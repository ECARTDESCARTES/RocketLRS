// RocketLRS stand-alone bridge: serial <-> SX1280 radio <-> serial.
//
// Air layer: RlrsAir (variable length frames, several RLRS messages per
// frame) scheduled by RlrsHop (50 ms TDMA frames + FHSS over 40 channels).
//   Ground (TARGET_TX, Ranger Nano, Java over USB) = RlrsHop master.
//   Rocket (TARGET_RX, RP1, STM32 over UART)      = RlrsHop slave.
// Radio presets come from RlrsProfiles.h (FlightLoRa by default,
// TransferFLRC on request), switched through RlrsHop on both sides.
//
// Scheduling: RlrsHop is driven from loop() with micros(), not from the
// ELRS hwTimer. RlrsHop/RlrsAir are not ISR code (no IRAM placement, the
// queue build and the SPI transfers would have to run in the timer ISR, and
// IRAM is short on the ESP8285). The DIO1 ISR of the driver does SPI itself
// (GetIrqStatus, FIFO read, ClearIrqStatus); our callbacks add
// GetRxBufferStatus and timestamp TxDone / RxDone with micros(), then post
// the frame in a mailbox, so the beacon sync does not depend on loop()
// jitter. loop() never blocks (no delay()).
//
// SPI exclusion: the SX1280 HAL is not reentrant. DIO1 only fires on TxDone
// / RxDone, i.e. while the radio is in TX or RX. Every loop() SPI sequence
// therefore starts with radioIdle(): SetTxIdleMode() (FS) under
// noInterrupts(), a few us. A DIO1 edge latched meanwhile runs its ISR right
// at interrupts(), before the next loop() SPI command; after that the radio
// stays silent until TXnb()/RXnb(), the last command of each sequence, so
// Config() (about 2 ms) runs with interrupts enabled. noInterrupts() masks
// the core that attached the DIO1 ISR (setup(), the loop() core on ESP32).
#include <Arduino.h>
#include <new>
#if defined(PLATFORM_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(PLATFORM_ESP32)
#include <WiFi.h>
#endif
#include "options.h"
#include "helpers.h"
#include "logging.h"
#include "SX1280Driver.h"
#include "RlrsParser.h"
#include "RlrsProtocol.h"
#include "RlrsAir.h"
#include "RlrsHop.h"
#include "RlrsLink.h"
#include "RlrsProfiles.h"
#include "FIFO.h"

// UART speed of the bridge (8N1), 420000 baud as the ELRS CRSF link. The
// Java program on the ground and the STM32 in the rocket must use the same
// value. STM32 USART, oversampling 16: BRR = fck / 420000 (84 MHz: 200,
// exact; 72/48/16/8 MHz: 0.25 % error), keep the error under 2 %. ESP8266
// integer divider: 80 MHz / 190 = 421053 baud (+0.25 %). The ground USB-UART
// adapter must accept a non standard rate: CH340, CP2102N and FTDI do;
// an old CP2102/CP2104 may need its baud rate alias table reprogrammed.
#ifndef RLRS_UART_BAUD
#define RLRS_UART_BAUD 420000
#endif

// Period of the LinkStats message (RLRS ID 0xF0) written to the ground UART.
#ifndef RLRS_LINKSTATS_PERIOD_US
#define RLRS_LINKSTATS_PERIOD_US 1000000UL
#endif

// RxDone IRQ latency compensated by RlrsHop when it syncs on a beacon: the
// timestamp is taken after the ISR entry, GetIrqStatus and the FIFO read of
// PayloadLength bytes (up to 127 o at 10 MHz, about 110 us). An estimate
// that is too high is harmless (the slave answers and closes its windows a
// little earlier, within windowUs); too low eats into the end guard of the
// master (RlrsHop keeps endGuard / 2 of margin for that).
#ifndef RLRS_RX_LATENCY_US
#define RLRS_RX_LATENCY_US 150
#endif

// Optional: -DRLRS_LINK_SEED=0x12345678 forces the FHSS seed / FLRC sync
// word. Otherwise the binding phrase UID is used, or a fixed default when
// no binding phrase is set (both modules must then use the same firmware).
// Optional: -DRLRS_POWER_INDEX=n caps the POWER_OUTPUT_VALUES index.
// Optional: -DRLRS_POWER_ALLOW_ABOVE_100MW lifts the 20 dBm EIRP cap.
// Optional: -DRLRS_ANTENNA_GAIN_DB=n (dBi, default 0) is subtracted from the
// 20 dBm EIRP cap before choosing the conducted level.
#ifndef RLRS_ANTENNA_GAIN_DB
#define RLRS_ANTENNA_GAIN_DB 0
#endif
// Optional: -DRLRS_KEEP_WIFI keeps the Wi-Fi as configured by the SDK.

static_assert(SX12XX_RX_BUFFER_SIZE >= RLRS_RADIO_FRAME_LIMIT,
              "SX12XX_RX_BUFFER_SIZE must hold the largest RlrsAir frame");
static_assert(RLRS_RADIO_FRAME_LIMIT <= RLRS_AIR_FRAME_MAX, "frame limit");

// Defined in SX1280.cpp. Used for the commands the driver does not expose:
// variable length packet params and GetRxBufferStatus (received length).
extern SX1280Hal hal;

namespace {
constexpr uint16_t UART_BUFFER_SIZE = 8 * RLRS_WIRE_MAX_SIZE;
constexpr uint32_t DEFAULT_LINK_SEED = 0x524C5253UL;  // "RLRS"
constexpr int POWER_LEVEL_COUNT = 8;                  // PowerLevels_e::PWR_COUNT
// Nominal conducted power of PowerLevels_e (10 mW .. 2000 mW).
constexpr int8_t POWER_LEVEL_DBM[POWER_LEVEL_COUNT] = {10, 14, 17, 20, 24, 27, 30, 33};
constexpr int EIRP_LIMIT_DBM = 20;                    // EN 300 328, 2.4 GHz, FHSS
constexpr uint8_t LINK_STATS_SIZE = 8;                // RlrsLink::enqueueLinkStats

#if defined(TARGET_TX)
constexpr RlrsHop::Role HOP_ROLE = RlrsHop::Master;
const char *const BRIDGE_ROLE = "JAVA/USB <-> radio (RlrsHop master)";
#else
constexpr RlrsHop::Role HOP_ROLE = RlrsHop::Slave;
const char *const BRIDGE_ROLE = "radio <-> STM32/UART (RlrsHop slave)";
#endif

struct RadioRegs
{
    uint8_t bw;   // LoRa BW or FLRC bitrate/BW
    uint8_t sf;   // LoRa SF or FLRC BT (SX1280Driver::Config passes it as BT)
    uint8_t cr;
    RadioBandMod::Combined modulation;
    bool valid;
};

SX1280Driver radio;
NullStream nullStream;
RlrsParser uartParser;
RlrsAirQueue airQueue;              // local UART messages waiting for the air
FIFO<UART_BUFFER_SIZE> uartTxQueue; // RLRS wire frames for the local UART
alignas(RlrsHop) uint8_t hopStorage[sizeof(RlrsHop)];
RlrsHop *hop = nullptr;             // placement new in startRadio(): no heap

// ---- ISR <-> loop() mailbox ------------------------------------------------
volatile bool rxEventPending = false;
volatile uint8_t rxEventLength = 0;    // 0 = CRC / header error
volatile uint32_t rxEventUs = 0;
volatile int8_t rxEventRssi = 0;
volatile int8_t rxEventSnr = 0;
volatile uint32_t rxOverruns = 0;
uint8_t rxMailbox[RLRS_AIR_FRAME_MAX];
volatile bool txEventPending = false;
volatile uint32_t txEventUs = 0;

// ---- loop() state ------------------------------------------------------------
bool radioReady = false;
bool radioTxInProgress = false;
bool radioRxActive = false;
bool rxDeadlineActive = false;
uint32_t rxDeadlineUs = 0;
uint32_t rxFrequencyHz = 0;
uint32_t timerAtUs = 0;
uint8_t activePreset = 0;
uint32_t linkSeed = DEFAULT_LINK_SEED;
uint32_t currentFrequencyHz = 0;
uint8_t radioTxFrame[RLRS_AIR_FRAME_MAX];
int8_t powerValue = 0;          // SX1280 SetOutputPower() value
uint8_t powerLevel = 0;         // PowerLevels_e of the selected index

// ---- statistics ----------------------------------------------------------------
uint32_t uartPacketsIn = 0;
uint32_t radioPacketsIn = 0;
uint32_t airQueueRefused = 0;
uint32_t airMalformed = 0;
uint32_t uartDrops = 0;
uint32_t rxIgnored = 0;
uint32_t presetLoads = 0;

struct SignalWindow
{
    int32_t rssiSum;
    int32_t snrSum;
    uint16_t count;
};
SignalWindow signalWindow = {0, 0, 0};
uint32_t linkStatsAtUs = 0;
uint32_t lastFramesTx = 0;
uint32_t lastFramesRx = 0;
uint32_t lastMissed = 0;
#if defined(TARGET_TX)
uint32_t profileRejected = 0;
uint8_t remoteUplink[3] = {0, 0, 0};   // rocket view: RSSI (-dBm), SNR, LQ
uint32_t remoteUplinkUs = 0;
bool remoteUplinkValid = false;
#endif

inline bool reached(uint32_t now, uint32_t at)
{
    return static_cast<int32_t>(now - at) >= 0;
}

uint32_t frequencyReg(uint32_t frequencyHz)
{
    return static_cast<uint32_t>(static_cast<double>(frequencyHz) / FREQ_STEP);
}

RadioRegs radioRegs(const RlrsRadioPresetParams &params)
{
    RadioRegs regs = {0, 0, 0, RadioBandMod::Combined::LORA_2G4, false};
    if (params.modulation == RlrsModulation::LoRa)
    {
        switch (params.loraBandwidthHz)
        {
        case 203125:  regs.bw = SX1280_LORA_BW_0200; break;
        case 406250:  regs.bw = SX1280_LORA_BW_0400; break;
        case 812500:  regs.bw = SX1280_LORA_BW_0800; break;
        case 1625000: regs.bw = SX1280_LORA_BW_1600; break;
        default: return regs;
        }
        switch (params.loraSf)
        {
        case 5:  regs.sf = SX1280_LORA_SF5; break;
        case 6:  regs.sf = SX1280_LORA_SF6; break;
        case 7:  regs.sf = SX1280_LORA_SF7; break;
        case 8:  regs.sf = SX1280_LORA_SF8; break;
        case 9:  regs.sf = SX1280_LORA_SF9; break;
        case 10: regs.sf = SX1280_LORA_SF10; break;
        default: return regs;
        }
        switch (params.loraCr)   // non interleaved coding rates only
        {
        case 1: regs.cr = SX1280_LORA_CR_4_5; break;
        case 2: regs.cr = SX1280_LORA_CR_4_6; break;
        case 3: regs.cr = SX1280_LORA_CR_4_7; break;
        case 4: regs.cr = SX1280_LORA_CR_4_8; break;
        default: return regs;
        }
        if (params.preamble == 0 || params.preamble > 15)
            return regs;
        regs.modulation = RadioBandMod::Combined::LORA_2G4;
        regs.valid = true;
        return regs;
    }

    switch (params.flrcBitrate)
    {
    case 1300000: regs.bw = SX1280_FLRC_BR_1_300_BW_1_2; break;
    case 1000000: regs.bw = SX1280_FLRC_BR_1_000_BW_1_2; break;
    case 650000:  regs.bw = SX1280_FLRC_BR_0_650_BW_0_6; break;
    default: return regs;
    }
    switch (params.flrcBtTenths)
    {
    case 0:  regs.sf = SX1280_FLRC_BT_DIS; break;
    case 5:  regs.sf = SX1280_FLRC_BT_0_5; break;
    case 10: regs.sf = SX1280_FLRC_BT_1; break;
    default: return regs;
    }
    switch (params.flrcCr)
    {
    case RLRS_FLRC_CR_1_2: regs.cr = SX1280_FLRC_CR_1_2; break;
    case RLRS_FLRC_CR_3_4: regs.cr = SX1280_FLRC_CR_3_4; break;
    case RLRS_FLRC_CR_1_0: regs.cr = SX1280_FLRC_CR_1_0; break;
    default: return regs;
    }
    if (params.preamble < 8 || params.preamble > 32 || (params.preamble % 4) != 0)
        return regs;
    regs.modulation = RadioBandMod::Combined::FLRC_2G4;
    regs.valid = true;
    return regs;
}

// Largest frame this node can receive with the active preset.
uint8_t rxFrameLimit()
{
    const RlrsHopPreset &preset = rlrsRadioPresetParams(activePreset).hop;
    return HOP_ROLE == RlrsHop::Master ? preset.downlinkMaxBytes : preset.uplinkMaxBytes;
}

// Radio to FS with the DIO1 ISR masked (see "SPI exclusion" above). Call it
// before any loop() SPI sequence; the radio raises no IRQ until TXnb/RXnb.
void radioIdle()
{
    noInterrupts();
    radio.SetTxIdleMode();
    interrupts();
}

// SX1280Driver::Config() always writes a fixed length (implicit) packet with
// the LoRa CRC off. This rewrites the packet params for the active preset:
// explicit header / variable length and hardware CRC. `length` is the TX
// length, or the largest accepted length before an RX.
void writePacketParams(uint8_t length)
{
    const RlrsRadioPresetParams &params = rlrsRadioPresetParams(activePreset);
    WORD_ALIGNED_ATTR uint8_t buf[7];
    if (params.modulation == RlrsModulation::FLRC)
    {
        buf[0] = static_cast<uint8_t>(((params.preamble / 4) - 1) << 4);  // AGC preamble
        buf[1] = SX1280_FLRC_SYNC_WORD_LEN_P32S;
        buf[2] = SX1280_FLRC_RX_MATCH_SYNC_WORD_1;
        buf[3] = SX1280_FLRC_PACKET_VARIABLE_LENGTH;
        buf[4] = length;
        buf[5] = SX1280_FLRC_CRC_3_BYTE;
        buf[6] = 0x08;   // whitening disabled (mandatory in FLRC)
    }
    else
    {
        buf[0] = params.preamble;
        buf[1] = params.explicitHeader ? SX1280_LORA_PACKET_EXPLICIT : SX1280_LORA_PACKET_IMPLICIT;
        buf[2] = length;
        buf[3] = params.crcOn ? SX1280_LORA_CRC_ON : SX1280_LORA_CRC_OFF;
        buf[4] = SX1280_LORA_IQ_NORMAL;
        buf[5] = 0x00;
        buf[6] = 0x00;
    }
    hal.WriteCommand(SX1280_RADIO_SET_PACKETPARAMS, buf, sizeof(buf), SX12XX_Radio_All, 20);
    // TXnb() writes PayloadLength bytes; RXnbISR() reads PayloadLength bytes.
    radio.PayloadLength = length;
}

bool applyPreset(uint8_t index)
{
    if (index >= RLRS_RADIO_PRESET_COUNT)
        return false;
    const RlrsRadioPresetParams &params = RLRS_RADIO_PRESETS[index];
    const RadioRegs regs = radioRegs(params);
    if (!regs.valid)
        return false;
    activePreset = index;
    radioIdle();   // the radio may still be in RX: no DIO1 during Config()
    // Config() goes through STDBY_RC (about 2 ms of SPI + busy time).
    radio.Config(regs.bw, regs.sf, regs.cr, frequencyReg(currentFrequencyHz),
                 params.preamble, false, rxFrameLimit(),
                 linkSeed, static_cast<uint16_t>(linkSeed ^ (linkSeed >> 16)),
                 regs.modulation);
    writePacketParams(rxFrameLimit());
    ++presetLoads;
    return true;
}

void setFrequency(uint32_t frequencyHz)
{
    currentFrequencyHz = frequencyHz;
    radio.SetFrequencyReg(frequencyReg(frequencyHz), SX12XX_Radio_All);
}

void stopRadio()
{
    radioIdle();             // FS: fast restart in TX or RX
    radioRxActive = false;
    rxDeadlineActive = false;
}

void startTransmit(uint32_t frequencyHz, const uint8_t *data, uint8_t length)
{
    stopRadio();
    if (!data || !length)
        return;
    memcpy(radioTxFrame, data, length);
    // FLRC needs >= 6 o: pad a bare beacon / header only downlink with
    // RLRS_AIR_PAD, which RlrsAirReader skips.
    const uint8_t minLength = rlrsRadioMinPayload(rlrsRadioPresetParams(activePreset));
    if (length < minLength)
        length = rlrsAirPad(radioTxFrame, length, minLength, sizeof(radioTxFrame));
    if (!length)
        return;
    writePacketParams(length);
    setFrequency(frequencyHz);
    noInterrupts();
    txEventPending = false;
    interrupts();
    radioTxInProgress = true;
    radio.TXnb(radioTxFrame, false, radioTxFrame, SX12XX_Radio_1);
}

void startReceive(uint32_t frequencyHz, uint32_t timeoutUs, uint32_t now)
{
    stopRadio();
    radioTxInProgress = false;
    writePacketParams(rxFrameLimit());
    setFrequency(frequencyHz);
    noInterrupts();
    rxEventPending = false;   // a result of the previous RX is stale now
    interrupts();
    radio.RXnb();             // RX continuous; the timeout is handled in loop()
    radioRxActive = true;
    rxFrequencyHz = frequencyHz;
    rxDeadlineActive = timeoutUs != 0;
    rxDeadlineUs = now + timeoutUs;
}

// Executes an RlrsHop action in the documented order.
void execute(const RlrsHopAction &action, uint32_t now)
{
    if (action.presetChanged && !applyPreset(action.preset))
        DBGLN("RocketLRS radio: preset %u rejected", action.preset);

    switch (action.type)
    {
    case RlrsHopAction::Transmit:
        startTransmit(action.frequencyHz, action.txData, action.txLength);
        break;
    case RlrsHopAction::Receive:
        startReceive(action.frequencyHz, action.rxTimeoutUs, now);
        break;
    case RlrsHopAction::Idle:
        stopRadio();
        radioTxInProgress = false;
        break;
    case RlrsHopAction::None:
    default:
        // Config() left the radio in standby: resume the current RX.
        if (action.presetChanged && radioRxActive)
        {
            const bool keepDeadline = rxDeadlineActive;
            const uint32_t deadline = rxDeadlineUs;
            startReceive(rxFrequencyHz, 0, now);
            rxDeadlineActive = keepDeadline;
            rxDeadlineUs = deadline;
        }
        break;
    }
    timerAtUs = action.timerAtUs;
}

// ---- radio ISR callbacks (ICACHE_RAM_ATTR, no allocation, no logging) ------

bool ICACHE_RAM_ATTR onRadioReceive(SX12xxDriverCommon::rx_status status)
{
    const uint32_t now = micros();
    if (rxEventPending)
    {
        ++rxOverruns;
        return false;
    }
    uint8_t length = 0;
    if (status == SX12xxDriverCommon::SX12XX_RX_OK)
    {
        // The driver only checks the CRC in FLRC mode; the LoRa CRC flag is
        // still set here (IRQs are cleared after this callback). Config()
        // does not enable HEADER_ERROR in the IRQ mask, and that is wanted:
        // a LoRa header error ends the reception without RX_DONE, and the
        // flag, not mapped on DIO1, would stay latched until the next good
        // packet and reject it. Only RX_DONE + CRC_ERROR matter here.
        const SX12XX_Radio_Number_t radioNumber = radio.GetProcessingPacketRadio();
        const uint16_t irq = radio.GetIrqStatus(radioNumber);
        if (!(irq & SX1280_IRQ_CRC_ERROR))
        {
            // Explicit header / variable length: the real length is in
            // GetRxBufferStatus; the driver copied PayloadLength bytes.
            WORD_ALIGNED_ATTR uint8_t bufferStatus[2] = {0, 0};
            hal.ReadCommand(SX1280_RADIO_GET_RXBUFFERSTATUS, bufferStatus, sizeof(bufferStatus), radioNumber);
            if (bufferStatus[0] > 0 && bufferStatus[0] <= radio.PayloadLength)
            {
                length = bufferStatus[0];
                memcpy(rxMailbox, radio.RXdataBuffer, length);
                radio.CheckForSecondPacket();
                radio.GetLastPacketStats();
                rxEventRssi = radio.LastPacketRSSI;
                rxEventSnr = radio.LastPacketSNRRaw;
            }
        }
    }
    rxEventLength = length;
    rxEventUs = now;
    rxEventPending = true;
    return length != 0;
}

void ICACHE_RAM_ATTR onRadioTransmit()
{
    txEventUs = micros();
    txEventPending = true;
}

// ---- UART <-> air ----------------------------------------------------------

void forwardToUart(uint8_t id, const uint8_t *payload, uint8_t length)
{
    uint8_t frame[RLRS_WIRE_MAX_SIZE];
    const size_t size = rlrsEncode(id, payload, length, frame, sizeof(frame));
    if (!size)
    {
        ++airMalformed;
        return;
    }
    if (uartTxQueue.free() < size)
    {
        ++uartDrops;
        return;
    }
    uartTxQueue.atomicPushBytes(frame, static_cast<uint16_t>(size));
    ++radioPacketsIn;
}

#if defined(TARGET_TX)
void requestProfile(const uint8_t *payload, uint8_t length)
{
    RlrsProfile profile;
    if (!hop || length < 1 || !rlrsProfileFromByte(payload[0], profile))
    {
        ++profileRejected;
        return;
    }
    const uint8_t preset = rlrsRadioPresetIndex(rlrsProfileSettings(profile).radio);
    if (preset == hop->preset())
        return;
    if (!hop->requestPreset(preset))   // link down or switch already pending
        ++profileRejected;
}

void storeRemoteUplink(const uint8_t *payload, uint8_t length)
{
    if (length < LINK_STATS_SIZE)
        return;
    remoteUplink[0] = payload[0];
    remoteUplink[1] = payload[1];
    remoteUplink[2] = payload[2];
    remoteUplinkUs = micros();
    remoteUplinkValid = true;
}
#endif

void onUartPacket(RlrsUartPacket packet, void *)
{
#if defined(TARGET_TX)
    if (packet.packet_id == RLRS_PROFILE_REQUEST_ID)
    {
        requestProfile(packet.payload, packet.payload_len);
        return;
    }
#endif
    if (airQueue.push(packet.packet_id, packet.payload, packet.payload_len))
        ++uartPacketsIn;
    else
        ++airQueueRefused;   // too big for the smallest frame, or queue full
}

void deliverAirFrame(const uint8_t *frame, uint8_t length)
{
    RlrsAirReader reader;
    if (!reader.begin(frame, length))
    {
        ++airMalformed;
        return;
    }
    uint8_t id = 0;
    const uint8_t *payload = nullptr;
    uint8_t size = 0;
    while (reader.next(id, payload, size))
    {
#if defined(TARGET_TX)
        if (id == RlrsLink::LinkStatsId)
        {
            storeRemoteUplink(payload, size);
            continue;
        }
        if (id == RLRS_PROFILE_REQUEST_ID)
        {
            requestProfile(payload, size);
            continue;
        }
#endif
        forwardToUart(id, payload, size);
    }
    if (reader.malformed())
        ++airMalformed;
}

// ---- link statistics ---------------------------------------------------------

uint8_t rssiToPositive(int32_t rssi)
{
    if (rssi >= 0)
        return 0;
    return rssi < -255 ? 255 : static_cast<uint8_t>(-rssi);
}

uint8_t percent(uint32_t part, uint32_t whole)
{
    if (!whole)
        return 0;
    const uint32_t value = part * 100u / whole;
    return value > 100u ? 100u : static_cast<uint8_t>(value);
}

void serviceLinkStats(uint32_t now)
{
    if (!hop || !reached(now, linkStatsAtUs + RLRS_LINKSTATS_PERIOD_US))
        return;
    linkStatsAtUs = now;

    uint8_t rssi = 0;
    int8_t snr = 0;
    if (signalWindow.count)
    {
        rssi = rssiToPositive(signalWindow.rssiSum / signalWindow.count);
        snr = static_cast<int8_t>(signalWindow.snrSum / signalWindow.count / RADIO_SNR_SCALE);
    }
    signalWindow.rssiSum = 0;
    signalWindow.snrSum = 0;
    signalWindow.count = 0;

    const RlrsHop::Stats &stats = hop->stats();
    const uint32_t framesTx = stats.framesTx - lastFramesTx;
    const uint32_t framesRx = stats.framesRx - lastFramesRx;
    const uint32_t missed = stats.missed - lastMissed;
    lastFramesTx = stats.framesTx;
    lastFramesRx = stats.framesRx;
    lastMissed = stats.missed;

    uint8_t payload[LINK_STATS_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};
    payload[6] = powerLevel;
    payload[7] = hop->preset();
#if defined(TARGET_TX)
    // Downlink measured here, uplink as reported by the rocket.
    (void)missed;
    const bool fresh = remoteUplinkValid &&
                       !reached(now, remoteUplinkUs + 3u * RLRS_LINKSTATS_PERIOD_US);
    if (fresh)
    {
        payload[0] = remoteUplink[0];
        payload[1] = remoteUplink[1];
        payload[2] = remoteUplink[2];
    }
    payload[3] = rssi;
    payload[4] = static_cast<uint8_t>(snr);
    payload[5] = percent(framesRx, framesTx);
    forwardToUart(RlrsLink::LinkStatsId, payload, sizeof(payload));
#else
    // Rocket: uplink (beacons) measured here, sent down to the ground.
    (void)framesTx;
    payload[0] = rssi;
    payload[1] = static_cast<uint8_t>(snr);
    payload[2] = percent(framesRx, framesRx + missed);
    if (hop->synced())
        airQueue.push(RlrsLink::LinkStatsId, payload, sizeof(payload));
#endif
}

// ---- radio servicing ---------------------------------------------------------

void handleRxEvent(const uint8_t *frame, uint8_t length, uint32_t rxUs, int8_t rssi, int8_t snr)
{
    if (!radioRxActive)
    {
        ++rxIgnored;   // arrived after the RX was closed
        return;
    }
    radioRxActive = false;
    rxDeadlineActive = false;
    uint32_t hopNow = rxUs;
    if (length)
        hopNow += rlrsHopSyncShiftUs(rlrsRadioPresetParams(activePreset), length);
    const RlrsHopAction action = hop->onRxDone(hopNow, length ? frame : nullptr, length);
    execute(action, rxUs);
    if (action.rxDelivered)
    {
        signalWindow.rssiSum += rssi;
        signalWindow.snrSum += snr;
        ++signalWindow.count;
        deliverAirFrame(frame, length);
    }
}

bool takeRxEvent(uint8_t *frame, uint8_t &length, uint32_t &rxUs, int8_t &rssi, int8_t &snr)
{
    bool ready = false;
    noInterrupts();
    if (rxEventPending)
    {
        length = rxEventLength;
        if (length)
            memcpy(frame, rxMailbox, length);
        rxUs = rxEventUs;
        rssi = rxEventRssi;
        snr = rxEventSnr;
        rxEventPending = false;
        ready = true;
    }
    interrupts();
    return ready;
}

void serviceRadio()
{
    if (!radioReady || !hop)
        return;

    bool txDone = false;
    uint32_t txUs = 0;
    noInterrupts();
    if (txEventPending)
    {
        txDone = true;
        txUs = txEventUs;
        txEventPending = false;
    }
    interrupts();
    if (txDone && radioTxInProgress)
    {
        radioTxInProgress = false;
        execute(hop->onTxDone(txUs), txUs);
    }

    uint8_t frame[RLRS_AIR_FRAME_MAX];
    uint8_t length = 0;
    uint32_t rxUs = 0;
    int8_t rssi = 0;
    int8_t snr = 0;
    if (takeRxEvent(frame, length, rxUs, rssi, snr))
        handleRxEvent(frame, length, rxUs, rssi, snr);

    uint32_t now = micros();
    if (radioRxActive && rxDeadlineActive && reached(now, rxDeadlineUs))
    {
        stopRadio();
        // A frame completed just before the radio stopped wins.
        if (takeRxEvent(frame, length, rxUs, rssi, snr))
        {
            radioRxActive = true;
            handleRxEvent(frame, length, rxUs, rssi, snr);
        }
        else
        {
            execute(hop->onRxDone(now, nullptr, 0), now);
        }
    }

    now = micros();
    if (reached(now, timerAtUs))
        execute(hop->onTimer(now), now);
}

// ---- setup helpers -----------------------------------------------------------

// Highest calibrated level of the hardware profile (last index of
// POWER_OUTPUT_VALUES, index 0 = power_min), capped by power_max, by
// RLRS_POWER_INDEX and, unless RLRS_POWER_ALLOW_ABOVE_100MW, by the 20 dBm
// EIRP limit (EN 300 328 at 2.4 GHz, FHSS active). PowerLevels_e values are
// CONDUCTED powers (100 mW = 20 dBm at the antenna connector, nominal): the
// highest level with level dBm + RLRS_ANTENNA_GAIN_DB <= 20 is kept, i.e.
// 100 mW conducted with a 0 dBi antenna, 50 mW with 2 to 3 dBi, ...
bool selectPower()
{
    const int count = POWER_OUTPUT_VALUES_COUNT;
    if (count < 1 || POWER_OUTPUT_VALUES == nullptr)
        return false;
    int index = count - 1;
    const int minLevel = hardware_int(HARDWARE_power_min);
    const int maxLevel = hardware_int(HARDWARE_power_max);
    if (maxLevel >= minLevel && index > maxLevel - minLevel)
        index = maxLevel - minLevel;
#if defined(RLRS_POWER_INDEX)
    if (index > (RLRS_POWER_INDEX))
        index = (RLRS_POWER_INDEX);
#endif
#if !defined(RLRS_POWER_ALLOW_ABOVE_100MW)
    int capLevel = -1;   // highest PowerLevels_e within the EIRP limit
    for (int level = 0; level < POWER_LEVEL_COUNT; ++level)
        if (POWER_LEVEL_DBM[level] + (RLRS_ANTENNA_GAIN_DB) <= EIRP_LIMIT_DBM)
            capLevel = level;
    if (index > capLevel - minLevel)
        index = capLevel - minLevel;   // < 0: power_min is already above it
#endif
    if (index < 0)
        index = 0;
    powerValue = static_cast<int8_t>(POWER_OUTPUT_VALUES[index]);
    powerLevel = static_cast<uint8_t>(minLevel + index);
    return true;
}

uint32_t selectLinkSeed()
{
#if defined(RLRS_LINK_SEED)
    return static_cast<uint32_t>(RLRS_LINK_SEED);
#else
    if (firmwareOptions.hasUID)
        return rlrsHopSeedFromUid(firmwareOptions.uid, sizeof(firmwareOptions.uid));
    return DEFAULT_LINK_SEED;
#endif
}

void disableWifi()
{
#if !defined(RLRS_KEEP_WIFI)
#if defined(PLATFORM_ESP8266) || defined(PLATFORM_ESP32)
    // The SDK keeps the last Wi-Fi mode in flash and restores it at boot
    // (access point on these test targets): turn it off explicitly.
    WiFi.persistent(false);
    WiFi.mode(WIFI_OFF);
#if defined(PLATFORM_ESP8266)
    WiFi.forceSleepBegin();
#endif
#endif
#endif
}

void startRadio()
{
    if (GPIO_PIN_SCK == UNDEF_PIN || GPIO_PIN_NSS == UNDEF_PIN ||
        GPIO_PIN_DIO1 == UNDEF_PIN || hardware_int(HARDWARE_power_control) != 0 ||
        !selectPower())
    {
        DBGLN("RocketLRS radio: unsupported/missing hardware layout");
        return;
    }
    for (uint8_t i = 0; i < RLRS_RADIO_PRESET_COUNT; ++i)
    {
        if (!radioRegs(RLRS_RADIO_PRESETS[i]).valid)
        {
            DBGLN("RocketLRS radio: preset %u not supported by the SX1280", i);
            return;
        }
    }
    if (rlrsRadioMaxFrameBytes() > RLRS_RADIO_FRAME_LIMIT)
    {
        DBGLN("RocketLRS radio: preset frame larger than the RX buffer");
        return;
    }

    linkSeed = selectLinkSeed();
    RlrsHopConfig config = rlrsHopConfigFromPresets(linkSeed);
    config.rxLatencyUs = RLRS_RX_LATENCY_US;
    if (!rlrsHopConfigValid(config))
    {
        DBGLN("RocketLRS radio: invalid RlrsHop configuration");
        return;
    }

    if (GPIO_PIN_ANT_CTRL != UNDEF_PIN)
    {
        pinMode(GPIO_PIN_ANT_CTRL, OUTPUT);
        digitalWrite(GPIO_PIN_ANT_CTRL, LOW);
    }
    if (!radio.Begin())
    {
        DBGLN("RocketLRS radio: SX1280 not detected");
        return;
    }

    // A message must fit the smallest frame of every preset in its direction.
    airQueue.setMaxFrameSize(HOP_ROLE == RlrsHop::Master ? rlrsRadioMinUplinkBytes()
                                                         : rlrsRadioMinDownlinkBytes());
    hop = new (hopStorage) RlrsHop(HOP_ROLE, config, &airQueue);
    currentFrequencyHz = config.firstFrequencyHz;

    radio.RXdoneCallback = onRadioReceive;
    radio.TXdoneCallback = onRadioTransmit;
    // Applied by the driver after the first TxDone (the first frame leaves
    // at the power set by Begin()).
    radio.SetOutputPower(powerValue);
    radioReady = true;

    const uint32_t now = micros();
    linkStatsAtUs = now;
    execute(hop->start(now), now);   // presetChanged: loads the default preset
    DBGLN("RocketLRS bridge: %s, seed %x", BRIDGE_ROLE, static_cast<unsigned>(linkSeed));
}

void readUart()
{
    uint8_t bytes[64];
    while (Serial.available() > 0)
    {
        const int available = Serial.available();
        const size_t count = available < static_cast<int>(sizeof(bytes))
            ? static_cast<size_t>(available) : sizeof(bytes);
        const size_t received = Serial.readBytes(bytes, count);
        if (received)
            uartParser.processBytes(bytes, static_cast<uint16_t>(received));
        else
            break;
    }
}

void writeUart()
{
    const int writable = Serial.availableForWrite();
    if (writable <= 0)
        return;

    uint8_t bytes[RLRS_WIRE_MAX_SIZE];
    const uint16_t limit = writable < static_cast<int>(sizeof(bytes))
        ? static_cast<uint16_t>(writable) : static_cast<uint16_t>(sizeof(bytes));
    uartTxQueue.lock();
    const uint16_t queued = uartTxQueue.size();
    const uint16_t count = queued < limit ? queued : limit;
    if (count)
        uartTxQueue.popBytes(bytes, count);
    uartTxQueue.unlock();

    if (count)
        Serial.write(bytes, count);
}
} // namespace

void setup()
{
    Serial.begin(RLRS_UART_BAUD);
    Serial.setTimeout(2);
    BackpackOrLogStrm = &nullStream;   // the UART carries binary RLRS frames

    disableWifi();
    uartParser.setHandler(onUartPacket, nullptr);

    if (options_init())
        startRadio();
    else
        DBGLN("RocketLRS bridge: hardware layout missing");
}

void loop()
{
    readUart();
    serviceRadio();
    serviceLinkStats(micros());
    writeUart();
    yield();
}
