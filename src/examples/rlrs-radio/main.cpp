// Transparent RocketLRS bridge over one fixed 69-byte SX1280 radio frame.
#include <Arduino.h>
#include "options.h"
#include "logging.h"
#include "SX1280Driver.h"
#include "RlrsParser.h"
#include "RlrsProtocol.h"
#include "FIFO.h"

namespace {
constexpr uint8_t RADIO_FRAME_SIZE = RLRS_WIRE_MAX_SIZE;
constexpr uint32_t RADIO_FREQUENCY_HZ = 2440000000UL;
constexpr uint16_t BRIDGE_BUFFER_SIZE = 8 * RADIO_FRAME_SIZE;

SX1280Driver radio;
RlrsParser uartParser;
RlrsParser radioParser;
FIFO<BRIDGE_BUFFER_SIZE> radioTxQueue;
FIFO<BRIDGE_BUFFER_SIZE> uartTxQueue;

volatile bool radioFrameReady = false;
volatile bool radioTxDone = false;
bool radioTxInProgress = false;
uint8_t radioMailbox[RADIO_FRAME_SIZE];
uint8_t radioTxFrame[RADIO_FRAME_SIZE];
bool radioReady = false;

uint32_t uartPacketsIn = 0;
uint32_t radioPacketsIn = 0;
uint32_t crcRejected = 0;
uint32_t queueDrops = 0;

#if defined(TARGET_TX)
constexpr char *BRIDGE_ROLE = "JAVA/USB <-> radio";
#else
constexpr char *BRIDGE_ROLE = "radio <-> STM32/UART";
#endif

bool ICACHE_RAM_ATTR onRadioReceive(SX12xxDriverCommon::rx_status status)
{
    if (status != SX12xxDriverCommon::SX12XX_RX_OK || radioFrameReady)
        return false;
    memcpy(radioMailbox, radio.RXdataBuffer, RADIO_FRAME_SIZE);
    radioFrameReady = true;
    return true;
}

void ICACHE_RAM_ATTR onRadioTransmit() { radioTxDone = true; }

void queueRadioPacket(RlrsUartPacket packet, void *)
{
    uint8_t frame[RADIO_FRAME_SIZE] = {};
    const size_t length = rlrsEncode(packet.packet_id, packet.payload,
                                     packet.payload_len, frame, sizeof(frame));
    if (!length || !rlrsValidate(frame, length))
    {
        ++crcRejected;
        return;
    }
    // Fixed-length radio mode: pad AFTER the logical RocketLRS frame and CRC.
    radioTxQueue.atomicPushBytes(frame, RADIO_FRAME_SIZE);
    ++uartPacketsIn;
}

void queueUartPacket(RlrsUartPacket packet, void *)
{
    uint8_t frame[RLRS_WIRE_MAX_SIZE];
    const size_t length = rlrsEncode(packet.packet_id, packet.payload,
                                     packet.payload_len, frame, sizeof(frame));
    if (!length)
    {
        ++crcRejected;
        return;
    }
    uartTxQueue.atomicPushBytes(frame, length);
    ++radioPacketsIn;
}

void startRadio()
{
    if (GPIO_PIN_SCK == UNDEF_PIN || GPIO_PIN_NSS == UNDEF_PIN ||
        GPIO_PIN_DIO1 == UNDEF_PIN || hardware_int(HARDWARE_power_control) != 0 ||
        POWER_OUTPUT_VALUES_COUNT < 1)
    {
        DBGLN("RocketLRS radio: unsupported/missing hardware layout");
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

    radio.Config(SX1280_LORA_BW_0800, SX1280_LORA_SF7, SX1280_LORA_CR_4_7,
        static_cast<uint32_t>(static_cast<double>(RADIO_FREQUENCY_HZ) / FREQ_STEP),
        12, false, RADIO_FRAME_SIZE, 0, 0, RadioBandMod::Combined::LORA_2G4);
    radio.SetOutputPower(static_cast<int8_t>(POWER_OUTPUT_VALUES[0]));
    radio.RXdoneCallback = onRadioReceive;
    radio.TXdoneCallback = onRadioTransmit;
    radio.SetFrequencyReg(static_cast<uint32_t>(
        static_cast<double>(RADIO_FREQUENCY_HZ) / FREQ_STEP), SX12XX_Radio_All);
    radioReady = true;
    radio.RXnb();
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

void handleRadioFrame()
{
    uint8_t frame[RADIO_FRAME_SIZE];
    bool ready = false;

    noInterrupts();
    if (radioFrameReady)
    {
        memcpy(frame, radioMailbox, sizeof(frame));
        radioFrameReady = false;
        ready = true;
    }
    interrupts();

    if (!ready)
        return;

    const size_t logicalLength = static_cast<size_t>(frame[1]) + 5;
    if (logicalLength > sizeof(frame) || !rlrsValidate(frame, logicalLength))
    {
        ++crcRejected;
        return;
    }

    // Each SX1280 frame is one complete RocketLRS message.
    radioParser.reset();
    radioParser.setHandler(queueUartPacket, nullptr);
    radioParser.processBytes(frame, static_cast<uint16_t>(logicalLength));
}

void writeUart()
{
    const int writable = Serial.availableForWrite();
    if (writable <= 0)
        return;

    uint8_t bytes[RLRS_WIRE_MAX_SIZE];
    uartTxQueue.lock();
    const uint16_t queued = uartTxQueue.size();
    const uint16_t count = queued < writable ? queued : static_cast<uint16_t>(writable);
    if (count)
        uartTxQueue.popBytes(bytes, count);
    uartTxQueue.unlock();

    if (count)
        Serial.write(bytes, count);
}

void sendRadioFrame()
{
    if (!radioReady || radioTxInProgress || !radioTxQueue.size())
        return;

    radioTxQueue.lock();
    if (radioTxQueue.size() < RADIO_FRAME_SIZE)
    {
        radioTxQueue.unlock();
        return;
    }
    radioTxQueue.popBytes(radioTxFrame, RADIO_FRAME_SIZE);
    radioTxQueue.unlock();

    radio.SetTxIdleMode();
    radioTxDone = false;
    radioTxInProgress = true;
    radio.TXnb(radioTxFrame, false, radioTxFrame, SX12XX_Radio_1);
}
} // namespace

void setup()
{
    Serial.begin(9600);
    Serial.setTimeout(2);
    BackpackOrLogStrm = new NullStream();

    uartParser.setHandler(queueRadioPacket, nullptr);
    radioParser.setHandler(queueUartPacket, nullptr);

    if (options_init())
        startRadio();
    else
        DBGLN("RocketLRS bridge: hardware layout missing");
}

void loop()
{
    readUart();
    if (radioTxDone)
    {
        radioTxDone = false;
        radioTxInProgress = false;
        radio.RXnb();
    }
    handleRadioFrame();
    writeUart();
    sendRadioFrame();
    delay(1);
}
