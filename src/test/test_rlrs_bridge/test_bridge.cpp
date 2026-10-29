// Host integration test of examples/rlrs-radio/main.cpp (DUT, the real
// file) against an RlrsHop peer of the opposite role, through a simulated
// air with injected latencies:
//   - SPI: a TX leaves the antenna 20..120 us after TXnb(), an RX opens
//     10..60 us after the peer's Receive action;
//   - ISR: DIO1 entry 5..20 us after the end of the packet, RxDone
//     timestamp 60..260 us after it (FIFO read), TxDone 15..40 us;
//   - loop(): stalls of 0..250 us (2 % of the iterations), peer timers late
//     by 0..200 us.
// Checks: no collision, every receiver that was listening when a packet
// started is still listening when it ends (no RX closed early), no RX opened
// after the start of a packet it then "receives" (no RX opened late), no
// loop() SPI while the radio can raise DIO1 with the interrupts unmasked,
// FLRC frames >= 6 o, uplink and downlink saturated with small messages
// delivered in order.
#include "Arduino.h"
#include "options.h"
#include "logging.h"
#include "WiFi.h"
#include "SX1280Driver.h"
#include <cstdio>
#include <vector>

uint32_t g_micros = 4294000000u;  // micros() wraps during the run
bool g_irqMasked = false;
bool g_inIsr = false;
bool g_radioCanIrq = false;
uint32_t g_spiRaces = 0;
uint32_t g_spiCommands = 0;
HardwareSerialStub Serial;
Stream *BackpackOrLogStrm = nullptr;
WiFiStub WiFi;
int g_hw[3] = {0, 0, 5};            // power_control, power_min (10 mW), power_max (500 mW)
int16_t g_powerValues[8] = {-17, -13, -9, -6, -2, 1, 0, 0};
int g_powerCount = 6;
firmware_options_t firmwareOptions = {0, {0, 0, 0, 0, 0, 0}};

#include "main.cpp"

SX1280Hal hal;

static int failures = 0, checks = 0;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

#ifndef BENCH_SEED
#define BENCH_SEED 0x2545F491u
#endif
static uint32_t rngState = BENCH_SEED;
static uint32_t rnd()
{
    rngState ^= rngState << 13;
    rngState ^= rngState >> 17;
    rngState ^= rngState << 5;
    return rngState;
}
static uint32_t jitter(uint32_t lo, uint32_t hi) { return lo + rnd() % (hi - lo + 1u); }

struct Air
{
    bool active, started, listenAtStart;
    uint32_t start, end, freqReg;
    uint8_t preset, len;
    uint8_t data[256];
};
static Air dutAir, peerAir;
static uint32_t collisions = 0, closedEarly = 0, openedLate = 0, flrcShort = 0;
// RX opened in the middle of a packet while parked in acquisition (slave
// listening continuously on a channel it just picked): a normal miss.
static uint32_t lateInAcquire = 0;
// Downlink end vs the master's next frame start (us): the closest one, and
// how many ended inside the end guard (endGuardUs) of the master.
static int32_t minDownlinkMarginUs = 1000000;
static uint32_t downlinksInEndGuard = 0;
static void downlinkMargin(const RlrsHop &master, uint32_t end)
{
    const int32_t margin = static_cast<int32_t>(master.frameStartUs() + master.config().framePeriodUs - end);
    if (margin < minDownlinkMarginUs) minDownlinkMarginUs = margin;
    if (margin < static_cast<int32_t>(master.config().endGuardUs)) ++downlinksInEndGuard;
}

// DUT radio ISRs waiting to run (entry time, timestamp)
struct Isr { bool pending; uint32_t at, stamp; uint8_t len; uint8_t data[256]; };
static Isr dutRxIsr, dutTxIsr;
static uint32_t isrOverlap = 0;

#if defined(TARGET_TX)
static const RlrsHop::Role PEER_ROLE = RlrsHop::Slave;
#else
static const RlrsHop::Role PEER_ROLE = RlrsHop::Master;
#endif
static RlrsAirQueue peerQueue;
alignas(RlrsHop) static uint8_t peerStorage[sizeof(RlrsHop)];
static RlrsHop *peer;
static uint8_t peerPreset = 0;
static bool peerRx = false;
static uint32_t peerRxFreq = 0, peerRxDeadline = 0, peerTimer = 0, peerTimerLate = 0, peerRxOpenedUs = 0;
static bool peerRxDeadlineOn = false;
struct Msg { uint8_t id; std::vector<uint8_t> p; };
static std::vector<Msg> peerGot;
static uint8_t peerLastFrame[256];
static uint8_t peerLastLen = 0;

static bool peerListening(const Air &a)
{
    return peerRx && peerRxFreq == a.freqReg && peerPreset == a.preset;
}
static bool dutListening(const Air &a)
{
    return radio.mode == SX1280Driver::Rx && radio.currFreq == a.freqReg && activePreset == a.preset;
}

static void peerApply(const RlrsHopAction &a, uint32_t now)
{
    if (a.presetChanged) peerPreset = a.preset;
    switch (a.type)
    {
    case RlrsHopAction::Transmit:
    {
        peerRx = false;
        CHECK(!peerAir.active);
        // The peer is another bridge: same FLRC padding as main.cpp.
        memcpy(peerAir.data, a.txData, a.txLength);
        uint8_t len = a.txLength;
        const uint8_t minLen = rlrsRadioMinPayload(rlrsRadioPresetParams(peerPreset));
        if (len < minLen) len = rlrsAirPad(peerAir.data, len, minLen, RLRS_AIR_FRAME_MAX);
        peerAir.active = true;
        peerAir.started = false;
        peerAir.start = now + jitter(20, 120);
        peerAir.end = peerAir.start + rlrsRadioAirtimeUs(rlrsRadioPresetParams(peerPreset), len);
        peerAir.freqReg = frequencyReg(a.frequencyHz);
        peerAir.preset = peerPreset;
        peerAir.len = len;
        break;
    }
    case RlrsHopAction::Receive:
        peerRx = true;
        peerRxFreq = frequencyReg(a.frequencyHz);
        peerRxDeadlineOn = a.rxTimeoutUs != 0;
        peerRxDeadline = now + a.rxTimeoutUs;
        peerRxOpenedUs = now + jitter(10, 60);
        break;
    case RlrsHopAction::Idle: peerRx = false; break;
    default: break;
    }
    peerTimer = a.timerAtUs;
    peerTimerLate = jitter(0, 200);
    if (a.rxDelivered)
    {
        RlrsAirReader r; uint8_t id; const uint8_t *p; uint8_t l;
        if (r.begin(peerLastFrame, peerLastLen))
            while (r.next(id, p, l)) { Msg m; m.id = id; m.p.assign(p, p + l); peerGot.push_back(m); }
        CHECK(!r.malformed());
    }
}

static std::vector<Msg> parseUart(const std::vector<uint8_t> &out)
{
    std::vector<Msg> v; size_t i = 0;
    while (i + 5 <= out.size())
    {
        if (out[i] != 0xAA) { ++i; continue; }
        const size_t n = out[i + 1] + 5u;
        if (i + n <= out.size() && rlrsValidate(&out[i], n))
        {
            Msg m; m.id = out[i + 2];
            m.p.assign(out.begin() + i + 3, out.begin() + i + 3 + out[i + 1]);
            v.push_back(m); i += n;
        }
        else ++i;
    }
    return v;
}

static void uartSend(uint8_t id, const uint8_t *p, uint8_t l)
{
    uint8_t f[RLRS_WIRE_MAX_SIZE];
    const size_t n = rlrsEncode(id, p, l, f, sizeof(f));
    for (size_t i = 0; i < n; ++i) Serial.in.push_back(f[i]);
}

static uint32_t dutTxSeen = 0;
static uint32_t lossCounter = 0;
static bool lostNow(unsigned lossPercent) { return (++lossCounter * 37u) % 100u < lossPercent; }

static void runIsr(Isr &isr, bool rx)
{
    isr.pending = false;
    g_inIsr = true;
    if (reached(isr.stamp, g_micros)) g_micros = isr.stamp;   // the ISR takes that long
    if (rx)
    {
        const uint8_t copy = isr.len < radio.PayloadLength ? isr.len : radio.PayloadLength;
        memcpy(radio.RXdataBuffer, isr.data, copy);
        hal.rxLength = isr.len;
        radio.irq = SX1280_IRQ_RX_DONE;
        radio.processingPacketRadio = SX12XX_Radio_1;
        radio.LastPacketRSSI = -81;
        radio.LastPacketSNRRaw = 22;
        radio.RXdoneCallback(SX12xxDriverCommon::SX12XX_RX_OK);
        radio.irq = 0;
    }
    else
    {
        radio.TXdoneCallback();
    }
    g_inIsr = false;
}

static void air(uint32_t t, unsigned lossPercent)
{
    // packet starts: who is listening
    if (dutAir.active && !dutAir.started && reached(t, dutAir.start))
    {
        dutAir.started = true;
        dutAir.listenAtStart = peerListening(dutAir);
        if (peerAir.active && peerAir.started) ++collisions;
    }
    if (peerAir.active && !peerAir.started && reached(t, peerAir.start))
    {
        peerAir.started = true;
        peerAir.listenAtStart = dutListening(peerAir);
        if (dutAir.active && dutAir.started) ++collisions;
    }
    // DUT packet ends
    if (dutAir.active && dutAir.started && reached(t, dutAir.end))
    {
        dutAir.active = false;
        if (PEER_ROLE == RlrsHop::Master) downlinkMargin(*peer, dutAir.end);
        radio.setMode(SX1280Driver::FS);
        if (dutTxIsr.pending) ++isrOverlap;
        dutTxIsr.pending = true;
        dutTxIsr.at = dutAir.end + jitter(5, 20);
        dutTxIsr.stamp = dutAir.end + jitter(15, 40);
        const bool listening = peerListening(dutAir);
        const bool openBefore = reached(dutAir.start, peerRxOpenedUs);
        if (listening && !openBefore)
        {
            if (peer->state() == RlrsHop::StateAcquire) ++lateInAcquire;
            else { ++openedLate; std::printf("late: peer RX opened %d us after the DUT packet start\n", static_cast<int>(peerRxOpenedUs - dutAir.start)); }
        }
        if (!listening && dutAir.listenAtStart) ++closedEarly;
        if (listening && openBefore && !lostNow(lossPercent))
        {
            peerRx = false;
            memcpy(peerLastFrame, dutAir.data, dutAir.len);
            peerLastLen = dutAir.len;
            // RxDone ISR of the peer bridge: timestamp late by 60..260 us
            const uint32_t stamp = dutAir.end + jitter(60, 260);
            uint32_t now = stamp;
            if (PEER_ROLE == RlrsHop::Slave)
                now += rlrsHopSyncShiftUs(rlrsRadioPresetParams(peerPreset), dutAir.len);
            peerApply(peer->onRxDone(now, peerLastFrame, peerLastLen), stamp);
        }
    }
    // peer packet ends
    if (peerAir.active && peerAir.started && reached(t, peerAir.end))
    {
        peerAir.active = false;
        if (PEER_ROLE == RlrsHop::Slave) downlinkMargin(*hop, peerAir.end);
        peerApply(peer->onTxDone(peerAir.end + jitter(15, 40)), peerAir.end);
        const bool listening = dutListening(peerAir);
        const bool openBefore = reached(peerAir.start, radio.rxOpenedUs);
        if (listening && !openBefore)
        {
            if (hop->state() == RlrsHop::StateAcquire) ++lateInAcquire;
            else { ++openedLate; std::printf("late: DUT RX opened %d us after the peer packet start\n", static_cast<int>(radio.rxOpenedUs - peerAir.start)); }
        }
        if (!listening && peerAir.listenAtStart) ++closedEarly;
        if (listening && openBefore && !lostNow(lossPercent))
        {
            if (dutRxIsr.pending) ++isrOverlap;
            dutRxIsr.pending = true;
            dutRxIsr.at = peerAir.end + jitter(5, 20);
            dutRxIsr.stamp = peerAir.end + jitter(60, 260);
            dutRxIsr.len = peerAir.len;
            memcpy(dutRxIsr.data, peerAir.data, peerAir.len);
        }
    }
    if (dutTxIsr.pending && reached(t, dutTxIsr.at)) runIsr(dutTxIsr, false);
    if (dutRxIsr.pending && reached(t, dutRxIsr.at)) runIsr(dutRxIsr, true);
}

static void peerService(uint32_t t)
{
    if (peerRx && peerRxDeadlineOn && reached(t, peerRxDeadline))
    {
        peerRx = false;
        peerApply(peer->onRxDone(t, nullptr, 0), t);
    }
    if (reached(t, peerTimer + peerTimerLate)) peerApply(peer->onTimer(t), t);
}

static void dutTxCheck(uint32_t t)
{
    if (radio.txCount == dutTxSeen)
        return;
    dutTxSeen = radio.txCount;
    CHECK(radio.txLen == hal.lastPacketParams[activePreset == 0 ? 2 : 4]);
    if (activePreset == 0)
    {
        CHECK(hal.lastPacketParams[1] == SX1280_LORA_PACKET_EXPLICIT);
        CHECK(hal.lastPacketParams[3] == SX1280_LORA_CRC_ON);
    }
    else
    {
        CHECK(hal.lastPacketParams[3] == SX1280_FLRC_PACKET_VARIABLE_LENGTH);
        CHECK(hal.lastPacketParams[5] == SX1280_FLRC_CRC_3_BYTE);
        if (radio.txLen < RLRS_FLRC_MIN_PAYLOAD) ++flrcShort;
    }
    CHECK(!dutAir.active);
    dutAir.active = true;
    dutAir.started = false;
    dutAir.start = t + jitter(20, 120);
    dutAir.end = dutAir.start + rlrsRadioAirtimeUs(rlrsRadioPresetParams(activePreset), radio.txLen);
    dutAir.freqReg = radio.currFreq;
    dutAir.preset = activePreset;
    memcpy(dutAir.data, radio.txData, radio.txLen);
    dutAir.len = radio.txLen;
}

// Small sequenced messages: [counter lo][counter hi](+ filler)
static size_t checkSequence(const std::vector<Msg> &msgs, uint8_t id, uint8_t size)
{
    size_t n = 0; int last = -1;
    for (size_t i = 0; i < msgs.size(); ++i)
    {
        if (msgs[i].id != id) continue;
        CHECK(msgs[i].p.size() == size);
        const int c = msgs[i].p[0] | (msgs[i].p[1] << 8);
        CHECK(c > last);   // in order, no duplicate
        last = c; ++n;
    }
    return n;
}

int main()
{
    setup();
    CHECK(Serial.baud == 420000);
    CHECK(WiFi.modeValue == WIFI_OFF && !WiFi.persistentFlag);
    CHECK(radioReady && hop != nullptr);
#if RLRS_ANTENNA_GAIN_DB == 3
    CHECK(powerValue == g_powerValues[2] && powerLevel == 2);   // 17 dBm + 3 dBi = 20 dBm EIRP
    CHECK(radio.power == g_powerValues[2]);
#else
    CHECK(powerValue == g_powerValues[3] && powerLevel == 3);   // 500 mW profile capped at 100 mW conducted
    CHECK(radio.power == g_powerValues[3]);
#endif
    CHECK(radio.cfgMod == RadioBandMod::Combined::LORA_2G4 && radio.cfgSf == SX1280_LORA_SF8 &&
          radio.cfgBw == SX1280_LORA_BW_0800 && radio.cfgCr == SX1280_LORA_CR_4_5 && radio.cfgPreamble == 8);
    CHECK(hop->config().rxLatencyUs == RLRS_RX_LATENCY_US);
    CHECK(rlrsHopConfigValid(rlrsHopConfigFromPresets(DEFAULT_LINK_SEED)));
    for (unsigned len = 0; len <= RLRS_RADIO_FRAME_LIMIT; ++len)   // FLRC timing model is an upper bound
        CHECK(rlrsFlrcAirtimeUs(RLRS_RADIO_PRESETS[1], static_cast<uint8_t>(len)) <=
              rlrsLoraAirtimeUs(RLRS_RADIO_PRESETS[1].hop.modem, static_cast<uint8_t>(len)));
    CHECK(rlrsRadioMaxFrameBytes() == RLRS_RADIO_FRAME_LIMIT);
    CHECK(rlrsRadioPresetIndex(rlrsProfileSettings(RlrsProfile::Armed).radio) == 0);
    CHECK(rlrsRadioPresetIndex(rlrsProfileSettings(RlrsProfile::Transfer).radio) == 1);

    RlrsHopConfig peerConfig = rlrsHopConfigFromPresets(DEFAULT_LINK_SEED);
    peerConfig.rxLatencyUs = RLRS_RX_LATENCY_US;   // the peer runs the same firmware
    peer = new (peerStorage) RlrsHop(PEER_ROLE, peerConfig, &peerQueue);
    peerQueue.setMaxFrameSize(PEER_ROLE == RlrsHop::Master ? rlrsRadioMinUplinkBytes() : rlrsRadioMinDownlinkBytes());
    const uint32_t t0 = g_micros;
    peerApply(peer->start(t0 + 7777), t0 + 7777);   // asynchronous start

    uint32_t sentUart = 0, sentPeer = 0, sentCmds = 0, sentSmallUp = 0, sentSmallDown = 0, sentMidDown = 0;
    uint32_t stepsInFlrc = 0;
    bool switched = false, switchedBack = false;
    const uint32_t stepUs = 10, totalUs = 12000000;
    uint32_t nextTel = 0, nextCmd = 0, nextStats = 0, nextUp = 0, nextDown = 0, nextMid = 0;
    bool req4 = false, req8 = false;
    uint8_t counter = 0;
    uint16_t upSeq = 0, downSeq = 0, midSeq = 0;
    uint32_t el = 0;
    while (el < totalUs)
    {
        uint32_t advance = stepUs;
        if (rnd() % 100u < 2u) advance += jitter(0, 250);   // loop() stall
        g_micros += advance;
        const uint32_t t = g_micros;
        el = t - t0;
        // ---- traffic ----
        // 6.5..7.5 s (FLRC): no telemetry nor command, bare beacons and
        // header only downlinks (< 6 o before padding).
        const bool quiet = el >= 6500000 && el < 7500000;
        // saturation phases, LoRa and FLRC: 1..6 s and 8.5..11 s
        const bool saturate = (el >= 1000000 && el < 6000000) || (el >= 8500000 && el < 11000000);
        if (quiet) { nextTel = el + 50000; nextCmd = el + 100000; }
        if (el >= nextTel)
        {
            nextTel += 50000;
            uint8_t tel[20];
            for (int i = 0; i < 20; ++i) tel[i] = static_cast<uint8_t>(counter + i);
#if defined(TARGET_TX)
            peerQueue.push(0x01, tel, 20); ++sentPeer;
#else
            uartSend(0x01, tel, 20); ++sentUart;
#endif
            ++counter;
        }
        if (el >= nextCmd)
        {
            nextCmd += 100000;
            ++sentCmds;
            uint8_t cmd[3] = {counter, 1, 2};
#if defined(TARGET_TX)
            uartSend(0x03, cmd, 3); ++sentUart;
#else
            peerQueue.push(0x03, cmd, 3); ++sentPeer;
#endif
        }
        if (el >= nextStats)
        {
            nextStats += 1000000;
#if defined(TARGET_TX)
            uint8_t ls[8] = {70, 10, 95, 0, 0, 0, 3, 0};
            peerQueue.push(0xF0, ls, 8);
#endif
        }
        // saturation: small sequenced messages, uplink every 10 ms (LED,
        // 4 o), downlink every 8 ms (id 0x10, 2 o) and every 4 ms (id 0x11,
        // 20 o: the 8 queue slots then hold more than a frame, so the
        // downlink length is set by the time left before the end guard)
        if (saturate && el >= nextMid)
        {
            nextMid = el + 4000;
            uint8_t m[20] = {static_cast<uint8_t>(midSeq), static_cast<uint8_t>(midSeq >> 8)};
            ++midSeq; ++sentMidDown;
#if defined(TARGET_TX)
            peerQueue.push(0x11, m, 20);
#else
            uartSend(0x11, m, 20);
#endif
        }
        if (saturate && el >= nextUp)
        {
            nextUp = el + 10000;
            uint8_t m[4] = {static_cast<uint8_t>(upSeq), static_cast<uint8_t>(upSeq >> 8), 0x11, 0x22};
            ++upSeq; ++sentSmallUp;
#if defined(TARGET_TX)
            uartSend(0x05, m, 4);
#else
            peerQueue.push(0x05, m, 4);
#endif
        }
        if (saturate && el >= nextDown)
        {
            nextDown = el + 8000;
            uint8_t m[2] = {static_cast<uint8_t>(downSeq), static_cast<uint8_t>(downSeq >> 8)};
            ++downSeq; ++sentSmallDown;
#if defined(TARGET_TX)
            peerQueue.push(0x10, m, 2);
#else
            uartSend(0x10, m, 2);
#endif
        }
#if defined(TARGET_TX)
        if (!req4 && el >= 4000000) { req4 = true; uint8_t pr = 3; uartSend(RLRS_PROFILE_REQUEST_ID, &pr, 1); }
        if (!req8 && el >= 8000000) { req8 = true; uint8_t pr = 1; uartSend(RLRS_PROFILE_REQUEST_ID, &pr, 1); }
#else
        if (!req4 && el >= 4000000) { req4 = true; CHECK(peer->requestPreset(1)); }
        if (!req8 && el >= 8000000) { req8 = true; CHECK(peer->requestPreset(0)); }
#endif
        // ---- simulation ----
        air(t, el > 10000000 ? 30 : 0);
        peerService(g_micros);
        loop();
        dutTxCheck(g_micros);
        if (activePreset == 1)
        {
            ++stepsInFlrc;
            if (!switched)
            {
                switched = true;
                CHECK(radio.cfgMod == RadioBandMod::Combined::FLRC_2G4 && radio.cfgBw == SX1280_FLRC_BR_1_300_BW_1_2 &&
                      radio.cfgCr == SX1280_FLRC_CR_1_2 && radio.cfgSf == SX1280_FLRC_BT_1 && radio.cfgPreamble == 32);
            }
        }
        if (switched && activePreset == 0) switchedBack = true;
    }
    const std::vector<Msg> uart = parseUart(Serial.out);
    CHECK(collisions == 0);
    CHECK(closedEarly == 0);
    CHECK(openedLate == 0);
    CHECK(g_spiRaces == 0);
    CHECK(flrcShort == 0);
    CHECK(isrOverlap == 0);
    CHECK(switched && switchedBack);
    CHECK(stepsInFlrc > 100000);            // more than 2 s in FLRC (10 us steps)
    CHECK(peer->preset() == 0 && activePreset == 0);
    CHECK(airMalformed == 0 && rxOverruns == 0);
#if defined(TARGET_TX)
    const std::vector<Msg> &downMsgs = uart;
    const std::vector<Msg> &upMsgs = peerGot;
#else
    const std::vector<Msg> &downMsgs = peerGot;
    const std::vector<Msg> &upMsgs = uart;
#endif
    const size_t smallDown = checkSequence(downMsgs, 0x10, 2);
    const size_t smallUp = checkSequence(upMsgs, 0x05, 4);
    const size_t midDown = checkSequence(downMsgs, 0x11, 20);
    std::printf("saturation: down 0x10 %zu/%u, down 0x11 %zu/%u, up 0x05 %zu/%u\n",
                smallDown, sentSmallDown, midDown, sentMidDown, smallUp, sentSmallUp);
    CHECK(midDown > 0);
    size_t tel = 0, cmds = 0, stats = 0;
    for (size_t i = 0; i < downMsgs.size(); ++i) if (downMsgs[i].id == 0x01) ++tel;
    for (size_t i = 0; i < upMsgs.size(); ++i) if (upMsgs[i].id == 0x03) ++cmds;
#if defined(TARGET_TX)
    size_t statsMerged = 0, statsDownOk = 0;
    for (size_t i = 0; i < uart.size(); ++i)
    {
        if (uart[i].id != 0xF0) continue;
        ++stats;
        CHECK(uart[i].p.size() == 8);
        if (uart[i].p[0] == 70 && uart[i].p[2] == 95) ++statsMerged;
        if (uart[i].p[3] == 81 && uart[i].p[5] >= 60) ++statsDownOk;
    }
    std::printf("TX DUT: telemetry %zu/%u, cmds %zu/%u, small down %zu/%u, small up %zu/%u, linkstats %zu (merged %zu, downlink ok %zu), profile rejected %u\n",
                tel, sentPeer, cmds, sentUart, smallDown, sentSmallDown, smallUp, sentSmallUp, stats, statsMerged, statsDownOk, profileRejected);
    CHECK(tel > sentPeer * 70 / 100);
    CHECK(stats >= 10 && statsMerged >= 8 && statsDownOk >= 8);
    CHECK(profileRejected == 0);
#else
    for (size_t i = 0; i < peerGot.size(); ++i)
        if (peerGot[i].id == 0xF0) { ++stats; CHECK(peerGot[i].p.size() == 8 && peerGot[i].p[0] == 81 && peerGot[i].p[7] <= 1); }
    std::printf("RX DUT: telemetry %zu/%u, cmds %zu/%u, small down %zu/%u, small up %zu/%u, linkstats from rocket %zu\n",
                tel, sentUart, cmds, sentPeer, smallDown, sentSmallDown, smallUp, sentSmallUp, stats);
    CHECK(tel > sentUart * 70 / 100);
    CHECK(stats >= 8);
    for (size_t i = 0; i < uart.size(); ++i) CHECK(uart[i].id != 0xF0 && uart[i].id != RLRS_PROFILE_REQUEST_ID);
#endif
    CHECK(cmds > sentCmds * 70 / 100);
    // Saturated (6.25 + 12.5 downlink messages per frame offered, bulk
    // priority, 8 queue slots): at least 10 % of the 2 o ones get through,
    // in order; uplink 5 per frame, 20 %.
    CHECK(smallDown > sentSmallDown * 10 / 100);   // lowest priority, 2 o
    CHECK(smallUp > sentSmallUp * 20 / 100);
    std::printf("hop: framesTx %u framesRx %u missed %u acq %u lost %u switches %u failed %u | peer switches %u | "
                "collisions %u closedEarly %u openedLate %u (in acquisition %u) spiRaces %u/%u flrcShort %u\n",
                hop->stats().framesTx, hop->stats().framesRx, hop->stats().missed, hop->stats().acquisitions,
                hop->stats().lost, hop->stats().presetSwitches, hop->stats().presetFailed,
                peer->stats().presetSwitches, collisions, closedEarly, openedLate, lateInAcquire, g_spiRaces, g_spiCommands, flrcShort);
    std::printf("downlink end before the next frame: min %d us, %u inside the end guard\n",
                static_cast<int>(minDownlinkMarginUs), downlinksInEndGuard);
    // Informative only. With the bridge presets (SF8, 50 ms) up + down
    // always total 40.5 + 5k symbols, so a time limited downlink ends about
    // 1 ms before the slave's deadline whatever the traffic: the master RX
    // margin is exercised by testMasterRxMargin (test_rlrs_air), which scans
    // the frame period to cover every residual slack.
    std::printf("%s: %d/%d checks failed\n", failures ? "FAILED" : "OK", failures, checks);
    return failures ? 1 : 0;
}
