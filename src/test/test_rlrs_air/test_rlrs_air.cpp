// Host tests for the RocketLRS stand-alone air layer (RlrsAir + RlrsHop).
// Build/run with: python test/test_rlrs_air/run_host_tests.py
//
// Covers the aggregation codec (priority, capacity, refusals, malformed
// frames, UART rebuild), the Semtech time on air and the 50 ms budget, the
// FHSS table, and a two-node simulator (master + slave + channel) with a
// virtual clock: deterministic losses 0/10/30/50 %, +/-100 ppm clock drift,
// micros() wrap-around, asynchronous start, preset switch, long outage and
// reacquisition. The simulator checks that the two nodes never transmit at
// the same time and that every message arrives intact and in order.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "RlrsAir.h"
#include "RlrsHop.h"
#include "RlrsProtocol.h"

namespace {

int g_failures = 0;
int g_checks = 0;

#define CHECK(cond)                                                              \
    do                                                                           \
    {                                                                            \
        ++g_checks;                                                              \
        if (!(cond))                                                             \
        {                                                                        \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            ++g_failures;                                                        \
        }                                                                        \
    } while (0)

typedef std::vector<uint8_t> Bytes;

Bytes payloadOf(uint8_t length, uint8_t seed)
{
    Bytes out(length);
    for (uint8_t i = 0; i < length; ++i)
        out[i] = static_cast<uint8_t>(seed + i * 3u);
    return out;
}

struct Msg
{
    uint8_t id;
    Bytes payload;
};

std::vector<Msg> readAll(const uint8_t *frame, size_t length, RlrsAirHeader *header, bool *malformed)
{
    std::vector<Msg> out;
    RlrsAirReader reader;
    const bool ok = reader.begin(frame, length);
    if (header)
        *header = reader.header();
    if (ok)
    {
        uint8_t id = 0, n = 0;
        const uint8_t *p = nullptr;
        while (reader.next(id, p, n))
        {
            Msg m;
            m.id = id;
            m.payload.assign(p, p + n);
            out.push_back(m);
        }
    }
    if (malformed)
        *malformed = reader.malformed();
    return out;
}

// ---------------------------------------------------------------------------
// Codec

void testCodecRoundTrip()
{
    RlrsAirQueue queue(33);
    CHECK(queue.maxPayload() == 26);
    const Bytes gps = payloadOf(20, 1), gyro = payloadOf(14, 2), cmd = payloadOf(4, 3);
    CHECK(queue.push(0x02, gps.data(), 20));
    CHECK(queue.push(0x01, gyro.data(), 14));
    CHECK(queue.push(0x03, cmd.data(), 4));   // pushed last, sent first

    RlrsAirHeader header = RlrsAirHeader();
    header.seq = 0x42;
    header.flags = RLRS_AIR_FLAG_HOP;
    header.hop = 17;
    uint8_t frame[RLRS_AIR_FRAME_MAX];
    const uint8_t length = queue.build(header, frame, 33);
    CHECK(length == 3 + 6 + 22);   // header + CMD + GPS; gyro (16) does not fit
    CHECK(length <= 33);

    RlrsAirHeader parsed;
    bool malformed = true;
    std::vector<Msg> msgs = readAll(frame, length, &parsed, &malformed);
    CHECK(!malformed);
    CHECK(parsed.seq == 0x42 && parsed.flags == RLRS_AIR_FLAG_HOP && parsed.hop == 17);
    CHECK(msgs.size() == 2);
    CHECK(msgs.size() == 2 && msgs[0].id == 0x03 && msgs[0].payload == cmd);
    CHECK(msgs.size() == 2 && msgs[1].id == 0x02 && msgs[1].payload == gps);

    // UART rebuild: AA + CRC16 reconstructed and valid.
    uint8_t wire[RLRS_WIRE_MAX_SIZE];
    const size_t wireLength = rlrsEncode(msgs[1].id, msgs[1].payload.data(),
                                         msgs[1].payload.size(), wire, sizeof(wire));
    CHECK(wireLength == 25 && rlrsValidate(wire, wireLength));

    // Next frame carries the remaining gyro message.
    header.seq = 0x43;
    const uint8_t second = queue.build(header, frame, 33);
    msgs = readAll(frame, second, nullptr, &malformed);
    CHECK(msgs.size() == 1 && msgs[0].id == 0x01 && msgs[0].payload == gyro);
    CHECK(queue.empty());
    CHECK(queue.stats().packed == 3 && queue.stats().frames == 2);

    // Empty queue: header only.
    CHECK(queue.build(header, frame, 33) == 3);
}

void testPresetHeader()
{
    RlrsAirHeader header = RlrsAirHeader();
    header.seq = 250;
    header.flags = RLRS_AIR_FLAG_PRESET | RLRS_AIR_FLAG_ACK;
    header.preset = 1;
    header.switchSeq = 4;
    uint8_t frame[8];
    CHECK(rlrsAirWriteHeader(header, frame, sizeof(frame)) == 4);
    CHECK(rlrsAirWriteHeader(header, frame, 3) == 0);
    RlrsAirHeader parsed;
    bool malformed = true;
    readAll(frame, 4, &parsed, &malformed);
    CHECK(!malformed && parsed.seq == 250 && parsed.preset == 1 && parsed.switchSeq == 4 &&
          parsed.flags == (RLRS_AIR_FLAG_PRESET | RLRS_AIR_FLAG_ACK));
    header.flags = 0x80;   // reserved
    CHECK(rlrsAirWriteHeader(header, frame, sizeof(frame)) == 0);
}

void testRefusals()
{
    // Too big for the uplink capacity: explicit, counted refusal.
    RlrsAirQueue up(33);
    const Bytes big = payloadOf(64, 9);
    CHECK(!up.push(0x02, big.data(), 27));
    CHECK(up.push(0x02, big.data(), 26));
    CHECK(up.stats().rejectedTooBig == 1);
    CHECK(!up.push(0x02, big.data(), 65));
    CHECK(up.stats().rejectedTooBig == 2);

    // A 64 byte RLRS payload fits the default downlink (72).
    RlrsAirQueue down(72);
    CHECK(down.maxPayload() == 64);
    CHECK(down.push(0x01, big.data(), 64));
    uint8_t frame[RLRS_AIR_FRAME_MAX];
    RlrsAirHeader header = RlrsAirHeader();
    const uint8_t length = down.build(header, frame, 72);
    CHECK(length == 2 + 66);
    bool malformed = true;
    std::vector<Msg> msgs = readAll(frame, length, nullptr, &malformed);
    CHECK(!malformed && msgs.size() == 1 && msgs[0].payload == big);

    // Full queue: telemetry evicted by a command, commands never evicted.
    RlrsAirQueue queue(72);
    const Bytes small = payloadOf(4, 1);
    for (unsigned i = 0; i < RLRS_AIR_QUEUE_COUNT; ++i)
        CHECK(queue.push(0x01, small.data(), 4));
    CHECK(!queue.push(0x01, small.data(), 4));
    CHECK(queue.stats().droppedFull == 1);
    for (unsigned i = 0; i < RLRS_AIR_QUEUE_COUNT; ++i)
        CHECK(queue.push(0x03, small.data(), 4));
    CHECK(queue.stats().evicted == RLRS_AIR_QUEUE_COUNT);
    CHECK(!queue.push(0x03, small.data(), 4));
    CHECK(queue.stats().droppedFull == 2);
}

void testFifoAndPriority()
{
    // Same priority: a big message that does not fit blocks the smaller one
    // queued after it (per-ID order kept); a lower level may still fill.
    RlrsAirQueue queue(72);
    const Bytes big = payloadOf(30, 1), small = payloadOf(4, 2), bulk = payloadOf(3, 3);
    CHECK(queue.push(0x01, big.data(), 30));      // telemetry
    CHECK(queue.push(0x07, small.data(), 4));     // telemetry
    CHECK(queue.push(0x55, bulk.data(), 3));      // bulk
    RlrsAirHeader header = RlrsAirHeader();
    uint8_t frame[RLRS_AIR_FRAME_MAX];
    const uint8_t length = queue.build(header, frame, 20);
    bool malformed = true;
    std::vector<Msg> msgs = readAll(frame, length, nullptr, &malformed);
    CHECK(msgs.size() == 1 && msgs[0].id == 0x55);
    const uint8_t next = queue.build(header, frame, 72);
    msgs = readAll(frame, next, nullptr, &malformed);
    CHECK(msgs.size() == 2 && msgs[0].id == 0x01 && msgs[1].id == 0x07);

    // Anti-starvation: a 64 byte bulk message is not starved forever by a
    // gyro sample queued before every frame.
    RlrsAirQueue starve(72);
    const Bytes huge = payloadOf(64, 4), gyro = payloadOf(14, 5);
    CHECK(starve.push(0x30, huge.data(), 64));
    bool sentHuge = false;
    unsigned frames = 0;
    for (; frames < 6 && !sentHuge; ++frames)
    {
        CHECK(starve.push(0x01, gyro.data(), 14));
        const uint8_t n = starve.build(header, frame, 72);
        msgs = readAll(frame, n, nullptr, &malformed);
        for (size_t i = 0; i < msgs.size(); ++i)
            sentHuge = sentHuge || (msgs[i].id == 0x30 && msgs[i].payload == huge);
    }
    CHECK(sentHuge && frames == RLRS_AIR_STARVE_FRAMES + 1);

    // Priority table matches RlrsLink::priorityForId.
    CHECK(rlrsAirPriorityForId(0x03) == 0 && rlrsAirPriorityForId(0x04) == 0);
    CHECK(rlrsAirPriorityForId(0x00) == 1 && rlrsAirPriorityForId(0xF0) == 1);
    CHECK(rlrsAirPriorityForId(0x01) == 2 && rlrsAirPriorityForId(0x08) == 2);
    CHECK(rlrsAirPriorityForId(0x99) == 3);
}

void testMalformed()
{
    RlrsAirReader reader;
    const uint8_t reserved[] = {1, 0x80, 0, 0};
    CHECK(!reader.begin(reserved, sizeof(reserved)) && reader.malformed());
    const uint8_t shortFrame[] = {1};
    CHECK(!reader.begin(shortFrame, 1));
    const uint8_t shortHop[] = {1, RLRS_AIR_FLAG_HOP | RLRS_AIR_FLAG_PRESET, 3, 1};
    CHECK(!reader.begin(shortHop, sizeof(shortHop)));

    uint8_t id, n;
    const uint8_t *p;
    const uint8_t tooLong[] = {1, 0, 65, 0x01};
    CHECK(reader.begin(tooLong, sizeof(tooLong)));
    CHECK(!reader.next(id, p, n) && reader.malformed());
    const uint8_t truncated[] = {1, 0, 2, 0x01, 9, 3, 0x02, 7};
    CHECK(reader.begin(truncated, sizeof(truncated)));
    CHECK(reader.next(id, p, n) && id == 0x01 && n == 2);
    CHECK(!reader.next(id, p, n) && reader.malformed());
    const uint8_t dangling[] = {1, 0, 0, 0x00, 5};
    CHECK(reader.begin(dangling, sizeof(dangling)));
    CHECK(reader.next(id, p, n) && n == 0);
    CHECK(!reader.next(id, p, n) && reader.malformed());
}


// FLRC minimum payload (6 o): padding is skipped by the reader, never
// delivered as a message, and anything after it is malformed.
void testPadding()
{
    uint8_t frame[16];
    RlrsAirHeader header = RlrsAirHeader();
    header.seq = 9;
    header.flags = RLRS_AIR_FLAG_HOP;
    header.hop = 4;
    uint8_t len = rlrsAirWriteHeader(header, frame, sizeof(frame));
    CHECK(len == 3);
    len = rlrsAirPad(frame, len, 6, sizeof(frame));
    CHECK(len == 6 && frame[3] == RLRS_AIR_PAD && frame[5] == RLRS_AIR_PAD);
    RlrsAirReader reader;
    uint8_t id, n;
    const uint8_t *p;
    CHECK(reader.begin(frame, len) && reader.header().hop == 4);
    CHECK(!reader.next(id, p, n) && !reader.malformed());
    CHECK(!reader.next(id, p, n) && !reader.malformed());

    // header only downlink (2 o) + one message: padding after the message
    RlrsAirQueue queue(16);
    const uint8_t payload[1] = {0x5A};
    CHECK(queue.push(0x07, payload, 1));
    header = RlrsAirHeader();
    header.seq = 1;
    len = queue.build(header, frame, sizeof(frame));
    CHECK(len == 5);
    len = rlrsAirPad(frame, len, 6, sizeof(frame));
    CHECK(len == 6);
    CHECK(reader.begin(frame, len));
    CHECK(reader.next(id, p, n) && id == 0x07 && n == 1 && p[0] == 0x5A);
    CHECK(!reader.next(id, p, n) && !reader.malformed());

    // already long enough: unchanged; bad arguments: 0
    CHECK(rlrsAirPad(frame, 7, 6, sizeof(frame)) == 7);
    CHECK(rlrsAirPad(frame, 1, 6, sizeof(frame)) == 0);
    CHECK(rlrsAirPad(frame, 3, 17, sizeof(frame)) == 0);
    CHECK(rlrsAirPad(nullptr, 3, 6, sizeof(frame)) == 0);

    // a message after padding is malformed; a lone trailing pad byte is fine
    const uint8_t bad[] = {1, 0, RLRS_AIR_PAD, 0, 0x01};
    CHECK(reader.begin(bad, sizeof(bad)));
    CHECK(!reader.next(id, p, n) && reader.malformed());
    const uint8_t lone[] = {1, 0, 0, 0x02, RLRS_AIR_PAD};
    CHECK(reader.begin(lone, sizeof(lone)));
    CHECK(reader.next(id, p, n) && id == 0x02 && n == 0);
    CHECK(!reader.next(id, p, n) && !reader.malformed());
}

// ---------------------------------------------------------------------------
// Time on air and budget

RlrsLoraModem modem(uint8_t sf, uint8_t cr, uint16_t preamble, bool explicitHeader, bool crc)
{
    RlrsLoraModem m;
    m.sf = sf;
    m.bandwidthHz = 812500;
    m.cr = cr;
    m.preambleSymbols = preamble;
    m.explicitHeader = explicitHeader;
    m.crcOn = crc;
    return m;
}

bool near(uint32_t value, double expected, double tolerance)
{
    return std::fabs(static_cast<double>(value) - expected) <= tolerance;
}

void testAirtime()
{
    // Values of UPGRADES-LIEN-RADIO.md (implicit header, no CRC).
    CHECK(near(rlrsLoraAirtimeUs(modem(7, 3, 12, false, false), 69), 24770, 10));
    CHECK(near(rlrsLoraAirtimeUs(modem(7, 1, 8, false, false), 20), 7130, 10));

    // Budget, SF8 BW800 CR4/5 preamble 8, explicit header, CRC on.
    const RlrsLoraModem sf8 = modem(8, 1, 8, true, true);
    const double tsym = 256.0 / 812500.0 * 1e6;   // 315.08 us
    const uint32_t up8 = rlrsLoraAirtimeUs(sf8, 8);
    const uint32_t down24 = rlrsLoraAirtimeUs(sf8, 24);
    const uint32_t down48 = rlrsLoraAirtimeUs(sf8, 48);
    CHECK(near(up8, 35.25 * tsym, 1));
    CHECK(near(down24, 55.25 * tsym, 1));
    CHECK(near(down48, 85.25 * tsym, 1));
    const RlrsHopConfig config = rlrsHopDefaultConfig(1);
    const uint32_t frame48 = up8 + config.guardUs + down48 + config.endGuardUs;
    const uint32_t frame24 = up8 + config.guardUs + down24 + config.endGuardUs;
    CHECK(frame48 <= 50000 && frame24 <= 50000);
    std::printf("  airtime SF8/BW800/CR4-5/pre8: up8=%u us down24=%u us down48=%u us, "
                "frame(48)=%u us frame(24)=%u us <= 50000\n",
                up8, down24, down48, frame48, frame24);

    // Inverse used by the slave to size its downlink.
    for (uint32_t budget = 5000; budget < 50000; budget += 777)
    {
        const int16_t fit = rlrsLoraMaxPayload(sf8, budget, 255);
        if (fit >= 0)
        {
            CHECK(rlrsLoraAirtimeUs(sf8, static_cast<uint8_t>(fit)) <= budget);
            if (fit < 255)
                CHECK(rlrsLoraAirtimeUs(sf8, static_cast<uint8_t>(fit + 1)) > budget);
        }
        else
        {
            CHECK(rlrsLoraAirtimeUs(sf8, 0) > budget);
        }
    }
    CHECK(rlrsLoraMaxPayload(sf8, 10, 255) == -1);

    CHECK(rlrsHopConfigValid(config));
    RlrsHopConfig bad = config;
    bad.presets[0].uplinkMaxBytes = 120;   // no room left for the downlink
    CHECK(!rlrsHopConfigValid(bad));
}

void testEventGuards()
{
    const RlrsHopConfig config = rlrsHopDefaultConfig(42);
    RlrsHop master(RlrsHop::Master, config, nullptr);
    const RlrsHopAction first = master.start(1000);
    CHECK(first.type == RlrsHopAction::Transmit && first.presetChanged && first.preset == 0);
    CHECK(first.txLength == 3 && first.timerAtUs == 1000 + config.framePeriodUs);
    // Early timer and out-of-state events leave the radio alone.
    CHECK(master.onTimer(2000).type == RlrsHopAction::None);
    CHECK(master.onRxDone(3000, nullptr, 0).type == RlrsHopAction::None);
    const RlrsHopAction rx = master.onTxDone(12000);
    CHECK(rx.type == RlrsHopAction::Receive && rx.frequencyHz == first.frequencyHz);
    CHECK(rx.rxTimeoutUs == 1000 + config.framePeriodUs - config.endGuardUs / 2 - 12000);
    CHECK(master.onTxDone(12001).type == RlrsHopAction::None);
    CHECK(!master.requestPreset(1));   // link not up yet
    // A beacon is not a valid downlink for the master.
    const uint8_t beacon[] = {0, RLRS_AIR_FLAG_HOP, 0};
    const RlrsHopAction bad = master.onRxDone(30000, beacon, sizeof(beacon));
    CHECK(!bad.rxDelivered && master.stats().rejected == 1);

    RlrsHop slave(RlrsHop::Slave, config, nullptr);
    const RlrsHopAction park = slave.start(5);
    CHECK(park.type == RlrsHopAction::Receive && park.rxTimeoutUs == 0);
    CHECK(park.frequencyHz == slave.frequencyForHop(0));
    // Beacon with the wrong hop index for the parked channel: keep listening.
    const uint8_t wrong[] = {0, RLRS_AIR_FLAG_HOP, 5};
    const RlrsHopAction again = slave.onRxDone(100, wrong, sizeof(wrong));
    CHECK(again.type == RlrsHopAction::Receive && !again.rxDelivered && !slave.synced());
    const RlrsHopAction got = slave.onRxDone(200, beacon, sizeof(beacon));
    CHECK(got.rxDelivered && slave.synced() && got.timerAtUs == 200 + config.guardUs);
    CHECK(!slave.requestPreset(1));   // master only
}

void testHopTable()
{
    const RlrsHopConfig config = rlrsHopDefaultConfig(0xC0FFEE);
    RlrsHop a(RlrsHop::Master, config, nullptr);
    RlrsHop b(RlrsHop::Slave, config, nullptr);
    RlrsHop c(RlrsHop::Master, rlrsHopDefaultConfig(0xBADC0DE), nullptr);
    bool seen[RLRS_HOP_MAX_CHANNELS] = {false};
    bool differs = false;
    for (uint8_t hop = 0; hop < config.channelCount; ++hop)
    {
        const uint8_t channel = a.channelForHop(hop);
        CHECK(channel < config.channelCount && !seen[channel]);
        seen[channel] = true;
        CHECK(channel == b.channelForHop(hop));
        differs = differs || channel != c.channelForHop(hop);
    }
    CHECK(differs);
    CHECK(a.channelFrequencyHz(0) == 2400400000u);
    CHECK(a.channelFrequencyHz(39) == 2479600000u);
    for (uint8_t ch = 1; ch < config.channelCount; ++ch)
        CHECK(a.channelFrequencyHz(ch) > a.channelFrequencyHz(ch - 1));
    const uint8_t uid[] = {1, 2, 3, 4, 5, 6};
    CHECK(rlrsHopSeedFromUid(uid, sizeof(uid)) == rlrsHopSeedFromUid(uid, sizeof(uid)));
    CHECK(rlrsHopSeedFromUid(nullptr, 0) != 0);
}

// ---------------------------------------------------------------------------
// Two-node simulator

const double INF = 1e300;

struct Packet
{
    int from;
    double start;
    double end;
    uint32_t frequency;
    uint8_t preset;
    Bytes data;
};

struct Traffic
{
    uint8_t id;
    uint8_t length;
    unsigned every;   // ticks of 10 ms
    unsigned phase;
};

struct Node
{
    RlrsAirQueue queue;
    RlrsHop hop;
    double ppm;
    double offset;
    double startAt;
    bool started;
    uint32_t timerLocal;
    double timerTrue;
    enum Mode { Standby, Tx, Rx } mode;
    uint32_t frequency;
    uint8_t preset;
    int txPacket;
    int rxPacket;
    double rxDeadline;
    // checks
    std::vector<std::pair<double, double> > txIntervals;
    uint32_t sentCounter[256];
    uint32_t lastReceived[256];
    uint32_t pushedAfterWarmup;
    uint32_t firstCounterAfterWarmup[256];
    uint32_t receivedAfterWarmup;
    uint32_t presetChangeSeq;
    double presetChangeAt;
    unsigned presetChanges;
    bool usedChannel[RLRS_HOP_MAX_CHANNELS];

    Node(RlrsHop::Role role, const RlrsHopConfig &config, uint8_t maxFrame)
        : queue(maxFrame), hop(role, config, &queue), ppm(0), offset(0), startAt(0), started(false),
          timerLocal(0), timerTrue(INF), mode(Standby), frequency(0), preset(0), txPacket(-1),
          rxPacket(-1), rxDeadline(INF), pushedAfterWarmup(0), receivedAfterWarmup(0),
          presetChangeSeq(0), presetChangeAt(-1), presetChanges(0)
    {
        std::memset(sentCounter, 0, sizeof(sentCounter));
        std::memset(lastReceived, 0, sizeof(lastReceived));
        std::memset(firstCounterAfterWarmup, 0, sizeof(firstCounterAfterWarmup));
        std::memset(usedChannel, 0, sizeof(usedChannel));
    }

    double rate() const { return 1.0 + ppm * 1e-6; }
    double localExact(double t) const { return offset + t * rate(); }
    uint32_t local(double t) const
    {
        return static_cast<uint32_t>(static_cast<uint64_t>(std::floor(localExact(t))) & 0xFFFFFFFFull);
    }
    double toTrue(uint32_t deadline, double t) const
    {
        const double exact = localExact(t);
        const double base = std::floor(exact);
        const int32_t diff = static_cast<int32_t>(deadline - local(t));
        const double trueTime = t + (base + diff - exact) / rate();
        return trueTime < t ? t : trueTime;
    }
};

struct Scenario
{
    const char *name;
    double lossPct;
    double masterPpm, slavePpm;
    double masterOffset, slaveOffset;
    double masterStart, slaveStart;
    double duration;
    double blackoutStart, blackoutEnd;
    double presetAt;
    uint8_t presetTarget;
    uint32_t seed;
};

struct Result
{
    unsigned overlaps;
    unsigned integrityErrors;
    unsigned orderErrors;
    unsigned aborted;
    double firstSync;
    double upRatio, downRatio;
    bool presetRequested;
};

class Sim
{
public:
    Sim(const Scenario &scenario, const RlrsHopConfig &config)
        : s(scenario), config(config), master(RlrsHop::Master, config, 33), slave(RlrsHop::Slave, config, 72),
          rng(scenario.seed), result()
    {
        master.ppm = s.masterPpm;
        slave.ppm = s.slavePpm;
        master.offset = s.masterOffset;
        slave.offset = s.slaveOffset;
        master.startAt = s.masterStart;
        slave.startAt = s.slaveStart;
        result.firstSync = -1;
    }

    Result run()
    {
        const double warmup = (s.masterStart > s.slaveStart ? s.masterStart : s.slaveStart) + 4.0e6;
        const double trafficEnd = s.duration - 0.5e6;
        double nextTick = 0;
        unsigned tick = 0;
        bool warm = false;
        bool presetDone = s.presetAt < 0;
        double t = 0;
        Node *nodes[2] = {&master, &slave};

        for (;;)
        {
            // Earliest event; ties: TX end, RX timeout, start, timer, traffic.
            double best = INF;
            int kind = -1;
            int who = -1;
            for (int i = 0; i < 2; ++i)
            {
                Node &n = *nodes[i];
                const double cand[4] = {
                    n.started && n.mode == Node::Tx ? packets[n.txPacket].end : INF,
                    n.started && n.mode == Node::Rx && n.rxPacket < 0 ? n.rxDeadline : INF,
                    n.started ? INF : n.startAt,
                    n.started ? n.timerTrue : INF};
                for (int k = 0; k < 4; ++k)
                {
                    if (cand[k] < best || (cand[k] == best && k < kind))
                    {
                        best = cand[k];
                        kind = k;
                        who = i;
                    }
                }
            }
            if (nextTick < best)
            {
                best = nextTick;
                kind = 4;
            }
            if (!presetDone && s.presetAt < best)
            {
                best = s.presetAt;
                kind = 5;
            }
            if (best > s.duration)
                break;
            t = best;

            if (kind == 4)
            {
                if (!warm && t >= warmup)
                {
                    warm = true;
                    markWarmup(master);
                    markWarmup(slave);
                }
                if (t < trafficEnd)
                    traffic(tick, warm);
                ++tick;
                nextTick = tick * 10000.0;
                continue;
            }
            if (kind == 5)
            {
                presetDone = true;
                result.presetRequested = master.hop.requestPreset(s.presetTarget);
                continue;
            }

            Node &n = *nodes[who];
            Node &other = *nodes[1 - who];
            if (kind == 0)
            {
                const int index = n.txPacket;
                n.mode = Node::Standby;
                n.txPacket = -1;
                if (other.mode == Node::Rx && other.rxPacket == index)
                {
                    other.mode = Node::Standby;
                    other.rxPacket = -1;
                    const Packet &p = packets[index];
                    const RlrsHopAction action = other.hop.onRxDone(other.local(t), p.data.data(),
                                                                     static_cast<uint8_t>(p.data.size()));
                    if (action.rxDelivered)
                        deliver(other, p.data);
                    apply(other, action, t);
                }
                apply(n, n.hop.onTxDone(n.local(t)), t);
            }
            else if (kind == 1)
            {
                n.mode = Node::Standby;
                apply(n, n.hop.onRxDone(n.local(t), nullptr, 0), t);
            }
            else if (kind == 2)
            {
                n.started = true;
                apply(n, n.hop.start(n.local(t)), t);
            }
            else
            {
                apply(n, n.hop.onTimer(n.timerLocal), t);
            }

            if (result.firstSync < 0 && slave.hop.synced() && master.hop.synced())
                result.firstSync = t;
        }

        // No double occupation: TX intervals of the two nodes never overlap.
        for (size_t i = 0; i < master.txIntervals.size(); ++i)
            for (size_t j = 0; j < slave.txIntervals.size(); ++j)
                if (master.txIntervals[i].first < slave.txIntervals[j].second &&
                    slave.txIntervals[j].first < master.txIntervals[i].second)
                    ++result.overlaps;

        result.upRatio = ratio(master, slave);
        result.downRatio = ratio(slave, master);
        return result;
    }

    const Scenario &s;
    const RlrsHopConfig &config;
    Node master;
    Node slave;
    std::vector<Packet> packets;
    uint32_t rng;
    Result result;

private:
    bool lost(double t)
    {
        rng = rng * 1664525u + 1013904223u;
        const double u = (rng >> 8) / 16777216.0;
        if (t >= s.blackoutStart && t < s.blackoutEnd)
            return true;
        return u < s.lossPct / 100.0;
    }

    void apply(Node &n, const RlrsHopAction &action, double t)
    {
        if (action.presetChanged)
        {
            n.preset = action.preset;
            n.presetChangeSeq = n.hop.seq();
            n.presetChangeAt = t;
            ++n.presetChanges;
        }
        n.timerLocal = action.timerAtUs;
        n.timerTrue = n.toTrue(action.timerAtUs, t);

        switch (action.type)
        {
        case RlrsHopAction::None:
            return;
        case RlrsHopAction::Idle:
            if (n.mode == Node::Tx)
                ++result.aborted;
            n.mode = Node::Standby;
            n.rxPacket = -1;
            return;
        case RlrsHopAction::Receive:
            if (n.mode == Node::Tx)
                ++result.aborted;
            n.mode = Node::Rx;
            n.frequency = action.frequencyHz;
            n.rxPacket = -1;
            n.rxDeadline = action.rxTimeoutUs ? t + action.rxTimeoutUs / n.rate() : INF;
            return;
        case RlrsHopAction::Transmit:
        {
            if (n.mode == Node::Tx)
                ++result.aborted;
            Packet p;
            p.from = &n == &master ? 0 : 1;
            p.start = t;
            p.end = t + rlrsLoraAirtimeUs(config.presets[n.preset].modem, action.txLength) / n.rate();
            p.frequency = action.frequencyHz;
            p.preset = n.preset;
            p.data.assign(action.txData, action.txData + action.txLength);
            packets.push_back(p);
            n.mode = Node::Tx;
            n.rxPacket = -1;
            n.txPacket = static_cast<int>(packets.size() - 1);
            n.txIntervals.push_back(std::make_pair(p.start, p.end));
            n.usedChannel[n.hop.channelForHop(n.hop.hopIndex())] = true;
            CHECK(action.frequencyHz == n.hop.frequencyForHop(n.hop.hopIndex()));
            Node &other = &n == &master ? slave : master;
            if (other.mode == Node::Tx)
                ++result.overlaps;
            const bool drop = lost(t);
            if (other.mode == Node::Rx && other.rxPacket < 0 && other.frequency == p.frequency &&
                other.preset == p.preset && !drop)
                other.rxPacket = n.txPacket;
            return;
        }
        }
    }

    static uint8_t filler(uint8_t id, uint32_t counter, unsigned i)
    {
        return static_cast<uint8_t>(id * 31u + counter * 7u + i * 13u);
    }

    void send(Node &n, uint8_t id, uint8_t length, bool warm)
    {
        if (!n.started)
            return;
        const uint32_t counter = ++n.sentCounter[id];
        uint8_t payload[RLRS_AIR_PAYLOAD_MAX];
        payload[0] = static_cast<uint8_t>(counter);
        payload[1] = static_cast<uint8_t>(counter >> 8);
        payload[2] = static_cast<uint8_t>(counter >> 16);
        payload[3] = static_cast<uint8_t>(counter >> 24);
        for (unsigned i = 4; i < length; ++i)
            payload[i] = filler(id, counter, i);
        if (n.queue.push(id, payload, length) && warm)
            ++n.pushedAfterWarmup;
    }

    void traffic(unsigned tick, bool warm)
    {
        static const Traffic down[] = {{0x01, 14, 5, 0}, {0x02, 20, 20, 3}, {0x07, 6, 100, 7}, {0x08, 4, 100, 57},
                                         {0x30, 64, 200, 150}};  // max RLRS payload, bulk
        static const Traffic up[] = {{0x03, 4, 30, 11}, {0x00, 4, 100, 21}};
        for (unsigned i = 0; i < sizeof(down) / sizeof(down[0]); ++i)
            if (tick % down[i].every == down[i].phase)
                send(slave, down[i].id, down[i].length, warm);
        for (unsigned i = 0; i < sizeof(up) / sizeof(up[0]); ++i)
            if (tick % up[i].every == up[i].phase)
                send(master, up[i].id, up[i].length, warm);
    }

    void markWarmup(Node &n)
    {
        for (unsigned id = 0; id < 256; ++id)
            n.firstCounterAfterWarmup[id] = n.sentCounter[id] + 1;
    }

    void deliver(Node &receiver, const Bytes &frame)
    {
        Node &sender = &receiver == &master ? slave : master;
        RlrsAirReader reader;
        if (!reader.begin(frame.data(), frame.size()))
        {
            ++result.integrityErrors;
            return;
        }
        uint8_t id = 0, length = 0;
        const uint8_t *payload = nullptr;
        while (reader.next(id, payload, length))
        {
            uint8_t wire[RLRS_WIRE_MAX_SIZE];
            const size_t size = rlrsEncode(id, payload, length, wire, sizeof(wire));
            if (size == 0 || !rlrsValidate(wire, size) || length < 4)
            {
                ++result.integrityErrors;
                continue;
            }
            const uint32_t counter = payload[0] | (payload[1] << 8) | (payload[2] << 16) |
                                     (static_cast<uint32_t>(payload[3]) << 24);
            for (unsigned i = 4; i < length; ++i)
                if (payload[i] != filler(id, counter, i))
                {
                    ++result.integrityErrors;
                    break;
                }
            if (counter <= receiver.lastReceived[id] || counter > sender.sentCounter[id])
                ++result.orderErrors;
            receiver.lastReceived[id] = counter;
            if (sender.firstCounterAfterWarmup[id] && counter >= sender.firstCounterAfterWarmup[id])
                ++receiver.receivedAfterWarmup;
        }
        if (reader.malformed())
            ++result.integrityErrors;
    }

    static double ratio(const Node &sender, const Node &receiver)
    {
        return sender.pushedAfterWarmup ? static_cast<double>(receiver.receivedAfterWarmup) / sender.pushedAfterWarmup
                                        : 0.0;
    }
};

const double WRAP = 4294967296.0;

Scenario baseScenario(const char *name, double loss, uint32_t seed)
{
    Scenario s;
    s.name = name;
    s.lossPct = loss;
    s.masterPpm = 100;
    s.slavePpm = -100;
    s.masterOffset = WRAP - 1.5e6;   // master micros() wraps after 1.5 s
    s.slaveOffset = 123456789.0;
    s.masterStart = 0;
    s.slaveStart = 1.37e6;           // asynchronous start
    s.duration = 25e6;
    s.blackoutStart = s.blackoutEnd = -1;
    s.presetAt = -1;
    s.presetTarget = 0;
    s.seed = seed;
    return s;
}

void report(const Scenario &s, const Sim &sim, const Result &r)
{
    const RlrsHop::Stats &ms = sim.master.hop.stats();
    const RlrsHop::Stats &ss = sim.slave.hop.stats();
    std::printf("  %-28s sync@%.2fs up=%.3f down=%.3f | M tx=%u rx=%u miss=%u lost=%u sw=%u fail=%u rev=%u "
                "| S tx=%u rx=%u miss=%u acq=%u lost=%u sw=%u rev=%u | presets M%u S%u\n",
                s.name, r.firstSync / 1e6, r.upRatio, r.downRatio, ms.framesTx, ms.framesRx, ms.missed, ms.lost,
                ms.presetSwitches, ms.presetFailed, ms.presetReverts, ss.framesTx, ss.framesRx, ss.missed,
                ss.acquisitions, ss.lost, ss.presetSwitches, ss.presetReverts, sim.master.hop.preset(),
                sim.slave.hop.preset());
}

void checkCommon(const Sim &sim, const Result &r)
{
    CHECK(r.overlaps == 0);
    CHECK(r.aborted == 0);
    CHECK(r.integrityErrors == 0);
    CHECK(r.orderErrors == 0);
    CHECK(r.firstSync >= 0);
    CHECK(sim.master.hop.preset() == sim.slave.hop.preset());
    CHECK(sim.master.preset == sim.slave.preset);
}

void testLossLevels()
{
    const RlrsHopConfig config = rlrsHopDefaultConfig(rlrsHopSeedFromUid(reinterpret_cast<const uint8_t *>("rocket"), 6));
    const double losses[] = {0, 10, 30, 50};
    for (unsigned i = 0; i < 4; ++i)
    {
        char name[40];
        std::snprintf(name, sizeof(name), "loss %2.0f%%", losses[i]);
        Scenario s = baseScenario(name, losses[i], 1000 + i);
        if (i % 2)
        {
            // Reverse: slave powered first, master later, drifts swapped.
            s.masterStart = 0.9e6;
            s.slaveStart = 0;
            s.masterPpm = -100;
            s.slavePpm = 100;
        }
        Sim sim(s, config);
        const Result r = sim.run();
        report(s, sim, r);
        checkCommon(sim, r);
        CHECK(sim.slave.hop.synced());
        CHECK(sim.master.hop.synced());
        const double keep = 1.0 - losses[i] / 100.0;
        CHECK(r.downRatio >= 0.85 * keep);
        CHECK(r.upRatio >= 0.85 * keep);
        if (losses[i] == 0)
        {
            CHECK(r.downRatio == 1.0 && r.upRatio == 1.0);
            CHECK(sim.slave.hop.stats().missed == 0);
            CHECK(sim.slave.hop.stats().acquisitions == 1);
            // acquisition within one park window plus one master cycle
            const double latest = (s.masterStart > s.slaveStart ? s.masterStart : s.slaveStart);
            CHECK(r.firstSync - latest <= 2.0 * (config.channelCount + 1) * config.framePeriodUs);
        }
        CHECK(r.firstSync <= 12e6);
        for (uint8_t ch = 0; ch < config.channelCount; ++ch)
            CHECK(sim.master.usedChannel[ch]);
    }
}

void testPresetSwitch()
{
    const RlrsHopConfig config = rlrsHopDefaultConfig(0x5EED1234);
    {
        Scenario s = baseScenario("preset switch 0%", 0, 77);
        s.presetAt = 6e6;
        s.presetTarget = 1;
        s.duration = 12e6;
        Sim sim(s, config);
        const Result r = sim.run();
        report(s, sim, r);
        checkCommon(sim, r);
        CHECK(r.presetRequested);
        CHECK(sim.master.hop.preset() == 1 && sim.slave.hop.preset() == 1);
        CHECK(sim.master.hop.stats().presetSwitches == 1 && sim.slave.hop.stats().presetSwitches == 1);
        // Same target frame on both sides (slave applies it when its RX window opens).
        CHECK(sim.master.presetChangeSeq == sim.slave.presetChangeSeq);
        CHECK(std::fabs(sim.master.presetChangeAt - sim.slave.presetChangeAt) < 3000);
        CHECK(sim.slave.hop.synced() && sim.master.hop.synced());
        CHECK(sim.slave.hop.stats().missed == 0);
        CHECK(r.downRatio == 1.0 && r.upRatio == 1.0);
    }
    {
        // Under 30 % loss the switch may fail, but both sides must agree.
        Scenario s = baseScenario("preset switch 30%", 30, 78);
        s.presetAt = 6e6;
        s.presetTarget = 1;
        s.duration = 14e6;
        Sim sim(s, config);
        const Result r = sim.run();
        report(s, sim, r);
        checkCommon(sim, r);
        CHECK(r.presetRequested);
        CHECK(sim.slave.hop.synced() && sim.master.hop.synced());
    }
}

void testLongOutage()
{
    const RlrsHopConfig config = rlrsHopDefaultConfig(0xFEEDBEEF);
    Scenario s = baseScenario("outage 2.5s + reacq (10%)", 10, 99);
    s.presetAt = 4e6;
    s.presetTarget = 1;
    s.blackoutStart = 7e6;
    s.blackoutEnd = 9.5e6;
    s.duration = 20e6;
    Sim sim(s, config);
    const Result r = sim.run();
    report(s, sim, r);
    checkCommon(sim, r);
    CHECK(r.presetRequested);
    // Back on the default preset on both sides, slave reacquired.
    CHECK(sim.master.hop.preset() == config.defaultPreset);
    CHECK(sim.slave.hop.preset() == config.defaultPreset);
    CHECK(sim.master.hop.stats().lost >= 1 && sim.slave.hop.stats().lost >= 1);
    CHECK(sim.master.hop.stats().presetReverts >= 1 || sim.master.hop.stats().presetSwitches == 0);
    CHECK(sim.slave.hop.stats().acquisitions >= 2);
    CHECK(sim.slave.hop.synced() && sim.master.hop.synced());
}

} // namespace


// Master RX close vs slave dating error. The slave dates the frame from its
// RxDone: late by the master TX start delay and by the IRQ latency it does
// not compensate (rxLatencyUs = 0 here), and its downlink leaves its antenna
// after its own TX start delay. Uplink and downlink queues are saturated so
// the downlink is time limited (40 ms frames, 4 o messages: fine length
// granularity); the frame period is scanned by 100 us steps
// (more than one 5 symbol block) to cover every residual slack of the LoRa
// symbol granularity. The master must still be listening at the end of
// every downlink: total lateness 200 + 150 + 100 = 450 us < endGuard / 2.
struct MarginPacket { bool on; uint32_t start, end, freq; uint8_t len; uint8_t data[RLRS_AIR_FRAME_MAX]; };

void testMasterRxMargin()
{
    const uint32_t masterTxDelay = 200, slaveIrqLatency = 150, slaveTxDelay = 100;
    int32_t worst = 1000000;
    for (uint32_t period = 40000; period <= 41600; period += 100)
    {
        RlrsHopConfig config = rlrsHopDefaultConfig(0x1234u);
        config.framePeriodUs = period;
        config.presetCount = 1;
        config.rxLatencyUs = 0;
        config.presets[0].downlinkMaxBytes = 40;
        CHECK(rlrsHopConfigValid(config));
        RlrsAirQueue mq(config.presets[0].uplinkMaxBytes), sq(config.presets[0].downlinkMaxBytes);
        RlrsHop master(RlrsHop::Master, config, &mq), slave(RlrsHop::Slave, config, &sq);
        const RlrsLoraModem &modem = config.presets[0].modem;
        MarginPacket up = MarginPacket(), down = MarginPacket();
        bool mRx = false, sRx = false;
        uint32_t mRxFreq = 0, mRxClose = 0, sRxFreq = 0, sRxClose = 0;
        bool mRxTimed = false, sRxTimed = false;
        uint32_t mLastFreq = 0, mLastOpen = 0, mLastClose = 0, mOpen = 0;
        uint32_t mTimer = 0, sTimer = 0;
        unsigned checked = 0, closedEarly = 0, timeLimited = 0;
        const uint8_t payload[20] = {0};

        struct Apply
        {
            static void run(const RlrsHopAction &a, uint32_t now, uint32_t txDelay, const RlrsLoraModem &m,
                            MarginPacket &pkt, bool &rx, uint32_t &rxFreq, uint32_t &rxClose, bool &rxTimed,
                            uint32_t &timer, uint32_t *open)
            {
                if (a.type == RlrsHopAction::Transmit)
                {
                    rx = false;
                    pkt.on = true;
                    pkt.start = now + txDelay;
                    pkt.end = pkt.start + rlrsLoraAirtimeUs(m, a.txLength);
                    pkt.freq = a.frequencyHz;
                    pkt.len = a.txLength;
                    memcpy(pkt.data, a.txData, a.txLength);
                }
                else if (a.type == RlrsHopAction::Receive)
                {
                    rx = true;
                    rxFreq = a.frequencyHz;
                    rxTimed = a.rxTimeoutUs != 0;
                    rxClose = now + a.rxTimeoutUs;
                    if (open) *open = now;
                }
                else if (a.type == RlrsHopAction::Idle)
                {
                    rx = false;
                }
                timer = a.timerAtUs;
            }
        };
        uint32_t t = 1000000;
        Apply::run(master.start(t), t, masterTxDelay, modem, up, mRx, mRxFreq, mRxClose, mRxTimed, mTimer, &mOpen);
        Apply::run(slave.start(t + 3333), t + 3333, slaveTxDelay, modem, down, sRx, sRxFreq, sRxClose, sRxTimed, sTimer, nullptr);
        for (uint32_t step = 0; step < 1500000 && checked < 20; ++step, t += 10)
        {
            while (mq.depth() < RLRS_AIR_QUEUE_COUNT && mq.push(0x05, payload, 4)) {}
            while (sq.depth() < RLRS_AIR_QUEUE_COUNT && sq.push(0x10, payload, 2)) {}
            if (up.on && static_cast<int32_t>(t - up.end) >= 0)
            {
                up.on = false;
                Apply::run(master.onTxDone(up.end + 20), up.end + 20, masterTxDelay, modem, up, mRx, mRxFreq, mRxClose, mRxTimed, mTimer, &mOpen);
                if (sRx && sRxFreq == up.freq && !(sRxTimed && static_cast<int32_t>(sRxClose - up.end) < 0))
                {
                    sRx = false;
                    const uint32_t now = up.end + slaveIrqLatency;
                    Apply::run(slave.onRxDone(now, up.data, up.len), now, slaveTxDelay, modem, down, sRx, sRxFreq, sRxClose, sRxTimed, sTimer, nullptr);
                }
            }
            if (down.on && static_cast<int32_t>(t - down.end) >= 0)
            {
                down.on = false;
                Apply::run(slave.onTxDone(down.end + 20), down.end + 20, slaveTxDelay, modem, down, sRx, sRxFreq, sRxClose, sRxTimed, sTimer, nullptr);
                const int32_t margin = static_cast<int32_t>(master.frameStartUs() + period - down.end);
                if (margin < worst) worst = margin;
                if (margin < static_cast<int32_t>(config.endGuardUs)) ++timeLimited;
                const bool listening = mRx && mRxFreq == down.freq && !(mRxTimed && static_cast<int32_t>(mRxClose - down.end) < 0);
                if (listening)
                {
                    ++checked;
                    mRx = false;
                    mLastFreq = 0;
                    Apply::run(master.onRxDone(down.end + 150, down.data, down.len), down.end + 150, masterTxDelay, modem, up, mRx, mRxFreq, mRxClose, mRxTimed, mTimer, &mOpen);
                }
                else if (mLastFreq == down.freq && static_cast<int32_t>(down.start - mLastOpen) >= 0 &&
                         static_cast<int32_t>(mLastClose - down.end) < 0)
                {
                    ++checked;
                    ++closedEarly;
                }
            }
            if (mRx && mRxTimed && static_cast<int32_t>(t - mRxClose) >= 0)
            {
                mRx = false;
                mLastFreq = mRxFreq; mLastOpen = mOpen; mLastClose = mRxClose;
                Apply::run(master.onRxDone(t, nullptr, 0), t, masterTxDelay, modem, up, mRx, mRxFreq, mRxClose, mRxTimed, mTimer, &mOpen);
            }
            if (sRx && sRxTimed && static_cast<int32_t>(t - sRxClose) >= 0)
            {
                sRx = false;
                Apply::run(slave.onRxDone(t, nullptr, 0), t, slaveTxDelay, modem, down, sRx, sRxFreq, sRxClose, sRxTimed, sTimer, nullptr);
            }
            if (static_cast<int32_t>(t - mTimer) >= 0)
                Apply::run(master.onTimer(t), t, masterTxDelay, modem, up, mRx, mRxFreq, mRxClose, mRxTimed, mTimer, &mOpen);
            if (static_cast<int32_t>(t - sTimer) >= 0)
                Apply::run(slave.onTimer(t), t, slaveTxDelay, modem, down, sRx, sRxFreq, sRxClose, sRxTimed, sTimer, nullptr);
        }
        CHECK(checked >= 20);
        CHECK(closedEarly == 0);
        if (closedEarly)
            std::printf("  margin: period %u us, %u/%u downlinks cut by the master RX close\n", period, closedEarly, checked);
        (void)timeLimited;
    }
    std::printf("  master RX margin: closest downlink end %d us before the next frame (end guard %u)\n",
                static_cast<int>(worst), 1000u);
    CHECK(worst < 1000);   // the scan really reached the end guard
}

int main()
{
    testCodecRoundTrip();
    testPresetHeader();
    testRefusals();
    testFifoAndPriority();
    testMalformed();
    testPadding();
    testAirtime();
    testEventGuards();
    testHopTable();
    testLossLevels();
    testPresetSwitch();
    testLongOutage();
    testMasterRxMargin();
    if (g_failures)
    {
        std::printf("test_rlrs_air: %d/%d checks FAILED\n", g_failures, g_checks);
        return 1;
    }
    std::printf("test_rlrs_air: all %d checks passed\n", g_checks);
    return 0;
}
