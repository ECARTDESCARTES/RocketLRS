// Host tests for the RocketLRS air link (framing, fragmentation, priority).
// Build/run with: python test/test_rlrs_link/run_host_tests.py
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "RlrsLink.h"
#include "RlrsProtocol.h"

namespace {

typedef std::vector<uint8_t> Bytes;

void feedFrame(RlrsLink &link, uint8_t id, const uint8_t *payload, uint8_t length)
{
    uint8_t wire[RLRS_MAX_SIZE];
    const size_t size = rlrsEncode(id, payload, length, wire, sizeof(wire));
    assert(size != 0);
    link.feedUart(wire, static_cast<uint16_t>(size));
}

Bytes readUart(RlrsLink &link)
{
    Bytes out;
    uint8_t buffer[64];
    for (;;)
    {
        const uint16_t size = link.takeUart(buffer, sizeof(buffer));
        if (size == 0)
            break;
        out.insert(out.end(), buffer, buffer + size);
    }
    return out;
}

// Pack everything currently queued, padding packets like OTA does.
std::vector<Bytes> drain(RlrsLink &link, uint8_t capacity, unsigned limit = 64)
{
    std::vector<Bytes> packets;
    uint8_t buffer[64];
    for (unsigned i = 0; i < limit; ++i)
    {
        const uint8_t size = link.packAir(buffer, capacity);
        if (size == 0)
            break;
        Bytes packet(capacity, 0xFF); // OTA pads with the idle marker
        std::memcpy(packet.data(), buffer, size);
        packets.push_back(packet);
    }
    return packets;
}

bool sameWire(const Bytes &wire, uint8_t id, const uint8_t *payload, uint8_t length)
{
    uint8_t expected[RLRS_MAX_SIZE];
    const size_t size = rlrsEncode(id, payload, length, expected, sizeof(expected));
    return size == wire.size() && std::memcmp(wire.data(), expected, size) == 0;
}

void testCrcVector()
{
    const uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    assert(rlrsCrc16(text, sizeof(text)) == 0x29B1);
}

void testSmallFrameIsSinglePacket()
{
    RlrsLink tx;
    RlrsLink rx;

    const uint8_t gyro[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
    feedFrame(tx, 0x01, gyro, sizeof(gyro));
    assert(tx.airPending());

    const std::vector<Bytes> packets = drain(tx, 10);
    assert(packets.size() == 1);
    assert(tx.stats().uartFramesIn == 1);
    assert(!tx.airPending());

    rx.unpackAir(packets[0].data(), 10);
    const Bytes wire = readUart(rx);
    assert(sameWire(wire, 0x01, gyro, sizeof(gyro)));
    assert(rx.stats().airFramesIn == 1);
}

void testLargeFrameFragmentsWithoutCrcOverhead()
{
    RlrsLink tx;
    RlrsLink rx;

    uint8_t gps[20];
    for (uint8_t i = 0; i < sizeof(gps); ++i)
        gps[i] = static_cast<uint8_t>(0xA0 + i);

    feedFrame(tx, 0x02, gps, sizeof(gps));
    const std::vector<Bytes> packets = drain(tx, 10);

    // 10 bytes: 3 header bytes + 7 data, then 1 + 9, then 1 + 4.
    assert(packets.size() == 3);
    for (size_t i = 0; i < packets.size(); ++i)
        rx.unpackAir(packets[i].data(), 10);

    const Bytes wire = readUart(rx);
    assert(sameWire(wire, 0x02, gps, sizeof(gps)));
    assert(rx.stats().airFramesIn == 1);
    assert(rx.stats().airLost == 0);
}

void testLostFragmentDiscardsWholeFrameOnly()
{
    RlrsLink tx;
    RlrsLink rx;

    uint8_t gps[20];
    std::memset(gps, 0x5A, sizeof(gps));
    feedFrame(tx, 0x02, gps, sizeof(gps));
    std::vector<Bytes> packets = drain(tx, 10);
    assert(packets.size() == 3);

    rx.unpackAir(packets[0].data(), 10);
    // packet 1 is lost
    rx.unpackAir(packets[2].data(), 10);
    assert(readUart(rx).empty());
    assert(rx.stats().airLost == 1);

    // The link is usable again immediately (self-describing length prefixes).
    const uint8_t battery[] = {0x0F, 0x1E};
    feedFrame(tx, 0x08, battery, sizeof(battery));
    packets = drain(tx, 10);
    assert(packets.size() == 1);
    rx.unpackAir(packets[0].data(), 10);
    const Bytes wire = readUart(rx);
    assert(sameWire(wire, 0x08, battery, sizeof(battery)));
}

void testPriorityAndLatestWins()
{
    RlrsLink tx;

    const uint8_t bulk[] = {1, 2, 3, 4};
    const uint8_t command[] = {0x00};
    tx.enqueue(0x09, bulk, sizeof(bulk), RlrsLink::PriorityBulk);
    tx.enqueue(0x03, command, sizeof(command), RlrsLink::PriorityCritical);
    const std::vector<Bytes> first = drain(tx, 10);
    // Both fit in one packet (3 + 6 bytes); the command still comes first.
    assert(first.size() == 1);
    assert(first[0][0] == sizeof(command) && first[0][1] == 0x03);
    assert(first[0][3] == sizeof(bulk) && first[0][4] == 0x09);
    assert(first[0][9] == 0xFF);

    // A newer telemetry sample replaces an older queued one.
    const uint8_t oldSample[] = {0x11};
    const uint8_t newSample[] = {0x22};
    tx.enqueue(0x01, oldSample, sizeof(oldSample), RlrsLink::PriorityTelemetry);
    tx.enqueue(0x01, newSample, sizeof(newSample), RlrsLink::PriorityTelemetry);
    const std::vector<Bytes> packets = drain(tx, 10);
    assert(packets.size() == 1);
    assert(packets[0][0] == 1 && packets[0][1] == 0x01 && packets[0][2] == 0x22);
}

void testReliableFrameIsAcknowledged()
{
    RlrsLink tx;
    RlrsLink rx;

    const uint8_t command[] = {0x01}; // arm
    tx.enqueue(0x03, command, sizeof(command), RlrsLink::PriorityCritical, true);
    assert(tx.airPending());

    std::vector<Bytes> packets = drain(tx, 10);
    assert(packets.size() == 1);
    assert((packets[0][0] & 0xF0) == 0x90); // reliable single

    rx.unpackAir(packets[0].data(), 10);
    const Bytes wire = readUart(rx);
    assert(sameWire(wire, 0x03, command, sizeof(command)));

    // The receiver queues an ACK; the sender repeats until it arrives.
    packets = drain(rx, 10);
    assert(packets.size() == 1);
    assert((packets[0][0] & 0xF0) == 0xA0);

    tx.unpackAir(packets[0].data(), 10);
    assert(!tx.airPending());

    // Without the ACK the frame would have been retransmitted.
    RlrsLink tx2;
    tx2.enqueue(0x03, command, sizeof(command), RlrsLink::PriorityCritical, true);
    drain(tx2, 10);
    bool repeated = false;
    for (unsigned i = 0; i < RLRS_LINK_RETRY_GAP + 2; ++i)
    {
        if (tx2.packAir(reinterpret_cast<uint8_t *>(packets[0].data()), 10) != 0)
            repeated = true;
    }
    assert(repeated);
    assert(tx2.airPending());
}

void testOta4Capacity()
{
    RlrsLink tx;
    RlrsLink rx;

    const uint8_t value[] = {0xDE, 0xAD, 0xBE, 0xEF};
    feedFrame(tx, 0x07, value, sizeof(value));
    const std::vector<Bytes> packets = drain(tx, 5);
    assert(packets.size() == 2); // 2 data bytes, then the last two

    for (size_t i = 0; i < packets.size(); ++i)
        rx.unpackAir(packets[i].data(), 5);
    assert(sameWire(readUart(rx), 0x07, value, sizeof(value)));
}

void testPartialUartInput()
{
    RlrsLink tx;
    RlrsLink rx;

    const uint8_t payload[] = {0x41, 0x42, 0x43};
    uint8_t wire[RLRS_MAX_SIZE];
    const size_t size = rlrsEncode(0x01, payload, sizeof(payload), wire, sizeof(wire));
    for (size_t i = 0; i < size; ++i)
        tx.feedUart(&wire[i], 1);

    assert(tx.stats().uartFramesIn == 1);
    const std::vector<Bytes> packets = drain(tx, 10);
    rx.unpackAir(packets[0].data(), 10);
    assert(sameWire(readUart(rx), 0x01, payload, sizeof(payload)));
}

void testLinkStats()
{
    RlrsLink tx;
    RlrsLink rx;

    RlrsLink::LinkStats stats;
    stats.uplinkRssi = 72;
    stats.uplinkSnr = -5;
    stats.uplinkLq = 98;
    stats.downlinkRssi = 60;
    stats.downlinkSnr = 7;
    stats.downlinkLq = 91;
    stats.txPower = 3;
    stats.rfMode = 8;
    assert(tx.enqueueLinkStats(stats));

    const std::vector<Bytes> packets = drain(tx, 10);
    assert(packets.size() == 1);
    rx.unpackAir(packets[0].data(), 10);
    const Bytes wire = readUart(rx);
    assert(wire.size() == 8 + 5);
    assert(wire[2] == RlrsLink::LinkStatsId);
    assert(wire[3] == 72 && wire[4] == static_cast<uint8_t>(-5));
    assert(wire[5] == 98 && wire[6] == 60);
    assert(wire[7] == 7 && wire[8] == 91);
    assert(wire[9] == 3 && wire[10] == 8);
}

void testIdlePacketIsIgnored()
{
    RlrsLink rx;
    uint8_t idle[10] = {0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    rx.unpackAir(idle, 10);
    assert(readUart(rx).empty());
    assert(rx.stats().airFramesIn == 0);
}

// Inside a LINKSTATS packet only 6 bytes are left for RLRS on OTA8
// (1 byte on OTA4). Frames must still flow, fragmenting when needed.
void testReducedLinkStatsContainer()
{
    RlrsLink tx;
    RlrsLink rx;

    const uint8_t battery[] = {0x0B, 0x16, 0x21, 0x2C};
    feedFrame(tx, 0x08, battery, sizeof(battery));
    std::vector<Bytes> packets = drain(tx, 6);
    assert(packets.size() == 1);
    rx.unpackAir(packets[0].data(), 6);
    assert(sameWire(readUart(rx), 0x08, battery, sizeof(battery)));

    // 20 bytes do not fit in 6: first fragment carries 3, then 5+5+5+2.
    uint8_t gps[20];
    std::memset(gps, 0x77, sizeof(gps));
    feedFrame(tx, 0x02, gps, sizeof(gps));
    packets = drain(tx, 6);
    assert(packets.size() == 5);
    for (size_t i = 0; i < packets.size(); ++i)
        rx.unpackAir(packets[i].data(), 6);
    assert(sameWire(readUart(rx), 0x02, gps, sizeof(gps)));
}

// A one-byte slot (OTA4 LINKSTATS) can only carry an acknowledgement.
void testOneByteSlotCarriesOnlyAck()
{
    RlrsLink tx;
    RlrsLink rx;

    const uint8_t command[] = {0x01};
    tx.enqueue(0x03, command, sizeof(command), RlrsLink::PriorityCritical, true);
    const std::vector<Bytes> packets = drain(tx, 10);
    assert(packets.size() == 1);

    rx.unpackAir(packets[0].data(), 10);
    assert(!readUart(rx).empty());

    // Nothing else can be transmitted in a single byte, but the ACK goes out.
    uint8_t slot[16] = {0};
    assert(rx.packAir(slot, 1) == 1);
    assert((slot[0] & 0xF0) == 0xA0);

    // A data frame is left queued rather than truncated.
    RlrsLink empty;
    uint8_t value[] = {0x42};
    empty.enqueue(0x01, value, sizeof(value));
    assert(empty.packAir(value, 1) == 0);
    assert(empty.airPending());
}

// ---------------------------------------------------------------------------
// Fragment coherence (bug n°2 of UPGRADES-LIEN-RADIO.md)
// ---------------------------------------------------------------------------

struct Decoded
{
    uint8_t id;
    Bytes payload;
};

// Split the host UART output of a link into RLRS frames (CRC checked).
std::vector<Decoded> decodeUart(RlrsLink &link)
{
    const Bytes wire = readUart(link);
    std::vector<Decoded> frames;
    size_t i = 0;
    while (i < wire.size())
    {
        assert(wire.size() - i >= 5 && wire[i] == RLRS_SYNCWORD);
        const size_t size = 5u + wire[i + 1];
        assert(i + size <= wire.size());
        assert(rlrsValidate(&wire[i], size));
        Decoded frame;
        frame.id = wire[i + 2];
        frame.payload.assign(wire.begin() + static_cast<long>(i + 3),
                             wire.begin() + static_cast<long>(i + 3 + wire[i + 1]));
        frames.push_back(frame);
        i += size;
    }
    return frames;
}

// Walk the segments of one sent packet: no segment may be empty or overrun
// the packet, and nothing but idle padding may follow a fragment.
void checkPacketShape(const uint8_t *packet, uint8_t written, uint8_t capacity)
{
    assert(written <= capacity);
    uint8_t pos = 0;
    while (pos < written)
    {
        const uint8_t op = packet[pos];
        const uint8_t space = static_cast<uint8_t>(capacity - pos);
        assert(op != 0xFF);
        if (op <= 0x3F)
        {
            assert(2u + op <= space);
            pos = static_cast<uint8_t>(pos + 2 + op);
        }
        else if (op <= 0x7F)
        {
            assert(space >= 4); // header + at least one data byte
            return;             // a fragment always ends the packet
        }
        else if (op <= 0x8F)
        {
            assert(space >= 2); // never an empty continuation
            return;
        }
        else if (op <= 0x9F)
        {
            assert(space >= 3 && 3u + packet[pos + 1] <= space);
            pos = static_cast<uint8_t>(pos + 3 + packet[pos + 1]);
        }
        else
        {
            assert((op & 0xF0) == 0xA0 && (op & 0x0F) != 0);
            ++pos;
        }
    }
    assert(pos == written);
}

// Losing the tail of frame A and the head of frame B must not splice B's
// continuations into A's reassembly buffer (per-frame fragment numbers all
// restarted at 1, so B's first continuation matched what A was waiting for).
void testFragmentsOfAnotherFrameAreRejected()
{
    RlrsLink tx;
    RlrsLink rx;

    uint8_t a[20];
    uint8_t b[20];
    std::memset(a, 0xAA, sizeof(a));
    std::memset(b, 0xBB, sizeof(b));
    tx.enqueue(0x09, a, sizeof(a), RlrsLink::PriorityBulk);
    const std::vector<Bytes> pa = drain(tx, 10);
    tx.enqueue(0x0A, b, sizeof(b), RlrsLink::PriorityBulk);
    const std::vector<Bytes> pb = drain(tx, 10);
    assert(pa.size() == 3 && pb.size() == 3);

    rx.unpackAir(pa[0].data(), 10); // A head
    // A tail (pa[1], pa[2]) and B head (pb[0]) are lost.
    rx.unpackAir(pb[1].data(), 10);
    rx.unpackAir(pb[2].data(), 10);

    const std::vector<Decoded> got = decodeUart(rx);
    assert(got.empty());
    assert(rx.stats().airLost == 1);
}

// Tiny slots: a 64-byte frame needs dozens of continuations. Losing exactly
// 16 of them aliases the 4-bit counter (documented limit), but the short
// reassembly then never reaches its announced length: nothing corrupted is
// ever delivered.
void testSixteenLostContinuationsAreDetected()
{
    RlrsLink tx;
    RlrsLink rx;

    uint8_t big[64];
    for (uint8_t i = 0; i < sizeof(big); ++i)
        big[i] = i;
    tx.enqueue(0x09, big, sizeof(big), RlrsLink::PriorityBulk);

    uint8_t buffer[64];
    unsigned dataPackets = 0;
    for (unsigned step = 0; step < 400 && tx.airPending(); ++step)
    {
        const uint8_t capacity = step == 0 ? 4 : (step < 300 ? 2 : 10);
        std::memset(buffer, 0xFF, sizeof(buffer));
        const uint8_t size = tx.packAir(buffer, capacity);
        if (size == 0)
            continue;
        checkPacketShape(buffer, size, capacity);
        ++dataPackets;
        if (dataPackets >= 2 && dataPackets <= 17)
            continue; // 16 consecutive continuations lost
        rx.unpackAir(buffer, capacity);
    }
    assert(!tx.airPending());

    const std::vector<Decoded> got = decodeUart(rx);
    for (size_t i = 0; i < got.size(); ++i)
        assert(got[i].id == 0x09 && got[i].payload == Bytes(big, big + sizeof(big)));
}

// The remaining slot of a packet may be a single byte: no empty fragment may
// be emitted there (and the frame must still arrive intact afterwards).
void testOneByteLeftNeverEmitsEmptyFragment()
{
    RlrsLink tx;
    RlrsLink rx;

    uint8_t gps[20];
    for (uint8_t i = 0; i < sizeof(gps); ++i)
        gps[i] = static_cast<uint8_t>(0x30 + i);
    tx.enqueue(0x02, gps, sizeof(gps), RlrsLink::PriorityTelemetry);

    // First fragment, then an ACK leaving one byte in a 2-byte slot.
    uint8_t buffer[16];
    std::memset(buffer, 0xFF, sizeof(buffer));
    uint8_t size = tx.packAir(buffer, 10);
    checkPacketShape(buffer, size, 10);
    rx.unpackAir(buffer, 10);

    const uint8_t reliable[] = {0x01};
    RlrsLink peer;
    peer.enqueue(0x03, reliable, sizeof(reliable), RlrsLink::PriorityCritical, true);
    uint8_t carrier[16];
    std::memset(carrier, 0xFF, sizeof(carrier));
    const uint8_t carried = peer.packAir(carrier, 10);
    tx.unpackAir(carrier, carried); // tx now owes an ACK

    std::memset(buffer, 0xFF, sizeof(buffer));
    size = tx.packAir(buffer, 2);
    checkPacketShape(buffer, size, 2);
    assert((buffer[0] & 0xF0) == 0xA0);
    rx.unpackAir(buffer, 2);

    for (unsigned step = 0; step < 8 && tx.airPending(); ++step)
    {
        std::memset(buffer, 0xFF, sizeof(buffer));
        size = tx.packAir(buffer, 10);
        if (size == 0)
            continue;
        checkPacketShape(buffer, size, 10);
        rx.unpackAir(buffer, 10);
    }
    const std::vector<Decoded> got = decodeUart(rx);
    assert(got.size() == 1 && got[0].id == 0x02);
    assert(got[0].payload == Bytes(gps, gps + sizeof(gps)));
    assert(rx.stats().airLost == 0 && rx.stats().malformed == 0);
}

// ---------------------------------------------------------------------------
// Aggregation
// ---------------------------------------------------------------------------

void testAggregatesSmallFrames()
{
    RlrsLink tx;
    RlrsLink rx;

    const uint8_t gyro[] = {0x11};
    const uint8_t baro[] = {0x21, 0x22};
    const uint8_t batt[] = {0x31};
    tx.enqueue(0x01, gyro, sizeof(gyro), RlrsLink::PriorityTelemetry);
    tx.enqueue(0x07, baro, sizeof(baro), RlrsLink::PriorityTelemetry);
    tx.enqueue(0x08, batt, sizeof(batt), RlrsLink::PriorityTelemetry);

    const std::vector<Bytes> packets = drain(tx, 10);
    assert(packets.size() == 1); // 3 + 4 + 3 = 10 bytes
    rx.unpackAir(packets[0].data(), 10);
    const std::vector<Decoded> got = decodeUart(rx);
    assert(got.size() == 3);
    assert(got[0].id == 0x01 && got[1].id == 0x07 && got[2].id == 0x08);
    assert(got[1].payload == Bytes(baro, baro + sizeof(baro)));
    assert(tx.stats().airFramesOut == 3 && rx.stats().airFramesIn == 3);
}

void testAckAggregatedWithData()
{
    RlrsLink tx;
    RlrsLink rx;

    const uint8_t command[] = {0x02};
    tx.enqueue(0x03, command, sizeof(command), RlrsLink::PriorityCritical, true);
    std::vector<Bytes> packets = drain(tx, 10);
    rx.unpackAir(packets[0].data(), 10);
    readUart(rx);

    const uint8_t gyro[] = {0x01, 0x02, 0x03};
    rx.enqueue(0x01, gyro, sizeof(gyro), RlrsLink::PriorityTelemetry);
    packets = drain(rx, 10);
    assert(packets.size() == 1);
    assert((packets[0][0] & 0xF0) == 0xA0);
    assert(packets[0][1] == sizeof(gyro) && packets[0][2] == 0x01);

    tx.unpackAir(packets[0].data(), 10);
    assert(!tx.airPending()); // ACK consumed
    const std::vector<Decoded> got = decodeUart(tx);
    assert(got.size() == 1 && got[0].id == 0x01);
}

// 0x00 is a valid empty frame (HEARTBEAT has id 0x00), so zero padding would
// be decoded as spurious frames: padding must be the 0xFF idle marker.
void testPaddingMustBeIdleMarker()
{
    RlrsLink rx;
    uint8_t packet[10] = {0x01, 0x05, 0x42, 0xFF, 0, 0, 0, 0, 0, 0};
    rx.unpackAir(packet, 10);
    std::vector<Decoded> got = decodeUart(rx);
    assert(got.size() == 1 && got[0].id == 0x05);

    RlrsLink heartbeat;
    uint8_t zeros[10] = {0x01, 0x05, 0x42, 0, 0, 0, 0, 0, 0, 0};
    heartbeat.unpackAir(zeros, 10);
    got = decodeUart(heartbeat);
    assert(got.size() == 4); // 1 real frame + 3 bogus empty id-0 frames
}

// A Critical frame never waits behind a split lower-priority frame.
void testCriticalGoesBeforeContinuation()
{
    RlrsLink tx;
    RlrsLink rx;

    uint8_t gps[30];
    std::memset(gps, 0x47, sizeof(gps));
    tx.enqueue(0x02, gps, sizeof(gps), RlrsLink::PriorityTelemetry);
    uint8_t buffer[16];
    std::memset(buffer, 0xFF, sizeof(buffer));
    uint8_t size = tx.packAir(buffer, 10);
    assert((buffer[0] & 0xC0) == 0x40);
    rx.unpackAir(buffer, 10);

    const uint8_t command[] = {0x0C};
    tx.enqueue(0x03, command, sizeof(command), RlrsLink::PriorityCritical);
    std::memset(buffer, 0xFF, sizeof(buffer));
    size = tx.packAir(buffer, 10);
    checkPacketShape(buffer, size, 10);
    assert(buffer[0] == sizeof(command) && buffer[1] == 0x03);
    assert((buffer[3] & 0xF0) == 0x80);
    rx.unpackAir(buffer, 10);

    for (unsigned step = 0; step < 10 && tx.airPending(); ++step)
    {
        std::memset(buffer, 0xFF, sizeof(buffer));
        size = tx.packAir(buffer, 10);
        checkPacketShape(buffer, size, 10);
        rx.unpackAir(buffer, 10);
    }
    const std::vector<Decoded> got = decodeUart(rx);
    assert(got.size() == 2 && got[0].id == 0x03 && got[1].id == 0x02);
    assert(got[1].payload == Bytes(gps, gps + sizeof(gps)));
}

// Leftover space is filled with smaller frames, but a lower-priority frame
// never starts a split there when a more important frame was left for the
// next packet (the split would then block it).
void testNoPriorityInversionInLeftover()
{
    RlrsLink tx;

    const uint8_t command[] = {0x01, 0x02, 0x03, 0x04};   // 6 bytes
    const uint8_t control[] = {0x05, 0x06, 0x07, 0x08, 0x09}; // 7 bytes
    uint8_t bulk[30];
    std::memset(bulk, 0x99, sizeof(bulk));
    tx.enqueue(0x03, command, sizeof(command), RlrsLink::PriorityCritical);
    tx.enqueue(0x00, control, sizeof(control), RlrsLink::PriorityControl);
    tx.enqueue(0x09, bulk, sizeof(bulk), RlrsLink::PriorityBulk);

    uint8_t buffer[16];
    std::memset(buffer, 0xFF, sizeof(buffer));
    uint8_t size = tx.packAir(buffer, 10);
    assert(size == 6 && buffer[1] == 0x03);

    std::memset(buffer, 0xFF, sizeof(buffer));
    size = tx.packAir(buffer, 10);
    assert(buffer[0] == sizeof(control) && buffer[1] == 0x00);
    // 3 bytes left: enough for a split header + data, but the bulk frame
    // started here is fine since nothing more important was skipped.
    checkPacketShape(buffer, size, 10);
}

void testNoPriorityInversionWhenSkipped()
{
    RlrsLink tx;

    const uint8_t first[] = {0x01, 0x02};                      // 4 bytes
    const uint8_t control[] = {1, 2, 3, 4, 5, 6, 7};           // 9 bytes
    uint8_t bulk[30];
    std::memset(bulk, 0x99, sizeof(bulk));
    tx.enqueue(0x03, first, sizeof(first), RlrsLink::PriorityCritical);
    tx.enqueue(0x00, control, sizeof(control), RlrsLink::PriorityControl);
    tx.enqueue(0x09, bulk, sizeof(bulk), RlrsLink::PriorityBulk);

    uint8_t buffer[16];
    std::memset(buffer, 0xFF, sizeof(buffer));
    const uint8_t size = tx.packAir(buffer, 10);
    // The control frame did not fit the 6 leftover bytes: the bulk frame must
    // not start a split there, or it would delay the control frame.
    assert(size == 4 && buffer[4] == 0xFF);
    std::memset(buffer, 0xFF, sizeof(buffer));
    tx.packAir(buffer, 10);
    assert(buffer[0] == sizeof(control) && buffer[1] == 0x00);
}

// ---------------------------------------------------------------------------
// Deterministic fuzz: random sizes, priorities, reliability, capacities
// 1..10 and losses in both directions.
// ---------------------------------------------------------------------------

struct Rng
{
    uint32_t state;
    explicit Rng(uint32_t seed) : state(seed ? seed : 1) {}
    uint32_t next()
    {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        return state;
    }
    uint32_t below(uint32_t n) { return next() % n; }
};

struct Sent
{
    uint8_t id;
    Bytes payload;
    bool reliable;
    unsigned delivered;
};

struct Side
{
    RlrsLink link;
    std::vector<Sent> sent;
    unsigned counter;
};

void maybeEnqueue(Side &side, Rng &rng, uint8_t idBase)
{
    if (rng.below(100) >= 35 || side.link.queueDepth() >= RLRS_LINK_FRAME_COUNT)
        return;
    Sent frame;
    frame.id = static_cast<uint8_t>(idBase + (side.counter & 0x7F));
    const uint8_t length = static_cast<uint8_t>(rng.below(4) == 0 ? rng.below(65) : rng.below(12));
    frame.payload.resize(length);
    for (uint8_t i = 0; i < length; ++i)
        frame.payload[i] = static_cast<uint8_t>(rng.next());
    if (length >= 2)
    {
        frame.payload[0] = static_cast<uint8_t>(side.counter);
        frame.payload[1] = static_cast<uint8_t>(side.counter >> 8);
    }
    frame.reliable = rng.below(5) == 0;
    frame.delivered = 0;
    const uint8_t priority = static_cast<uint8_t>(rng.below(4));
    if (side.link.enqueue(frame.id, length ? frame.payload.data() : nullptr, length,
                          priority, frame.reliable))
    {
        side.sent.push_back(frame);
        ++side.counter;
    }
}

void transfer(Side &from, Side &to, Rng &rng, unsigned lossPercent, bool tiny)
{
    uint8_t buffer[64];
    std::memset(buffer, 0xFF, sizeof(buffer));
    const uint8_t capacity = static_cast<uint8_t>(tiny ? 1 + rng.below(10) : 10);
    const uint8_t size = from.link.packAir(buffer, capacity);
    for (uint8_t i = capacity; i < sizeof(buffer); ++i)
        assert(buffer[i] == 0xFF); // never writes past the slot
    if (size == 0)
        return;
    checkPacketShape(buffer, size, capacity);
    if (rng.below(100) < lossPercent)
        return;
    to.link.unpackAir(buffer, capacity);
}

void collect(Side &receiver, Side &sender)
{
    const std::vector<Decoded> got = decodeUart(receiver.link);
    for (size_t i = 0; i < got.size(); ++i)
    {
        // Identical frames (e.g. empty payloads) are credited to the oldest
        // undelivered copy first; only reliable frames may arrive twice.
        bool matched = false;
        for (int pass = 0; pass < 2 && !matched; ++pass)
        {
            for (size_t j = 0; j < sender.sent.size() && !matched; ++j)
            {
                Sent &candidate = sender.sent[j];
                if (candidate.id != got[i].id || candidate.payload != got[i].payload)
                    continue;
                if (pass == 0 ? candidate.delivered != 0 : !candidate.reliable)
                    continue;
                ++candidate.delivered;
                matched = true;
            }
        }
        assert(matched); // no corrupted, spliced or duplicated best-effort frame
    }
}

void runFuzz(uint32_t seed, unsigned lossPercent)
{
    Rng rng(seed);
    Side a;
    Side b;
    a.counter = 0;
    b.counter = 0;

    for (unsigned step = 0; step < 4000; ++step)
    {
        maybeEnqueue(a, rng, 0x10);
        maybeEnqueue(b, rng, 0x90);
        transfer(a, b, rng, lossPercent, true);
        transfer(b, a, rng, lossPercent, true);
        collect(b, a);
        collect(a, b);
    }
    // Lossless flush with full slots.
    for (unsigned step = 0; step < 400; ++step)
    {
        transfer(a, b, rng, 0, false);
        transfer(b, a, rng, 0, false);
        collect(b, a);
        collect(a, b);
    }

    assert(a.link.stats().malformed == 0 && b.link.stats().malformed == 0);
    assert(a.link.stats().uartOverflow == 0 && b.link.stats().uartOverflow == 0);
    if (lossPercent == 0)
    {
        assert(!a.link.airPending() && !b.link.airPending());
        for (size_t i = 0; i < a.sent.size(); ++i)
            assert(a.sent[i].delivered == 1);
        for (size_t i = 0; i < b.sent.size(); ++i)
            assert(b.sent[i].delivered == 1);
        assert(a.link.stats().airLost == 0 && b.link.stats().airLost == 0);
    }
    else
    {
        // The link keeps working: most frames still arrive.
        size_t delivered = 0;
        for (size_t i = 0; i < a.sent.size(); ++i)
            delivered += a.sent[i].delivered != 0;
        assert(a.sent.size() > 100 && delivered * 4 > a.sent.size());
    }
}

void testFuzzRoundTrip()
{
    for (uint32_t seed = 1; seed <= 12; ++seed)
    {
        runFuzz(seed * 2654435761u, 0);
        runFuzz(seed * 40503u + 7u, 10);
        runFuzz(seed * 97u + 3u, 35);
    }
}

} // namespace

// Defect 1: the rocket resets its link on (re)connection while the ground
// may still hold the head of a split frame. The next frame's continuations
// must never complete that old head (CRC is rebuilt, so a splice is silent).
void testResetDoesNotSpliceAcrossReconnection()
{
    for (unsigned lost = 1; lost <= 2; ++lost)
    {
        RlrsLink rocket;
        RlrsLink ground;
        uint8_t a[30];
        uint8_t b[30];
        std::memset(a, 0xAA, sizeof(a));
        std::memset(b, 0xBB, sizeof(b));

        rocket.enqueue(0x09, a, sizeof(a), RlrsLink::PriorityBulk);
        const std::vector<Bytes> pa = drain(rocket, 10, 2); // head + 1 continuation
        assert(pa.size() == 2);
        ground.unpackAir(pa[0].data(), 10);
        ground.unpackAir(pa[1].data(), 10);

        rocket.reset(); // rx_main GotConnection()
        rocket.enqueue(0x0A, b, sizeof(b), RlrsLink::PriorityBulk);
        const std::vector<Bytes> pb = drain(rocket, 10);
        assert(pb.size() == 4); // 7 + 9 + 9 + 5

        // B's head (and maybe its first continuation) are lost.
        for (size_t i = lost; i < pb.size(); ++i)
            ground.unpackAir(pb[i].data(), 10);
        assert(decodeUart(ground).empty());
    }
}

// Defect 1, ground side: a reassembly reset on (re)connection drops the head.
void testGroundReassemblyResetOnConnection()
{
    RlrsLink rocket;
    RlrsLink ground;
    uint8_t a[30];
    std::memset(a, 0xAA, sizeof(a));
    rocket.enqueue(0x09, a, sizeof(a), RlrsLink::PriorityBulk);
    const std::vector<Bytes> pa = drain(rocket, 10);
    assert(pa.size() == 4);

    ground.unpackAir(pa[0].data(), 10);
    ground.resetReassembly(); // tx_main: link (re)connected
    for (size_t i = 1; i < pa.size(); ++i)
        ground.unpackAir(pa[i].data(), 10);
    assert(decodeUart(ground).empty());

    // The reassembly works again from the next head.
    rocket.enqueue(0x09, a, sizeof(a), RlrsLink::PriorityBulk);
    const std::vector<Bytes> again = drain(rocket, 10);
    for (size_t i = 0; i < again.size(); ++i)
        ground.unpackAir(again[i].data(), 10);
    const std::vector<Decoded> got = decodeUart(ground);
    assert(got.size() == 1 && got[0].id == 0x09 && got[0].payload == Bytes(a, a + 30));
}

// Defect 3: the 16-bit arrival order wraps; the older frame must still win.
void testOrderSurvivesWrap()
{
    RlrsLink tx;
    const uint8_t value = 0x11;
    // Same-id bulk frames replace each other in place: 65536 enqueues bring
    // the frame to order 65535 and the counter back to 0.
    for (uint32_t i = 0; i < 65536u; ++i)
        assert(tx.enqueue(0x20, &value, 1, RlrsLink::PriorityBulk));
    assert(tx.enqueue(0x21, &value, 1, RlrsLink::PriorityBulk)); // order 0, newer

    uint8_t buffer[3]; // exactly one whole frame per packet
    assert(tx.packAir(buffer, 3) == 3);
    assert(buffer[1] == 0x20);
    assert(tx.packAir(buffer, 3) == 3);
    assert(buffer[1] == 0x21);
}

// Defect 4: a late or duplicated ACK of a finished reliable frame must not
// acknowledge the next reliable frame.
void testStaleAckDoesNotClearNextReliableFrame()
{
    RlrsLink tx;
    RlrsLink rx;
    const uint8_t command[] = {0x01};

    tx.enqueue(0x03, command, sizeof(command), RlrsLink::PriorityCritical, true);
    const std::vector<Bytes> first = drain(tx, 10, 1);
    assert(first.size() == 1 && (first[0][0] & 0xF0) == 0x90);
    const uint8_t seqA = first[0][0] & 0x0F;

    rx.unpackAir(first[0].data(), 10);
    const std::vector<Bytes> ack = drain(rx, 10);
    assert(ack.size() == 1 && ack[0][0] == (0xA0 | seqA));
    tx.unpackAir(ack[0].data(), 10);
    assert(!tx.airPending());

    tx.enqueue(0x03, command, sizeof(command), RlrsLink::PriorityCritical, true);
    uint8_t buffer[10];
    assert(tx.packAir(buffer, 10) == 4 && (buffer[0] & 0xF0) == 0x90);
    assert((buffer[0] & 0x0F) != seqA);

    tx.unpackAir(ack[0].data(), 10); // the stale ACK arrives again
    assert(tx.airPending());
}

// Defect 5: OTA8 downlink alternates LINKSTATS (6 bytes) and DATA (10 bytes)
// containers; a frame that fits whole in the DATA one waits for it.
void testDownlinkWaitsForLargerSlot()
{
    RlrsLink rocket;
    RlrsLink ground;
    uint8_t buffer[10];

    const uint8_t gps[6] = {1, 2, 3, 4, 5, 6}; // 8 bytes on air
    rocket.enqueue(0x02, gps, sizeof(gps), RlrsLink::PriorityTelemetry);
    assert(rocket.packAir(buffer, 6, 10) == 0); // LINKSTATS: wait
    assert(rocket.airPending());
    assert(rocket.packAir(buffer, 10, 10) == 8 && buffer[0] == 6);
    Bytes packet(10, 0xFF);
    std::memcpy(packet.data(), buffer, 8);
    ground.unpackAir(packet.data(), 10);
    const std::vector<Decoded> got = decodeUart(ground);
    assert(got.size() == 1 && got[0].payload == Bytes(gps, gps + 6));

    // Smaller frames still use the small slot.
    const uint8_t beat[2] = {7, 8};
    rocket.enqueue(0x00, beat, sizeof(beat), RlrsLink::PriorityControl);
    assert(rocket.packAir(buffer, 6, 10) == 4);

    // A frame that never fits whole is still split in the small slot.
    uint8_t big[20];
    std::memset(big, 0x5A, sizeof(big));
    rocket.enqueue(0x09, big, sizeof(big), RlrsLink::PriorityBulk);
    assert(rocket.packAir(buffer, 6, 10) == 6 && (buffer[0] & 0xC0) == 0x40);

    // A reliable frame keeps its sequence number for the larger slot.
    RlrsLink r2;
    const uint8_t cmd[4] = {1, 2, 3, 4}; // 7 bytes reliable, 6 plain
    r2.enqueue(0x03, cmd, sizeof(cmd), RlrsLink::PriorityCritical, true);
    assert(r2.packAir(buffer, 6, 10) == 0);
    assert(r2.packAir(buffer, 10, 10) == 7 && (buffer[0] & 0xF0) == 0x90);
}

int main()
{
    testFragmentsOfAnotherFrameAreRejected();
    testSixteenLostContinuationsAreDetected();
    testOneByteLeftNeverEmitsEmptyFragment();
    testCrcVector();
    testSmallFrameIsSinglePacket();
    testLargeFrameFragmentsWithoutCrcOverhead();
    testLostFragmentDiscardsWholeFrameOnly();
    testPriorityAndLatestWins();
    testReliableFrameIsAcknowledged();
    testOta4Capacity();
    testPartialUartInput();
    testLinkStats();
    testIdlePacketIsIgnored();
    testReducedLinkStatsContainer();
    testOneByteSlotCarriesOnlyAck();
    testAggregatesSmallFrames();
    testAckAggregatedWithData();
    testPaddingMustBeIdleMarker();
    testCriticalGoesBeforeContinuation();
    testNoPriorityInversionInLeftover();
    testNoPriorityInversionWhenSkipped();
    testFuzzRoundTrip();
    testResetDoesNotSpliceAcrossReconnection();
    testGroundReassemblyResetOnConnection();
    testOrderSurvivesWrap();
    testStaleAckDoesNotClearNextReliableFrame();
    testDownlinkWaitsForLargerSlot();
    std::puts("PASS: RocketLRS link framing, fragmentation, priority, ACK and link stats");
    return 0;
}
