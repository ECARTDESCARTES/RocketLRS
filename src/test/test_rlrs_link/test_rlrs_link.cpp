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
        Bytes packet(capacity, 0);
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

    // 10 bytes: 2 header bytes + 8 data, then 1 + 9, then 1 + 3.
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
    assert(first.size() == 2);
    assert(first[0][0] == sizeof(command) && first[0][1] == 0x03);

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
    assert(packets.size() == 2); // 3 data bytes, then the last one

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

    // 20 bytes do not fit in 6: two fragments (4 then 16 -> 4+5+5+5+1).
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

} // namespace

int main()
{
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
    std::puts("PASS: RocketLRS link framing, fragmentation, priority, ACK and link stats");
    return 0;
}
