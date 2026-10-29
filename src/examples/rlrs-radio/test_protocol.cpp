#include "RlrsParser.h"
#include "RlrsProtocol.h"
#include <cassert>
#include <cstring>
#include <cstdio>

static unsigned calls;
static RlrsUartPacket last;
static void receive(RlrsUartPacket packet, void *) { ++calls; last = packet; }
int main()
{
    const uint8_t check[] = "123456789";
    assert(rlrsCrc16(check, 9) == 0x29B1);
    const uint8_t heartbeat[] = {0xAA,4,0,'T','E','S','T',0xC9,0x24};
    const uint8_t empty[] = {0xAA,0,0,0xB6,0xC1};
    assert(rlrsValidate(heartbeat, sizeof(heartbeat)));
    assert(rlrsValidate(empty, sizeof(empty)));
    RlrsParser parser;
    parser.setHandler(receive, nullptr);
    uint8_t payload[64], frame[69];
    for (unsigned i = 0; i < 64; ++i) payload[i] = i;
    payload[20] = 0xAA;
    for (unsigned len = 0; len <= 64; ++len)
    {
        size_t size = rlrsEncode(0x02, payload, len, frame, sizeof(frame));
        assert(size == len + 5 && rlrsValidate(frame, size));
        for (size_t split = 0; split <= size; ++split)
        {
            unsigned before = calls;
            parser.processBytes(frame, split);
            parser.processBytes(frame + split, size - split);
            assert(calls == before + 1 && last.payload_len == len && last.packet_id == 2);
            assert(memcmp(last.payload, payload, len) == 0);
        }
    }
    assert(!rlrsEncode(0, payload, 65, frame, sizeof(frame)));
    assert(!rlrsEncode(0, payload, 64, frame, 68));
    assert(!rlrsEncode(0, nullptr, 1, frame, sizeof(frame)));
    memcpy(frame, heartbeat, sizeof(heartbeat));
    frame[4] ^= 1;
    unsigned before = calls;
    parser.processBytes(frame, sizeof(heartbeat));
    assert(calls == before);
    memcpy(frame, heartbeat, sizeof(heartbeat));
    uint8_t temp = frame[7]; frame[7] = frame[8]; frame[8] = temp;
    parser.processBytes(frame, sizeof(heartbeat));
    assert(calls == before); // wrong CRC byte order
    const uint8_t noise[] = {0, 0x55, 0xAA, 65, 0x12, 0xAA};
    parser.processBytes(noise, sizeof(noise));
    parser.processBytes(heartbeat, sizeof(heartbeat));
    parser.processBytes(empty, sizeof(empty));
    assert(calls == before + 2); // sync recovery and back-to-back frames
    for (size_t size = 0; size < sizeof(heartbeat); ++size)
        assert(!rlrsValidate(heartbeat, size));
    assert(!rlrsValidate(heartbeat, sizeof(heartbeat) + 1));
    std::printf("PASS: CCITT vector, protocol examples, 0..64 bytes, all split positions, corruption and bounds (%u callbacks)\n", calls);
}
