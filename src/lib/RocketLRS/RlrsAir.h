#ifndef H_RLRS_AIR
#define H_RLRS_AIR

#include <cstddef>
#include <cstdint>

// RocketLRS stand-alone air layer: aggregation codec.
//
// Replaces the fixed 69 byte radio frame of the SX1280 bridge by a variable
// length frame that carries several RLRS messages at once. The SX1280 runs
// with an explicit LoRa header and its hardware CRC enabled, so the RLRS
// sync word (0xAA) and the RLRS CRC16 are NOT sent over the air: the receiver
// rebuilds the UART wire frame with rlrsEncode() (RlrsProtocol.h).
//
// Frame layout:
//   [seq:8][flags:8]                    always
//   [hop:8]                             if flags & RLRS_AIR_FLAG_HOP
//   [preset:8][switchSeq:8]             if flags & RLRS_AIR_FLAG_PRESET
//   { [len:8][id:8][payload:len] } * N  compacted RLRS messages, len <= 64
//   [0xFF] * P                          optional padding (RLRS_AIR_PAD)
//
// Padding: a length byte equal to RLRS_AIR_PAD ends the message list; every
// following byte must also be RLRS_AIR_PAD (otherwise the frame is
// malformed). It lets a modem with a minimum payload length (SX1280 FLRC:
// 6 bytes) carry a bare beacon or a header only downlink, see rlrsAirPad().
//
// Flags (bits not listed are reserved and must be zero, otherwise the frame
// is rejected as malformed):
//   0x01 HOP     uplink beacon: hop index present (RlrsHop resynchronisation)
//   0x02 PRESET  preset switch fields present (announce on the uplink, echo
//                on the downlink)
//   0x04 ACK     downlink: the slave acknowledges the announced preset switch
//
// No fragmentation: a message that cannot fit in a frame of the queue's
// maximum size is refused at push() and counted (Stats::rejectedTooBig).
// A queued message that does not fit in the space left in the current frame
// waits for the next one; messages of the same priority stay in FIFO order,
// so the per-ID order is always preserved. Anti-starvation: a big message
// repeatedly passed over (RLRS_AIR_STARVE_FRAMES) is packed right after the
// commands, before the periodic telemetry that would otherwise always win.
//
// Pure logic: no Arduino, no driver, no dynamic allocation, no exceptions.

#ifndef RLRS_AIR_QUEUE_COUNT
#define RLRS_AIR_QUEUE_COUNT 8
#endif
// A head-of-level message passed over this many frames for lack of room
// (while it would fit in an empty frame) is packed right after the commands.
#ifndef RLRS_AIR_STARVE_FRAMES
#define RLRS_AIR_STARVE_FRAMES 2
#endif

// Functions that may run from a radio interrupt. Define it to ICACHE_RAM_ATTR
// in the build flags when RlrsHop/RlrsAirQueue are driven from an ISR on
// ESP8266/ESP32; leave it empty when they are driven from loop().
#ifndef RLRS_ISR_ATTR
#define RLRS_ISR_ATTR
#endif

constexpr uint8_t RLRS_AIR_FLAG_HOP = 0x01;
constexpr uint8_t RLRS_AIR_FLAG_PRESET = 0x02;
constexpr uint8_t RLRS_AIR_FLAG_ACK = 0x04;
constexpr uint8_t RLRS_AIR_FLAG_KNOWN = 0x07;

constexpr uint8_t RLRS_AIR_HEADER_MIN = 2;
constexpr uint8_t RLRS_AIR_HEADER_MAX = 5;
constexpr uint8_t RLRS_AIR_MSG_OVERHEAD = 2;   // [len][id]
constexpr uint8_t RLRS_AIR_PAYLOAD_MAX = 64;   // RLRS payload limit
constexpr uint8_t RLRS_AIR_FRAME_MAX = 255;    // SX1280 LoRa payload limit
constexpr uint8_t RLRS_AIR_PAD = 0xFF;         // padding byte (> RLRS_AIR_PAYLOAD_MAX)

struct RlrsAirHeader
{
    uint8_t seq;
    uint8_t flags;
    uint8_t hop;        // valid with RLRS_AIR_FLAG_HOP
    uint8_t preset;     // valid with RLRS_AIR_FLAG_PRESET
    uint8_t switchSeq;  // valid with RLRS_AIR_FLAG_PRESET
};

// Size of the header for a given flag set.
uint8_t rlrsAirHeaderSize(uint8_t flags);
// Write the header; returns its size, or 0 if it does not fit / bad flags.
uint8_t rlrsAirWriteHeader(const RlrsAirHeader &header, uint8_t *out, uint8_t capacity);
// Pad a built frame with RLRS_AIR_PAD up to minLength (<= capacity).
// Returns the new length (unchanged if already long enough), 0 on bad args.
uint8_t rlrsAirPad(uint8_t *frame, uint8_t length, uint8_t minLength, uint8_t capacity);
// Default priority of an RLRS packet ID (same table as RlrsLink::priorityForId:
// 0 commands, 1 control, 2 telemetry, 3 bulk).
uint8_t rlrsAirPriorityForId(uint8_t id);

class RlrsAirQueue
{
public:
    enum Priority : uint8_t
    {
        PriorityCritical = 0,   // CMD / ACK (arm, parachute...)
        PriorityControl = 1,    // heartbeat, LED, servo, link stats
        PriorityTelemetry = 2,  // INFO/gyro, GPS, pressure, battery
        PriorityBulk = 3        // everything else
    };

    struct Stats
    {
        uint32_t accepted;        // messages queued
        uint32_t rejectedTooBig;  // len + 2 > maxFrameSize - RLRS_AIR_HEADER_MAX
        uint32_t droppedFull;     // queue full, nothing less important to evict
        uint32_t evicted;         // older, less important message replaced
        uint32_t packed;          // messages written into frames
        uint32_t frames;          // frames built
    };

    // maxFrameSize: largest frame this queue will ever be asked to build
    // (e.g. 33 for the uplink, 64..127 for the downlink).
    explicit RlrsAirQueue(uint8_t maxFrameSize = RLRS_AIR_FRAME_MAX);

    void reset();                       // drop every message, keep stats
    void setMaxFrameSize(uint8_t maxFrameSize);
    uint8_t maxFrameSize() const { return maxFrameSize_; }
    // Largest payload accepted by push() with the current maxFrameSize.
    uint8_t maxPayload() const;

    // Queue one RLRS message (payload without AA/len/id/CRC). Returns false
    // when refused (too big, bad arguments or queue full); every refusal is
    // counted in stats().
    bool push(uint8_t id, const uint8_t *payload, uint8_t length);
    bool push(uint8_t id, const uint8_t *payload, uint8_t length, uint8_t priority);

    // Build one frame: header first, then as many queued messages as fit in
    // `capacity` bytes, most important first. Returns the frame length, or 0
    // if the header itself does not fit.
    uint8_t build(const RlrsAirHeader &header, uint8_t *out, uint8_t capacity);

    uint8_t depth() const { return count_; }
    bool empty() const { return count_ == 0; }
    const Stats &stats() const { return stats_; }

private:
    struct Slot
    {
        uint8_t payload[RLRS_AIR_PAYLOAD_MAX];
        uint32_t order;
        uint8_t length;
        uint8_t id;
        uint8_t priority;
        uint8_t skips;      // frames this head message was passed over
        bool used;
    };

    int oldest(uint8_t priority) const;
    void packSlot(int index, uint8_t *out, uint8_t &pos);

    Slot slots_[RLRS_AIR_QUEUE_COUNT];
    Stats stats_;
    uint32_t nextOrder_;
    uint8_t count_;
    uint8_t maxFrameSize_;
};

// Iterates over a received air frame.
//   RlrsAirReader reader;
//   if (reader.begin(data, len)) { uint8_t id, n; const uint8_t *p;
//       while (reader.next(id, p, n)) { rlrsEncode(id, p, n, wire, sizeof wire); ... } }
//   reader.malformed() tells whether parsing stopped on a bad message.
class RlrsAirReader
{
public:
    RlrsAirReader();
    // Parse the header. Returns false (and malformed()) on a short frame or
    // reserved flags.
    bool begin(const uint8_t *frame, size_t length);
    const RlrsAirHeader &header() const { return header_; }
    // Next message; false at the end of the frame, at the padding, or on a
    // malformed message (len > 64, truncated, or non pad byte after the
    // padding), in which case the rest of the frame is ignored.
    bool next(uint8_t &id, const uint8_t *&payload, uint8_t &length);
    bool malformed() const { return malformed_; }

private:
    const uint8_t *frame_;
    size_t length_;
    size_t position_;
    RlrsAirHeader header_;
    bool malformed_;
};

#endif
