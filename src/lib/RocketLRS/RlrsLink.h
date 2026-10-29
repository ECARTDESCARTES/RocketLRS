#ifndef H_RLRS_LINK
#define H_RLRS_LINK

#include <cstddef>
#include <cstdint>

#include "RlrsParser.h"
#include "RlrsProtocol.h"

// Number of frames that can wait for an air slot.
#ifndef RLRS_LINK_FRAME_COUNT
#define RLRS_LINK_FRAME_COUNT 8
#endif
// Bytes buffered for the UART/host side.
#ifndef RLRS_LINK_UART_TX_SIZE
#define RLRS_LINK_UART_TX_SIZE 512
#endif
// Reliable frames are repeated after this many radio packets...
#ifndef RLRS_LINK_RETRY_GAP
#define RLRS_LINK_RETRY_GAP 10
#endif
// ...up to this many transmissions.
#ifndef RLRS_LINK_MAX_ATTEMPTS
#define RLRS_LINK_MAX_ATTEMPTS 3
#endif

// Air framing for RocketLRS over an ELRS data channel.
//
// A complete RLRS frame is sent in a single radio packet whenever it fits.
// The RLRS sync word and CRC are not transmitted: the radio packet already
// carries a CRC and the UART frame is rebuilt on the other side. Only frames
// larger than the radio payload are split, and each split packet then costs a
// single control byte (no extra CRC).
//
// Segment layout, where C is the radio data payload size (10 bytes on OTA8,
// 5 bytes on OTA4):
//   0x00..0x3F  whole frame      [len][id][payload...]          len + 2 <= C
//   0x40..0x7F  first fragment   [0x40|totalLen][id][data...]
//   0x80..0x8F  next fragment    [0x80|seq][data...]
//   0x90..0x9F  reliable frame   [0x90|seq][len][id][payload...]
//   0xA0..0xAF  acknowledgement  [0xA0|seq]
//   0xFF        idle             nothing to send
//
// Small periodic packets (gyro, heartbeat, battery...) therefore never get
// fragmented and never pay any per-packet overhead beyond their own length.
//
// Delivery policy: frames coming from the UART are ALWAYS best effort. The
// RLRS protocol already has its own CMD/ACK exchange (0x03/0x04) and that
// application-level acknowledgement stays the only authority for arm,
// parachute and other sensitive commands, so the radio never duplicates them.
// Radio-level ARQ (enqueue(..., reliable = true)) is opt-in and reserved for
// future state/telemetry frames that tolerate a duplicated copy.
class RlrsLink
{
public:
    enum Priority : uint8_t
    {
        PriorityCritical = 0,   // commands (arm, parachute...)
        PriorityControl = 1,    // heartbeats, ACKs, link stats
        PriorityTelemetry = 2,  // gyro, GPS, pressure, battery...
        PriorityBulk = 3        // everything else
    };

    struct Stats
    {
        uint32_t uartFramesIn;
        uint32_t uartFramesOut;
        uint32_t airFramesOut;
        uint32_t airFramesIn;
        uint32_t framesDropped;
        uint32_t uartOverflow;
        uint32_t airLost;
        uint32_t malformed;
    };

    struct LinkStats
    {
        uint8_t uplinkRssi;      // positive value, i.e. -dBm
        int8_t uplinkSnr;
        uint8_t uplinkLq;        // percent
        uint8_t downlinkRssi;    // positive value, i.e. -dBm
        int8_t downlinkSnr;
        uint8_t downlinkLq;      // percent
        uint8_t txPower;
        uint8_t rfMode;
    };

    // Payload IDs are the RLRS packet IDs (see protocole.md).
    static const uint8_t LinkStatsId = 0xF0;

    RlrsLink();

    // Drop every queued frame and parser state (e.g. on (re)connection).
    void reset();

    // ---- UART / host side ------------------------------------------------
    // Feed raw bytes received from the host. Complete, CRC-valid frames are
    // queued for the air; partial frames are kept by the parser.
    void feedUart(const uint8_t *data, uint16_t length);
    // Copy up to maxLength bytes of RLRS wire frames (AA/CRC rebuilt) ready
    // for the host UART. Returns the number of bytes copied.
    uint16_t takeUart(uint8_t *destination, uint16_t maxLength);
    uint16_t uartPending() const { return uartCount; }

    // ---- Air side --------------------------------------------------------
    // Write one radio data payload. Returns the number of bytes written, or 0
    // when there is nothing to send (caller should send an idle packet).
    uint8_t packAir(uint8_t *destination, uint8_t capacity);
    // Decode one received radio data payload (capacity = payload size of the
    // current OTA mode). Complete frames are queued for the host UART.
    void unpackAir(const uint8_t *source, uint8_t capacity);

    bool airPending() const;
    uint8_t queueDepth() const;

    // ---- Queueing --------------------------------------------------------
    bool enqueue(uint8_t id, const uint8_t *payload, uint8_t length,
                 uint8_t priority = PriorityTelemetry, bool reliable = false);
    bool enqueueLinkStats(const LinkStats &stats);

    static uint8_t priorityForId(uint8_t id);

    const Stats &stats() const { return stats_; }

private:
    struct Frame
    {
        uint8_t data[RLRS_MAX_SIZE_PAYLOAD];
        uint8_t length;
        uint8_t id;
        uint8_t priority;
        uint8_t seq;        // 1..15 for reliable frames, 0 otherwise
        uint8_t attempts;
        uint8_t offset;     // bytes already handed to the air
        uint8_t fragment;   // next fragment sequence number
        uint8_t retryIn;    // radio packets to wait before a repeat
        uint16_t order;
        bool inUse;
    };

    static void onUartFrame(RlrsUartPacket packet, void *context);
    void deliver(uint8_t id, const uint8_t *payload, uint8_t length);
    void finishReassembly();
    void enqueueAck(uint8_t seq);
    void clearReliable(uint8_t seq);
    bool pushUart(const uint8_t *data, uint16_t length);
    void freeFrame(Frame *frame);
    Frame *bestFrame();
    uint8_t nextReliableSeq();

    RlrsParser inputParser;

    Frame frames[RLRS_LINK_FRAME_COUNT];
    uint16_t nextOrder;

    // One-frame reassembly buffer; a lost fragment discards the whole frame.
    uint8_t rxBuffer[RLRS_MAX_SIZE_PAYLOAD];
    uint8_t rxId;
    uint8_t rxTotal;
    uint8_t rxHave;
    uint8_t rxFragment;
    bool rxActive;

    // Bits 1..15: acknowledgements waiting for the next radio packet.
    uint16_t pendingAcks;

    uint8_t uartOut[RLRS_LINK_UART_TX_SIZE];
    uint16_t uartHead;
    uint16_t uartTail;
    uint16_t uartCount;

    Stats stats_;
};

#endif // H_RLRS_LINK
