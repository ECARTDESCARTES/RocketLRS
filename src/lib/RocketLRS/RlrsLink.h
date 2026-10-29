#ifndef H_RLRS_LINK
#define H_RLRS_LINK

#include <cstddef>
#include <cstdint>

#include "RlrsParser.h"
#include "RlrsProtocol.h"

// packAir()/unpackAir()/airPending() run in the radio ISRs (TX: timer and
// RX-done callbacks, RX: HandleSendDataDl/ProcessRfPacket_DataUl) while
// enqueue()/feedUart()/takeUart()/reset() run in loop(). Every public entry
// touching the shared state takes a short critical section, and the ISR path
// lives in IRAM (ESP8285: the flash cache can be off when an ISR fires).
// The ISR path never builds UART wire frames (no CRC): it stores raw records,
// takeUart() encodes them in loop() context.
#if defined(PLATFORM_ESP32)
#include <freertos/FreeRTOS.h>
#include <esp_attr.h>
#define RLRS_IRAM IRAM_ATTR
#elif defined(PLATFORM_ESP8266)
#include <Arduino.h>
#define RLRS_IRAM IRAM_ATTR // ICACHE_RAM_ATTR is deprecated in core 3.x
#else
#define RLRS_IRAM
#endif

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
// A radio packet carries one or more segments. Whole frames, reliable frames
// and acknowledgements are self-delimited, so as many of them as fit are
// stacked in the same packet (in priority order). The RLRS sync word and CRC
// are not transmitted: the radio packet already carries a CRC and the UART
// frame is rebuilt on the other side. Only frames larger than the radio
// payload are split.
//
// Segment layout, where C is the radio data payload size (10 bytes on OTA8,
// 5 bytes on OTA4, less inside a LINKSTATS packet):
//   0x00..0x3F  whole frame      [len][id][payload...]
//   0x40..0x7F  first fragment   [0x40|totalLen][id][tag][data...]
//   0x80..0x8F  next fragment    [0x80|n][data...]
//   0x90..0x9F  reliable frame   [0x90|seq][len][id][payload...]
//   0xA0..0xAF  acknowledgement  [0xA0|seq]
//   0xFF        idle / padding   end of the packet
//
// A fragment (first or next) always ends the packet: its data runs to the end
// of the slot, or to the end of the frame. Unused bytes MUST be 0xFF: 0x00
// would decode as an empty frame of id 0x00 (HEARTBEAT).
//
// Continuations are numbered by a link-wide 4-bit counter that never restarts
// at a new frame; the first fragment carries (tag) the number of its first
// continuation. A receiver that missed the tail of one frame and the head of
// the next therefore rejects the next frame's continuations instead of
// splicing them into the old buffer. Only exactly 16k consecutive lost
// continuations, with no first fragment received in between, can alias.
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
    // The continuation counter and the reliable sequence number survive (and
    // the counter skips one value): the peer may still hold the head of a
    // split frame, or an ACK in flight, from before the reset.
    void reset();
    // Drop only a split frame being reassembled (e.g. ground side on
    // (re)connection: the peer may have reset in between).
    void resetReassembly();

    // ---- UART / host side ------------------------------------------------
    // Feed raw bytes received from the host. Complete, CRC-valid frames are
    // queued for the air; partial frames are kept by the parser.
    void feedUart(const uint8_t *data, uint16_t length);
    // Copy up to maxLength bytes of RLRS wire frames (AA/CRC rebuilt) ready
    // for the host UART. Returns the number of bytes copied.
    uint16_t takeUart(uint8_t *destination, uint16_t maxLength);
    // Raw bytes still waiting for the host (not the exact wire size).
    uint16_t uartPending() const;

    // ---- Air side --------------------------------------------------------
    // Write one radio data payload (several segments when they fit). Returns
    // the number of bytes written, or 0 when there is nothing to send (caller
    // should send an idle packet). The caller pads the rest with 0xFF.
    // maxCapacity is the largest payload this direction ever offers (0: same
    // as capacity). A frame that fits whole in maxCapacity is never split in
    // a smaller slot: it waits (OTA8 downlink alternates LINKSTATS = 6 bytes
    // and DATA = 10 bytes containers).
    uint8_t packAir(uint8_t *destination, uint8_t capacity, uint8_t maxCapacity = 0);
    // Decode one received radio data payload (capacity = payload size of the
    // current OTA mode), segment after segment until 0xFF or the end.
    // Complete frames are queued for the host UART.
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
    // Short critical section shared by loop() and the radio ISRs.
    class Guard
    {
    public:
        explicit inline __attribute__((always_inline)) Guard(const RlrsLink &link)
#if defined(PLATFORM_ESP32)
            : mux(&link.mux)
        {
#if defined(portENTER_CRITICAL_SAFE)
            portENTER_CRITICAL_SAFE(mux);
#else
            portENTER_CRITICAL(mux);
#endif
        }
        inline __attribute__((always_inline)) ~Guard()
        {
#if defined(portEXIT_CRITICAL_SAFE)
            portEXIT_CRITICAL_SAFE(mux);
#else
            portEXIT_CRITICAL(mux);
#endif
        }
    private:
        portMUX_TYPE *mux;
#elif defined(PLATFORM_ESP8266)
            : saved(xt_rsil(15)) // also safe inside an ISR: restores the level
        {
            (void)link;
        }
        inline __attribute__((always_inline)) ~Guard() { xt_wsr_ps(saved); }
    private:
        uint32_t saved;
#else
        {
            (void)link;
        }
#endif
        Guard(const Guard &);
        Guard &operator=(const Guard &);
    };

    // Arrival order comparison that survives the 16-bit wrap.
    static inline bool olderThan(uint16_t a, uint16_t b)
    {
        return static_cast<int16_t>(static_cast<uint16_t>(a - b)) < 0;
    }

    struct Frame
    {
        uint8_t data[RLRS_MAX_SIZE_PAYLOAD];
        uint8_t length;
        uint8_t id;
        uint8_t priority;
        uint8_t seq;        // 1..15 for reliable frames, 0 otherwise
        uint8_t attempts;
        uint8_t offset;     // bytes already handed to the air
        uint8_t retryIn;    // radio packets to wait before a repeat
        uint16_t order;
        bool inUse;
    };

    static void onUartFrame(RlrsUartPacket packet, void *context);
    void deliver(uint8_t id, const uint8_t *payload, uint8_t length);
    void finishReassembly();
    void enqueueAck(uint8_t seq);
    void clearReliable(uint8_t seq);
    bool pushUart(uint8_t id, const uint8_t *data, uint8_t length);
    void freeFrame(Frame *frame);
    void clearReassembly();
    uint8_t nextReliableSeq();
    uint32_t tickRetries();
    Frame *partialFrame();
    Frame *bestCandidate(uint32_t eligible, uint32_t considered, uint8_t priorityBelow);
    static bool fitsWhole(const Frame &frame, uint8_t space);
    bool writeWhole(Frame &frame, uint8_t *destination, uint8_t &pos, uint8_t limit,
                    uint8_t capacity);
    uint8_t packWhole(uint8_t *destination, uint8_t pos, uint8_t limit, uint8_t capacity,
                      uint8_t priorityBelow, uint32_t eligible, uint32_t &considered);
    uint8_t writeFirstFragment(Frame &frame, uint8_t *destination, uint8_t pos,
                               uint8_t capacity);
    uint8_t writeNextFragment(Frame &frame, uint8_t *destination, uint8_t pos,
                              uint8_t capacity);

    RlrsParser inputParser;

    Frame frames[RLRS_LINK_FRAME_COUNT];
    uint16_t nextOrder;
    uint8_t txFragment;     // number of the next continuation sent (4 bits)
    uint8_t lastReliableSeq; // last sequence number handed out (rotates)

    // One-frame reassembly buffer; a lost fragment discards the whole frame.
    uint8_t rxBuffer[RLRS_MAX_SIZE_PAYLOAD];
    uint8_t rxId;
    uint8_t rxTotal;
    uint8_t rxHave;
    uint8_t rxFragment;
    bool rxActive;

    // Bits 1..15: acknowledgements waiting for the next radio packet.
    uint16_t pendingAcks;

    // Raw records [id][len][payload...] filled by the ISR path.
    uint8_t uartOut[RLRS_LINK_UART_TX_SIZE];
    uint16_t uartHead;
    uint16_t uartTail;
    uint16_t uartCount;
    // loop() side: one record rebuilt as an RLRS wire frame (AA, CRC).
    uint8_t uartWire[RLRS_MAX_SIZE];
    uint8_t uartWireLength;
    uint8_t uartWirePos;

#if defined(PLATFORM_ESP32)
    mutable portMUX_TYPE mux;
#endif

    Stats stats_;
};

#endif // H_RLRS_LINK
