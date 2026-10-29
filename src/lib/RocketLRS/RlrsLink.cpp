#include "RlrsLink.h"

#include <cstring>

RlrsLink::RlrsLink() : nextOrder(0), txFragment(1), lastReliableSeq(0)
{
#if defined(PLATFORM_ESP32)
    portMUX_TYPE unlocked = portMUX_INITIALIZER_UNLOCKED;
    mux = unlocked;
#endif
    reset();
}

void RlrsLink::reset()
{
    // The UART parser is only used from loop(): no lock needed for it.
    inputParser.reset();
    inputParser.setHandler(&RlrsLink::onUartFrame, this);

    Guard guard(*this);
    for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
    {
        std::memset(&frames[i], 0, sizeof(frames[i]));
        frames[i].inUse = false;
    }

    nextOrder = 0;
    // Not restarted: the peer may hold the head of a frame dropped here, and
    // the skipped value keeps the next frame's continuations from matching
    // the one it expects (a first fragment lost right after the reset would
    // otherwise splice two frames, the rebuilt CRC hiding it).
    txFragment = static_cast<uint8_t>((txFragment + 1) & 0x0F);
    clearReassembly();
    pendingAcks = 0;

    uartHead = 0;
    uartTail = 0;
    uartCount = 0;
    uartWireLength = 0;
    uartWirePos = 0;

    stats_ = Stats();
}

void RlrsLink::resetReassembly()
{
    Guard guard(*this);
    if (rxActive)
        ++stats_.airLost;
    clearReassembly();
}

void RLRS_IRAM RlrsLink::clearReassembly()
{
    rxActive = false;
    rxId = 0;
    rxTotal = 0;
    rxHave = 0;
    rxFragment = 0;
}

// ---------------------------------------------------------------------------
// UART / host side
// ---------------------------------------------------------------------------

void RlrsLink::onUartFrame(RlrsUartPacket packet, void *context)
{
    RlrsLink *link = static_cast<RlrsLink *>(context);
    if (link == nullptr)
        return;
    ++link->stats_.uartFramesIn;
    // Commands are not made reliable here: the RLRS CMD/ACK exchange is the
    // authority, and retransmitting a frame would execute it twice.
    link->enqueue(packet.packet_id, packet.payload, packet.payload_len,
                  priorityForId(packet.packet_id));
}

void RlrsLink::feedUart(const uint8_t *data, uint16_t length)
{
    if (data == nullptr || length == 0)
        return;
    inputParser.processBytes(data, length);
}

uint16_t RlrsLink::uartPending() const
{
    return static_cast<uint16_t>(uartCount + (uartWireLength - uartWirePos));
}

uint16_t RlrsLink::takeUart(uint8_t *destination, uint16_t maxLength)
{
    if (destination == nullptr || maxLength == 0)
        return 0;

    uint16_t copied = 0;
    while (copied < maxLength)
    {
        if (uartWirePos >= uartWireLength)
        {
            // Pop one raw record under the lock, encode it outside.
            uint8_t id = 0;
            uint8_t length = 0;
            uint8_t payload[RLRS_MAX_SIZE_PAYLOAD];
            {
                Guard guard(*this);
                if (uartCount < 2)
                    break;
                id = uartOut[uartTail];
                length = uartOut[(uartTail + 1) % RLRS_LINK_UART_TX_SIZE];
                for (uint8_t i = 0; i < length; ++i)
                    payload[i] = uartOut[(uartTail + 2 + i) % RLRS_LINK_UART_TX_SIZE];
                uartTail = static_cast<uint16_t>((uartTail + 2 + length) % RLRS_LINK_UART_TX_SIZE);
                uartCount = static_cast<uint16_t>(uartCount - 2 - length);
            }
            uartWirePos = 0;
            uartWireLength = static_cast<uint8_t>(
                rlrsEncode(id, payload, length, uartWire, sizeof(uartWire)));
            if (uartWireLength == 0)
                continue; // cannot happen: deliver() checks the length
        }
        uint16_t chunk = static_cast<uint16_t>(uartWireLength - uartWirePos);
        if (chunk > maxLength - copied)
            chunk = static_cast<uint16_t>(maxLength - copied);
        std::memcpy(&destination[copied], &uartWire[uartWirePos], chunk);
        uartWirePos = static_cast<uint8_t>(uartWirePos + chunk);
        copied = static_cast<uint16_t>(copied + chunk);
    }
    return copied;
}

// ISR path (caller holds the lock): store [id][len][payload...].
bool RLRS_IRAM RlrsLink::pushUart(uint8_t id, const uint8_t *data, uint8_t length)
{
    const uint16_t needed = static_cast<uint16_t>(2u + length);
    if (uartCount + needed > RLRS_LINK_UART_TX_SIZE)
    {
        ++stats_.uartOverflow;
        return false;
    }

    uartOut[uartHead] = id;
    uartHead = static_cast<uint16_t>((uartHead + 1) % RLRS_LINK_UART_TX_SIZE);
    uartOut[uartHead] = length;
    uartHead = static_cast<uint16_t>((uartHead + 1) % RLRS_LINK_UART_TX_SIZE);
    for (uint8_t i = 0; i < length; ++i)
    {
        uartOut[uartHead] = data[i];
        uartHead = static_cast<uint16_t>((uartHead + 1) % RLRS_LINK_UART_TX_SIZE);
    }
    uartCount = static_cast<uint16_t>(uartCount + needed);
    return true;
}

// ---------------------------------------------------------------------------
// Queueing
// ---------------------------------------------------------------------------

uint8_t RlrsLink::priorityForId(uint8_t id)
{
    switch (id)
    {
    case 0x03: // CMD
    case 0x04: // ACK
        return PriorityCritical;
    case 0x00: // HEARTBEAT
    case 0x05: // LED
    case 0x06: // SERVO
    case LinkStatsId:
        return PriorityControl;
    case 0x01: // INFO
    case 0x02: // GPS
    case 0x07: // reserved for pressure
    case 0x08: // reserved for battery
        return PriorityTelemetry;
    default:
        return PriorityBulk;
    }
}

uint8_t RlrsLink::nextReliableSeq()
{
    // Rotate from the last number handed out: a late or duplicated ACK of a
    // finished frame must not acknowledge the next one.
    for (uint8_t attempt = 1; attempt <= 15; ++attempt)
    {
        const uint8_t seq = static_cast<uint8_t>((lastReliableSeq + attempt - 1) % 15 + 1);

        bool used = false;
        for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
        {
            if (frames[i].inUse && frames[i].seq == seq)
            {
                used = true;
                break;
            }
        }
        if (!used)
        {
            lastReliableSeq = seq;
            return seq;
        }
    }
    return 0;
}

void RLRS_IRAM RlrsLink::freeFrame(Frame *frame)
{
    if (frame == nullptr)
        return;
    std::memset(frame, 0, sizeof(*frame));
    frame->inUse = false;
}

bool RlrsLink::enqueue(uint8_t id, const uint8_t *payload, uint8_t length,
                       uint8_t priority, bool reliable)
{
    if (length > RLRS_MAX_SIZE_PAYLOAD || (length != 0 && payload == nullptr) ||
        priority >= 4)
        return false;

    Guard guard(*this);

    // Periodic telemetry is state, not a stream: a newer sample replaces an
    // older one that has not started transmitting yet.
    if (priority >= PriorityTelemetry)
    {
        for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
        {
            Frame &frame = frames[i];
            if (frame.inUse && frame.id == id && frame.offset == 0 &&
                frame.attempts == 0 && frame.seq == 0)
            {
                if (length != 0)
                    std::memcpy(frame.data, payload, length);
                frame.length = length;
                frame.priority = priority;
                frame.order = nextOrder++;
                return true;
            }
        }
    }

    Frame *slot = nullptr;
    for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
    {
        if (!frames[i].inUse)
        {
            slot = &frames[i];
            break;
        }
    }

    if (slot == nullptr)
    {
        // Reclaim the least important not-yet-sent frame, but never a
        // reliable one (it may already have been seen by the other side).
        Frame *victim = nullptr;
        for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
        {
            Frame &frame = frames[i];
            if (!frame.inUse || frame.offset != 0 || frame.seq != 0)
                continue;
            if (victim == nullptr || frame.priority > victim->priority ||
                (frame.priority == victim->priority && olderThan(frame.order, victim->order)))
                victim = &frame;
        }

        if (victim == nullptr || victim->priority <= priority)
        {
            ++stats_.framesDropped;
            return false;
        }
        slot = victim;
    }

    std::memset(slot, 0, sizeof(*slot));
    slot->inUse = true;
    slot->id = id;
    slot->length = length;
    slot->priority = priority;
    slot->order = nextOrder++;
    if (length != 0)
        std::memcpy(slot->data, payload, length);

    if (reliable)
    {
        slot->seq = nextReliableSeq();
        if (slot->seq == 0)
        {
            // No sequence number available; fall back to best effort.
            reliable = false;
        }
    }

    if (!reliable)
        slot->seq = 0;

    return true;
}

bool RlrsLink::enqueueLinkStats(const LinkStats &stats)
{
    uint8_t payload[8];
    payload[0] = stats.uplinkRssi;
    payload[1] = static_cast<uint8_t>(stats.uplinkSnr);
    payload[2] = stats.uplinkLq;
    payload[3] = stats.downlinkRssi;
    payload[4] = static_cast<uint8_t>(stats.downlinkSnr);
    payload[5] = stats.downlinkLq;
    payload[6] = stats.txPower;
    payload[7] = stats.rfMode;
    return enqueue(LinkStatsId, payload, sizeof(payload), PriorityControl);
}

// ---------------------------------------------------------------------------
// Air side
// ---------------------------------------------------------------------------

namespace
{
const uint8_t kWholeMaxLength = 0x3F;  // 0x40 and above are other opcodes
const uint8_t kOpFirstFragment = 0x40;
const uint8_t kOpNextFragment = 0x80;
const uint8_t kOpReliable = 0x90;
const uint8_t kOpAck = 0xA0;
const uint8_t kAnyPriority = 4;
} // namespace

static_assert(RLRS_LINK_FRAME_COUNT <= 32, "frame masks are 32 bits wide");

// Once per radio packet: age the reliable frames waiting for an ACK, drop the
// exhausted ones, and return the mask of those due for a repeat.
uint32_t RLRS_IRAM RlrsLink::tickRetries()
{
    uint32_t eligible = 0;
    for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
    {
        Frame &frame = frames[i];
        if (!frame.inUse || frame.seq == 0 || frame.attempts == 0 || frame.offset != 0)
            continue;
        if (frame.retryIn > 0)
        {
            --frame.retryIn;
            continue;
        }
        if (frame.attempts >= RLRS_LINK_MAX_ATTEMPTS)
        {
            freeFrame(&frame);
            continue;
        }
        eligible |= (1u << i);
    }
    return eligible;
}

RLRS_IRAM RlrsLink::Frame *RlrsLink::partialFrame()
{
    for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
    {
        if (frames[i].inUse && frames[i].offset > 0)
            return &frames[i];
    }
    return nullptr;
}

// Most important frame ready to go (new, or a reliable one due for a repeat),
// not yet considered for this packet, with priority < priorityBelow.
RLRS_IRAM RlrsLink::Frame *RlrsLink::bestCandidate(uint32_t eligible, uint32_t considered,
                                         uint8_t priorityBelow)
{
    Frame *best = nullptr;
    for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
    {
        Frame &frame = frames[i];
        if (!frame.inUse || frame.offset != 0 || (considered & (1u << i)) != 0 ||
            frame.priority >= priorityBelow)
            continue;
        if (frame.attempts != 0 && (eligible & (1u << i)) == 0)
            continue;
        if (best == nullptr || frame.priority < best->priority ||
            (frame.priority == best->priority && olderThan(frame.order, best->order)))
            best = &frame;
    }
    return best;
}

// True when the frame fits whole (reliable or plain) in `space` bytes.
bool RLRS_IRAM RlrsLink::fitsWhole(const Frame &frame, uint8_t space)
{
    if (frame.seq != 0 && 3u + frame.length <= space)
        return true;
    return frame.length <= kWholeMaxLength && 2u + frame.length <= space;
}

// Write the frame whole at destination[pos] if it fits before `limit`.
// A reliable frame keeps its ACK whenever the slot size allows it; one that
// can never carry its sequence number at this capacity goes best effort.
bool RLRS_IRAM RlrsLink::writeWhole(Frame &frame, uint8_t *destination, uint8_t &pos,
                          uint8_t limit, uint8_t capacity)
{
    const uint8_t space = static_cast<uint8_t>(limit - pos);
    if (frame.seq != 0 && 3u + frame.length <= capacity)
    {
        if (3u + frame.length > space)
            return false;
        destination[pos] = static_cast<uint8_t>(kOpReliable | (frame.seq & 0x0F));
        destination[pos + 1] = frame.length;
        destination[pos + 2] = frame.id;
        if (frame.length != 0)
            std::memcpy(&destination[pos + 3], frame.data, frame.length);
        pos = static_cast<uint8_t>(pos + 3 + frame.length);
        ++frame.attempts;
        frame.retryIn = RLRS_LINK_RETRY_GAP;
        return true;
    }

    if (frame.length > kWholeMaxLength || 2u + frame.length > space)
        return false;
    destination[pos] = frame.length;
    destination[pos + 1] = frame.id;
    if (frame.length != 0)
        std::memcpy(&destination[pos + 2], frame.data, frame.length);
    pos = static_cast<uint8_t>(pos + 2 + frame.length);
    freeFrame(&frame);
    return true;
}

// Stack whole frames (priority < priorityBelow) in [pos, limit), most
// important first, skipping those that do not fit. Never splits.
uint8_t RLRS_IRAM RlrsLink::packWhole(uint8_t *destination, uint8_t pos, uint8_t limit,
                            uint8_t capacity, uint8_t priorityBelow,
                            uint32_t eligible, uint32_t &considered)
{
    while (pos < limit && limit - pos >= 2)
    {
        Frame *frame = bestCandidate(eligible, considered, priorityBelow);
        if (frame == nullptr)
            break;
        considered |= (1u << static_cast<uint8_t>(frame - frames));
        if (writeWhole(*frame, destination, pos, limit, capacity))
            ++stats_.airFramesOut;
    }
    return pos;
}

uint8_t RLRS_IRAM RlrsLink::writeFirstFragment(Frame &frame, uint8_t *destination, uint8_t pos,
                                     uint8_t capacity)
{
    // A split frame loses its radio-level ACK (it is no small control frame).
    frame.seq = 0;
    // 0 encodes a 64-byte payload: an empty frame is never fragmented.
    destination[pos] = static_cast<uint8_t>(kOpFirstFragment | (frame.length & 0x3F));
    destination[pos + 1] = frame.id;
    destination[pos + 2] = txFragment;
    uint8_t chunk = static_cast<uint8_t>(capacity - pos - 3);
    if (chunk > frame.length)
        chunk = frame.length;
    if (chunk != 0)
        std::memcpy(&destination[pos + 3], frame.data, chunk);
    frame.offset = chunk;
    ++stats_.airFramesOut;
    if (frame.offset >= frame.length)
        freeFrame(&frame);
    return static_cast<uint8_t>(pos + 3 + chunk);
}

uint8_t RLRS_IRAM RlrsLink::writeNextFragment(Frame &frame, uint8_t *destination, uint8_t pos,
                                    uint8_t capacity)
{
    // Callers guarantee at least one data byte: never an empty continuation.
    const uint8_t remaining = static_cast<uint8_t>(frame.length - frame.offset);
    uint8_t chunk = static_cast<uint8_t>(capacity - pos - 1);
    if (chunk > remaining)
        chunk = remaining;
    destination[pos] = static_cast<uint8_t>(kOpNextFragment | (txFragment & 0x0F));
    std::memcpy(&destination[pos + 1], &frame.data[frame.offset], chunk);
    frame.offset = static_cast<uint8_t>(frame.offset + chunk);
    txFragment = static_cast<uint8_t>((txFragment + 1) & 0x0F);
    ++stats_.airFramesOut;
    if (frame.offset >= frame.length)
        freeFrame(&frame);
    return static_cast<uint8_t>(pos + 1 + chunk);
}

uint8_t RLRS_IRAM RlrsLink::packAir(uint8_t *destination, uint8_t capacity,
                                    uint8_t maxCapacity)
{
    if (destination == nullptr || capacity < 1)
        return 0;
    if (maxCapacity < capacity)
        maxCapacity = capacity;

    Guard guard(*this);

    const uint32_t eligible = tickRetries();
    uint8_t pos = 0;

    // 1) Acknowledgements: one byte each and time sensitive.
    for (uint8_t seq = 1; seq <= 15 && pendingAcks != 0 && pos < capacity; ++seq)
    {
        const uint16_t mask = static_cast<uint16_t>(1u << seq);
        if ((pendingAcks & mask) != 0)
        {
            pendingAcks = static_cast<uint16_t>(pendingAcks & ~mask);
            destination[pos++] = static_cast<uint8_t>(kOpAck | seq);
            ++stats_.airFramesOut;
        }
    }

    // Every data segment needs at least two bytes.
    if (capacity - pos < 2)
        return pos;

    uint32_t considered = 0;
    Frame *partial = partialFrame();

    if (partial != nullptr)
    {
        // 2a) A split frame must finish before another one starts (single
        // reassembly buffer), but strictly more important frames go first.
        pos = packWhole(destination, pos, capacity, maxCapacity, partial->priority,
                        eligible, considered);
        const uint8_t remaining = static_cast<uint8_t>(partial->length - partial->offset);
        const uint8_t space = static_cast<uint8_t>(capacity - pos);
        if (space >= 1u + remaining)
        {
            // The split frame ends in this packet: stack other frames in
            // front of its last fragment, which must stay last.
            const uint8_t limit = static_cast<uint8_t>(capacity - 1 - remaining);
            pos = packWhole(destination, pos, limit, maxCapacity, kAnyPriority,
                            eligible, considered);
            pos = writeNextFragment(*partial, destination, pos, capacity);
        }
        else if (space >= 2)
        {
            pos = writeNextFragment(*partial, destination, pos, capacity);
        }
        return pos;
    }

    // 2b) Whole frames in priority order. A frame that fits a fresh packet
    // (of maxCapacity) but not the leftover waits for the next packet; a frame
    // that never fits whole is split, using the rest of this packet, unless a
    // more important frame was left behind (the split would then block it).
    bool skipped = false;
    while (capacity - pos >= 2)
    {
        Frame *frame = bestCandidate(eligible, considered, kAnyPriority);
        if (frame == nullptr)
            break;
        considered |= (1u << static_cast<uint8_t>(frame - frames));

        if (writeWhole(*frame, destination, pos, capacity, maxCapacity))
        {
            ++stats_.airFramesOut;
            continue;
        }
        if (!fitsWhole(*frame, maxCapacity) && !skipped && capacity - pos >= 4)
        {
            pos = writeFirstFragment(*frame, destination, pos, capacity);
            break; // a fragment always ends the packet
        }
        skipped = true;
    }
    return pos;
}

void RLRS_IRAM RlrsLink::unpackAir(const uint8_t *source, uint8_t capacity)
{
    if (source == nullptr || capacity == 0)
        return;

    Guard guard(*this);

    uint8_t pos = 0;
    while (pos < capacity)
    {
        const uint8_t opcode = source[pos];
        const uint8_t space = static_cast<uint8_t>(capacity - pos);

        if (opcode <= 0x3F)
        {
            const uint8_t length = opcode;
            if (2u + length > space)
            {
                ++stats_.malformed;
                return;
            }
            deliver(source[pos + 1], &source[pos + 2], length);
            pos = static_cast<uint8_t>(pos + 2 + length);
        }
        else if (opcode <= 0x7F)
        {
            if (space < 4)
            {
                ++stats_.malformed;
                return;
            }
            if (rxActive)
                ++stats_.airLost; // the previous split frame never completed

            rxTotal = static_cast<uint8_t>(opcode & 0x3F);
            if (rxTotal == 0)
                rxTotal = RLRS_MAX_SIZE_PAYLOAD; // 64 is encoded as zero
            rxId = source[pos + 1];
            rxFragment = static_cast<uint8_t>(source[pos + 2] & 0x0F);
            rxActive = true;

            uint8_t chunk = static_cast<uint8_t>(space - 3);
            if (chunk > rxTotal)
                chunk = rxTotal;
            std::memcpy(rxBuffer, &source[pos + 3], chunk);
            rxHave = chunk;

            if (rxHave >= rxTotal)
                finishReassembly();
            return; // a fragment always ends the packet
        }
        else if (opcode <= 0x8F)
        {
            if (space < 2)
            {
                ++stats_.malformed;
                return;
            }
            if (!rxActive)
                return; // continuation of a frame whose head was lost
            const uint8_t fragment = static_cast<uint8_t>(opcode & 0x0F);
            if (fragment != rxFragment)
            {
                // Lost continuation, or one belonging to another frame.
                rxActive = false;
                ++stats_.airLost;
                return;
            }

            const uint8_t remaining = static_cast<uint8_t>(rxTotal - rxHave);
            uint8_t chunk = static_cast<uint8_t>(space - 1);
            if (chunk > remaining)
                chunk = remaining;
            std::memcpy(&rxBuffer[rxHave], &source[pos + 1], chunk);
            rxHave = static_cast<uint8_t>(rxHave + chunk);
            rxFragment = static_cast<uint8_t>((rxFragment + 1) & 0x0F);

            if (rxHave >= rxTotal)
                finishReassembly();
            return; // a fragment always ends the packet
        }
        else if (opcode <= 0x9F)
        {
            if (space < 3)
            {
                ++stats_.malformed;
                return;
            }
            const uint8_t seq = static_cast<uint8_t>(opcode & 0x0F);
            const uint8_t length = source[pos + 1];
            if (length > RLRS_MAX_SIZE_PAYLOAD || 3u + length > space)
            {
                ++stats_.malformed;
                return;
            }
            deliver(source[pos + 2], &source[pos + 3], length);
            enqueueAck(seq);
            pos = static_cast<uint8_t>(pos + 3 + length);
        }
        else if (opcode <= 0xAF)
        {
            clearReliable(static_cast<uint8_t>(opcode & 0x0F));
            ++pos;
        }
        else
        {
            // 0xFF is the idle/padding marker (end of packet); anything else
            // is reserved and also ends decoding.
            return;
        }
    }
}

void RLRS_IRAM RlrsLink::finishReassembly()
{
    deliver(rxId, rxBuffer, rxTotal);
    clearReassembly();
}

void RLRS_IRAM RlrsLink::enqueueAck(uint8_t seq)
{
    if (seq >= 1 && seq <= 15)
        pendingAcks = static_cast<uint16_t>(pendingAcks | (1u << seq));
}

void RLRS_IRAM RlrsLink::clearReliable(uint8_t seq)
{
    if (seq == 0)
        return;
    for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
    {
        Frame &frame = frames[i];
        if (frame.inUse && frame.seq == seq)
        {
            freeFrame(&frame);
            return;
        }
    }
}

void RLRS_IRAM RlrsLink::deliver(uint8_t id, const uint8_t *payload, uint8_t length)
{
    // The wire frame (AA, CRC) is rebuilt by takeUart() in loop() context.
    if (length > RLRS_MAX_SIZE_PAYLOAD)
    {
        ++stats_.malformed;
        return;
    }
    if (pushUart(id, payload, length))
    {
        ++stats_.airFramesIn;
        ++stats_.uartFramesOut;
    }
}

bool RLRS_IRAM RlrsLink::airPending() const
{
    Guard guard(*this);
    if (pendingAcks != 0)
        return true;
    for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
    {
        if (frames[i].inUse)
            return true;
    }
    return false;
}

uint8_t RlrsLink::queueDepth() const
{
    Guard guard(*this);
    uint8_t count = 0;
    for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
    {
        if (frames[i].inUse)
            ++count;
    }
    return count;
}
