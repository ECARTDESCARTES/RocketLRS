#include "RlrsLink.h"

#include <cstring>

RlrsLink::RlrsLink() : nextOrder(0)
{
    reset();
}

void RlrsLink::reset()
{
    for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
    {
        std::memset(&frames[i], 0, sizeof(frames[i]));
        frames[i].inUse = false;
    }

    nextOrder = 0;
    rxId = 0;
    rxTotal = 0;
    rxHave = 0;
    rxFragment = 0;
    rxActive = false;
    pendingAcks = 0;

    uartHead = 0;
    uartTail = 0;
    uartCount = 0;

    stats_ = Stats();

    inputParser.reset();
    inputParser.setHandler(&RlrsLink::onUartFrame, this);
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

uint16_t RlrsLink::takeUart(uint8_t *destination, uint16_t maxLength)
{
    if (destination == nullptr || maxLength == 0 || uartCount == 0)
        return 0;

    uint16_t copied = 0;
    while (copied < maxLength && uartCount > 0)
    {
        destination[copied++] = uartOut[uartTail];
        uartTail = static_cast<uint16_t>((uartTail + 1) % RLRS_LINK_UART_TX_SIZE);
        --uartCount;
    }
    return copied;
}

bool RlrsLink::pushUart(const uint8_t *data, uint16_t length)
{
    if (length > RLRS_LINK_UART_TX_SIZE ||
        uartCount + length > RLRS_LINK_UART_TX_SIZE)
    {
        ++stats_.uartOverflow;
        return false;
    }

    for (uint16_t i = 0; i < length; ++i)
    {
        uartOut[uartHead] = data[i];
        uartHead = static_cast<uint16_t>((uartHead + 1) % RLRS_LINK_UART_TX_SIZE);
    }
    uartCount = static_cast<uint16_t>(uartCount + length);
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
    for (uint8_t attempt = 0; attempt < 15; ++attempt)
    {
        uint8_t seq = static_cast<uint8_t>((attempt + 1) & 0x0F);
        if (seq == 0)
            continue;

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
            return seq;
    }
    return 0;
}

void RlrsLink::freeFrame(Frame *frame)
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
                (frame.priority == victim->priority && frame.order < victim->order))
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

RlrsLink::Frame *RlrsLink::bestFrame()
{
    Frame *best = nullptr;
    for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
    {
        Frame &frame = frames[i];
        if (!frame.inUse || frame.offset != 0 || frame.attempts != 0)
            continue;
        if (best == nullptr || frame.priority < best->priority ||
            (frame.priority == best->priority && frame.order < best->order))
            best = &frame;
    }
    return best;
}

uint8_t RlrsLink::packAir(uint8_t *destination, uint8_t capacity)
{
    if (destination == nullptr || capacity < 1)
        return 0;

    // Acknowledgements first: they are one byte and time sensitive.
    if (pendingAcks != 0)
    {
        for (uint8_t seq = 1; seq <= 15; ++seq)
        {
            const uint16_t mask = static_cast<uint16_t>(1u << seq);
            if ((pendingAcks & mask) != 0)
            {
                pendingAcks = static_cast<uint16_t>(pendingAcks & ~mask);
                destination[0] = static_cast<uint8_t>(0xA0 | seq);
                ++stats_.airFramesOut;
                return 1;
            }
        }
    }

    Frame *frame = nullptr;

    // One byte is only enough for an acknowledgement.
    if (capacity < 2)
        return 0;

    // 1) A frame that is already split must finish; never interleave two
    // partial frames on the single reassembly buffer of the receiver.
    for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
    {
        if (frames[i].inUse && frames[i].offset > 0)
        {
            frame = &frames[i];
            break;
        }
    }

    // 2) Reliable frames waiting for an acknowledgement.
    if (frame == nullptr)
    {
        for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
        {
            Frame &candidate = frames[i];
            if (!candidate.inUse || candidate.seq == 0 ||
                candidate.offset != 0 || candidate.attempts == 0)
                continue;

            if (candidate.retryIn > 0)
            {
                --candidate.retryIn;
                continue;
            }
            if (candidate.attempts >= RLRS_LINK_MAX_ATTEMPTS)
            {
                freeFrame(&candidate);
                continue;
            }
            ++candidate.attempts;
            candidate.fragment = 1;
            frame = &candidate;
            break;
        }
    }

    // 3) A new frame.
    if (frame == nullptr)
        frame = bestFrame();
    if (frame == nullptr)
        return 0;

    uint8_t written = 0;

    if (frame->offset == 0 && frame->seq != 0 &&
        static_cast<uint8_t>(3 + frame->length) <= capacity)
    {
        destination[0] = static_cast<uint8_t>(0x90 | (frame->seq & 0x0F));
        destination[1] = frame->length;
        destination[2] = frame->id;
        if (frame->length != 0)
            std::memcpy(&destination[3], frame->data, frame->length);
        frame->offset = frame->length;
        written = static_cast<uint8_t>(3 + frame->length);
    }
    else if (frame->offset == 0 &&
             static_cast<uint8_t>(2 + frame->length) <= capacity)
    {
        destination[0] = frame->length;
        destination[1] = frame->id;
        if (frame->length != 0)
            std::memcpy(&destination[2], frame->data, frame->length);
        frame->offset = frame->length;
        written = static_cast<uint8_t>(2 + frame->length);
    }
    else if (frame->offset == 0 && capacity >= 3)
    {
        // Too large for one packet: split. Reliable frames are small control
        // messages; if one is too large it loses its ACK but is still sent.
        frame->seq = 0;
        // 0 encodes a 64-byte payload: an empty frame is never fragmented.
        destination[0] = static_cast<uint8_t>(0x40 | (frame->length & 0x3F));
        destination[1] = frame->id;
        uint8_t chunk = static_cast<uint8_t>(capacity - 2);
        if (chunk > frame->length)
            chunk = frame->length;
        if (chunk != 0)
            std::memcpy(&destination[2], frame->data, chunk);
        frame->offset = chunk;
        frame->fragment = 1;
        written = static_cast<uint8_t>(2 + chunk);
    }
    else if (frame->offset != 0)
    {
        const uint8_t remaining = static_cast<uint8_t>(frame->length - frame->offset);
        uint8_t chunk = static_cast<uint8_t>(capacity - 1);
        if (chunk > remaining)
            chunk = remaining;
        destination[0] = static_cast<uint8_t>(0x80 | (frame->fragment & 0x0F));
        if (chunk != 0)
            std::memcpy(&destination[1], &frame->data[frame->offset], chunk);
        frame->offset = static_cast<uint8_t>(frame->offset + chunk);
        frame->fragment = static_cast<uint8_t>((frame->fragment + 1) & 0x0F);
        written = static_cast<uint8_t>(1 + chunk);
    }
    else
    {
        // The slot is too small for a header plus data (one byte only). Leave
        // the frame queued for a larger slot.
        return 0;
    }

    if (frame->offset >= frame->length)
    {
        frame->offset = 0;
        frame->fragment = 0;
        if (frame->seq != 0)
        {
            if (frame->attempts == 0)
                frame->attempts = 1;
            frame->retryIn = RLRS_LINK_RETRY_GAP;
        }
        else
        {
            freeFrame(frame);
        }
    }

    ++stats_.airFramesOut;
    return written;
}

void RlrsLink::unpackAir(const uint8_t *source, uint8_t capacity)
{
    if (source == nullptr || capacity == 0)
        return;

    const uint8_t opcode = source[0];

    if (opcode <= 0x3F)
    {
        const uint8_t length = opcode;
        if (static_cast<uint8_t>(2 + length) > capacity)
        {
            ++stats_.malformed;
            return;
        }
        deliver(source[1], &source[2], length);
    }
    else if (opcode >= 0x40 && opcode <= 0x7F)
    {
        if (capacity < 3)
        {
            ++stats_.malformed;
            return;
        }

        rxTotal = static_cast<uint8_t>(opcode & 0x3F);
        if (rxTotal == 0)
            rxTotal = RLRS_MAX_SIZE_PAYLOAD; // 64 is encoded as zero
        rxId = source[1];
        rxHave = 0;
        rxFragment = 1;
        rxActive = true;

        uint8_t chunk = static_cast<uint8_t>(capacity - 2);
        if (chunk > rxTotal)
            chunk = rxTotal;
        std::memcpy(rxBuffer, &source[2], chunk);
        rxHave = chunk;

        if (rxHave >= rxTotal)
            finishReassembly();
    }
    else if (opcode >= 0x80 && opcode <= 0x8F)
    {
        if (capacity < 2)
        {
            ++stats_.malformed;
            return;
        }
        const uint8_t fragment = static_cast<uint8_t>(opcode & 0x0F);
        if (!rxActive || fragment != rxFragment)
        {
            rxActive = false;
            ++stats_.airLost;
            return;
        }

        const uint8_t remaining = static_cast<uint8_t>(rxTotal - rxHave);
        uint8_t chunk = static_cast<uint8_t>(capacity - 1);
        if (chunk > remaining)
            chunk = remaining;
        std::memcpy(&rxBuffer[rxHave], &source[1], chunk);
        rxHave = static_cast<uint8_t>(rxHave + chunk);
        rxFragment = static_cast<uint8_t>((rxFragment + 1) & 0x0F);

        if (rxHave >= rxTotal)
            finishReassembly();
    }
    else if (opcode >= 0x90 && opcode <= 0x9F)
    {
        if (capacity < 3)
        {
            ++stats_.malformed;
            return;
        }
        const uint8_t seq = static_cast<uint8_t>(opcode & 0x0F);
        const uint8_t length = source[1];
        if (static_cast<uint8_t>(3 + length) > capacity)
        {
            ++stats_.malformed;
            return;
        }
        deliver(source[2], &source[3], length);
        enqueueAck(seq);
    }
    else if (opcode >= 0xA0 && opcode <= 0xAF)
    {
        clearReliable(static_cast<uint8_t>(opcode & 0x0F));
    }
    // 0xFF is the idle marker, anything else is reserved.
}

void RlrsLink::finishReassembly()
{
    deliver(rxId, rxBuffer, rxTotal);
    rxActive = false;
    rxId = 0;
    rxTotal = 0;
    rxHave = 0;
    rxFragment = 0;
}

void RlrsLink::enqueueAck(uint8_t seq)
{
    if (seq >= 1 && seq <= 15)
        pendingAcks = static_cast<uint16_t>(pendingAcks | (1u << seq));
}

void RlrsLink::clearReliable(uint8_t seq)
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

void RlrsLink::deliver(uint8_t id, const uint8_t *payload, uint8_t length)
{
    uint8_t wire[RLRS_MAX_SIZE];
    const size_t wireLength = rlrsEncode(id, payload, length, wire, sizeof(wire));
    if (wireLength == 0)
    {
        ++stats_.malformed;
        return;
    }
    if (pushUart(wire, static_cast<uint16_t>(wireLength)))
    {
        ++stats_.airFramesIn;
        ++stats_.uartFramesOut;
    }
}

bool RlrsLink::airPending() const
{
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
    uint8_t count = 0;
    for (uint8_t i = 0; i < RLRS_LINK_FRAME_COUNT; ++i)
    {
        if (frames[i].inUse)
            ++count;
    }
    return count;
}
