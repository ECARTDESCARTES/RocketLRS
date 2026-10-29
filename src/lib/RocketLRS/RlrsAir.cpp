#include "RlrsAir.h"

#include <cstring>

RLRS_ISR_ATTR uint8_t rlrsAirHeaderSize(uint8_t flags)
{
    uint8_t size = RLRS_AIR_HEADER_MIN;
    if (flags & RLRS_AIR_FLAG_HOP)
        size += 1;
    if (flags & RLRS_AIR_FLAG_PRESET)
        size += 2;
    return size;
}

RLRS_ISR_ATTR uint8_t rlrsAirWriteHeader(const RlrsAirHeader &header, uint8_t *out, uint8_t capacity)
{
    if (out == nullptr || (header.flags & ~RLRS_AIR_FLAG_KNOWN))
        return 0;
    const uint8_t size = rlrsAirHeaderSize(header.flags);
    if (capacity < size)
        return 0;
    uint8_t pos = 0;
    out[pos++] = header.seq;
    out[pos++] = header.flags;
    if (header.flags & RLRS_AIR_FLAG_HOP)
        out[pos++] = header.hop;
    if (header.flags & RLRS_AIR_FLAG_PRESET)
    {
        out[pos++] = header.preset;
        out[pos++] = header.switchSeq;
    }
    return pos;
}

RLRS_ISR_ATTR uint8_t rlrsAirPriorityForId(uint8_t id)
{
    switch (id)
    {
    case 0x03: // CMD
    case 0x04: // ACK
        return RlrsAirQueue::PriorityCritical;
    case 0x00: // HEARTBEAT
    case 0x05: // LED
    case 0x06: // SERVO
    case 0xF0: // link stats (RlrsLink::LinkStatsId)
        return RlrsAirQueue::PriorityControl;
    case 0x01: // INFO / gyro
    case 0x02: // GPS
    case 0x07: // pressure
    case 0x08: // battery
        return RlrsAirQueue::PriorityTelemetry;
    default:
        return RlrsAirQueue::PriorityBulk;
    }
}

// ---------------------------------------------------------------------------
// Queue

RlrsAirQueue::RlrsAirQueue(uint8_t maxFrameSize)
    : stats_(), nextOrder_(0), count_(0), maxFrameSize_(maxFrameSize)
{
    for (unsigned i = 0; i < RLRS_AIR_QUEUE_COUNT; ++i)
        slots_[i].used = false;
}

void RlrsAirQueue::reset()
{
    for (unsigned i = 0; i < RLRS_AIR_QUEUE_COUNT; ++i)
        slots_[i].used = false;
    count_ = 0;
}

void RlrsAirQueue::setMaxFrameSize(uint8_t maxFrameSize)
{
    maxFrameSize_ = maxFrameSize;
}

uint8_t RlrsAirQueue::maxPayload() const
{
    const unsigned room = maxFrameSize_ > RLRS_AIR_HEADER_MAX + RLRS_AIR_MSG_OVERHEAD
                              ? maxFrameSize_ - RLRS_AIR_HEADER_MAX - RLRS_AIR_MSG_OVERHEAD
                              : 0;
    return static_cast<uint8_t>(room > RLRS_AIR_PAYLOAD_MAX ? RLRS_AIR_PAYLOAD_MAX : room);
}

RLRS_ISR_ATTR bool RlrsAirQueue::push(uint8_t id, const uint8_t *payload, uint8_t length)
{
    return push(id, payload, length, rlrsAirPriorityForId(id));
}

RLRS_ISR_ATTR bool RlrsAirQueue::push(uint8_t id, const uint8_t *payload, uint8_t length,
                                      uint8_t priority)
{
    if (length > RLRS_AIR_PAYLOAD_MAX || (length && payload == nullptr) ||
        maxFrameSize_ < RLRS_AIR_HEADER_MAX + RLRS_AIR_MSG_OVERHEAD ||
        length > maxPayload())
    {
        ++stats_.rejectedTooBig;
        return false;
    }
    if (priority > PriorityBulk)
        priority = PriorityBulk;

    int free = -1;
    for (unsigned i = 0; i < RLRS_AIR_QUEUE_COUNT; ++i)
    {
        if (!slots_[i].used)
        {
            free = static_cast<int>(i);
            break;
        }
    }
    if (free < 0)
    {
        // Full: replace the oldest message of the least important level,
        // provided it is strictly less important than the new one.
        for (int level = PriorityBulk; level > priority; --level)
        {
            free = oldest(static_cast<uint8_t>(level));
            if (free >= 0)
                break;
        }
        if (free < 0)
        {
            ++stats_.droppedFull;
            return false;
        }
        slots_[free].used = false;
        --count_;
        ++stats_.evicted;
    }

    Slot &slot = slots_[free];
    if (length)
        std::memcpy(slot.payload, payload, length);
    slot.length = length;
    slot.id = id;
    slot.priority = priority;
    slot.order = nextOrder_++;
    slot.skips = 0;
    slot.used = true;
    ++count_;
    ++stats_.accepted;
    return true;
}

RLRS_ISR_ATTR int RlrsAirQueue::oldest(uint8_t priority) const
{
    int best = -1;
    for (unsigned i = 0; i < RLRS_AIR_QUEUE_COUNT; ++i)
    {
        const Slot &slot = slots_[i];
        if (!slot.used || slot.priority != priority)
            continue;
        if (best < 0 || static_cast<int32_t>(slot.order - slots_[best].order) < 0)
            best = static_cast<int>(i);
    }
    return best;
}

RLRS_ISR_ATTR void RlrsAirQueue::packSlot(int index, uint8_t *out, uint8_t &pos)
{
    Slot &slot = slots_[index];
    out[pos++] = slot.length;
    out[pos++] = slot.id;
    if (slot.length)
        std::memcpy(out + pos, slot.payload, slot.length);
    pos = static_cast<uint8_t>(pos + slot.length);
    slot.used = false;
    --count_;
    ++stats_.packed;
}

RLRS_ISR_ATTR uint8_t RlrsAirQueue::build(const RlrsAirHeader &header, uint8_t *out,
                                          uint8_t capacity)
{
    uint8_t pos = rlrsAirWriteHeader(header, out, capacity);
    if (pos == 0)
        return 0;
    const uint8_t headerSize = pos;
    // Pass 0: commands. Pass 1: the oldest starving head message, if any.
    // Passes 2..4: control, telemetry, bulk.
    for (uint8_t pass = 0; pass <= PriorityBulk + 1; ++pass)
    {
        if (pass == 1)
        {
            int starving = -1;
            for (unsigned i = 0; i < RLRS_AIR_QUEUE_COUNT; ++i)
            {
                const Slot &slot = slots_[i];
                if (slot.used && slot.skips >= RLRS_AIR_STARVE_FRAMES &&
                    (starving < 0 || static_cast<int32_t>(slot.order - slots_[starving].order) < 0))
                    starving = static_cast<int>(i);
            }
            if (starving >= 0 && pos + RLRS_AIR_MSG_OVERHEAD + slots_[starving].length <= capacity)
                packSlot(starving, out, pos);
            continue;
        }
        const uint8_t level = static_cast<uint8_t>(pass == 0 ? 0 : pass - 1);
        for (;;)
        {
            const int index = oldest(level);
            if (index < 0)
                break;
            Slot &slot = slots_[index];
            const unsigned need = RLRS_AIR_MSG_OVERHEAD + slot.length;
            if (pos + need > capacity)
            {
                // Keep FIFO order inside this level; count the frame as
                // passed over only if an empty frame would have taken it.
                if (headerSize + need <= capacity && slot.skips < 0xFF)
                    ++slot.skips;
                break;
            }
            packSlot(index, out, pos);
        }
    }
    ++stats_.frames;
    return pos;
}

// ---------------------------------------------------------------------------
// Reader

RlrsAirReader::RlrsAirReader()
    : frame_(nullptr), length_(0), position_(0), header_(), malformed_(false)
{
}

RLRS_ISR_ATTR bool RlrsAirReader::begin(const uint8_t *frame, size_t length)
{
    frame_ = frame;
    length_ = length;
    position_ = 0;
    header_ = RlrsAirHeader();
    malformed_ = true;
    if (frame == nullptr || length < RLRS_AIR_HEADER_MIN || length > RLRS_AIR_FRAME_MAX)
        return false;
    header_.seq = frame[0];
    header_.flags = frame[1];
    if (header_.flags & ~RLRS_AIR_FLAG_KNOWN)
        return false;
    const uint8_t size = rlrsAirHeaderSize(header_.flags);
    if (length < size)
        return false;
    size_t pos = RLRS_AIR_HEADER_MIN;
    if (header_.flags & RLRS_AIR_FLAG_HOP)
        header_.hop = frame[pos++];
    if (header_.flags & RLRS_AIR_FLAG_PRESET)
    {
        header_.preset = frame[pos++];
        header_.switchSeq = frame[pos++];
    }
    position_ = pos;
    malformed_ = false;
    return true;
}

RLRS_ISR_ATTR bool RlrsAirReader::next(uint8_t &id, const uint8_t *&payload, uint8_t &length)
{
    if (frame_ == nullptr || malformed_ || position_ >= length_)
        return false;
    if (length_ - position_ < RLRS_AIR_MSG_OVERHEAD && frame_[position_] != RLRS_AIR_PAD)
    {
        malformed_ = true;
        return false;
    }
    const uint8_t len = frame_[position_];
    if (len == RLRS_AIR_PAD)
    {
        // Padding up to the end of the frame: not a message.
        for (size_t i = position_; i < length_; ++i)
        {
            if (frame_[i] != RLRS_AIR_PAD)
            {
                malformed_ = true;
                return false;
            }
        }
        position_ = length_;
        return false;
    }
    if (len > RLRS_AIR_PAYLOAD_MAX || length_ - position_ - RLRS_AIR_MSG_OVERHEAD < len)
    {
        malformed_ = true;
        return false;
    }
    id = frame_[position_ + 1];
    payload = frame_ + position_ + RLRS_AIR_MSG_OVERHEAD;
    length = len;
    position_ += RLRS_AIR_MSG_OVERHEAD + len;
    return true;
}

uint8_t rlrsAirPad(uint8_t *frame, uint8_t length, uint8_t minLength, uint8_t capacity)
{
    if (frame == nullptr || length < RLRS_AIR_HEADER_MIN || minLength > capacity || length > capacity)
        return 0;
    while (length < minLength)
        frame[length++] = RLRS_AIR_PAD;
    return length;
}
