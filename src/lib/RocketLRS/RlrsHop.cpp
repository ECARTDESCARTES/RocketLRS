#include "RlrsHop.h"

namespace {

RLRS_ISR_ATTR inline bool reached(uint32_t now, uint32_t deadline)
{
    return static_cast<int32_t>(now - deadline) >= 0;
}

RLRS_ISR_ATTR inline uint32_t remaining(uint32_t now, uint32_t deadline)
{
    return reached(now, deadline) ? 0 : deadline - now;
}

uint32_t xorshift32(uint32_t &state)
{
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

RlrsLoraModem loraModem(uint8_t sf)
{
    RlrsLoraModem modem;
    modem.sf = sf;
    modem.bandwidthHz = 812500;
    modem.cr = 1;
    modem.preambleSymbols = 8;
    modem.explicitHeader = true;
    modem.crcOn = true;
    return modem;
}

} // namespace

// ---------------------------------------------------------------------------
// Time on air

RLRS_ISR_ATTR uint32_t rlrsLoraAirtimeUs(const RlrsLoraModem &modem, uint8_t payloadLength)
{
    const int32_t sf = modem.sf < 5 ? 5 : (modem.sf > 12 ? 12 : modem.sf);
    const uint32_t cr = modem.cr < 1 ? 1 : (modem.cr > 4 ? 4 : modem.cr);
    if (modem.bandwidthHz == 0)
        return 0;

    int32_t bits = 8 * static_cast<int32_t>(payloadLength) + (modem.crcOn ? 16 : 0) - 4 * sf +
                   (sf >= 7 ? 8 : 0) + (modem.explicitHeader ? 20 : 0);
    if (bits < 0)
        bits = 0;
    const int32_t perBlock = sf >= 11 ? 4 * (sf - 2) : 4 * sf;
    const uint32_t blocks = static_cast<uint32_t>((bits + perBlock - 1) / perBlock);
    // Everything counted in quarter symbols (4.25 and 6.25 symbol terms).
    const uint64_t quarters = 4ull * modem.preambleSymbols + (sf <= 6 ? 25u : 17u) + 32u +
                              4ull * blocks * (cr + 4u);
    const uint64_t numerator = quarters * (1ull << sf) * 1000000ull;
    const uint64_t denominator = 4ull * modem.bandwidthHz;
    return static_cast<uint32_t>((numerator + denominator - 1) / denominator);
}

RLRS_ISR_ATTR int16_t rlrsLoraMaxPayload(const RlrsLoraModem &modem, uint32_t budgetUs, uint8_t limit)
{
    if (rlrsLoraAirtimeUs(modem, 0) > budgetUs)
        return -1;
    // Time on air is non-decreasing with the length: binary search.
    uint16_t low = 0;        // fits
    uint16_t high = limit;   // candidate
    if (rlrsLoraAirtimeUs(modem, limit) <= budgetUs)
        return limit;
    while (high - low > 1)
    {
        const uint16_t mid = static_cast<uint16_t>((low + high) / 2);
        if (rlrsLoraAirtimeUs(modem, static_cast<uint8_t>(mid)) <= budgetUs)
            low = mid;
        else
            high = mid;
    }
    return static_cast<int16_t>(low);
}

// ---------------------------------------------------------------------------
// Configuration

RlrsHopConfig rlrsHopDefaultConfig(uint32_t seed)
{
    RlrsHopConfig config = RlrsHopConfig();
    config.framePeriodUs = 50000;
    config.firstFrequencyHz = 2400400000u;
    config.lastFrequencyHz = 2479600000u;
    config.channelCount = 40;
    config.seed = seed;
    config.guardUs = 500;
    config.endGuardUs = 1000;
    config.windowUs = 1000;
    config.rxLatencyUs = 0;
    config.missLimit = 20;
    config.linkLossUs = 1000000;
    config.presetLeadFrames = 10;
    config.presetCount = 2;
    config.defaultPreset = 0;
    config.presets[0].modem = loraModem(8);
    config.presets[0].uplinkMaxBytes = 33;
    config.presets[0].downlinkMaxBytes = 72;
    config.presets[1].modem = loraModem(7);
    config.presets[1].uplinkMaxBytes = 33;
    config.presets[1].downlinkMaxBytes = 127;
    return config;
}

bool rlrsHopConfigValid(const RlrsHopConfig &config)
{
    if (config.framePeriodUs == 0 || config.framePeriodUs > 0x40000000u ||
        config.channelCount == 0 || config.channelCount > RLRS_HOP_MAX_CHANNELS ||
        config.firstFrequencyHz > config.lastFrequencyHz ||
        config.presetCount == 0 || config.presetCount > RLRS_HOP_MAX_PRESETS ||
        config.defaultPreset >= config.presetCount || config.missLimit == 0 ||
        config.presetLeadFrames < 2 || config.presetLeadFrames > 100 ||
        config.windowUs >= config.framePeriodUs / 4)
        return false;
    const uint32_t tail = config.endGuardUs > config.windowUs ? config.endGuardUs : config.windowUs;
    for (uint8_t i = 0; i < config.presetCount; ++i)
    {
        const RlrsHopPreset &preset = config.presets[i];
        const RlrsLoraModem &modem = preset.modem;
        if (modem.sf < 5 || modem.sf > 12 || modem.cr < 1 || modem.cr > 4 ||
            modem.bandwidthHz == 0 || preset.uplinkMaxBytes < RLRS_AIR_HEADER_MAX ||
            preset.downlinkMaxBytes < RLRS_AIR_HEADER_MAX)
            return false;
        const uint64_t fullUp = static_cast<uint64_t>(rlrsLoraAirtimeUs(modem, preset.uplinkMaxBytes)) +
                                config.guardUs + rlrsLoraAirtimeUs(modem, RLRS_AIR_HEADER_MAX) + tail;
        const uint64_t fullDown = static_cast<uint64_t>(rlrsLoraAirtimeUs(modem, 3)) + config.guardUs +
                                  rlrsLoraAirtimeUs(modem, preset.downlinkMaxBytes) + tail;
        if (fullUp > config.framePeriodUs || fullDown > config.framePeriodUs)
            return false;
    }
    return true;
}

uint32_t rlrsHopSeedFromUid(const uint8_t *uid, size_t length)
{
    uint32_t hash = 2166136261u;
    for (size_t i = 0; uid != nullptr && i < length; ++i)
    {
        hash ^= uid[i];
        hash *= 16777619u;
    }
    return hash ? hash : 0x9E3779B9u;
}

// ---------------------------------------------------------------------------
// RlrsHop

RlrsHop::RlrsHop(Role role, const RlrsHopConfig &config, RlrsAirQueue *txQueue)
    : config_(config), queue_(txQueue), stats_(), lastHeader_(), frameStart_(0), timerAt_(0),
      lastRxUs_(0), role_(role), state_(StateStopped), seq_(0), hop_(0), parkHop_(0), missed_(0),
      preset_(0), pendingPreset_(0), pendingSeq_(0), pendingValid_(false), pendingAcked_(false),
      presetChanged_(false), linkAlive_(false)
{
    if (config_.channelCount == 0)
        config_.channelCount = 1;
    if (config_.channelCount > RLRS_HOP_MAX_CHANNELS)
        config_.channelCount = RLRS_HOP_MAX_CHANNELS;
    if (config_.presetCount == 0)
        config_.presetCount = 1;
    if (config_.presetCount > RLRS_HOP_MAX_PRESETS)
        config_.presetCount = RLRS_HOP_MAX_PRESETS;
    if (config_.defaultPreset >= config_.presetCount)
        config_.defaultPreset = 0;
    if (config_.missLimit == 0)
        config_.missLimit = 1;
    if (config_.presetLeadFrames < 2)
        config_.presetLeadFrames = 2;
    preset_ = config_.defaultPreset;

    // Fisher-Yates permutation driven by a deterministic xorshift32.
    const uint8_t count = config_.channelCount;
    for (uint8_t i = 0; i < count; ++i)
        perm_[i] = i;
    uint32_t state = config_.seed ? config_.seed : 0x9E3779B9u;
    for (uint8_t i = static_cast<uint8_t>(count - 1); i > 0; --i)
    {
        const uint8_t j = static_cast<uint8_t>(xorshift32(state) % (i + 1u));
        const uint8_t tmp = perm_[i];
        perm_[i] = perm_[j];
        perm_[j] = tmp;
    }
}

uint8_t RlrsHop::channelForHop(uint8_t hop) const
{
    return perm_[hop % config_.channelCount];
}

uint32_t RlrsHop::channelFrequencyHz(uint8_t channel) const
{
    if (config_.channelCount < 2)
        return config_.firstFrequencyHz;
    const uint64_t span = config_.lastFrequencyHz - config_.firstFrequencyHz;
    return config_.firstFrequencyHz +
           static_cast<uint32_t>(span * (channel % config_.channelCount) / (config_.channelCount - 1u));
}

uint32_t RlrsHop::frequencyForHop(uint8_t hop) const
{
    return channelFrequencyHz(channelForHop(hop));
}

bool RlrsHop::synced() const
{
    if (role_ == Master)
        return linkAlive_;
    return linkAlive_ && state_ != StateAcquire && state_ != StateStopped;
}

bool RlrsHop::requestPreset(uint8_t preset)
{
    if (role_ != Master || state_ == StateStopped || !linkAlive_ || pendingValid_ ||
        preset >= config_.presetCount || preset == preset_)
        return false;
    pendingPreset_ = preset;
    pendingSeq_ = static_cast<uint8_t>(seq_ + config_.presetLeadFrames);
    pendingValid_ = true;
    pendingAcked_ = false;
    return true;
}

RLRS_ISR_ATTR RlrsHopAction RlrsHop::makeAction(RlrsHopAction::Type type)
{
    RlrsHopAction action;
    action.type = type;
    action.frequencyHz = frequencyForHop(hop_);
    action.txData = nullptr;
    action.txLength = 0;
    action.rxTimeoutUs = 0;
    action.presetChanged = presetChanged_;
    action.preset = preset_;
    action.rxDelivered = false;
    action.timerAtUs = timerAt_;
    presetChanged_ = false;
    return action;
}

RLRS_ISR_ATTR void RlrsHop::setPreset(uint8_t preset)
{
    if (preset != preset_)
    {
        preset_ = preset;
        presetChanged_ = true;
    }
}

RLRS_ISR_ATTR void RlrsHop::advanceFrame()
{
    ++seq_;
    hop_ = static_cast<uint8_t>((hop_ + 1u) % config_.channelCount);
    if (pendingValid_ && seq_ == pendingSeq_)
    {
        pendingValid_ = false;
        if (role_ == Slave || pendingAcked_)
        {
            setPreset(pendingPreset_);
            ++stats_.presetSwitches;
        }
        else
        {
            ++stats_.presetFailed;
        }
    }
}

RlrsHopAction RlrsHop::start(uint32_t now)
{
    state_ = StateIdle;
    seq_ = 0;
    hop_ = 0;
    parkHop_ = 0;
    missed_ = 0;
    pendingValid_ = false;
    pendingAcked_ = false;
    linkAlive_ = false;
    lastRxUs_ = now;
    preset_ = config_.defaultPreset;
    presetChanged_ = true;   // make the caller load the default preset
    if (role_ == Master)
    {
        frameStart_ = now;
        return masterFrame(now);
    }
    return slaveAcquire(now);
}

RLRS_ISR_ATTR RlrsHopAction RlrsHop::onTimer(uint32_t now)
{
    if (state_ == StateStopped || !reached(now, timerAt_))
        return makeAction(RlrsHopAction::None);

    if (role_ == Master)
    {
        frameStart_ = timerAt_;
        advanceFrame();
        // Caller late by whole frames: keep the schedule, skip those frames.
        while (static_cast<uint32_t>(now - frameStart_) >= config_.framePeriodUs)
        {
            frameStart_ += config_.framePeriodUs;
            advanceFrame();
        }
        return masterFrame(now);
    }

    switch (state_)
    {
    case StateAcquire:
        parkHop_ = static_cast<uint8_t>((parkHop_ + 1u) % config_.channelCount);
        return slaveAcquire(now);
    case StateTxGuard:
        return slaveTransmit(now);
    case StateRxBeacon:
        return slaveMiss(now);
    default:  // Idle, or Tx whose TxDone never came: open the next window
        return slaveOpenWindow(now);
    }
}

RLRS_ISR_ATTR RlrsHopAction RlrsHop::onTxDone(uint32_t now)
{
    if (state_ != StateTx)
    {
        ++stats_.ignored;
        return makeAction(RlrsHopAction::None);
    }
    if (role_ == Master)
    {
        // The slave dates the frame from its RxDone, late by the master TX
        // start (loop + SPI) and its own IRQ latency, so its downlink may end
        // up to that much after frameStart_ + period - endGuard. Listen
        // until endGuard / 2 before the next frame: the other half is the
        // margin for those latencies and the RX->TX turnaround of the master.
        const uint32_t deadline = frameStart_ + config_.framePeriodUs - config_.endGuardUs / 2u;
        const uint32_t timeout = remaining(now, deadline);
        if (timeout == 0)
        {
            state_ = StateIdle;
            return makeAction(RlrsHopAction::Idle);
        }
        state_ = StateRxDownlink;
        RlrsHopAction action = makeAction(RlrsHopAction::Receive);
        action.rxTimeoutUs = timeout;
        return action;
    }
    state_ = StateIdle;
    return makeAction(RlrsHopAction::Idle);
}

RLRS_ISR_ATTR RlrsHopAction RlrsHop::onRxDone(uint32_t now, const uint8_t *data, uint8_t length)
{
    if (data == nullptr)
        length = 0;
    if (role_ == Master)
    {
        if (state_ != StateRxDownlink)
        {
            ++stats_.ignored;
            return makeAction(RlrsHopAction::None);
        }
        return masterRx(now, data, length);
    }

    RlrsAirReader reader;
    const bool parsed = length > 0 && reader.begin(data, length);
    const RlrsAirHeader &header = reader.header();
    const bool beacon = parsed && (header.flags & RLRS_AIR_FLAG_HOP) && header.hop < config_.channelCount;

    switch (state_)
    {
    case StateAcquire:
        if (beacon && header.hop == parkHop_)
        {
            ++stats_.acquisitions;
            return slaveSync(now, header, length);
        }
        if (length)
            ++stats_.rejected;
        {
            // Single RX ended (error or foreign frame): listen again.
            RlrsHopAction action = makeAction(RlrsHopAction::Receive);
            action.frequencyHz = frequencyForHop(parkHop_);
            return action;
        }
    case StateRxBeacon:
        if (beacon && header.hop == hop_)
            return slaveSync(now, header, length);
        if (length)
            ++stats_.rejected;
        return slaveMiss(now);
    default:
        ++stats_.ignored;
        return makeAction(RlrsHopAction::None);
    }
}

// ---- master ---------------------------------------------------------------

RLRS_ISR_ATTR RlrsHopAction RlrsHop::masterFrame(uint32_t now)
{
    if (linkAlive_ && reached(now, lastRxUs_ + config_.linkLossUs))
    {
        linkAlive_ = false;
        ++stats_.lost;
        if (pendingValid_)
        {
            pendingValid_ = false;
            ++stats_.presetFailed;
        }
    }
    if (!linkAlive_ && preset_ != config_.defaultPreset && reached(now, lastRxUs_ + config_.linkLossUs))
    {
        setPreset(config_.defaultPreset);
        ++stats_.presetReverts;
    }

    RlrsAirHeader header = RlrsAirHeader();
    header.seq = seq_;
    header.flags = RLRS_AIR_FLAG_HOP;
    header.hop = hop_;
    if (pendingValid_)
    {
        header.flags |= RLRS_AIR_FLAG_PRESET;
        header.preset = pendingPreset_;
        header.switchSeq = pendingSeq_;
    }
    const uint8_t capacity = config_.presets[preset_].uplinkMaxBytes;
    const uint8_t length = queue_ != nullptr ? queue_->build(header, txBuffer_, capacity)
                                             : rlrsAirWriteHeader(header, txBuffer_, capacity);
    timerAt_ = frameStart_ + config_.framePeriodUs;
    if (length == 0)
    {
        state_ = StateIdle;
        return makeAction(RlrsHopAction::Idle);
    }
    state_ = StateTx;
    ++stats_.framesTx;
    RlrsHopAction action = makeAction(RlrsHopAction::Transmit);
    action.txData = txBuffer_;
    action.txLength = length;
    return action;
}

RLRS_ISR_ATTR RlrsHopAction RlrsHop::masterRx(uint32_t now, const uint8_t *data, uint8_t length)
{
    state_ = StateIdle;
    RlrsAirReader reader;
    if (length == 0 || !reader.begin(data, length))
    {
        if (length)
            ++stats_.rejected;
        ++stats_.missed;
        return makeAction(RlrsHopAction::Idle);
    }
    const RlrsAirHeader &header = reader.header();
    if ((header.flags & RLRS_AIR_FLAG_HOP) || header.seq != seq_)
    {
        ++stats_.rejected;
        ++stats_.missed;
        return makeAction(RlrsHopAction::Idle);
    }
    lastHeader_ = header;
    lastRxUs_ = now;
    linkAlive_ = true;
    ++stats_.framesRx;
    if (pendingValid_ && (header.flags & RLRS_AIR_FLAG_ACK) && (header.flags & RLRS_AIR_FLAG_PRESET) &&
        header.preset == pendingPreset_ && header.switchSeq == pendingSeq_)
        pendingAcked_ = true;
    RlrsHopAction action = makeAction(RlrsHopAction::Idle);
    action.rxDelivered = true;
    return action;
}

// ---- slave ----------------------------------------------------------------

RLRS_ISR_ATTR RlrsHopAction RlrsHop::slaveAcquire(uint32_t now)
{
    if (state_ != StateAcquire && preset_ != config_.defaultPreset)
    {
        setPreset(config_.defaultPreset);
        ++stats_.presetReverts;
    }
    state_ = StateAcquire;
    pendingValid_ = false;
    missed_ = 0;
    linkAlive_ = false;
    timerAt_ = now + (config_.channelCount + 1u) * config_.framePeriodUs;
    RlrsHopAction action = makeAction(RlrsHopAction::Receive);
    action.frequencyHz = frequencyForHop(parkHop_);
    return action;
}

RLRS_ISR_ATTR RlrsHopAction RlrsHop::slaveSync(uint32_t now, const RlrsAirHeader &header, uint8_t length)
{
    frameStart_ = now - config_.rxLatencyUs - rlrsLoraAirtimeUs(modem(), length);
    seq_ = header.seq;
    hop_ = header.hop;
    parkHop_ = header.hop;
    missed_ = 0;
    lastRxUs_ = now;
    linkAlive_ = true;
    lastHeader_ = header;
    ++stats_.framesRx;

    if (header.flags & RLRS_AIR_FLAG_PRESET)
    {
        const uint8_t ahead = static_cast<uint8_t>(header.switchSeq - header.seq);
        if (header.preset < config_.presetCount && ahead != 0 && ahead <= 127)
        {
            pendingValid_ = true;
            pendingPreset_ = header.preset;
            pendingSeq_ = header.switchSeq;
        }
    }
    else
    {
        pendingValid_ = false;  // the master no longer announces a switch
    }

    state_ = StateTxGuard;
    timerAt_ = now + config_.guardUs;
    RlrsHopAction action = makeAction(RlrsHopAction::Idle);
    action.rxDelivered = true;
    return action;
}

RLRS_ISR_ATTR RlrsHopAction RlrsHop::slaveTransmit(uint32_t now)
{
    const uint32_t tail = config_.endGuardUs > config_.windowUs ? config_.endGuardUs : config_.windowUs;
    const uint32_t deadline = frameStart_ + config_.framePeriodUs - tail;
    timerAt_ = frameStart_ + config_.framePeriodUs - config_.windowUs;

    RlrsAirHeader header = RlrsAirHeader();
    header.seq = seq_;
    if (pendingValid_)
    {
        header.flags = RLRS_AIR_FLAG_PRESET | RLRS_AIR_FLAG_ACK;
        header.preset = pendingPreset_;
        header.switchSeq = pendingSeq_;
    }
    const int16_t fit = rlrsLoraMaxPayload(modem(), remaining(now, deadline),
                                           config_.presets[preset_].downlinkMaxBytes);
    if (fit < rlrsAirHeaderSize(header.flags))
    {
        state_ = StateIdle;
        return makeAction(RlrsHopAction::Idle);
    }
    const uint8_t capacity = static_cast<uint8_t>(fit);
    const uint8_t length = queue_ != nullptr ? queue_->build(header, txBuffer_, capacity)
                                             : rlrsAirWriteHeader(header, txBuffer_, capacity);
    state_ = StateTx;
    ++stats_.framesTx;
    RlrsHopAction action = makeAction(RlrsHopAction::Transmit);
    action.txData = txBuffer_;
    action.txLength = length;
    return action;
}

RLRS_ISR_ATTR RlrsHopAction RlrsHop::slaveOpenWindow(uint32_t now)
{
    frameStart_ += config_.framePeriodUs;
    advanceFrame();
    const uint32_t deadline = frameStart_ +
                              rlrsLoraAirtimeUs(modem(), config_.presets[preset_].uplinkMaxBytes) +
                              config_.windowUs;
    timerAt_ = deadline;
    const uint32_t timeout = remaining(now, deadline);
    if (timeout == 0)
        return slaveMiss(now);
    state_ = StateRxBeacon;
    RlrsHopAction action = makeAction(RlrsHopAction::Receive);
    action.rxTimeoutUs = timeout;
    return action;
}

RLRS_ISR_ATTR RlrsHopAction RlrsHop::slaveMiss(uint32_t now)
{
    ++stats_.missed;
    if (++missed_ >= config_.missLimit)
    {
        ++stats_.lost;
        return slaveAcquire(now);
    }
    state_ = StateIdle;
    timerAt_ = frameStart_ + config_.framePeriodUs - config_.windowUs;
    return makeAction(RlrsHopAction::Idle);
}
