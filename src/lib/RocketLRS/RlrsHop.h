#ifndef H_RLRS_HOP
#define H_RLRS_HOP

#include <cstddef>
#include <cstdint>

#include "RlrsAir.h"

// RocketLRS stand-alone air layer: TDMA + FHSS, master/slave.
//
// Ground (TX module) = master, rocket (RX) = slave. Every frame period
// (default 50 ms = 20 Hz):
//   1. at frame start F the master sends its uplink (beacon + commands) on
//      channel k: [seq][flags|HOP][hop index][preset announce?][messages]
//   2. the slave receives it, waits `guardUs` and answers on the same channel
//      with its aggregated downlink (capacity adapted so that it ends before
//      F + period - endGuardUs, F as dated by the slave); the master listens
//      until F + period - endGuardUs / 2: half of the end guard absorbs the
//      slave's dating error (master TX start latency + slave IRQ latency
//      not covered by rxLatencyUs)
//   3. both hop to channel k + 1.
// The channel table (N channels, default 40 from 2400.4 to 2479.6 MHz) is
// permuted by a 32 bit seed (link word / UID, see rlrsHopSeedFromUid) with a
// deterministic PRNG; each channel is used once per cycle of N frames.
//
// Slave: acquisition = parked on one channel in continuous RX until a beacon
// whose hop index matches that channel (the master sweeps every channel once
// per cycle; the parking channel changes every N + 1 frames), then tracking.
// A missed beacon advances by timer; `missLimit` consecutive misses (default
// 20 = 1 s) go back to acquisition. The slave never transmits without having
// received the beacon of the current frame (no double occupation).
//
// Preset switch: master.requestPreset(p) announces (p, seq + lead) in every
// beacon up to the target seq; the slave echoes it with the ACK flag. At the
// target seq the slave switches if it heard the announcement, the master
// switches if it received the ACK (otherwise it aborts: Stats::presetFailed).
// Link loss > linkLossUs (master: no valid downlink; slave: missLimit missed
// beacons) brings both sides back to the default preset, so any residual
// mismatch converges.
//
// Event API with injected time (local micros(), wrap-around safe). Each call
// returns an RlrsHopAction that the caller executes, in this order:
//   - if presetChanged: load preset `preset` into the radio,
//   - Transmit: send txData/txLength on frequencyHz,
//     Receive: RX on frequencyHz (rxTimeoutUs, 0 = continuous; an RX timeout
//     or CRC error is reported with onRxDone(now, nullptr, 0)),
//     Idle: radio standby, None: leave the radio as it is,
//   - (re)arm the timer: call onTimer(now) when micros() reaches timerAtUs.
// onRxDone returns rxDelivered = true when the frame is valid for this node:
// iterate its messages with RlrsAirReader (same data/length) and rebuild the
// UART frames with rlrsEncode().
//
// Time on air, Semtech SX1280 datasheet (LoRa, SF7..SF10):
//   Tsym = 2^SF / BW
//   Npayload = ceil(max(8*PL + 16*CRC - 4*SF + 8 + 20*H, 0) / (4*SF)) * (CR + 4)
//   Ntotal   = Npreamble + 4.25 + 8 + Npayload        (SF5/6: 6.25, no +8;
//   ToA      = Ntotal * Tsym                           SF11/12: 4*(SF-2))
//   H = 1 explicit header, CRC = 1 hardware CRC on, CR = 1..4 for 4/5..4/8.
// Budget, SF8 BW800 (812.5 kHz, Tsym = 315.08 us), CR 4/5, preamble 8,
// explicit header, CRC on (checked by test_rlrs_air):
//   uplink    8 o : 35.25 sym = 11.11 ms
//   downlink 24 o : 55.25 sym = 17.41 ms
//   downlink 48 o : 85.25 sym = 26.86 ms
//   frame = up 11.11 + guard 0.50 + down 26.86 + end guard 1.00 = 39.47 ms
//           (24 o downlink: 30.02 ms)  <= 50 ms period.
// Worst case uplink (33 o, 20.56 ms) still leaves room for a 50 o downlink.

#ifndef RLRS_HOP_MAX_CHANNELS
#define RLRS_HOP_MAX_CHANNELS 80
#endif
#ifndef RLRS_HOP_MAX_PRESETS
#define RLRS_HOP_MAX_PRESETS 4
#endif

struct RlrsLoraModem
{
    uint8_t sf;               // 5..12
    uint32_t bandwidthHz;     // 203125, 406250, 812500, 1625000
    uint8_t cr;               // 1..4 = 4/5..4/8
    uint16_t preambleSymbols;
    bool explicitHeader;
    bool crcOn;
};

struct RlrsHopPreset
{
    RlrsLoraModem modem;
    uint8_t uplinkMaxBytes;    // whole air frame, header included
    uint8_t downlinkMaxBytes;  // whole air frame, header included
};

struct RlrsHopConfig
{
    uint32_t framePeriodUs;     // 50000
    uint32_t firstFrequencyHz;  // 2400400000
    uint32_t lastFrequencyHz;   // 2479600000
    uint8_t channelCount;       // 40
    uint32_t seed;              // link word / UID hash
    uint32_t guardUs;           // slave RX->TX turnaround after the beacon
    uint32_t endGuardUs;        // kept free at the end of every frame
    uint32_t windowUs;          // slave RX opens this early / stays this late
    uint32_t rxLatencyUs;       // RxDone IRQ latency (subtracted on sync)
    uint8_t missLimit;          // consecutive missed beacons before acquisition
    uint32_t linkLossUs;        // master: no downlink for this long = lost
    uint8_t presetLeadFrames;   // frames between request and switch
    uint8_t presetCount;
    uint8_t defaultPreset;
    RlrsHopPreset presets[RLRS_HOP_MAX_PRESETS];
};

// Defaults: 50 ms, 40 channels 2400.4..2479.6 MHz, guard 500 us, end guard
// 1000 us, window 1500 us, missLimit 20, link loss 1 s, lead 10 frames,
// preset 0 = SF8 BW800 CR4/5 pre 8 (up 33 / down 72), preset 1 = SF7 BW800
// CR4/5 pre 8 (up 33 / down 127). Default preset 0.
RlrsHopConfig rlrsHopDefaultConfig(uint32_t seed);
// Checks ranges and that, for every preset, (a) a full uplink plus a header
// only downlink and (b) a bare beacon plus a full downlink fit in a frame.
bool rlrsHopConfigValid(const RlrsHopConfig &config);
// FNV-1a over a link word / UID, never 0.
uint32_t rlrsHopSeedFromUid(const uint8_t *uid, size_t length);

// Semtech SX1280 LoRa time on air (us, rounded up).
uint32_t rlrsLoraAirtimeUs(const RlrsLoraModem &modem, uint8_t payloadLength);
// Largest payload length <= limit whose time on air is <= budgetUs, or -1.
int16_t rlrsLoraMaxPayload(const RlrsLoraModem &modem, uint32_t budgetUs, uint8_t limit);

struct RlrsHopAction
{
    enum Type : uint8_t
    {
        None = 0,     // keep the radio as it is
        Idle,         // standby
        Transmit,     // send txData/txLength on frequencyHz
        Receive       // RX on frequencyHz, rxTimeoutUs (0 = continuous)
    };
    Type type;
    uint32_t frequencyHz;
    const uint8_t *txData;
    uint8_t txLength;
    uint32_t rxTimeoutUs;
    bool presetChanged;   // apply `preset` to the radio before TX/RX
    uint8_t preset;
    bool rxDelivered;     // onRxDone only: frame accepted, read it with RlrsAirReader
    uint32_t timerAtUs;   // always valid: call onTimer() at this local time
};

class RlrsHop
{
public:
    enum Role : uint8_t
    {
        Master = 0,   // ground
        Slave = 1     // rocket
    };

    enum State : uint8_t
    {
        StateStopped = 0,
        StateAcquire,     // slave: parked, continuous RX
        StateRxBeacon,    // slave: RX window around the expected beacon
        StateTxGuard,     // slave: beacon received, waiting guardUs
        StateTx,          // transmitting
        StateRxDownlink,  // master: listening for the downlink
        StateIdle         // waiting for the next frame
    };

    struct Stats
    {
        uint32_t framesTx;
        uint32_t framesRx;          // valid frames delivered
        uint32_t rejected;          // received but not valid for this node
        uint32_t missed;            // slave: beacons missed; master: downlinks missed
        uint32_t acquisitions;      // slave: acquisition -> tracking
        uint32_t lost;              // link losses
        uint32_t presetSwitches;
        uint32_t presetFailed;      // master: no ACK by the target seq
        uint32_t presetReverts;     // back to default after a link loss
        uint32_t ignored;           // events that did not match the state
    };

    // txQueue: messages to aggregate in this node's frames (may be null).
    RlrsHop(Role role, const RlrsHopConfig &config, RlrsAirQueue *txQueue);

    RlrsHopAction start(uint32_t now);
    RlrsHopAction onTimer(uint32_t now);
    RlrsHopAction onTxDone(uint32_t now);
    // data == nullptr or length == 0: RX timeout or CRC error.
    RlrsHopAction onRxDone(uint32_t now, const uint8_t *data, uint8_t length);

    // Master only: announce a preset switch. False if not master, link down,
    // a switch is already pending, preset invalid or already active.
    bool requestPreset(uint8_t preset);

    bool synced() const;        // slave: tracking; master: downlink within linkLossUs
    bool presetPending() const { return pendingValid_; }
    uint8_t preset() const { return preset_; }
    uint8_t seq() const { return seq_; }
    uint8_t hopIndex() const { return hop_; }
    State state() const { return state_; }
    Role role() const { return role_; }
    uint8_t channelForHop(uint8_t hop) const;          // permuted channel number
    uint32_t frequencyForHop(uint8_t hop) const;       // Hz
    uint32_t channelFrequencyHz(uint8_t channel) const;
    uint32_t frameStartUs() const { return frameStart_; }
    const RlrsAirHeader &lastHeader() const { return lastHeader_; }
    const Stats &stats() const { return stats_; }
    const RlrsHopConfig &config() const { return config_; }

private:
    RlrsHopAction makeAction(RlrsHopAction::Type type);
    void advanceFrame();
    void setPreset(uint8_t preset);
    const RlrsLoraModem &modem() const { return config_.presets[preset_].modem; }

    RlrsHopAction masterFrame(uint32_t now);
    RlrsHopAction masterRx(uint32_t now, const uint8_t *data, uint8_t length);
    RlrsHopAction slaveAcquire(uint32_t now);
    RlrsHopAction slaveOpenWindow(uint32_t now);
    RlrsHopAction slaveTransmit(uint32_t now);
    RlrsHopAction slaveMiss(uint32_t now);
    RlrsHopAction slaveSync(uint32_t now, const RlrsAirHeader &header, uint8_t length);

    RlrsHopConfig config_;
    RlrsAirQueue *queue_;
    Stats stats_;
    RlrsAirHeader lastHeader_;
    uint8_t perm_[RLRS_HOP_MAX_CHANNELS];
    uint8_t txBuffer_[RLRS_AIR_FRAME_MAX];
    uint32_t frameStart_;
    uint32_t timerAt_;
    uint32_t lastRxUs_;
    Role role_;
    State state_;
    uint8_t seq_;
    uint8_t hop_;
    uint8_t parkHop_;
    uint8_t missed_;
    uint8_t preset_;
    uint8_t pendingPreset_;
    uint8_t pendingSeq_;
    bool pendingValid_;
    bool pendingAcked_;
    bool presetChanged_;
    bool linkAlive_;
};

#endif
