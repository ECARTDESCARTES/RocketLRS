#ifndef H_RlrsProfiles
#define H_RlrsProfiles

#include <cstdint>

#include "RlrsHop.h"

// Flight phases. The bridge maps each phase to a radio preset and switches
// presets through RlrsHop (announce + ACK + switch at the same seq).
enum class RlrsProfile : uint8_t {
    Disarmed   = 0,
    Armed      = 1,
    Postflight = 2,
    Transfer   = 3
};

// The enum value is also the RlrsHop preset index (see RLRS_RADIO_PRESETS).
enum class RlrsRadioPreset : uint8_t {
    FlightLoRa   = 0,
    TransferFLRC = 1
};

constexpr uint8_t RLRS_RADIO_PRESET_COUNT = 2;
static_assert(RLRS_RADIO_PRESET_COUNT <= RLRS_HOP_MAX_PRESETS, "RlrsHop holds too few presets");

struct RlrsProfileSettings {
    RlrsRadioPreset radio;
    uint32_t telemetryPeriodMs;
    bool firmwareTransferAllowed;
};

// Périodes de télémétrie proposées.
// Ce ne sont PAS les intervalles entre créneaux radio.
// 20 ms = 50 trames/s possibles pour les petits paquets ; les débits réels
// sont : gyro 20 Hz, GPS 5 Hz, pression 1 Hz, batterie 1 Hz.
constexpr RlrsProfileSettings DISARMED_SETTINGS {
    RlrsRadioPreset::FlightLoRa, 1000, false
};

constexpr RlrsProfileSettings ARMED_SETTINGS {
    RlrsRadioPreset::FlightLoRa, 20, false
};

constexpr RlrsProfileSettings POSTFLIGHT_SETTINGS {
    RlrsRadioPreset::FlightLoRa, 500, false
};

constexpr RlrsProfileSettings TRANSFER_SETTINGS {
    RlrsRadioPreset::TransferFLRC, 1000, true
};

// RLRS message intercepted by the ground bridge (never forwarded to a UART):
// payload[0] = RlrsProfile. It may come from the Java program (ground UART)
// or from the STM32 (rocket UART, carried over the air). The ground bridge,
// RlrsHop master, then calls requestPreset() for the profile's radio preset.
constexpr uint8_t RLRS_PROFILE_REQUEST_ID = 0xF1;

// ---------------------------------------------------------------------------
// Radio presets
// ---------------------------------------------------------------------------

enum class RlrsModulation : uint8_t {
    LoRa = 0,
    FLRC = 1
};

// FLRC coding rates (SX1280 FLRC_CR_1_2 / 3_4 / 1_0).
constexpr uint8_t RLRS_FLRC_CR_1_2 = 0;
constexpr uint8_t RLRS_FLRC_CR_3_4 = 1;
constexpr uint8_t RLRS_FLRC_CR_1_0 = 2;

// Radio-independent description. main.cpp converts it to SX1280 register
// values (SX1280_LORA_SF8, SX1280_LORA_BW_0800, SX1280_LORA_CR_4_5,
// SX1280_FLRC_BR_1_300_BW_1_2, ...).
struct RlrsRadioPresetParams {
    RlrsRadioPreset preset;
    RlrsModulation modulation;
    // LoRa
    uint32_t loraBandwidthHz;   // 812500 = SX1280 "BW 800"
    uint8_t loraSf;             // 5..12
    uint8_t loraCr;             // 1..4 = 4/5..4/8, NOT interleaved (CR_LI)
    // FLRC
    uint32_t flrcBitrate;       // 1300000, 1000000 or 650000 bit/s
    uint8_t flrcCr;             // RLRS_FLRC_CR_*
    uint8_t flrcBtTenths;       // Gaussian filter: 10 = BT 1.0, 5 = BT 0.5, 0 = off
    // Common
    uint8_t preamble;           // LoRa: symbols (<= 15); FLRC: bits (8..32, multiple of 4)
    bool explicitHeader;        // variable length frames
    bool crcOn;                 // hardware CRC (LoRa 16 bits, FLRC 24 bits)
    // Timing model used by RlrsHop (always LoRa). For a LoRa preset it is the
    // real modem. For FLRC it is a LoRa modem whose time on air is >= the FLRC
    // time on air for every length (checked by test_profiles): RlrsHop
    // then sizes the slots conservatively, and main.cpp corrects the beacon
    // sync with rlrsRadioAirtimeUs() (see rlrsHopSyncShiftUs()).
    RlrsHopPreset hop;
};

// Largest air frame of any preset; SX12XX_RX_BUFFER_SIZE must be >= this.
constexpr uint8_t RLRS_RADIO_FRAME_LIMIT = 127;

constexpr RlrsRadioPresetParams RLRS_RADIO_PRESETS[RLRS_RADIO_PRESET_COUNT] = {
    // Flight: LoRa SF8, BW 812.5 kHz, CR 4/5, preamble 8, explicit header,
    // hardware CRC. About -115 dBm. Uplink 33 o (beacon + commands), downlink
    // 72 o (aggregated telemetry), both inside a 50 ms TDMA frame.
    { RlrsRadioPreset::FlightLoRa, RlrsModulation::LoRa,
      812500, 8, 1,
      0, RLRS_FLRC_CR_1_2, 0,
      8, true, true,
      { { 8, 812500, 1, 8, true, true }, 33, 72 } },
    // Ground transfer: FLRC 1.3 Mbit/s (SX1280_FLRC_BR_1_300_BW_1_2), CR 1/2,
    // BT 1.0, 32 bit preamble, 32 bit sync word = link seed, CRC 24 bits.
    // Less sensitive than LoRa: never used in flight.
    { RlrsRadioPreset::TransferFLRC, RlrsModulation::FLRC,
      0, 0, 0,
      1300000, RLRS_FLRC_CR_1_2, 10,
      32, true, true,
      { { 5, 1625000, 1, 8, true, true }, 64, 127 } },
};

inline const RlrsProfileSettings &rlrsProfileSettings(RlrsProfile profile)
{
    switch (profile)
    {
    case RlrsProfile::Armed:      return ARMED_SETTINGS;
    case RlrsProfile::Postflight: return POSTFLIGHT_SETTINGS;
    case RlrsProfile::Transfer:   return TRANSFER_SETTINGS;
    case RlrsProfile::Disarmed:
    default:                      return DISARMED_SETTINGS;
    }
}

inline bool rlrsProfileFromByte(uint8_t value, RlrsProfile &profile)
{
    if (value > static_cast<uint8_t>(RlrsProfile::Transfer))
        return false;
    profile = static_cast<RlrsProfile>(value);
    return true;
}

inline uint8_t rlrsRadioPresetIndex(RlrsRadioPreset preset)
{
    return static_cast<uint8_t>(preset);
}

inline const RlrsRadioPresetParams &rlrsRadioPresetParams(uint8_t index)
{
    return RLRS_RADIO_PRESETS[index < RLRS_RADIO_PRESET_COUNT ? index : 0];
}

// FLRC time on air, rounded up (us): preamble + 32 bit sync word, then
// 16 bit header + payload + 24 bit CRC + 6 tail bits through the FEC.
inline uint32_t rlrsFlrcAirtimeUs(const RlrsRadioPresetParams &params, uint8_t payloadLength)
{
    const uint32_t coded = 16u + 8u * payloadLength + 24u + 6u;
    uint32_t codedBits = coded;
    if (params.flrcCr == RLRS_FLRC_CR_1_2)
        codedBits = coded * 2u;
    else if (params.flrcCr == RLRS_FLRC_CR_3_4)
        codedBits = (coded * 4u + 2u) / 3u;
    const uint64_t bits = static_cast<uint64_t>(params.preamble) + 32u + codedBits;
    if (params.flrcBitrate == 0)
        return 0;
    return static_cast<uint32_t>((bits * 1000000u + params.flrcBitrate - 1u) / params.flrcBitrate);
}

// SX1280 FLRC payload length range is 6..127 o (datasheet, FLRC packet
// params): shorter frames (bare beacon, header only downlink) are padded
// with RLRS_AIR_PAD (rlrsAirPad), which RlrsAirReader skips.
constexpr uint8_t RLRS_FLRC_MIN_PAYLOAD = 6;
inline uint8_t rlrsRadioMinPayload(const RlrsRadioPresetParams &params)
{
    return params.modulation == RlrsModulation::FLRC ? RLRS_FLRC_MIN_PAYLOAD : 1;
}
static_assert(RLRS_FLRC_MIN_PAYLOAD <= RLRS_RADIO_FRAME_LIMIT, "FLRC minimum payload");

// Real time on air of a frame with this preset.
inline uint32_t rlrsRadioAirtimeUs(const RlrsRadioPresetParams &params, uint8_t payloadLength)
{
    if (params.modulation == RlrsModulation::FLRC)
        return rlrsFlrcAirtimeUs(params, payloadLength);
    return rlrsLoraAirtimeUs(params.hop.modem, payloadLength);
}

// RlrsHop dates a received beacon with its LoRa timing model
// (frameStart = now - airtime_model(length)). For FLRC the real frame is
// shorter: add this to the RxDone time before calling onRxDone().
inline uint32_t rlrsHopSyncShiftUs(const RlrsRadioPresetParams &params, uint8_t payloadLength)
{
    const uint32_t model = rlrsLoraAirtimeUs(params.hop.modem, payloadLength);
    const uint32_t real = rlrsRadioAirtimeUs(params, payloadLength);
    return model > real ? model - real : 0;
}

// RlrsHop configuration of the bridge: RlrsHop defaults (50 ms, 40 channels
// 2400.4..2479.6 MHz, guards) with the presets of RLRS_RADIO_PRESETS.
inline RlrsHopConfig rlrsHopConfigFromPresets(uint32_t seed)
{
    RlrsHopConfig config = rlrsHopDefaultConfig(seed);
    config.presetCount = RLRS_RADIO_PRESET_COUNT;
    config.defaultPreset = rlrsRadioPresetIndex(RlrsRadioPreset::FlightLoRa);
    for (uint8_t i = 0; i < RLRS_RADIO_PRESET_COUNT; ++i)
        config.presets[i] = RLRS_RADIO_PRESETS[i].hop;
    return config;
}

// Smallest uplink / downlink frame over every preset: the air queues are
// sized with it so that a queued message fits whatever the active preset.
inline uint8_t rlrsRadioMinUplinkBytes()
{
    uint8_t value = RLRS_AIR_FRAME_MAX;
    for (uint8_t i = 0; i < RLRS_RADIO_PRESET_COUNT; ++i)
        if (RLRS_RADIO_PRESETS[i].hop.uplinkMaxBytes < value)
            value = RLRS_RADIO_PRESETS[i].hop.uplinkMaxBytes;
    return value;
}

inline uint8_t rlrsRadioMinDownlinkBytes()
{
    uint8_t value = RLRS_AIR_FRAME_MAX;
    for (uint8_t i = 0; i < RLRS_RADIO_PRESET_COUNT; ++i)
        if (RLRS_RADIO_PRESETS[i].hop.downlinkMaxBytes < value)
            value = RLRS_RADIO_PRESETS[i].hop.downlinkMaxBytes;
    return value;
}

inline uint8_t rlrsRadioMaxFrameBytes()
{
    uint8_t value = 0;
    for (uint8_t i = 0; i < RLRS_RADIO_PRESET_COUNT; ++i)
    {
        const RlrsHopPreset &hop = RLRS_RADIO_PRESETS[i].hop;
        if (hop.uplinkMaxBytes > value)
            value = hop.uplinkMaxBytes;
        if (hop.downlinkMaxBytes > value)
            value = hop.downlinkMaxBytes;
    }
    return value;
}
#endif
