#ifndef H_RlrsProfiles
#define H_RlrsProfiles

#include <cstdint>

enum class RlrsProfile : uint8_t {
    Disarmed   = 0,
    Armed      = 1,
    Postflight = 2,
    Transfer   = 3
};

enum class RlrsRadioPreset : uint8_t {
    FlightLoRa,
    TransferFLRC
};

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
#endif