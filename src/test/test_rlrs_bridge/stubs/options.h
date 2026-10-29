#pragma once
#include <cstdint>
enum { HARDWARE_power_control = 0, HARDWARE_power_min = 1, HARDWARE_power_max = 2 };
extern int g_hw[3];
inline int hardware_int(int key) { return g_hw[key]; }
extern int16_t g_powerValues[8];
extern int g_powerCount;
#define POWER_OUTPUT_VALUES (g_powerCount ? g_powerValues : nullptr)
#define POWER_OUTPUT_VALUES_COUNT g_powerCount
#define GPIO_PIN_SCK 1
#define GPIO_PIN_NSS 2
#define GPIO_PIN_DIO1 3
#define GPIO_PIN_ANT_CTRL UNDEF_PIN
struct firmware_options_t { uint8_t hasUID; uint8_t uid[6]; };
extern firmware_options_t firmwareOptions;
inline bool options_init() { return true; }
