#pragma once
// ESP32 FreeRTOS critical section stub (used by RlrsLink::Guard).
#include "../Arduino.h"
struct portMUX_TYPE { int owner; };
#define portMUX_INITIALIZER_UNLOCKED {0}
#define portENTER_CRITICAL(m) do { (void)(m); g_irqMasked = true; } while (0)
#define portEXIT_CRITICAL(m) do { (void)(m); g_irqMasked = false; } while (0)
