// Standalone RocketLRS serial <-> SX1280 radio bridge for the Wi-Fi test targets.
#include <Arduino.h>
#include "options.h"
#include "logging.h"
#include "SX1280Driver.h"
#include "RlrsParser.h"
#include "RlrsProtocol.h"
#include "FIFO.h"
// Seen by the PlatformIO library finder here (it does not follow the
// relative include below): framework Wi-Fi library used by disableWifi().
#if defined(PLATFORM_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(PLATFORM_ESP32)
#include <WiFi.h>
#endif
#include "helpers.h"
#include "../examples/rlrs-radio/main.cpp"
