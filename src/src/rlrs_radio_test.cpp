// Standalone RocketLRS serial <-> SX1280 radio bridge for the Wi-Fi test targets.
#include <Arduino.h>
#include "options.h"
#include "logging.h"
#include "SX1280Driver.h"
#include "RlrsParser.h"
#include "RlrsProtocol.h"
#include "FIFO.h"
#include "../examples/rlrs-radio/main.cpp"
