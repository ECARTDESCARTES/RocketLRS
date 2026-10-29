#pragma once
#include "targets.h"
#include <deque>
#include <vector>
extern uint32_t g_micros;
// Interrupt model: g_irqMasked between noInterrupts()/interrupts(), g_inIsr
// while the bench runs a radio callback.
extern bool g_irqMasked;
extern bool g_inIsr;
inline uint32_t micros() { return g_micros; }
inline void noInterrupts() { g_irqMasked = true; }
inline void interrupts() { g_irqMasked = false; }
// ESP8266 core interrupt level primitives (used by RlrsLink::Guard)
inline uint32_t xt_rsil(int) { const uint32_t was = g_irqMasked ? 15u : 0u; g_irqMasked = true; return was; }
inline void xt_wsr_ps(uint32_t ps) { g_irqMasked = ps != 0; }
inline void yield() {}
inline void delay(unsigned long) {}
#define OUTPUT 1
#define LOW 0
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
class Stream { public: virtual ~Stream() {} };
class HardwareSerialStub
{
public:
    unsigned long baud = 0;
    std::deque<uint8_t> in;
    std::vector<uint8_t> out;
    void begin(unsigned long b) { baud = b; }
    void setTimeout(unsigned long) {}
    int available() { return static_cast<int>(in.size()); }
    size_t readBytes(uint8_t *b, size_t n)
    {
        size_t i = 0;
        for (; i < n && !in.empty(); ++i) { b[i] = in.front(); in.pop_front(); }
        return i;
    }
    int availableForWrite() { return 64; }
    size_t write(const uint8_t *b, size_t n) { out.insert(out.end(), b, b + n); return n; }
};
extern HardwareSerialStub Serial;
