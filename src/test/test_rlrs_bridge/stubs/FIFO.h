#pragma once
#include "targets.h"
template <uint32_t N> class FIFO
{
    uint8_t b[N]; uint32_t head = 0, count = 0;
public:
    void lock() {} void unlock() {}
    uint16_t size() const { return static_cast<uint16_t>(count); }
    uint16_t free() const { return static_cast<uint16_t>(N - count); }
    void atomicPushBytes(const uint8_t *d, uint16_t n) { if (n > N - count) return; for (uint16_t i = 0; i < n; ++i) b[(head + count++) % N] = d[i]; }
    void popBytes(uint8_t *d, uint16_t n) { for (uint16_t i = 0; i < n && count; ++i) { d[i] = b[head]; head = (head + 1) % N; --count; } }
};
