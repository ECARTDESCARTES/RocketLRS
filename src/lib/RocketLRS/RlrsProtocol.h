#pragma once

#include <cstddef>
#include <cstdint>

constexpr uint8_t RLRS_SYNCWORD = 0xAA;
constexpr size_t RLRS_WIRE_MAX_SIZE = 69;

// CRC-16/CCITT-FALSE, including AA, length, ID and the actual payload.
inline uint16_t rlrsCrc16(const uint8_t *data, size_t length, uint16_t crc = 0xFFFF)
{
    while (length--)
    {
        crc ^= static_cast<uint16_t>(*data++) << 8;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
    }
    return crc;
}

// Returns the logical frame length (5..69), or zero for invalid arguments.
// Padding to a fixed radio size is the caller's responsibility.
inline size_t rlrsEncode(uint8_t id, const uint8_t *payload, size_t length,
                         uint8_t *out, size_t capacity)
{
    if (length > 64 || !out || capacity < length + 5 || (length && !payload))
        return 0;
    out[0] = RLRS_SYNCWORD;
    out[1] = static_cast<uint8_t>(length);
    out[2] = id;
    for (size_t i = 0; i < length; ++i)
        out[3 + i] = payload[i];
    const uint16_t crc = rlrsCrc16(out, length + 3);
    out[length + 3] = static_cast<uint8_t>(crc >> 8);
    out[length + 4] = static_cast<uint8_t>(crc);
    return length + 5;
}

inline bool rlrsValidate(const uint8_t *frame, size_t size)
{
    if (!frame || size < 5 || frame[0] != RLRS_SYNCWORD || frame[1] > 64)
        return false;
    const size_t body = 3 + frame[1];
    if (size != body + 2)
        return false;
    const uint16_t received = (static_cast<uint16_t>(frame[body]) << 8) | frame[body + 1];
    return received == rlrsCrc16(frame, body);
}
