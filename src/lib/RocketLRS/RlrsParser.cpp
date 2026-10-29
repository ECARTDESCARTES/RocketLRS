#include "RlrsParser.h"
#include "RlrsProtocol.h"

void RlrsParser::setHandler(PacketHandler handler, void *context)
{
    handler_ = handler;
    context_ = context;
}

void RlrsParser::processBytes(const uint8_t *bytes, uint16_t size)
{
    for (uint16_t i = 0; i < size; ++i)
    {
        if (processByte(bytes[i]))
        {
            if (handler_ != nullptr)
                handler_(current_build, context_);
            current_build = RlrsUartPacket{};
            current_position = 0;
        }
    }
}

void RlrsParser::reset()
{
    current_build = RlrsUartPacket{};
    current_position = 0;
    handler_ = nullptr;
    context_ = nullptr;
}

bool RlrsParser::processByte(uint8_t byte)
{
    if (current_position == 0)
    {
        if (byte != RLRS_SYNCWORD)
            return false;
        current_build.syncword = byte;
    }
    else if (current_position == 1)
    {
        current_build.payload_len = byte;
        if (byte > RLRS_MAX_SIZE_PAYLOAD)
        {
            current_build = RlrsUartPacket{};
            current_position = 0;
            // AA cannot be a valid length; it may start the next frame.
            if (byte == RLRS_SYNCWORD)
            {
                current_build.syncword = byte;
                current_position = 1;
            }
            return false;
        }
    }
    else if (current_position == 2)
        current_build.packet_id = byte;
    else if (current_position < 3 + current_build.payload_len)
        current_build.payload[current_position - 3] = byte;
    else if (current_position == 3 + current_build.payload_len)
        current_build.crc = static_cast<uint16_t>(byte) << 8;
    else if (current_position == 4 + current_build.payload_len)
    {
        current_build.crc |= byte;
        const uint8_t header[] = {current_build.syncword, current_build.payload_len, current_build.packet_id};
        uint16_t crc = rlrsCrc16(header, sizeof(header));
        crc = rlrsCrc16(current_build.payload, current_build.payload_len, crc);
        if (crc == current_build.crc)
            return true;
        current_build = RlrsUartPacket{};
        current_position = 0;
        return false;
    }
    ++current_position;
    return false;
}
