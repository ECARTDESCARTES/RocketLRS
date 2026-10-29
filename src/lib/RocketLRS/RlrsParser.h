#ifndef H_RlrsParser
#define H_RlrsParser

#include <cstdint>

#define RLRS_MAX_SIZE_PAYLOAD 64
#define RLRS_MAX_SIZE RLRS_MAX_SIZE_PAYLOAD + 5

struct RlrsUartPacket{
    uint8_t syncword;
    uint8_t payload_len;
    uint8_t packet_id;
    uint8_t payload[RLRS_MAX_SIZE_PAYLOAD];
    uint16_t crc;
};

class RlrsParser
{
public:
    using PacketHandler = void (*)(
        RlrsUartPacket packet,
        void *context);
    
    void setHandler(PacketHandler handler, void *context);
    void processBytes(const uint8_t *bytes, uint16_t size);
    void reset();

private:
    bool processByte(uint8_t byte);

    RlrsUartPacket current_build{};
    uint16_t current_position = 0;

    PacketHandler handler_ = nullptr;
    void *context_ = nullptr;
};

#endif