#pragma once

#include "OTA.h"
#include "SerialIO.h"
#include "RlrsLink.h"

class SerialRLRS final : public SerialIO {
public:
    explicit SerialRLRS(Stream &out, Stream &in);
    ~SerialRLRS() override = default;

    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

    int getMaxSerialReadSize() override;
    void sendQueuedData(uint32_t maxBytesToSend) override;

    // RocketLRS air side, driven from rx_main. isTlmQueued/packAir/unpackAir
    // are called from the radio ISRs: defined in IRAM in SerialRLRS.cpp.
    bool isTlmQueued() const;
    uint8_t packAir(uint8_t *destination, uint8_t capacity, uint8_t maxCapacity = 0);
    void unpackAir(const uint8_t *source, uint8_t capacity);
    void resetLink() { link.reset(); } // loop() context (GotConnection)

    // Queue a LINK_STATS (0xF0) frame for the STM32 with the current RF info.
    void queueLinkStats();

private:
    void processBytes(uint8_t *bytes, uint16_t size) override;

    RlrsLink link;
};
