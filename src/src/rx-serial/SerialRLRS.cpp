#if defined(TARGET_RX)

#include "SerialRLRS.h"
#include "common.h"
#include "device.h"

SerialRLRS::SerialRLRS(Stream &out, Stream &in) : SerialIO(&out, &in)
{
    link.reset();
}

uint32_t SerialRLRS::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData)
{
    return DURATION_IMMEDIATELY;
}

int SerialRLRS::getMaxSerialReadSize()
{
    return RLRS_MAX_SIZE;
}

void SerialRLRS::processBytes(uint8_t *bytes, uint16_t size)
{
    if (connectionState == connected)
    {
        link.feedUart(bytes, size);
    }
}

void SerialRLRS::sendQueuedData(uint32_t maxBytesToSend)
{
    uint8_t buffer[64];
    uint32_t budget = maxBytesToSend != 0 ? maxBytesToSend : sizeof(buffer);

    while (budget > 0)
    {
        const uint16_t chunk = budget < sizeof(buffer) ? static_cast<uint16_t>(budget)
                                                        : static_cast<uint16_t>(sizeof(buffer));
        const uint16_t length = link.takeUart(buffer, chunk);
        if (length == 0)
            break;
        _outputPort->write(buffer, length);
        budget -= length;
    }
}

void SerialRLRS::queueLinkStats()
{
    RlrsLink::LinkStats stats;
    stats.uplinkRssi = linkStats.uplink_RSSI_1;
    stats.uplinkSnr = linkStats.uplink_SNR;
    stats.uplinkLq = linkStats.uplink_Link_quality;
    stats.downlinkRssi = linkStats.downlink_RSSI_1;
    stats.downlinkSnr = linkStats.downlink_SNR;
    stats.downlinkLq = linkStats.downlink_Link_quality;
    stats.txPower = linkStats.uplink_TX_Power;
    stats.rfMode = linkStats.rf_Mode;
    link.enqueueLinkStats(stats);
}

#endif
