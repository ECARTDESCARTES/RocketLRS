#pragma once
// Stub of the SX1280 driver/HAL. Real SX1280_Regs.h / SX12xxDriverCommon.h.
// Every SPI command checks the reentrancy rule of main.cpp: a loop() SPI
// command while the radio is in RX/TX (DIO1 may fire) must run with the
// interrupts masked, otherwise g_spiRaces is incremented.
#include "targets.h"
#include "Arduino.h"
#include "SX1280_Regs.h"
#include "SX12xxDriverCommon.h"
#define RADIO_SNR_SCALE 4
extern bool g_radioCanIrq;      // radio in RX or TX
extern uint32_t g_spiRaces;
extern uint32_t g_spiCommands;
inline void spiCommand()
{
    ++g_spiCommands;
    if (g_radioCanIrq && !g_irqMasked && !g_inIsr)
        ++g_spiRaces;
}
class SX1280Hal
{
public:
    uint8_t lastPacketParams[7] = {0};
    uint32_t packetParamWrites = 0;
    uint8_t rxLength = 0;
    void WriteCommand(SX1280_RadioCommands_t op, uint8_t *buf, uint8_t size, SX12XX_Radio_Number_t, uint32_t = 15)
    { spiCommand(); if (op == SX1280_RADIO_SET_PACKETPARAMS && size == 7) { memcpy(lastPacketParams, buf, 7); ++packetParamWrites; } }
    uint8_t ReadCommand(SX1280_RadioCommands_t op, uint8_t *buf, uint8_t size, SX12XX_Radio_Number_t)
    { spiCommand(); if (op == SX1280_RADIO_GET_RXBUFFERSTATUS && size == 2) { buf[0] = rxLength; buf[1] = 0x80; } return 0; }
};
class SX1280Driver : public SX12xxDriverCommon
{
public:
    enum Mode { Sleep, Standby, FS, Rx, Tx };
    Mode mode = Sleep;
    uint32_t rxOpenedUs = 0;     // micros() of the last RXnb()
    uint8_t cfgBw = 0, cfgSf = 0, cfgCr = 0, cfgPreamble = 0;
    RadioBandMod::Combined cfgMod = RadioBandMod::Combined::LORA_2G4;
    uint32_t cfgSync = 0, configCount = 0, txCount = 0;
    int8_t power = -99;
    uint8_t txData[256];
    uint8_t txLen = 0;
    uint16_t irq = 0;
    void setMode(Mode m) { mode = m; g_radioCanIrq = (m == Rx || m == Tx); }
    bool Begin() { setMode(Standby); return true; }
    void SetTxIdleMode() { spiCommand(); setMode(FS); }
    void Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq, uint8_t pre, bool, uint8_t len,
                uint32_t sync, uint16_t, RadioBandMod::Combined m)
    { spiCommand(); cfgBw = bw; cfgSf = sf; cfgCr = cr; cfgPreamble = pre; cfgMod = m; cfgSync = sync; ++configCount;
      setMode(Standby); currFreq = freq; PayloadLength = len; }
    void SetFrequencyReg(uint32_t f, SX12XX_Radio_Number_t, bool = false) { spiCommand(); currFreq = f; }
    void SetOutputPower(int8_t p) { power = p; }   // no SPI: applied after TxDone
    void TXnb(uint8_t *d, bool, uint8_t *, SX12XX_Radio_Number_t)
    { spiCommand(); memcpy(txData, d, PayloadLength); txLen = PayloadLength; setMode(Tx); ++txCount; }
    void RXnb() { spiCommand(); setMode(Rx); rxOpenedUs = micros(); }
    uint16_t GetIrqStatus(SX12XX_Radio_Number_t) { spiCommand(); return irq; }
    void CheckForSecondPacket() {}
    void GetLastPacketStats() { spiCommand(); }
};
