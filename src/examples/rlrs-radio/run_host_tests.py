"""Run protocol tests and the actual SX1280 FIFO methods with a fake SPI bus."""
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / ".pio" / "radio-host-tests"
BUILD.mkdir(parents=True, exist_ok=True)
CXX = os.environ.get("CXX") or shutil.which("g++")
if not CXX:
    raise SystemExit("g++ is required (or set CXX).")
SUFFIX = ".exe" if os.name == "nt" else ""

def run_test(name, sources):
    binary = BUILD / (name + SUFFIX)
    subprocess.run([CXX, "-std=c++11", "-Wall", "-Wextra", "-pedantic",
                    "-I" + str(ROOT / "lib/RocketLRS"),
                    *map(str, sources), "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)

run_test("protocol", [ROOT / "examples/rlrs-radio/test_protocol.cpp",
                      ROOT / "lib/RocketLRS/RlrsParser.cpp"])

hal = (ROOT / "lib/SX1280Driver/SX1280_hal.cpp").read_text()
start = hal.index("void ICACHE_RAM_ATTR SX1280Hal::WriteBuffer(")
end = hal.index("bool ICACHE_RAM_ATTR SX1280Hal::WaitOnBusy(", start)
actual_methods = hal[start:end]
harness = r"""
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#define ICACHE_RAM_ATTR
#define WORD_ALIGNED_ATTR alignas(4)
using SX12XX_Radio_Number_t = uint8_t;
constexpr uint8_t SX1280_RADIO_WRITE_BUFFER = 0x1A;
constexpr uint8_t SX1280_RADIO_READ_BUFFER = 0x1B;
struct FakeSPI {
    uint8_t memory[256];
    unsigned reads = 0, writes = 0, synchronousWrites = 0;
    void write(uint8_t radio, uint8_t *data, uint32_t size) {
        assert(radio == 1 && size >= 3 && size <= 64);
        assert(reinterpret_cast<uintptr_t>(data) % 4 == 0);
        assert(data[0] == SX1280_RADIO_WRITE_BUFFER);
        assert(data[1] + size - 2 <= sizeof(memory));
        memcpy(memory + data[1], data + 2, size - 2);
        ++writes;
    }
    void read(uint8_t radio, uint8_t *data, uint32_t size) {
        if (data[0] == SX1280_RADIO_WRITE_BUFFER) { write(radio, data, size); ++synchronousWrites; return; }
        assert(radio == 1 && size >= 4 && size <= 64);
        assert(reinterpret_cast<uintptr_t>(data) % 4 == 0);
        assert(data[0] == SX1280_RADIO_READ_BUFFER && data[2] == 0);
        assert(data[1] + size - 3 <= sizeof(memory));
        memcpy(data + 3, memory + data[1], size - 3);
        ++reads;
    }
} SPIEx;
class SX1280Hal {
public:
    unsigned waits = 0;
    void WriteBuffer(uint8_t, uint8_t *, uint8_t, SX12XX_Radio_Number_t);
    void ReadBuffer(uint8_t, uint8_t *, uint8_t, SX12XX_Radio_Number_t);
    bool WaitOnBusy(uint8_t radio) { assert(radio == 1); ++waits; return true; }
    void BusyDelay(uint32_t us) { assert(us == 15); }
};
"""
harness += actual_methods
harness += r"""
int main() {
    SX1280Hal hal;
    uint8_t data[128];
    for (unsigned i = 0; i < sizeof(data); ++i) data[i] = i ^ 0x5A;
    for (unsigned offset : {0u, 128u}) {
        for (unsigned length = 0; length <= 128; ++length) {
            memset(SPIEx.memory, 0xCC, sizeof(SPIEx.memory));
            SPIEx.reads = SPIEx.writes = SPIEx.synchronousWrites = hal.waits = 0;
            hal.WriteBuffer(offset, data, length, 1);
            for (unsigned i = 0; i < 256; ++i) {
                const uint8_t want = i >= offset && i < offset + length ? data[i-offset] : 0xCC;
                assert(SPIEx.memory[i] == want);
            }
            uint8_t dest[130];
            memset(dest, 0xA5, sizeof(dest));
            hal.ReadBuffer(offset, dest + 1, length, 1);
            assert(dest[0] == 0xA5 && dest[length+1] == 0xA5);
            assert(memcmp(dest + 1, data, length) == 0);
            assert(SPIEx.writes == (length + 61) / 62);
            assert(SPIEx.reads == (length + 60) / 61);
            assert(SPIEx.synchronousWrites == (SPIEx.writes ? SPIEx.writes - 1 : 0));
            assert(hal.waits == SPIEx.writes + SPIEx.reads);
        }
    }
    puts("PASS: actual HAL methods, lengths 0..128, TX/RX offsets, FIFO limits and buffer guards (258 cases)");
}
"""
harness = "#include <initializer_list>\n" + harness
source = BUILD / "spi_hal_test.cpp"
source.write_text(harness)
run_test("spi_hal", [source])
