"""Build and run the RocketLRS bridge host test (examples/rlrs-radio/main.cpp
against a simulated RlrsHop peer), ground role, rocket role, and ground role
with a 3 dBi antenna (-DRLRS_ANTENNA_GAIN_DB=3).

Extra arguments are passed to g++ (e.g. -DRLRS_RX_LATENCY_US=0)."""
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
BUILD = ROOT / ".pio" / "rlrs-bridge-tests"
BUILD.mkdir(parents=True, exist_ok=True)
CXX = os.environ.get("CXX") or shutil.which("g++")
if not CXX and Path("C:/msys64/ucrt64/bin/g++.exe").exists():
    CXX = "C:/msys64/ucrt64/bin/g++.exe"
if not CXX:
    raise SystemExit("g++ is required (or set CXX).")
SUFFIX = ".exe" if os.name == "nt" else ""

FLAGS = ["-std=c++11", "-O1", "-Wall", "-Wextra", "-Werror", "-fno-exceptions", "-fno-rtti",
         "-DSX12XX_RX_BUFFER_SIZE=128",
         # stubs first: Arduino, Wi-Fi, options, FIFO, logging, helpers, SX1280 driver/HAL
         "-I" + str(HERE / "stubs"),
         "-I" + str(ROOT / "examples/rlrs-radio"),
         # real headers: RocketLRS, SX1280_Regs.h, SX12xxDriverCommon.h
         "-I" + str(ROOT / "lib/RocketLRS"),
         "-I" + str(ROOT / "lib/SX12xxDriverCommon"),
         "-I" + str(ROOT / "lib/SX1280Driver")]
LIBS = [str(ROOT / "lib/RocketLRS" / f) for f in ("RlrsAir.cpp", "RlrsHop.cpp", "RlrsParser.cpp")]
VARIANTS = [
    ("bridge_rx", ["-DTARGET_RX=1", "-DPLATFORM_ESP8266=1"]),
    ("bridge_tx", ["-DTARGET_TX=1", "-DPLATFORM_ESP32=1"]),
    ("bridge_tx_gain3", ["-DTARGET_TX=1", "-DPLATFORM_ESP32=1", "-DRLRS_ANTENNA_GAIN_DB=3"]),
]

failed = 0
for name, defines in VARIANTS:
    binary = BUILD / (name + SUFFIX)
    subprocess.run([CXX, *FLAGS, *defines, *sys.argv[1:], str(HERE / "test_bridge.cpp"), *LIBS,
                    "-o", str(binary)], check=True)
    print("==", name)
    sys.stdout.flush()
    if subprocess.run([str(binary)]).returncode != 0:
        failed += 1
if failed:
    raise SystemExit("%d bridge variant(s) FAILED" % failed)
print("bridge host tests: all variants passed")
