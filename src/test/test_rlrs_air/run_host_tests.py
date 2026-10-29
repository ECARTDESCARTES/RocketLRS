"""Build and run the RocketLRS stand-alone air layer host tests (RlrsAir + RlrsHop)."""
import os
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / ".pio" / "rlrs-air-tests"
BUILD.mkdir(parents=True, exist_ok=True)
CXX = os.environ.get("CXX") or shutil.which("g++")
if not CXX and Path("C:/msys64/ucrt64/bin/g++.exe").exists():
    CXX = "C:/msys64/ucrt64/bin/g++.exe"
if not CXX:
    raise SystemExit("g++ is required (or set CXX).")
SUFFIX = ".exe" if os.name == "nt" else ""

binary = BUILD / ("rlrs_air" + SUFFIX)
sources = [
    ROOT / "test/test_rlrs_air/test_rlrs_air.cpp",
    ROOT / "lib/RocketLRS/RlrsAir.cpp",
    ROOT / "lib/RocketLRS/RlrsHop.cpp",
]
subprocess.run(
    [CXX, "-std=c++11", "-O1", "-Wall", "-Wextra", "-pedantic", "-Werror",
     "-I" + str(ROOT / "lib/RocketLRS"),
     *map(str, sources), "-o", str(binary)],
    check=True,
)
subprocess.run([str(binary)], check=True)
