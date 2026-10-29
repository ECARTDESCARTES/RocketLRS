"""Build and run the RocketLRS link layer host tests."""
import os
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / ".pio" / "rlrs-link-tests"
BUILD.mkdir(parents=True, exist_ok=True)
CXX = os.environ.get("CXX") or shutil.which("g++")
if not CXX:
    raise SystemExit("g++ is required (or set CXX).")
SUFFIX = ".exe" if os.name == "nt" else ""

binary = BUILD / ("rlrs_link" + SUFFIX)
sources = [
    ROOT / "test/test_rlrs_link/test_rlrs_link.cpp",
    ROOT / "lib/RocketLRS/RlrsParser.cpp",
    ROOT / "lib/RocketLRS/RlrsLink.cpp",
]
subprocess.run(
    [CXX, "-std=c++11", "-Wall", "-Wextra", "-pedantic",
     "-I" + str(ROOT / "lib/RocketLRS"),
     *map(str, sources), "-o", str(binary)],
    check=True,
)
subprocess.run([str(binary)], check=True)
