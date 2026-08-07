#!/usr/bin/env bash
# One-time setup on either original Ubuntu 18.04 device.
set -euo pipefail

ROLE="${1:-}"
case "$ROLE" in
  vehicle|infrastructure) ;;
  *) echo "Usage: bash scripts/setup_ubuntu18.sh vehicle|infrastructure" >&2; exit 2 ;;
esac
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

test -f /opt/ros/melodic/setup.bash || {
  echo "ROS 1 Melodic is required at /opt/ros/melodic." >&2
  exit 3
}

if ! command -v conda >/dev/null 2>&1; then
  for prefix in "$HOME/miniforge3" "$HOME/anaconda3"; do
    if [ -x "$prefix/bin/conda" ]; then
      export PATH="$prefix/bin:$PATH"
      break
    fi
  done
fi
if ! command -v conda >/dev/null 2>&1; then
  echo "[setup] installing Miniforge into $HOME/miniforge3"
  curl -fL --retry 3 -o /tmp/Miniforge3.sh \
    https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
  bash /tmp/Miniforge3.sh -b -p "$HOME/miniforge3"
  export PATH="$HOME/miniforge3/bin:$PATH"
fi

eval "$(conda shell.bash hook)"
if ! conda env list | awk '{print $1}' | grep -qx etx; then
  conda create -n etx python=3.12 -y
fi
conda activate etx
# The bundled Verizon examples are pure Python. Import them directly instead
# of asking PEP 517 to create an isolated build environment, which would try
# to download setuptools even when all runtime dependencies are already
# installed (and fails on field machines during a temporary DNS outage).
export PYTHONPATH="$PROJECT_ROOT/app:$PROJECT_ROOT/vendor/python-etx-samples/src${PYTHONPATH:+:$PYTHONPATH}"
if python - <<'PY'
import google.protobuf
import geohash
import paho.mqtt.client
import requests
import socks
print("[setup] existing Python runtime dependencies: OK")
PY
then
  :
else
  echo "[setup] missing Python runtime dependency; installing from PyPI"
  python -m pip install "protobuf>=6.31.1,<7" "paho-mqtt>=2.1,<3" \
    "requests>=2.32,<3" "PySocks>=1.7,<2"
fi

# Build only the small TrackingObjectArray message overlay. It intentionally
# excludes autoware_msgs and therefore does not require jsk_recognition_msgs.
source /opt/ros/melodic/setup.bash
cd "$PROJECT_ROOT/catkin_ws"
[ -f src/CMakeLists.txt ] || catkin_init_workspace src

# The ETX conda environment may contain CMake 4.x. ROS Melodic's catkin
# toplevel still declares pre-3.5 compatibility, which CMake 4 rejects before
# any package is configured. Always build this ROS1 overlay with Ubuntu 18's
# system CMake. If a previous attempt cached another CMake executable, preserve
# the failed build/devel directories and start with a clean cache.
SYSTEM_CMAKE=/usr/bin/cmake
[ -x "$SYSTEM_CMAKE" ] || {
  echo "[setup] required system CMake not found: $SYSTEM_CMAKE" >&2
  exit 4
}
CACHED_CMAKE=""
if [ -f build/CMakeCache.txt ]; then
  CACHED_CMAKE="$(
    awk -F= '$1 == "CMAKE_COMMAND:INTERNAL" {print $2; exit}' \
      build/CMakeCache.txt
  )"
fi
if [ -n "$CACHED_CMAKE" ] && [ "$CACHED_CMAKE" != "$SYSTEM_CMAKE" ]; then
  CACHE_STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
  echo "[setup] CMake cache used $CACHED_CMAKE; backing it up."
  mv build "build.cmake_mismatch_$CACHE_STAMP"
  if [ -d devel ]; then
    mv devel "devel.cmake_mismatch_$CACHE_STAMP"
  fi
fi
echo "[setup] ROS overlay CMake: $("$SYSTEM_CMAKE" --version | head -n 1)"
PATH="/usr/bin:/bin:$PATH" /opt/ros/melodic/bin/catkin_make
cd "$PROJECT_ROOT"

chmod 600 clients/*/registration.json
chmod +x scripts/*.sh

ETX_FORCE_CODEC_LITE=1 PYTHONPATH="$PROJECT_ROOT/app:$PROJECT_ROOT/vendor/python-etx-samples/src" python - <<'PY'
from codec_lite import Codec, GeoRoutedHeader
from examples.utils.client import create_etx_client
c = Codec()
value = c.decode_etx(c.encode_etx('{"ok":1}', GeoRoutedHeader(34.07, -118.44)))
assert value == b'{"ok":1}'
print("[setup] Python ETX imports and codec_lite: OK")
PY

bash "$PROJECT_ROOT/scripts/preflight.sh" "$ROLE" --offline
echo "[setup] complete for role=$ROLE"
