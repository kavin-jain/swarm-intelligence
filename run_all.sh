#!/usr/bin/env bash
# Every check in the repo. Exits non-zero if anything that must pass fails.
#   bash run_all.sh            (firmware build is skipped if PlatformIO isn't installed)
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p build

echo "== core unit tests (ASan + UBSan)"
c++ -std=c++17 -Wall -Wextra -g -fsanitize=address,undefined -I core test/test_core.cpp -o build/test_core
build/test_core

echo "== simulator: scenarios, then 20 seeds each"
c++ -std=c++17 -O2 -Wall -Wextra -I core -I sim sim/sim.cpp -o build/sim
build/sim
build/sim --seeds 20

echo "== simulator: 120 random arenas (benchmark, reported not enforced)"
build/sim --random 120 | tail -1 || true

echo "== satellite (Python)"
[ -x .venv/bin/python ] || { python3 -m venv .venv && .venv/bin/pip install -q -r satellite/requirements.txt; }
(cd satellite && ../.venv/bin/python -m unittest test_satellite)

if command -v pio >/dev/null; then
    echo "== firmware build (robot1-3, gateway, motortest)"
    (cd firmware && pio run -s)
fi
if command -v zig >/dev/null && command -v node >/dev/null; then
    echo "== WebAssembly build + smoke test"
    bash web/build.sh >/dev/null
    node web/test_wasm.mjs
fi
echo "ALL CHECKS PASSED"
