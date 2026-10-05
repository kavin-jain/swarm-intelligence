#!/usr/bin/env bash
# Compile the simulator (real robot brain inside) to WebAssembly for the website.
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build
zig c++ -target wasm32-wasi -mexec-model=reactor -Os -fno-exceptions -fno-rtti -DNDEBUG -s \
    -I core -I sim web/engine_wasm.cpp -o build/swarm-engine.wasm
ls -la build/swarm-engine.wasm
