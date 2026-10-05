// Smoke test: the wasm build must solve the same jobs as the native simulator.
//   node web/test_wasm.mjs
import { readFileSync } from 'fs';
const stub = new Proxy({}, { get: () => () => 0 });
const { instance } = await WebAssembly.instantiate(readFileSync(new URL('../build/swarm-engine.wasm', import.meta.url)), { wasi_snapshot_preview1: stub });
const w = instance.exports; w._initialize();
function run(robots, loads, secs) {
  w.reset(1500, 1000, 1320, 500, 160, 1234, 0);
  robots.forEach(([x, y]) => w.add_robot(x, y, 0));
  loads.forEach(([x, y, k]) => w.add_object(x, y, k || 1));
  for (let t = 0; t < secs * 50; t += 50) {
    w.step(50);
    const s = new Float32Array(w.memory.buffer, w.state(), 8 + 12 * 10 + 16 * 8);
    const no = s[2], base = 8 + 120; let done = 0;
    for (let j = 0; j < no; j++) done += s[base + j * 8 + 5] === 1;
    if (done === no) return { done, no, t: s[0] };
  }
  const s = new Float32Array(w.memory.buffer, w.state(), 8);
  return { done: -1, t: s[0] };
}
const a = run([[150, 250], [150, 750]], [[500, 200], [650, 820], [400, 520], [880, 330], [760, 640]], 180);
const b = run([[150, 250], [150, 800]], [[560, 330, 2], [520, 800]], 180);
console.log('5 pencils / 2 robots:', a, '| heavy box + pencil:', b);
if (a.done !== 5 || b.done !== 2) { console.error('FAIL'); process.exit(1); }
console.log('wasm smoke test passed');
