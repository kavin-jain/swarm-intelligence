// Smoke test: the wasm build must solve the same jobs as the native simulator.
//   node web/test_wasm.mjs
import { readFileSync } from 'fs';
const stub = new Proxy({}, { get: () => () => 0 });
const { instance } = await WebAssembly.instantiate(readFileSync(new URL('../build/swarm-engine.wasm', import.meta.url)), { wasi_snapshot_preview1: stub });
const w = instance.exports; w._initialize();
function run(robots, loads, secs, docks) {
  w.reset(1500, 1000, 1320, 500, 160, 1234, 5);   // docks ship loads after 5 s, like the website
  (docks || []).forEach(([x, y, r]) => w.add_dock(x, y, r));
  robots.forEach(([x, y]) => w.add_robot(x, y, 0));
  loads.forEach(([x, y, k, kind]) => w.add_object(x, y, k || 1, kind || 0));
  for (let t = 0; t < secs * 50; t += 50) {
    w.step(50);
    const h = new Float32Array(w.memory.buffer, w.state(), 8), OB = 8 + 3 * 3 + h[6] * 10;
    const s = new Float32Array(w.memory.buffer, w.state(), OB + h[7] * 9);
    const no = loads.length; let done = s[3];   // shipped, plus delivered loads still on the dock
    for (let j = 0; j < s[2]; j++) done += s[OB + j * 9 + 5] === 1;
    if (done === no) return { done, no, t: s[0] };
  }
  const s = new Float32Array(w.memory.buffer, w.state(), 8);
  return { done: -1, t: s[0] };
}
const a = run([[150, 250], [150, 750]], [[500, 200], [650, 820], [400, 520], [880, 330], [760, 640]], 180);
const b = run([[150, 250], [150, 800]], [[560, 330, 2], [520, 800]], 180);
// sorting: dock A (index 0) at the bottom right, dock B on top; kinds alternate
const c = run([[150, 200], [150, 500], [150, 800]], [[480, 260, 1, 0], [600, 760, 1, 1], [420, 520, 1, 0], [820, 330, 1, 1], [760, 620, 1, 0], [950, 500, 1, 1]], 180, [[1320, 250, 150]]);
console.log('5 pencils / 2 robots:', a, '| heavy box + pencil:', b, '| sorting to 2 docks:', c);
if (a.done !== 5 || b.done !== 2 || c.done !== 6) { console.error('FAIL'); process.exit(1); }
console.log('wasm smoke test passed');
