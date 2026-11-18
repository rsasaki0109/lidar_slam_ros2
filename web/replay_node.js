// Replays an LSWC2 clip through the WebAssembly module in Node.js, in the same order as the
// page's worker, and writes a TUM trajectory. With --reference it compares every pose against a
// trajectory written by the native build (web/README.md) and exits 1 on any difference.
//
//   node web/replay_node.js MODULE.js CLIP.lswc2.gz PARAMS.json OUT.tum [--reference REF.tum]
'use strict';
const fs = require('fs');
const path = require('path');
const zlib = require('zlib');

const args = process.argv.slice(2);
const refIndex = args.indexOf('--reference');
const reference = refIndex >= 0 ? args.splice(refIndex, 2)[1] : null;
const [modulePath, clipPath, paramsPath, outPath] = args;
if (!outPath) {
  console.error('usage: node replay_node.js MODULE.js CLIP.lswc2.gz PARAMS.json OUT.tum [--reference REF.tum]');
  process.exit(2);
}

function zigzagDeltaDecode(src, n, out, stride, offset) {
  let value = 0;
  for (let i = 0; i < n; i++) {
    const z = src[i];
    value += (z >>> 1) ^ -(z & 1);
    out[i * stride + offset] = value;
  }
}

require(path.resolve(modulePath))().then((M) => {
  let bytes = fs.readFileSync(clipPath);
  if (clipPath.endsWith('.gz')) bytes = zlib.gunzipSync(bytes);
  const buffer = bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
  const dv = new DataView(buffer);
  if (Buffer.from(buffer, 0, 5).toString() !== 'LSWC2') throw new Error('not an LSWC2 clip');
  const jsonLen = dv.getUint32(8, true);
  const header = JSON.parse(Buffer.from(buffer, 12, jsonLen).toString());
  let off = 12 + jsonLen;
  if (!M.ccall('slam_create', 'number', ['string'], [fs.readFileSync(paramsPath, 'utf8')])) process.exit(1);

  const imus = [];
  for (let i = 0; i < header.n_imu; i++, off += 56) {
    imus.push({ sec: dv.getInt32(off, true), nsec: dv.getUint32(off + 4, true), off: off + 8 });
  }
  const imuPtr = M._malloc(48);
  const feedImu = (m) => {
    M.HEAPU8.set(new Uint8Array(buffer, m.off, 48), imuPtr);
    M._slam_add_imu(m.sec, m.nsec, imuPtr);
  };
  const before = (a, sec, nsec) => a.sec < sec || (a.sec === sec && a.nsec <= nsec);
  let next = 0;
  let cap = 0;
  let xyzPtr = 0;
  let dtPtr = 0;
  let xyz = new Int16Array(0);
  let dt = new Uint16Array(0);
  for (let k = 0; k < header.n_scans; k++) {
    const sec = dv.getInt32(off, true);
    const nsec = dv.getUint32(off + 4, true);
    const n = dv.getUint32(off + 8, true);
    off += 12;
    if (n > cap) {
      if (cap) { M._free(xyzPtr); M._free(dtPtr); }
      cap = n;
      xyzPtr = M._malloc(n * 6);
      dtPtr = M._malloc(n * 2);
      xyz = new Int16Array(n * 3);
      dt = new Uint16Array(n);
    }
    for (let c = 0; c < 3; c++, off += n * 2) zigzagDeltaDecode(new Uint16Array(buffer.slice(off, off + n * 2)), n, xyz, 3, c);
    zigzagDeltaDecode(new Uint16Array(buffer.slice(off, off + n * 2)), n, dt, 1, 0);
    off += n * 2;
    M.HEAPU8.set(new Uint8Array(xyz.buffer, 0, n * 6), xyzPtr);
    M.HEAPU8.set(new Uint8Array(dt.buffer, 0, n * 2), dtPtr);
    while (next < imus.length && before(imus[next], sec, nsec)) feedImu(imus[next++]);
    M._slam_add_scan(sec, nsec, n, xyzPtr, dtPtr);
  }
  while (next < imus.length) feedImu(imus[next++]);

  const poses = [];
  for (let i = 0; i < M._slam_output_count(); i++) {
    const p = M._slam_output_pose(i) / 8;
    poses.push(Array.from(M.HEAPF64.subarray(p, p + 8)));
  }
  fs.writeFileSync(outPath, poses.map((v) => v.map((x) => x.toFixed(9)).join(' ')).join('\n') + '\n');
  console.error(`poses ${poses.length}`);
  if (reference) {
    const ref = fs.readFileSync(reference, 'utf8').trim().split('\n').map((l) => l.split(' ').map(Number));
    let worst = 0;
    if (ref.length !== poses.length) {
      console.error(`reference has ${ref.length} poses, this run ${poses.length}`);
      process.exit(1);
    }
    ref.forEach((r, i) => r.forEach((x, j) => { worst = Math.max(worst, Math.abs(x - poses[i][j])); }));
    console.error(`largest difference from the reference: ${worst}`);
    if (worst > 1e-9) process.exit(1);
  }
});
