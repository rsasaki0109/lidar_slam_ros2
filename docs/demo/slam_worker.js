// Runs RKO-LIO (WebAssembly, built by web/build_wasm.sh) on an LSWC2 clip (web/make_clip.py)
// and streams poses and map points to the page. Parameters come from params.json.
/* global importScripts, createSlamModule */
importScripts('slam_web.js');


let paused = false;
let speed = 1; // multiple of real time; 0 = as fast as possible

self.onmessage = async (event) => {
  const msg = event.data;
  if (msg.type === 'start') run(msg.clipUrl, msg.paramsUrl).catch((e) => self.postMessage({ type: 'error', message: String(e) }));
  if (msg.type === 'pause') paused = msg.paused;
  if (msg.type === 'speed') speed = msg.speed;
};

async function fetchClip(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`clip download failed: ${response.status}`);
  const total = Number(response.headers.get('content-length')) || 0;
  const reader = response.body.getReader();
  const chunks = [];
  let received = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) break;
    chunks.push(value);
    received += value.length;
    self.postMessage({ type: 'download', received, total });
  }
  const gz = new Blob(chunks).stream().pipeThrough(new DecompressionStream('gzip'));
  return new Response(gz).arrayBuffer();
}

function zigzagDeltaDecode(src, n, out, stride, offset) {
  let value = 0;
  for (let i = 0; i < n; i++) {
    const z = src[i];
    value += (z >>> 1) ^ -(z & 1);
    out[i * stride + offset] = value;
  }
}

const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));

async function run(clipUrl, paramsUrl) {
  const [M, buffer, params] = await Promise.all([
    createSlamModule(), fetchClip(clipUrl), fetch(paramsUrl).then((r) => r.text())]);
  const dv = new DataView(buffer);
  const magic = new TextDecoder().decode(new Uint8Array(buffer, 0, 5));
  if (magic !== 'LSWC2') throw new Error('not an LSWC2 clip');
  const jsonLen = dv.getUint32(8, true);
  const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 12, jsonLen)));
  self.postMessage({ type: 'header', header });
  let off = 12 + jsonLen;
  if (!M.ccall('slam_create', 'number', ['string'], [params])) throw new Error('slam_create failed');

  const imus = [];
  for (let i = 0; i < header.n_imu; i++, off += 56) {
    imus.push({ sec: dv.getInt32(off, true), nsec: dv.getUint32(off + 4, true), off: off + 8 });
  }
  const imuPtr = M._malloc(48);
  let nextImu = 0;
  const feedImu = (m) => {
    M.HEAPU8.set(new Uint8Array(buffer, m.off, 48), imuPtr);
    M._slam_add_imu(m.sec, m.nsec, imuPtr);
  };
  const before = (a, sec, nsec) => a.sec < sec || (a.sec === sec && a.nsec <= nsec);

  let cap = 0;
  let xyzPtr = 0;
  let dtPtr = 0;
  let xyz = new Int16Array(0);
  let dt = new Uint16Array(0);
  let firstStamp = null;
  const wallStart = performance.now();
  let computeMs = 0;

  for (let k = 0; k < header.n_scans; k++) {
    while (paused) await sleep(100);
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
    for (let c = 0; c < 3; c++) {
      zigzagDeltaDecode(new Uint16Array(buffer.slice(off, off + n * 2)), n, xyz, 3, c);
      off += n * 2;
    }
    zigzagDeltaDecode(new Uint16Array(buffer.slice(off, off + n * 2)), n, dt, 1, 0);
    off += n * 2;

    const stamp = sec + nsec * 1e-9;
    if (firstStamp === null) firstStamp = stamp;
    if (speed > 0) {
      const due = (stamp - firstStamp) / speed * 1000 - (performance.now() - wallStart);
      if (due > 0) await sleep(due);
    }

    const t0 = performance.now();
    M.HEAPU8.set(new Uint8Array(xyz.buffer, 0, n * 6), xyzPtr);
    M.HEAPU8.set(new Uint8Array(dt.buffer, 0, n * 2), dtPtr);
    while (nextImu < imus.length && before(imus[nextImu], sec, nsec)) feedImu(imus[nextImu++]);
    M._slam_add_scan(sec, nsec, n, xyzPtr, dtPtr);
    computeMs += performance.now() - t0;

    const count = M._slam_output_count();
    const poses = new Float64Array(count * 8);
    let pointTotal = 0;
    for (let i = 0; i < count; i++) pointTotal += M._slam_output_point_count(i);
    const points = new Float32Array(pointTotal * 3);
    let p = 0;
    for (let i = 0; i < count; i++) {
      const posePtr = M._slam_output_pose(i) / 8;
      poses.set(M.HEAPF64.subarray(posePtr, posePtr + 8), i * 8);
      const m = M._slam_output_point_count(i) * 3;
      const ptsPtr = M._slam_output_points(i) / 4;
      points.set(M.HEAPF32.subarray(ptsPtr, ptsPtr + m), p);
      p += m;
    }
    M._slam_clear_outputs();
    self.postMessage({
      type: 'progress', scan: k + 1, nScans: header.n_scans, scanPoints: n,
      dataSeconds: stamp - firstStamp, computeMs, poses, points,
    }, [poses.buffer, points.buffer]);
  }
  while (nextImu < imus.length) feedImu(imus[nextImu++]);
  self.postMessage({ type: 'done', computeMs });
}
