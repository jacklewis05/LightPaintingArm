// Viewer: shows the planned light painting like a slicer preview, plus the arm's live pose.
import * as THREE from 'three';
import { OrbitControls } from './vendor/controls/OrbitControls.js';
import { LineSegments2 } from './vendor/lines/LineSegments2.js';
import { LineSegmentsGeometry } from './vendor/lines/LineSegmentsGeometry.js';
import { LineMaterial } from './vendor/lines/LineMaterial.js';

const $ = (id) => document.getElementById(id);
const rad = (deg) => deg * Math.PI / 180;

// ------------------------------------------------------------------ server calls

async function api(path, body, raw) {
  const res = await fetch(path, {
    method: 'POST',
    headers: raw ? {} : { 'Content-Type': 'application/json' },
    body: raw ?? JSON.stringify(body ?? {}),
  });
  if (!res.ok) {
    let detail = res.statusText;
    try { detail = (await res.json()).detail; } catch { /* not JSON */ }
    throw new Error(detail);
  }
  return res.json();
}

let toastTimer;
function toast(message) {
  $('toast').textContent = message;
  $('toast').hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { $('toast').hidden = true; }, 6000);
}
const guard = (fn) => async (...args) => {
  try { await fn(...args); } catch (err) { toast(err.message); }
};

// ------------------------------------------------------------------ scene

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);        // the arm's world is Z-up
const canvas = $('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0b0d12);
const camera = new THREE.PerspectiveCamera(40, 1, 5, 20000);
camera.position.set(1050, -620, 560);
const controls = new OrbitControls(camera, canvas);
controls.target.set(120, 0, 260);
controls.enableDamping = true;

scene.add(new THREE.HemisphereLight(0xffffff, 0x20242e, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.4);
sun.position.set(400, -600, 900);
scene.add(sun);
const grid = new THREE.GridHelper(1400, 28, 0x39404f, 0x1c212b);
grid.rotation.x = Math.PI / 2;
scene.add(grid);
scene.add(new THREE.AxesHelper(90));

const lineMaterials = [];
function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  for (const m of lineMaterials) m.resolution.set(w, h);
}
new ResizeObserver(resize).observe(canvas);

// ------------------------------------------------------------------ arm model

function glowTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  const grad = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  grad.addColorStop(0, 'rgba(255,255,255,1)');
  grad.addColorStop(0.25, 'rgba(255,255,255,.55)');
  grad.addColorStop(1, 'rgba(255,255,255,0)');
  g.fillStyle = grad;
  g.fillRect(0, 0, 64, 64);
  return new THREE.CanvasTexture(c);
}

// Builds the arm in the style of the CAD model: flared base, stacked base drive, a
// horizontal drum for each cycloidal drive, the upper arm bolted to the side of the
// shoulder drum, and a square forearm clamped to the elbow drum. Sizes come from
// arm_config.json; joint conventions match lightpaint/kinematics.py.
function buildArm(geo, ghost) {
  const shell = ghost
    ? new THREE.MeshBasicMaterial({ color: 0x6aa9ff, transparent: true, opacity: 0.16, depthWrite: false })
    : new THREE.MeshStandardMaterial({ color: 0x9aa1b2, roughness: 0.42, metalness: 0.45 });
  const dark = ghost ? shell : new THREE.MeshStandardMaterial({ color: 0x5b6170, roughness: 0.5, metalness: 0.4 });
  const outline = new THREE.LineBasicMaterial({ color: 0x14161c });

  // Adds a part at (x, y, z) in its parent's frame, with CAD-style dark edges.
  const part = (parent, geometry, material, x = 0, y = 0, z = 0) => {
    const mesh = new THREE.Mesh(geometry, material);
    mesh.position.set(x, y, z);
    if (!ghost) mesh.add(new THREE.LineSegments(new THREE.EdgesGeometry(geometry, 30), outline));
    parent.add(mesh);
    return mesh;
  };
  const drum = (radius, length, axis, topRadius = radius) => {
    const g = new THREE.CylinderGeometry(topRadius, radius, length, 48);
    if (axis === 'z') g.rotateX(Math.PI / 2);
    if (axis === 'x') g.rotateZ(-Math.PI / 2);
    return g;
  };
  const box = (x, y, z) => new THREE.BoxGeometry(x, y, z);

  const R = 44;                               // drive housing radius
  const side = 72;                            // the upper arm sits this far to the side
  const root = new THREE.Group();

  // Fixed base: flared foot, then the base drive as a stack of rings.
  const neck = Math.max(geo.h - R - 4, 40);   // height where the shoulder housing starts
  const foot = Math.min(60, neck * 0.5);
  part(root, drum(80, foot, 'z', 52), dark, 0, 0, foot / 2);
  const ring = (neck - foot) / 3;
  part(root, drum(52, ring, 'z'), shell, 0, 0, foot + ring * 0.5);

  const yaw = new THREE.Group();
  root.add(yaw);
  part(yaw, drum(54, ring, 'z'), dark, 0, 0, foot + ring * 1.5);
  part(yaw, drum(52, ring, 'z'), shell, 0, 0, foot + ring * 2.5);

  // Shoulder drive: a drum lying across the top of the base, on a saddle.
  part(yaw, box(R * 1.7, 104, R), dark, geo.a1, -6, geo.h - R * 0.6);
  part(yaw, drum(R, 122, 'y'), shell, geo.a1, -6, geo.h);
  part(yaw, drum(R + 2, 8, 'y'), dark, geo.a1, 58, geo.h);

  // Upper arm, pivoting about the shoulder axis; its local +X runs toward the elbow.
  const shoulder = new THREE.Group();
  shoulder.position.set(geo.a1, 0, geo.h);
  yaw.add(shoulder);
  const hub = 78, clamp = 96;                 // lengths of the wide parts at each end
  part(shoulder, drum(R - 2, 24, 'y'), shell, 0, side, 0);
  part(shoulder, box(hub, 24, 2 * R - 4), shell, hub / 2, side, 0);
  const bar = Math.max(geo.l1 - hub - clamp + 30, 10);
  part(shoulder, box(bar, 20, 34), dark, hub + bar / 2, side, -8);
  part(shoulder, box(clamp, 26, 2 * R - 6), shell, geo.l1 - clamp / 2 + 30, side - 2, 0);

  // Elbow drive and forearm, pivoting about the elbow axis.
  const elbow = new THREE.Group();
  elbow.position.x = geo.l1;
  shoulder.add(elbow);
  part(elbow, drum(R - 2, 118, 'y'), shell, 0, 0, 0);
  part(elbow, drum(R, 8, 'y'), dark, 0, 55, 0);
  part(elbow, box(64, 40, 44), dark, R + 12, geo.d, 0);            // forearm clamp
  for (const up of [1, -1]) {                                      // gussets above and below it
    const tri = new THREE.Shape([
      new THREE.Vector2(R - 8, up * 20), new THREE.Vector2(R + 100, up * 11), new THREE.Vector2(R - 8, up * R),
    ]);
    const g = new THREE.ExtrudeGeometry(tri, { depth: 5, bevelEnabled: false });
    g.rotateX(Math.PI / 2);                   // shape plane -> the arm's X-Z plane
    part(elbow, g, dark, 0, geo.d + 2.5, 0);
  }
  const tube = Math.max(geo.l2 - 30, 10);
  part(elbow, box(tube, 22, 22), shell, 30 + tube / 2, geo.d, 0);

  const ledMaterial = new THREE.MeshBasicMaterial({ color: 0x222222 });
  const led = new THREE.Mesh(new THREE.SphereGeometry(8, 20, 14), ghost ? shell : ledMaterial);
  led.position.set(geo.l2, geo.d, 0);
  elbow.add(led);
  const glow = new THREE.Sprite(new THREE.SpriteMaterial({
    map: glowTexture(), blending: THREE.AdditiveBlending, depthWrite: false, transparent: true,
  }));
  glow.scale.set(110, 110, 1);
  glow.visible = false;
  if (!ghost) led.add(glow);

  return {
    root,
    setPose(q) {
      yaw.rotation.z = rad(q[0]);
      shoulder.rotation.y = -rad(q[1]);
      elbow.rotation.y = -rad(q[2]);
    },
    setLed(rgb) {
      const on = rgb[0] + rgb[1] + rgb[2] > 0;
      glow.visible = on;
      if (on) {
        const peak = Math.max(...rgb);
        // Show the hue at full strength; the glow's opacity carries the brightness.
        glow.material.color.setRGB(rgb[0] / peak, rgb[1] / peak, rgb[2] / peak, THREE.SRGBColorSpace);
        glow.material.opacity = 0.35 + 0.65 * peak / 255;
        ledMaterial.color.copy(glow.material.color);
      } else {
        ledMaterial.color.set(0x222222);
      }
    },
  };
}

let liveArm, ghostArm, armConfig;

// ------------------------------------------------------------------ job preview

let job = null;            // the plan returned by the server
let jobObjects = [];
let brightLines = null;    // the part of the drawing done up to the playhead
let litBefore = null;      // litBefore[k] = lit segments completed by sample k
let playhead = 0, playing = false, playClock = 0;
let liveRun = false;       // true while the real arm is drawing this job
let travelLines = null;

// Wide lines. They are drawn opaque: the short segments overlap at every joint, so any
// blending would build up into bright blobs. `gain` scales the colour instead.
function makeLines(positions, colors, width, gain) {
  const geo = new LineSegmentsGeometry();
  geo.setPositions(positions);
  geo.setColors(colors.map((c) => c * gain));
  const mat = new LineMaterial({ vertexColors: true, linewidth: width });
  mat.resolution.set(canvas.clientWidth, canvas.clientHeight);
  lineMaterials.push(mat);
  return new LineSegments2(geo, mat);
}

function basicLines(positions, color, opacity) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  return new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ color, transparent: true, opacity }));
}

function clearJob() {
  for (const obj of jobObjects) {
    scene.remove(obj);
    obj.geometry.dispose();
    const i = lineMaterials.indexOf(obj.material);
    if (i >= 0) lineMaterials.splice(i, 1);
    obj.material.dispose();
  }
  jobObjects = [];
  brightLines = null;
  travelLines = null;
}

function showJob(next) {
  clearJob();
  job = next;
  const { tip, path } = job;
  const count = path.length;
  const palette = job.stroke_rgb.map(
    ([r, g, b]) => new THREE.Color().setRGB(r / 255, g / 255, b / 255, THREE.SRGBColorSpace));
  const lit = [], litColors = [], dark = [];
  litBefore = new Uint32Array(count);
  for (let k = 1; k < count; k++) {
    const seg = [tip[3 * k - 3], tip[3 * k - 2], tip[3 * k - 1], tip[3 * k], tip[3 * k + 1], tip[3 * k + 2]];
    if (path[k] >= 0) {
      const c = palette[path[k]];
      lit.push(...seg);
      litColors.push(c.r, c.g, c.b, c.r, c.g, c.b);
    } else {
      dark.push(...seg);
    }
    litBefore[k] = lit.length / 6;
  }
  if (lit.length) {
    const faint = makeLines(lit, litColors, 1.5, 0.22);
    brightLines = makeLines(lit, litColors, 3.5, 1.0);
    jobObjects.push(faint, brightLines);
  }
  if (dark.length) {
    travelLines = basicLines(dark, 0x8b92a5, 0.22);
    travelLines.visible = $('show_travel').checked;
    jobObjects.push(travelLines);
  }
  const red = [];
  for (const line of job.unreachable) {
    for (let i = 3; i < line.length; i += 3) red.push(...line.slice(i - 3, i + 3));
  }
  if (red.length) jobObjects.push(basicLines(red, 0xff4040, 0.9));
  for (const obj of jobObjects) scene.add(obj);

  $('scrub').max = count - 1;
  playing = false;
  setPlayhead(count - 1);
  showStats();
}

function setPlayhead(k) {
  if (!job) return;
  playhead = Math.max(0, Math.min(job.path.length - 1, Math.round(k)));
  if (brightLines) brightLines.geometry.instanceCount = litBefore[playhead];
  $('scrub').value = playhead;
  const total = (job.path.length - 1) * job.dt;
  $('clock').textContent = `${clock(playhead * job.dt)} / ${clock(total)}`;
  const atEnd = playhead === job.path.length - 1;
  ghostArm.root.visible = !liveRun && !atEnd;
  ghostArm.setPose([job.q[3 * playhead], job.q[3 * playhead + 1], job.q[3 * playhead + 2]]);
  $('play').textContent = playing ? '❚❚' : '▶';
}

function clock(seconds) {
  const s = Math.round(seconds);
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
}

function showStats() {
  const s = job.stats, src = job.source;
  const rows = [
    ['Source', `${src.strokes} strokes from ${src.kind}`],
    ['Lines drawn', `${(s.draw_mm / 1000).toFixed(2)} m in ${s.strokes} strokes`],
    ['Drawing time', clock(s.draw_s)],
    ['Dark travel', clock(s.travel_s)],
    ['Exposure needed', clock(s.total_s)],
  ];
  $('stats').innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
  const warnings = [];
  if (s.unreachable_mm > 0) {
    const share = 100 * s.unreachable_mm / (s.unreachable_mm + s.draw_mm);
    warnings.push(`${share.toFixed(0)}% of the drawing is out of reach (red) and will be skipped. ` +
                  'Move or shrink it, or press Auto-fit.');
  }
  if (s.below_table) warnings.push('A travel move dips below the minimum tip height.');
  $('warn').hidden = warnings.length === 0;
  $('warn').textContent = warnings.join(' ');
}

// ------------------------------------------------------------------ inputs -> plan

const SOURCE_IDS = ['feature_angle', 'image_mode', 'detail', 'invert'];
const PLACE_IDS = ['size', 'x', 'y', 'z', 'rx', 'ry', 'rz'];
const LIGHT_IDS = ['mode', 'color', 'speed', 'brightness', 'even_exposure', 'return_home'];
const value = (id) => {
  const el = $(id);
  if (el.type === 'checkbox') return el.checked;
  return el.type === 'number' || el.type === 'range' ? Number(el.value) : el.value;
};
const collect = (ids) => Object.fromEntries(ids.map((id) => [id, value(id)]));
const settings = () => ({ source: collect(SOURCE_IDS), placement: collect(PLACE_IDS), light: collect(LIGHT_IDS) });

let loaded = false, planTimer, planSerial = 0;
function showOutputs() {
  $('feature_angle_out').textContent = `${value('feature_angle')}°`;
  $('detail_out').textContent = `${Math.round(value('detail') * 100)}%`;
  $('speed_out').textContent = `${value('speed')} mm/s`;
  $('brightness_out').textContent = `${Math.round(value('brightness') * 100)}%`;
  $('color').style.visibility = value('mode') === 'single' ? 'visible' : 'hidden';
}

async function replan() {
  if (!loaded) return;
  const serial = ++planSerial;
  const next = await api('/api/plan', settings());
  if (serial === planSerial) showJob(next);       // ignore answers to superseded requests
}
function schedulePlan() {
  showOutputs();
  clearTimeout(planTimer);
  planTimer = setTimeout(guard(replan), 200);
}

async function autofit() {
  if (!loaded) return;
  const fit = await api('/api/autofit', settings());
  for (const id of PLACE_IDS) $(id).value = fit[id];
  await replan();
}

async function loadFile(name, data) {
  const info = await api(`/api/load?name=${encodeURIComponent(name)}`, null, data);
  loaded = true;
  $('src-name').textContent = name;
  $('mesh-opts').hidden = info.kind !== 'mesh';
  $('image-opts').hidden = info.kind !== 'image';
  $('mode').value = info.kind === 'image' ? 'source' : 'palette';
  if (info.image_mode) $('image_mode').value = info.image_mode;
  $('mode').querySelector('[value=source]').disabled = info.kind !== 'image';
  // A model looks best tilted so no edges hide behind each other; an image faces the camera.
  $('rx').value = info.kind === 'mesh' ? 20 : 0;
  $('ry').value = 0;
  $('rz').value = info.kind === 'mesh' ? 30 : 0;
  showOutputs();
  await autofit();
}

for (const id of [...SOURCE_IDS, ...PLACE_IDS, ...LIGHT_IDS]) $(id).addEventListener('input', schedulePlan);
$('autofit').onclick = guard(autofit);
$('pick').onclick = () => $('file').click();
$('file').onchange = guard(async () => {
  const f = $('file').files[0];
  if (f) await loadFile(f.name, await f.arrayBuffer());
});
const drop = $('drop');
window.addEventListener('dragover', (e) => { e.preventDefault(); drop.classList.add('over'); });
window.addEventListener('dragleave', () => drop.classList.remove('over'));
window.addEventListener('drop', guard(async (e) => {
  e.preventDefault();
  drop.classList.remove('over');
  const f = e.dataTransfer.files[0];
  if (f) await loadFile(f.name, await f.arrayBuffer());
}));
for (const link of document.querySelectorAll('[data-example]')) {
  link.onclick = guard(async (e) => {
    e.preventDefault();
    const name = link.dataset.example;
    await loadFile(name, await (await fetch(`/examples/${name}`)).arrayBuffer());
  });
}

// ------------------------------------------------------------------ preview playback

$('scrub').oninput = () => { playing = false; setPlayhead(Number($('scrub').value)); };
$('play').onclick = () => {
  if (!job || liveRun) return;
  playing = !playing;
  if (playing && playhead >= job.path.length - 1) playhead = 0;
  playClock = playhead * job.dt;
  setPlayhead(playhead);
};
$('show_travel').onchange = () => { if (travelLines) travelLines.visible = $('show_travel').checked; };

// ------------------------------------------------------------------ arm controls

async function refreshPorts() {
  const ports = await (await fetch('/api/ports')).json();
  const keep = $('port').value;
  $('port').innerHTML = ports.map((p) => `<option value="${p.port}">${p.label}</option>`).join('');
  if (ports.some((p) => p.port === keep)) $('port').value = keep;
}
$('refresh').onclick = guard(refreshPorts);
$('connect').onclick = guard(async () => {
  if (status.connected) await api('/api/disconnect');
  else await api('/api/connect', { port: $('port').value });
});
$('home').onclick = guard(() => api('/api/home'));
$('home_marks').onclick = guard(() => api('/api/home_marks'));
$('park').onclick = guard(() => api('/api/park'));
$('motors').onclick = guard(() => api('/api/motors', { on: !status.motors }));
$('resync').onclick = guard(() => api('/api/resync'));
$('save_marks').onclick = guard(async (e) => {
  e.preventDefault();
  if (confirm('Save the marks?\n\nOnly do this with every joint sitting exactly on its home mark. ' +
              'The encoder angles at this pose are stored on the ESP32 and used by "Home at marks" from now on.')) {
    await api('/api/save_marks');
  }
});
$('run').onclick = guard(() => { playing = false; return api('/api/run', { hold_s: value('hold') }); });
$('stop').onclick = guard(() => api('/api/stop'));

function buildJog() {
  const button = (i, d) => `<button data-joint="${i}" data-delta="${d}">${d > 0 ? '+' : ''}${d}°</button>`;
  $('jog').innerHTML = armConfig.joints.map((j, i) =>
    `<span>${j.name}</span>${button(i, -10)}${button(i, -1)}` +
    `<span class="angle" id="angle${i}">–</span>${button(i, 1)}${button(i, 10)}`).join('');
  for (const b of $('jog').querySelectorAll('button')) {
    b.onclick = guard(() => api('/api/jog', { joint: Number(b.dataset.joint), delta: Number(b.dataset.delta) }));
  }
}

// ------------------------------------------------------------------ live status

let status = { connected: false, run: { state: 'idle' } };

function chip(text, kind = '') { return `<span class="chip ${kind}">${text}</span>`; }

function onStatus(next) {
  status = next;
  const running = next.run.state === 'running';
  const ready = next.connected && next.online && next.homed && next.state === 'idle' && !running;
  $('connect').textContent = next.connected ? 'Disconnect' : 'Connect';
  $('port').disabled = next.connected;
  const idle = next.connected && next.online && !running && next.state === 'idle';
  $('home').disabled = !idle || !next.motors;
  $('home_marks').disabled = !idle;
  $('motors').disabled = !idle;
  $('motors').textContent = next.connected && !next.motors ? 'Hold motors' : 'Release motors';
  $('park').disabled = !ready || !next.motors;
  $('run').disabled = !ready || !next.motors || !job || job.stats.strokes === 0;
  $('stop').disabled = !next.connected;
  // Jogging works before homing too (slowly, without limits) so joints can be put on their marks.
  for (const b of $('jog').querySelectorAll('button')) b.disabled = !idle || !next.motors;
  const slip = next.connected ? next.slip : [null, null, null];
  $('slip').innerHTML = slip.map((d, i) =>
    `<span class="${next.connected && next.slipped[i] ? 'bad' : ''}">${armConfig.joints[i].name[0].toUpperCase()} ` +
    `${d === null ? '–' : `${d > 0 ? '+' : ''}${d.toFixed(2)}°`}</span>`).join('');
  $('resync').disabled = !idle || !slip.some((d) => d !== null);
  $('scrub').disabled = running;
  $('play').disabled = running;

  const chips = [];
  if (!next.connected) {
    chips.push(chip('not connected'));
  } else {
    chips.push(chip(next.device, next.online ? 'ok' : 'bad'));
    if (!next.online) chips.push(chip('no response', 'bad'));
    chips.push(next.homed ? chip('homed', 'ok') : chip('not homed', 'bad'));
    if (next.state === 'home') chips.push(chip('moving…', 'busy'));
    if (!next.motors) chips.push(chip('motors released', 'busy'));
    if (next.slipped.some(Boolean)) {
      const names = armConfig.joints.filter((_, i) => next.slipped[i]).map((j) => j.name);
      chips.push(chip(`missed steps: ${names.join(', ')}`, 'bad'));
    }
    if (running) chips.push(chip(`${next.run.label} ${Math.round(next.run.progress * 100)}%`, 'busy'));
    if (next.error) chips.push(chip(next.error, 'bad'));
  }
  $('chips').innerHTML = chips.join('');
  $('progress').firstElementChild.style.width = `${running ? next.run.progress * 100 : 0}%`;
  $('message').textContent = next.run.message || '';
  $('log').textContent = (next.log || []).join('\n');

  liveArm.root.visible = next.connected;
  if (next.connected) {
    liveArm.setPose(next.q);
    liveArm.setLed(next.rgb);
    next.q.forEach((angle, i) => { $(`angle${i}`).textContent = `${angle.toFixed(1)}°`; });
  }

  // While the arm draws, the playhead follows what it has actually finished.
  const wasLive = liveRun;
  liveRun = running && next.run.k !== null;
  if (liveRun) setPlayhead(next.run.k);
  else if (wasLive && job) setPlayhead(job.path.length - 1);
}

function openSocket() {
  const ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = (e) => onStatus(JSON.parse(e.data));
  ws.onclose = () => {
    onStatus({ connected: false, run: { state: 'idle', message: 'Lost the connection to the program; is it still running?' } });
    setTimeout(openSocket, 1500);
  };
}

// ------------------------------------------------------------------ start

let lastFrame = performance.now();
function frame(now) {
  const elapsed = (now - lastFrame) / 1000;
  lastFrame = now;
  if (playing && job && !liveRun) {
    playClock += elapsed * Number($('rate').value);
    const k = playClock / job.dt;
    if (k >= job.path.length - 1) playing = false;
    setPlayhead(k);
  }
  controls.update();
  renderer.render(scene, camera);
  requestAnimationFrame(frame);
}

async function start() {
  armConfig = await (await fetch('/api/config')).json();
  liveArm = buildArm(armConfig.geometry, false);
  ghostArm = buildArm(armConfig.geometry, true);
  liveArm.setPose(armConfig.joints.map((j) => j.home));
  ghostArm.root.visible = false;
  scene.add(liveArm.root, ghostArm.root);
  buildJog();
  showOutputs();
  resize();
  await refreshPorts();
  openSocket();
  requestAnimationFrame(frame);
  // http://127.0.0.1:8765/?example=octahedron.obj opens with that example loaded
  const example = new URLSearchParams(location.search).get('example');
  if (example) await loadFile(example, await (await fetch(`/examples/${example}`)).arrayBuffer());
}
start().catch((err) => toast(err.message));
