import * as THREE from 'three';
import { flightScene, setFlightCamera } from './scene.mjs';

const query = new URLSearchParams(location.search);
const width = +(query.get('w') || innerWidth), height = +(query.get('h') || innerHeight);
try {
const labels = [];
function label(text, pos, color = '#405c55', scale = 1) {
  const c = document.createElement('canvas'); c.width = 512; c.height = 128;
  const ctx = c.getContext('2d'); ctx.fillStyle = color; ctx.font = '600 48px system-ui'; ctx.textAlign = 'center';
  ctx.fillText(text, 256, 77);
  const sprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: new THREE.CanvasTexture(c), depthWrite: false }));
  sprite.position.set(...pos); sprite.scale.set(1.6 * scale, 0.4 * scale, 1); labels.push(sprite);
}
const { scene, drone } = flightScene(label);
for (const sprite of labels) scene.add(sprite);
const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
renderer.setSize(width, height); renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.outputColorSpace = THREE.SRGBColorSpace; renderer.setClearColor('#f3f1ec');
renderer.domElement.setAttribute('aria-label', 'Three dimensional replay of a drone flying through gates');
document.body.prepend(renderer.domElement);
const camera = new THREE.PerspectiveCamera(43, width / height, 0.05, 100);

  const trackPath = query.get('track');
  if (!trackPath) throw new Error('Choose a recorded track: ?track=/tmp/codex-drone/tracks/seed-101.json&trail=1');
  const response = await fetch(trackPath);
  if (!response.ok) throw new Error(`Track request returned ${response.status}`);
  const track = await response.json();
  if (!Array.isArray(track.frames) || !track.frames.length) throw new Error('Track contains no flight frames');
  const { frames, result } = track, end = frames.at(-1).t, passColor = result.success ? '#2c9a70' : '#c65c46';
  if (query.get('hud') === '0') for (const id of ['panel', 'controls']) { const el = document.getElementById(id); if (el) el.style.display = 'none'; }
  if (query.get('hud') === 'min') { const el = document.getElementById('panel'); if (el) el.style.display = 'none'; }
  document.querySelector('#status').textContent = result.success ? 'PASS · three gates and a soft landing' : `FAIL · ${result.reason}`;
  document.querySelector('#status').style.color = passColor;
  document.querySelector('#detail').textContent = `Seed ${track.seed} · ${result.gatesPassed}/3 gates · ${end.toFixed(2)} s flight\nLanding error ${result.landingError.toFixed(3)} m. Simplified flight dynamics.`;
  document.querySelector('#trail-dot').style.background = passColor;
  document.querySelector('#trail-label').textContent = result.success ? 'Successful flight path' : 'Failed flight path';
  const pathGeometry = new THREE.BufferGeometry().setFromPoints(frames.map((f) => new THREE.Vector3(f.pos.x, f.pos.y, f.pos.z)));
  const trail = new THREE.Line(pathGeometry, new THREE.LineBasicMaterial({ color: passColor }));
  trail.visible = query.has('trail'); scene.add(trail);
  // Periodic markers make trajectory depth legible in still images.
  if (trail.visible) for (let i = 0; i < frames.length; i += 15) {
    const point = new THREE.Mesh(new THREE.SphereGeometry(0.016, 8, 6), new THREE.MeshStandardMaterial({ color: passColor, roughness: 0.75 }));
    point.position.copy(new THREE.Vector3(frames[i].pos.x, frames[i].pos.y, frames[i].pos.z)); scene.add(point);
  }
  const scrub = document.querySelector('#scrub'), playButton = document.querySelector('#play'), select = document.querySelector('#camera');
  scrub.max = end;
  let playing = query.has('play'), time = query.get('t') === 'end' || !query.has('t') ? end : Math.max(0, Math.min(end, +query.get('t')));
  if (playing) time = 0;
  let cameraName = query.get('cam') || 'overview';
  select.value = ['overview', 'chase', 'low', 'top'].includes(cameraName) ? cameraName : 'overview';
  select.onchange = () => { cameraName = select.value; render(); };
  playButton.textContent = playing ? 'Pause' : 'Play';
  playButton.onclick = () => { if (time >= end) time = 0; playing = !playing; playButton.textContent = playing ? 'Pause' : 'Play'; };
  scrub.oninput = () => { playing = false; time = +scrub.value; playButton.textContent = 'Play'; render(); };
  function render() {
    let i = 0;
    while (i + 1 < frames.length && frames[i + 1].t <= time) i++;
    const a = frames[i], b = frames[Math.min(i + 1, frames.length - 1)], u = b.t > a.t ? (time - a.t) / (b.t - a.t) : 0;
    const mix = (x, y) => x + (y - x) * u;
    drone.position.set(mix(a.pos.x, b.pos.x), mix(a.pos.y, b.pos.y), mix(a.pos.z, b.pos.z));
    drone.quaternion.setFromEuler(new THREE.Euler(-mix(a.att.roll, b.att.roll), mix(a.att.yaw, b.att.yaw), -mix(a.att.pitch, b.att.pitch), 'YZX'));
    setFlightCamera(camera, cameraName, drone);
    pathGeometry.setDrawRange(0, query.has('play') ? i + 1 : frames.length);
    scrub.value = time; document.querySelector('#clock').textContent = `${time.toFixed(2)} / ${end.toFixed(2)} s`;
    renderer.render(scene, camera);
    window.__frame = { t: time, gateIndex: a.gateIndex };
  }
  let previous = performance.now();
  function tick(now) {
    if (playing) {
      time = Math.min(end, time + Math.min((now - previous) / 1000, 0.1));
      if (time >= end) { playing = false; playButton.textContent = 'Play'; window.__done = true; }
      render();
    }
    previous = now; requestAnimationFrame(tick);
  }
  render(); window.__result = result; window.__ready = true;
  requestAnimationFrame(tick);
} catch (error) {
  window.__error = error.message;
  const panel = document.querySelector('#error'); panel.style.display = 'block'; panel.textContent = error.message;
}
