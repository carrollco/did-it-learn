// Browser viewer: rebuilds the scene, re-runs the same trial the judge ran, and renders it.
// URL: /harness/view.html?scene=/scenes/x.js&seed=1&cam=overview | cam=az,el,dist &t=end|seconds &play=1 &w=1280&h=720
import * as THREE from 'three';
import * as CANNON from 'cannon-es';
import { TASK, VAL_CAMERAS } from './task.mjs';
import { checkSource } from './world.mjs';
import { runTrial } from './trialrun.mjs';

const q = new URLSearchParams(location.search);
const W = +(q.get('w') || 1280), H = +(q.get('h') || 720);

export function cameraFor(spec) {
  const named = VAL_CAMERAS.find((c) => c.name === spec);
  if (named) return named;
  const [az, el, dist] = String(spec).split(',').map(Number); // degrees, degrees, metres around the desk centre
  const a = az * Math.PI / 180, e = el * Math.PI / 180;
  return { name: spec, position: [dist * Math.cos(e) * Math.sin(a), 0.2 + dist * Math.sin(e), dist * Math.cos(e) * Math.cos(a)], target: [0, 0.2, 0] };
}

async function main() {
  const sceneUrl = q.get('scene');
  const src = await (await fetch(sceneUrl, { cache: 'no-store' })).text();
  checkSource(src);
  const build = (await import(sceneUrl + '?v=' + Date.now())).default;
  const seed = +(q.get('seed') || 1);
  const bump = q.get('bump') ? JSON.parse(q.get('bump')) : null;
  const { result, track, world } = runTrial({ THREE, CANNON, build, seed, graphics: true, record: true, bumpOverride: bump });
  window.__result = result;

  const renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true });
  renderer.setSize(W, H); renderer.setPixelRatio(1);
  renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  document.body.appendChild(renderer.domElement);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color('#f3f1ec');
  scene.add(new THREE.HemisphereLight('#ffffff', '#c8b89a', 1.6));
  const sun = new THREE.DirectionalLight('#fff4e0', 2.2);
  sun.position.set(1.2, 2.5, 1.0); sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048); Object.assign(sun.shadow.camera, { left: -1.5, right: 1.5, top: 1.5, bottom: -1.5 });
  scene.add(sun);
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(8, 8), new THREE.MeshStandardMaterial({ color: '#dcd6cc' }));
  floor.rotation.x = -Math.PI / 2; floor.position.y = -0.75; floor.receiveShadow = true; scene.add(floor);
  scene.add(world.root);

  const cam = cameraFor(q.get('cam') || 'overview');
  const camera = new THREE.PerspectiveCamera(45, W / H, 0.01, 50);
  camera.position.set(...cam.position); camera.lookAt(new THREE.Vector3(...cam.target));

  const apply = (k) => {
    const s = track[Math.max(0, Math.min(track.length - 1, k))];
    world.marbleMesh.position.set(...s.p); world.marbleMesh.quaternion.set(...s.q);
    world.movers.forEach((m, i) => { if (m.mesh && s.movers[i]) { m.mesh.position.set(...s.movers[i].p); m.mesh.quaternion.set(...s.movers[i].q); } });
  };
  if (q.get('play')) {
    const t0 = performance.now();
    const tick = () => {
      const t = (performance.now() - t0) / 1000;
      const k = track.findIndex((s) => s.t >= t);
      apply(k < 0 ? track.length - 1 : k);
      renderer.render(scene, camera);
      if (k < 0 && t > track[track.length - 1].t + 0.8) { window.__done = true; return; }
      requestAnimationFrame(tick);
    };
    tick();
  } else {
    const at = q.get('t');
    const k = !at || at === 'end' ? track.length - 1 : track.findIndex((s) => s.t >= +at);
    apply(k < 0 ? track.length - 1 : k);
    renderer.render(scene, camera);
  }
  window.__ready = true;
}
main().catch((e) => { window.__error = String(e && e.message || e); document.body.textContent = 'ERROR: ' + window.__error; });
