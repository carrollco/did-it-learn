import * as THREE from '../../harness/node_modules/three/build/three.module.js';
import { TASK, GATES, TAKEOFF, PAD } from './task.mjs';

export function flightScene(label = () => {}) {
const scene = new THREE.Scene(); scene.background = new THREE.Color('#f3f1ec');
scene.add(new THREE.HemisphereLight('#ffffff', '#c5bda9', 2.4));
const sun = new THREE.DirectionalLight('#fff9e8', 3.0); sun.position.set(-3, 8, 3); sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048); sun.shadow.camera.left = -6; sun.shadow.camera.right = 6;
sun.shadow.camera.top = 6; sun.shadow.camera.bottom = -6; sun.shadow.normalBias = 0.015;
scene.add(sun);
const material = (color, other = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.75, ...other });
const floorMat = material('#e7e6dc'), frameMat = material('#d39a4f'), padMat = material('#72b7a3');
function box(size, pos, mat, parent = scene) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(...size), mat);
  mesh.position.set(...pos); mesh.castShadow = true; mesh.receiveShadow = true; parent.add(mesh); return mesh;
}
box([6.05, 0.07, 6.05], [0, -0.045, 0], floorMat);
const grid = new THREE.GridHelper(6, 12, '#c4c9bd', '#d1d5c9'); grid.position.y = 0.002; scene.add(grid);
const wallMat = material('#dde3da', { transparent: true, opacity: 0.24, depthWrite: false, side: THREE.DoubleSide });
box([0.025, 3, 6], [-3.02, 1.5, 0], wallMat); box([6, 3, 0.025], [0, 1.5, -3.02], wallMat);
const boundaryMat = new THREE.LineBasicMaterial({ color: '#acb8aa', transparent: true, opacity: 0.6 });
const edge = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(6, 3, 6)), boundaryMat);
edge.position.y = 1.5; scene.add(edge);


for (let i = 0; i < GATES.length; i++) {
  const { x, y, z } = GATES[i].centre, h = TASK.gateOpening / 2, f = TASK.gateFrame;
  box([0.065, f, 2 * h + 2 * f], [x, y + h + f / 2, z], frameMat);
  box([0.065, f, 2 * h + 2 * f], [x, y - h - f / 2, z], frameMat);
  for (const sign of [-1, 1]) {
    box([0.065, 2 * h, f], [x, y, z + sign * (h + f / 2)], frameMat);
    box([0.035, Math.max(0.01, y - h - f), 0.035], [x, (y - h - f) / 2, z + sign * (h + f / 2)], material('#b7b7a3'));
  }
  label('GATE ' + (i + 1), [x, y + h + 0.26, z], '#8b632e', 0.65);
}
function pad(pos, text) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(0.28, 0.28, 0.015, 48), padMat);
  mesh.position.set(pos.x, 0.012, pos.z); mesh.receiveShadow = true; scene.add(mesh);
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.14, 0.153, 64), new THREE.MeshBasicMaterial({ color: '#f7fbf0', side: THREE.DoubleSide }));
  ring.rotation.x = -Math.PI / 2; ring.position.set(pos.x, 0.022, pos.z); scene.add(ring);
  label(text, [pos.x, 0.20, pos.z + 0.46], '#417365', 0.65);
}
pad(TAKEOFF, 'TAKE-OFF'); pad(PAD.centre, 'LAND');
const drone = new THREE.Group(); scene.add(drone);
box([0.13, 0.065, 0.10], [0, 0, 0], material('#f7faf4'), drone);
box([0.055, 0.018, 0.055], [0, 0.045, 0], material('#d08c49'), drone);
box([0.27, 0.018, 0.025], [0, 0.0, 0], material('#596c65'), drone);
box([0.025, 0.018, 0.27], [0, 0.0, 0], material('#596c65'), drone);
const rotors = [];
for (const x of [-TASK.arm, TASK.arm]) for (const z of [-TASK.arm, TASK.arm]) {
  const arm = box([Math.hypot(x, z), 0.016, 0.017], [x / 2, 0, z / 2], material('#71867b'), drone);
  arm.rotation.y = -Math.atan2(z, x);
  const hub = new THREE.Mesh(new THREE.CylinderGeometry(0.024, 0.024, 0.035, 12), material('#789382'));
  hub.position.set(x, 0.022, z); drone.add(hub);
  const rotor = new THREE.Mesh(new THREE.CylinderGeometry(0.056, 0.056, 0.003, 32), material('#8fa99a', { transparent: true, opacity: 0.6 }));
  rotor.position.set(x, 0.044, z); drone.add(rotor); rotors.push(rotor);
}
for (const z of [-0.055, 0.055]) box([0.15, 0.012, 0.012], [0, -0.047, z], material('#596c65'), drone);

return { scene, drone };
}

export function setFlightCamera(camera, name, drone) {
  if (name === 'overview') { camera.position.set(6.8, 5.8, -7.8); camera.lookAt(0, 0.7, 0); }
  else if (name === 'low') { camera.position.set(6, 1.55, 5.5); camera.lookAt(0, 1, 0); }
  else if (name === 'top') { camera.position.set(0.05, 10.3, 0.05); camera.lookAt(0, 0, 0); }
  else if (name === 'chase') { camera.position.copy(drone.position).add(new THREE.Vector3(-2.8, 1.8, 2.8)); camera.lookAt(drone.position.clone().add(new THREE.Vector3(0.45, 0, 0))); }
  else {
    const values = name.split(',').map(Number);
    if (values.length !== 3 || !values.every(Number.isFinite) || values[2] <= 0) throw new Error('camera must be overview, chase, low, top, or azimuth,elevation,distance');
    const [az, el, distance] = values, a = az * Math.PI / 180, e = el * Math.PI / 180;
    camera.position.set(distance * Math.cos(e) * Math.cos(a), 0.9 + distance * Math.sin(e), distance * Math.cos(e) * Math.sin(a)); camera.lookAt(0, 0.9, 0);
  }
}
