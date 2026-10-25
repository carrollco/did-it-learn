// Two halves, kept apart on purpose.
// describeScene: runs a model's scene code (in the judge, inside a locked-down child process) and records what it
//   asks for: static pieces, paddle sizes, and paddle path functions that stay in that process.
// buildWorld: trusted. Checks a recorded description against the budget and builds the physics (and graphics).
// The model's code never runs in the process that knows which trial is being run.
import { TASK, rng } from './task.mjs';

export class SceneError extends Error {}

const vec = (v, n, what) => {
  if (!Array.isArray(v) || v.length !== n || !v.every((x) => typeof x === 'number' && Number.isFinite(x)))
    throw new SceneError(`${what} must be an array of ${n} finite numbers`);
  return v;
};

// Source rules for scene files: one default export, no imports, no reaching outside the arguments it is given.
// The process boundary is the real barrier; this filter is a second one and gives readable errors.
const FORBIDDEN = [/^\s*import\s/m, /\bimport\s*\(/, /\brequire\b/, /\bglobalThis\b/, /\bwindow\b/, /\bdocument\b/, /\bself\b/,
  /\bprocess\b/, /\bfetch\b/, /\beval\b/, /\bFunction\b/, /\bconstructor\b/, /\bprototype\b/, /__proto__/, /\bReflect\b/,
  /\bProxy\b/, /\bDate\b/, /\bperformance\b/, /\bsetTimeout\b/, /\bsetInterval\b/, /\bWebAssembly\b/, /\bAtomics\b/];

export function checkSource(src) {
  for (const re of FORBIDDEN) if (re.test(src)) throw new SceneError(`scene source uses a forbidden construct: ${re}`);
  if (!/export\s+default\s+function/.test(src)) throw new SceneError('scene must `export default function build({ THREE, add, task })`');
}

const deepFreeze = (o) => { Object.values(o).forEach((v) => v && typeof v === 'object' && deepFreeze(v)); return Object.freeze(o); };
const num = (x) => +(+x).toFixed(6);

// Run the scene's build with a recording API. Returns plain data plus the paddle path functions.
export function describeScene({ THREE, build, keepDecor = false }) {
  const pieces = [], movers = [], paths = [], decor = [];
  const add = Object.freeze({
    box({ size, position, rotation = [0, 0, 0], color = '#b8b8b8' } = {}) {
      pieces.push({ kind: 'box', size: vec(size, 3, 'box size').map(num), position: vec(position, 3, 'box position').map(num),
        rotation: vec(rotation, 3, 'box rotation').map(num), color: String(color).slice(0, 32) });
    },
    cylinder({ radius, height, position, rotation = [0, 0, 0], color = '#b8b8b8' } = {}) {
      pieces.push({ kind: 'cylinder', size: vec([2 * radius, height, 2 * radius], 3, 'cylinder size').map(num),
        position: vec(position, 3, 'cylinder position').map(num), rotation: vec(rotation, 3, 'cylinder rotation').map(num),
        color: String(color).slice(0, 32) });
    },
    mover({ size, path, color = '#d05a3c' } = {}) {
      if (typeof path !== 'function') throw new SceneError('mover path must be a function (t, marble) => ({ position, rotation })');
      movers.push({ size: vec(size, 3, 'mover size').map(num), color: String(color).slice(0, 32) });
      paths.push(path);
    },
    decor(object3D) {
      if (!object3D || !object3D.isObject3D) throw new SceneError('decor takes a THREE.Object3D');
      if (keepDecor) decor.push(object3D);
    },
  });
  const realRandom = Math.random;
  Math.random = rng(424242);
  try { build({ THREE, add, task: deepFreeze(structuredClone(TASK)) }); }
  catch (e) { if (e instanceof SceneError) throw e; throw new SceneError(`scene build threw: ${e && e.message}`); }
  finally { Math.random = realRandom; }
  const M = TASK.marble;
  const m0 = { position: [M.start[0], M.start[1], M.start[2]], velocity: [0, 0, 0] };
  const desc = { pieces, movers: movers.map((m, i) => ({ ...m, initial: pose(paths[i], 0, m0) })) };
  return { desc, paths, decor };
}

// A paddle's pose at time t given what it sees of the marble. Runs wherever the scene code runs.
export function pose(path, t, marble) {
  let p;
  try { p = path(t, { position: [...marble.position], velocity: [...marble.velocity] }) || {}; }
  catch (e) { throw new SceneError(`mover path threw: ${e && e.message}`); }
  return { position: vec(p.position, 3, 'mover position').map(num), rotation: vec(p.rotation || [0, 0, 0], 3, 'mover rotation').map(num) };
}

// Trusted: check a description against the frozen budget. Throws SceneError with a readable reason.
export function validateDesc(desc) {
  const B = TASK.bounds, L = TASK.budget;
  const inB = (p, what) => {
    if (p[0] < B.minX || p[0] > B.maxX || p[1] < B.minY || p[1] > B.maxY || p[2] < B.minZ || p[2] > B.maxZ)
      throw new SceneError(`${what} at [${p.map((x) => x.toFixed(3))}] is outside the allowed volume`);
  };
  if (!desc || !Array.isArray(desc.pieces) || !Array.isArray(desc.movers)) throw new SceneError('malformed scene description');
  if (desc.pieces.length > L.pieces) throw new SceneError(`${desc.pieces.length} static pieces; the budget is ${L.pieces}`);
  if (desc.movers.length > L.movers) throw new SceneError(`${desc.movers.length} movers; the budget is ${L.movers}`);
  let material = 0;
  desc.pieces.forEach((p, i) => {
    if (!['box', 'cylinder'].includes(p.kind)) throw new SceneError(`piece ${i}: unknown kind`);
    vec(p.size, 3, `piece ${i} size`); vec(p.position, 3, `piece ${i} position`); vec(p.rotation, 3, `piece ${i} rotation`);
    if (p.size.some((x) => x < L.minSize || x > L.maxSize)) throw new SceneError(`piece ${i}: every side must be ${L.minSize}..${L.maxSize} m`);
    inB(p.position, `piece ${i}`);
    const s = [...p.size].sort((a, b) => b - a);
    material += s[0] * s[1];
  });
  if (material > L.material + 1e-9) throw new SceneError(`material ${material.toFixed(3)} m^2 is over the ${L.material} m^2 budget (sum of each piece's largest face)`);
  desc.movers.forEach((m, i) => {
    vec(m.size, 3, `mover ${i} size`);
    if (m.size.some((x) => x < L.minSize || x > L.moverMaxSize)) throw new SceneError(`mover ${i}: every side must be ${L.minSize}..${L.moverMaxSize} m`);
    inB(m.initial.position, `mover ${i} start`);
  });
  return { material: +material.toFixed(4) };
}

const quatOf = (THREE, rot) => new THREE.Quaternion().setFromEuler(new THREE.Euler(rot[0], rot[1], rot[2], 'XYZ'));

// Trusted: build the physics world (and meshes when graphics is on) from a validated description.
export function buildWorld({ THREE, CANNON, desc, decor = [], trial, graphics = true }) {
  const usage = validateDesc(desc);
  const world = new CANNON.World({ gravity: new CANNON.Vec3(0, -9.81, 0) });
  world.broadphase = new CANNON.SAPBroadphase(world);
  world.allowSleep = false;
  world.solver.iterations = 20;
  const marbleMat = new CANNON.Material('marble'), pieceMat = new CANNON.Material('piece');
  world.addContactMaterial(new CANNON.ContactMaterial(marbleMat, pieceMat, { friction: trial.friction, restitution: trial.restitution }));
  const root = new THREE.Group();
  const material = (color) => new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.05 });
  const place = (body, mesh, position, rotation) => {
    const q = quatOf(THREE, rotation);
    body.position.set(...position); body.quaternion.set(q.x, q.y, q.z, q.w); world.addBody(body);
    if (mesh) { mesh.position.set(...position); mesh.quaternion.copy(q); mesh.castShadow = mesh.receiveShadow = true; root.add(mesh); }
  };
  const boxBody = (s, type) => new CANNON.Body({ mass: 0, type, material: pieceMat, shape: new CANNON.Box(new CANNON.Vec3(s[0] / 2, s[1] / 2, s[2] / 2)) });

  const d = TASK.desk, deskSize = [d.maxX - d.minX, 0.04, d.maxZ - d.minZ];
  place(boxBody(deskSize, CANNON.Body.STATIC), graphics ? new THREE.Mesh(new THREE.BoxGeometry(...deskSize), material('#9a6b43')) : null, [0, -0.02, 0], [0, 0, 0]);
  const C = TASK.cup, ring = C.innerRadius + C.wallThickness / 2;
  for (let i = 0; i < C.segments; i++) {
    const a = (i / C.segments) * 2 * Math.PI, w = (2 * Math.PI * ring / C.segments) * 1.08;
    const s = [C.wallThickness, C.wallHeight, w];
    place(boxBody(s, CANNON.Body.STATIC), graphics ? new THREE.Mesh(new THREE.BoxGeometry(...s), material('#f4f1ea')) : null,
      [C.center[0] + ring * Math.cos(a), C.wallHeight / 2, C.center[2] + ring * Math.sin(a)], [0, -a, 0]);
  }
  for (const p of desc.pieces) {
    if (p.kind === 'box') {
      place(boxBody(p.size, CANNON.Body.STATIC), graphics ? new THREE.Mesh(new THREE.BoxGeometry(...p.size), material(p.color)) : null, p.position, p.rotation);
    } else {
      const r = p.size[0] / 2, h = p.size[1];
      const body = new CANNON.Body({ mass: 0, type: CANNON.Body.STATIC, material: pieceMat, shape: new CANNON.Cylinder(r, r, h, 16) });
      place(body, graphics ? new THREE.Mesh(new THREE.CylinderGeometry(r, r, h, 24), material(p.color)) : null, p.position, p.rotation);
    }
  }
  const movers = desc.movers.map((m) => {
    const body = boxBody(m.size, CANNON.Body.KINEMATIC);
    const mesh = graphics ? new THREE.Mesh(new THREE.BoxGeometry(...m.size), material(m.color)) : null;
    place(body, mesh, m.initial.position, m.initial.rotation);
    return { body, mesh };
  });
  if (graphics) decor.forEach((o) => root.add(o));

  const M = TASK.marble;
  const marble = new CANNON.Body({ mass: M.mass, shape: new CANNON.Sphere(M.radius), material: marbleMat, linearDamping: 0.01, angularDamping: 0.01 });
  marble.position.set(M.start[0] + trial.startOffset[0], M.start[1], M.start[2] + trial.startOffset[2]);
  marble.velocity.set(...trial.startVelocity);
  world.addBody(marble);
  let marbleMesh = null;
  if (graphics) {
    marbleMesh = new THREE.Mesh(new THREE.SphereGeometry(M.radius, 32, 16), new THREE.MeshStandardMaterial({ color: '#2f7fd8', roughness: 0.15, metalness: 0.3 }));
    marbleMesh.castShadow = true; root.add(marbleMesh);
  }

  // Steer each paddle toward its requested pose, no faster than the speed limits; physics moves it between ticks.
  const B = TASK.bounds, L = TASK.budget, hz = TASK.controlHz;
  const steer = (poses) => {
    movers.forEach((m, i) => {
      const p = poses[i]; if (!p) return;
      const b = m.body;
      const target = p.position.map((x, k) => Math.min([B.maxX, B.maxY, B.maxZ][k], Math.max([B.minX, B.minY, B.minZ][k], x)));
      let v = [target[0] - b.position.x, target[1] - b.position.y, target[2] - b.position.z].map((x) => x * hz);
      const sp = Math.hypot(...v); if (sp > L.moverMaxSpeed) v = v.map((x) => x * L.moverMaxSpeed / sp);
      b.velocity.set(...v);
      const q = quatOf(THREE, p.rotation);
      const cq = new THREE.Quaternion(b.quaternion.x, b.quaternion.y, b.quaternion.z, b.quaternion.w);
      const dq = q.clone().multiply(cq.clone().invert());
      if (dq.w < 0) { dq.x = -dq.x; dq.y = -dq.y; dq.z = -dq.z; dq.w = -dq.w; }
      const ang = 2 * Math.acos(Math.min(1, dq.w)), s = Math.sqrt(Math.max(1e-12, 1 - dq.w * dq.w));
      let w = ang > 1e-6 ? [dq.x / s * ang * hz, dq.y / s * ang * hz, dq.z / s * ang * hz] : [0, 0, 0];
      const ws = Math.hypot(...w); if (ws > L.moverMaxSpin) w = w.map((x) => x * L.moverMaxSpin / ws);
      b.angularVelocity.set(...w);
    });
  };
  const syncMeshes = () => movers.forEach((m) => { if (m.mesh) { m.mesh.position.copy(m.body.position); m.mesh.quaternion.copy(m.body.quaternion); } });
  return { world, root, marble, marbleMesh, movers, steer, syncMeshes, usage };
}

// Judge the end state: is the marble resting inside the cup?
export function judgeEnd(marble) {
  const C = TASK.cup;
  const r = Math.hypot(marble.position.x - C.center[0], marble.position.z - C.center[2]);
  return { success: r < C.innerRadius && marble.position.y < C.wallHeight, distToCup: r };
}
