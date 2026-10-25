// Builds one trial's world: the harness's desk, cup and marble, plus whatever the scene adds through `add`.
// Runs unchanged in Node (physics trials) and in the browser (renders). THREE and CANNON are passed in.
import { TASK, rng } from './task.mjs';

export class SceneError extends Error {}

const vec = (v, n, what) => {
  if (!Array.isArray(v) || v.length !== n || !v.every(Number.isFinite)) throw new SceneError(`${what} must be an array of ${n} finite numbers`);
  return v;
};

// Source rules for scene files: one default export, no imports, no reaching outside the arguments it is given.
const FORBIDDEN = [/^\s*import\s/m, /\bimport\s*\(/, /\brequire\s*\(/, /\bglobalThis\b/, /\bwindow\b/, /\bdocument\b/,
  /\bprocess\b/, /\bfetch\b/, /\beval\b/, /\bFunction\s*\(/, /\bDate\b/, /\bperformance\b/, /\bsetTimeout\b/, /\bsetInterval\b/];

export function checkSource(src) {
  for (const re of FORBIDDEN) if (re.test(src)) throw new SceneError(`scene source uses a forbidden construct: ${re}`);
  if (!/export\s+default\s+function/.test(src)) throw new SceneError('scene must `export default function build({ THREE, add, task })`');
}

export function buildWorld({ THREE, CANNON, build, trial, graphics = true }) {
  const world = new CANNON.World({ gravity: new CANNON.Vec3(0, -9.81, 0) });
  world.broadphase = new CANNON.SAPBroadphase(world);
  world.allowSleep = false;
  world.solver.iterations = 20;
  const marbleMat = new CANNON.Material('marble');
  const pieceMat = new CANNON.Material('piece');
  world.addContactMaterial(new CANNON.ContactMaterial(marbleMat, pieceMat, {
    friction: trial.friction, restitution: trial.restitution,
  }));
  const root = new THREE.Group();
  const quat = (rot) => new THREE.Quaternion().setFromEuler(new THREE.Euler(rot[0], rot[1], rot[2], 'XYZ'));
  const material = (color) => new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.05 });
  const B = TASK.bounds, LIM = TASK.budget;
  let pieces = 0, movers = 0;
  const moverList = [];

  const place = (body, mesh, position, rotation) => {
    const q = quat(rotation);
    body.position.set(...position);
    body.quaternion.set(q.x, q.y, q.z, q.w);
    world.addBody(body);
    if (mesh) { mesh.position.set(...position); mesh.quaternion.copy(q); mesh.castShadow = mesh.receiveShadow = true; root.add(mesh); }
  };
  const inBounds = (p, what) => {
    if (p[0] < B.minX || p[0] > B.maxX || p[1] < B.minY || p[1] > B.maxY || p[2] < B.minZ || p[2] > B.maxZ)
      throw new SceneError(`${what} at [${p.map((x) => x.toFixed(3))}] is outside the allowed volume`);
  };
  const size = (s, what) => {
    vec(s, 3, `${what} size`);
    if (s.some((x) => x < LIM.minSize || x > LIM.maxSize)) throw new SceneError(`${what} size must be within ${LIM.minSize}..${LIM.maxSize} m on every axis`);
    return s;
  };
  const countPiece = (what) => {
    if (++pieces > LIM.pieces) throw new SceneError(`more than ${LIM.pieces} physical pieces (${what})`);
  };

  // Harness-owned pieces: desk top (as an infinite plane) and the cup.
  const d = TASK.desk, deskSize = [d.maxX - d.minX, 0.04, d.maxZ - d.minZ];
  const desk = new CANNON.Body({ mass: 0, type: CANNON.Body.STATIC, material: pieceMat,
    shape: new CANNON.Box(new CANNON.Vec3(deskSize[0] / 2, deskSize[1] / 2, deskSize[2] / 2)) });
  place(desk, graphics ? new THREE.Mesh(new THREE.BoxGeometry(...deskSize), material('#9a6b43')) : null, [0, -0.02, 0], [0, 0, 0]);
  const C = TASK.cup;
  const ring = C.innerRadius + C.wallThickness / 2;
  for (let i = 0; i < C.segments; i++) {
    const a = (i / C.segments) * 2 * Math.PI;
    const w = (2 * Math.PI * ring / C.segments) * 1.08;
    const p = [C.center[0] + ring * Math.cos(a), C.wallHeight / 2, C.center[2] + ring * Math.sin(a)];
    const body = new CANNON.Body({ mass: 0, type: CANNON.Body.STATIC, material: pieceMat,
      shape: new CANNON.Box(new CANNON.Vec3(C.wallThickness / 2, C.wallHeight / 2, w / 2)) });
    const mesh = graphics ? new THREE.Mesh(new THREE.BoxGeometry(C.wallThickness, C.wallHeight, w), material('#f4f1ea')) : null;
    place(body, mesh, p, [0, -a, 0]);
  }

  // The only API a scene gets. No handle on the world, the marble or time other than a mover's own path.
  const add = Object.freeze({
    box({ size: s, position, rotation = [0, 0, 0], color = '#b8b8b8' }) {
      countPiece('box'); size(s, 'box'); vec(position, 3, 'box position'); vec(rotation, 3, 'box rotation'); inBounds(position, 'box');
      const body = new CANNON.Body({ mass: 0, type: CANNON.Body.STATIC, material: pieceMat,
        shape: new CANNON.Box(new CANNON.Vec3(s[0] / 2, s[1] / 2, s[2] / 2)) });
      place(body, graphics ? new THREE.Mesh(new THREE.BoxGeometry(...s), material(color)) : null, position, rotation);
    },
    cylinder({ radius, height, position, rotation = [0, 0, 0], color = '#b8b8b8' }) {
      countPiece('cylinder'); size([radius * 2, height, radius * 2], 'cylinder'); vec(position, 3, 'cylinder position'); inBounds(position, 'cylinder');
      const body = new CANNON.Body({ mass: 0, type: CANNON.Body.STATIC, material: pieceMat,
        shape: new CANNON.Cylinder(radius, radius, height, 16) });
      place(body, graphics ? new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, height, 24), material(color)) : null, position, rotation);
    },
    mover({ size: s, path, color = '#d05a3c' }) {
      if (++movers > LIM.movers) throw new SceneError(`more than ${LIM.movers} movers`);
      size(s, 'mover');
      if (typeof path !== 'function') throw new SceneError('mover path must be a function of time t (seconds)');
      const pose = (t) => {
        const p = path(t) || {};
        return { position: vec(p.position, 3, 'mover position'), rotation: vec(p.rotation || [0, 0, 0], 3, 'mover rotation') };
      };
      const p0 = pose(0);
      inBounds(p0.position, 'mover');
      const body = new CANNON.Body({ mass: 0, type: CANNON.Body.KINEMATIC, material: pieceMat,
        shape: new CANNON.Box(new CANNON.Vec3(s[0] / 2, s[1] / 2, s[2] / 2)) });
      const mesh = graphics ? new THREE.Mesh(new THREE.BoxGeometry(...s), material(color)) : null;
      place(body, mesh, p0.position, p0.rotation);
      moverList.push({ body, mesh, pose });
    },
    decor(object3D) {
      if (!graphics) return;
      if (!object3D || !object3D.isObject3D) throw new SceneError('decor takes a THREE.Object3D');
      root.add(object3D);
    },
  });

  // Run the scene's build with a seeded Math.random so every trial builds the identical scene.
  const realRandom = Math.random;
  Math.random = rng(424242);
  try { build({ THREE, add, task: TASK }); }
  catch (e) { if (e instanceof SceneError) throw e; throw new SceneError(`scene build threw: ${e && e.message}`); }
  finally { Math.random = realRandom; }

  const M = TASK.marble;
  const marble = new CANNON.Body({ mass: M.mass, shape: new CANNON.Sphere(M.radius), material: marbleMat,
    linearDamping: 0.01, angularDamping: 0.01 });
  marble.position.set(M.start[0] + trial.startOffset[0], M.start[1], M.start[2] + trial.startOffset[2]);
  marble.velocity.set(...trial.startVelocity);
  world.addBody(marble);
  let marbleMesh = null;
  if (graphics) {
    marbleMesh = new THREE.Mesh(new THREE.SphereGeometry(M.radius, 32, 16),
      new THREE.MeshStandardMaterial({ color: '#2f7fd8', roughness: 0.15, metalness: 0.3 }));
    marbleMesh.castShadow = true; root.add(marbleMesh);
  }

  const setMovers = (t, dt) => {
    for (const m of moverList) {
      const p = m.pose(t), q = quat(p.rotation);
      const prev = m.body.position.clone(), prevQ = m.body.quaternion.clone();
      m.body.position.set(...p.position);
      m.body.quaternion.set(q.x, q.y, q.z, q.w);
      if (dt > 0) {
        m.body.velocity.set((p.position[0] - prev.x) / dt, (p.position[1] - prev.y) / dt, (p.position[2] - prev.z) / dt);
        const dq = m.body.quaternion.mult(prevQ.conjugate());
        const ang = 2 * Math.acos(Math.min(1, Math.abs(dq.w)));
        const s = Math.sqrt(Math.max(1e-12, 1 - dq.w * dq.w)) * Math.sign(dq.w || 1);
        m.body.angularVelocity.set(ang > 1e-9 ? (dq.x / s) * ang / dt : 0, ang > 1e-9 ? (dq.y / s) * ang / dt : 0, ang > 1e-9 ? (dq.z / s) * ang / dt : 0);
      }
      if (m.mesh) { m.mesh.position.set(...p.position); m.mesh.quaternion.copy(q); }
    }
  };

  return { world, root, marble, marbleMesh, movers: moverList, setMovers, pieces: () => pieces };
}

// Judge the end state: is the marble resting inside the cup?
export function judgeEnd(marble) {
  const C = TASK.cup;
  const dx = marble.position.x - C.center[0], dz = marble.position.z - C.center[2];
  const r = Math.hypot(dx, dz);
  return { success: r < C.innerRadius && marble.position.y < C.wallHeight, distToCup: r };
}
