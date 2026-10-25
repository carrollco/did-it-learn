// The trial loop, shared by the Node judge and the browser viewer so a replay is the run the judge scored.
import { TASK, trial as makeTrial } from './task.mjs';
import { buildWorld, judgeEnd } from './world.mjs';

export function runTrial({ THREE, CANNON, build, seed, graphics = false, record = false, bumpOverride = null }) {
  const tr = makeTrial(seed);
  if (bumpOverride) tr.bump = bumpOverride;
  const w = buildWorld({ THREE, CANNON, build, trial: tr, graphics });
  const { world, marble } = w;
  const steps = Math.round(TASK.duration / TASK.dt);
  const every = Math.round(1 / 30 / TASK.dt);
  const track = [];
  let bumped = false, fell = false, still = 0, t = 0, maxSpeed = 0;
  const snap = () => track.push({ t: +t.toFixed(4), p: [marble.position.x, marble.position.y, marble.position.z],
    q: [marble.quaternion.x, marble.quaternion.y, marble.quaternion.z, marble.quaternion.w],
    movers: w.movers.map((m) => ({ p: [m.body.position.x, m.body.position.y, m.body.position.z],
      q: [m.body.quaternion.x, m.body.quaternion.y, m.body.quaternion.z, m.body.quaternion.w] })) });
  w.setMovers(0, 0);
  for (let i = 0; i < steps; i++) {
    t = i * TASK.dt;
    if (record && i % every === 0) snap();
    if (w.movers.length) w.setMovers(t + TASK.dt, TASK.dt);
    if (!bumped && t >= tr.bump.time) {
      marble.velocity.x += tr.bump.dv[0]; marble.velocity.y += tr.bump.dv[1]; marble.velocity.z += tr.bump.dv[2];
      bumped = true;
    }
    world.step(TASK.dt);
    const v = marble.velocity.length();
    maxSpeed = Math.max(maxSpeed, v);
    if (marble.position.y < -0.3) { fell = true; break; }
    // Early stop once the marble has rested for a second after the bump (static scenes only).
    still = v < 0.002 ? still + TASK.dt : 0;
    if (!w.movers.length && bumped && still > 1.0) break;
  }
  if (record) snap();
  const end = judgeEnd(marble);
  return {
    result: {
      seed, success: !fell && end.success, fell, distToCup: +end.distToCup.toFixed(4), simulatedSeconds: +t.toFixed(2),
      final: [marble.position.x, marble.position.y, marble.position.z].map((x) => +x.toFixed(4)),
      maxSpeed: +maxSpeed.toFixed(3), pieces: w.pieces(), bumpTime: +tr.bump.time.toFixed(2),
    },
    track, world: w,
  };
}
