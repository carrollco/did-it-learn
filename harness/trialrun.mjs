// The trial loop, shared by the Node judge and the browser viewer so a replay is the run the judge scored.
// `host.tick(t, marble)` returns the paddles' requested poses. In the judge that is a locked-down child process;
// in the browser viewer it is the scene's own functions.
import { TASK, trial as makeTrial } from './task.mjs';
import { buildWorld, judgeEnd, SceneError } from './world.mjs';

export async function runTrial({ THREE, CANNON, desc, decor = [], host = null, seed, graphics = false, record = false, bumpsOverride = null }) {
  const tr = makeTrial(seed);
  if (bumpsOverride) tr.bumps = bumpsOverride;
  const w = buildWorld({ THREE, CANNON, desc, decor, trial: tr, graphics });
  const { world, marble } = w;
  const steps = Math.round(TASK.duration / TASK.dt);
  const every = Math.round(1 / 30 / TASK.dt), tickEvery = Math.round(1 / TASK.controlHz / TASK.dt);
  const applied = tr.bumps.map(() => false);
  const track = [];
  let fell = false, still = 0, t = 0, maxSpeed = 0, reason = null;
  const snap = () => track.push({ t: +t.toFixed(4), p: [marble.position.x, marble.position.y, marble.position.z],
    q: [marble.quaternion.x, marble.quaternion.y, marble.quaternion.z, marble.quaternion.w],
    movers: w.movers.map((m) => ({ p: [m.body.position.x, m.body.position.y, m.body.position.z],
      q: [m.body.quaternion.x, m.body.quaternion.y, m.body.quaternion.z, m.body.quaternion.w] })) });
  for (let i = 0; i < steps; i++) {
    t = i * TASK.dt;
    if (record && i % every === 0) snap();
    if (w.movers.length && i % tickEvery === 0) {
      try {
        w.steer(await host.tick(+t.toFixed(5), { position: [marble.position.x, marble.position.y, marble.position.z],
          velocity: [marble.velocity.x, marble.velocity.y, marble.velocity.z] }));
      } catch (e) {
        reason = `paddle code failed at t=${t.toFixed(2)}s: ${e && e.message}`;
        break;
      }
    }
    tr.bumps.forEach((b, k) => {
      if (!applied[k] && t >= b.time) {
        marble.velocity.x += b.dv[0]; marble.velocity.y += b.dv[1]; marble.velocity.z += b.dv[2];
        applied[k] = true;
      }
    });
    world.step(TASK.dt);
    const v = marble.velocity.length();
    maxSpeed = Math.max(maxSpeed, v);
    if (marble.position.y < -0.3) { fell = true; break; }
    // Early stop once the marble has rested for a second after the last bump (scenes without paddles only).
    still = v < 0.002 ? still + TASK.dt : 0;
    if (!w.movers.length && applied.every(Boolean) && still > 1.0) break;
  }
  w.syncMeshes();
  if (record) snap();
  const end = judgeEnd(marble);
  return {
    result: {
      seed, success: !fell && !reason && end.success, fell, reason: reason || (fell ? 'fell off the desk' : end.success ? 'in the cup' : 'missed'),
      distToCup: +end.distToCup.toFixed(4), simulatedSeconds: +t.toFixed(2),
      final: [marble.position.x, marble.position.y, marble.position.z].map((x) => +x.toFixed(4)), maxSpeed: +maxSpeed.toFixed(3),
      bumpTimes: tr.bumps.map((b) => +b.time.toFixed(2)), material: w.usage.material,
    },
    track, world: w,
  };
}

export { SceneError };
