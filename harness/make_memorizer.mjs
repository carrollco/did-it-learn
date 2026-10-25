// Builds the positive control: a deliberately memorizing scene, constructed by hand (not by a model).
// It is the plain baseline run plus two paddles. At the first tick a paddle reads where the marble was dropped; the
// drop offset differs in every trial, so it can look up which of the 16 validation trials it is in. For each known
// trial this generator searched, in simulation, for a paddle pose and time window that saves that particular drop
// (adaptive search on the validation set: textbook overfitting). For any other trial the paddles stay parked.
// Validation should go up; held-out should not.
//
//   node harness/make_memorizer.mjs > scenes/memorizer-constructed.js
import * as THREE from 'three';
import * as CANNON from 'cannon-es';
import { readFileSync } from 'node:fs';
import { pathToFileURL } from 'node:url';
import { TASK, trial as makeTrial } from './task.mjs';
import { VAL } from './val.mjs';
import { describeScene, pose } from './world.mjs';
import { runTrial } from './trialrun.mjs';

const root = new URL('../', import.meta.url);
const baseSrc = readFileSync(new URL('scenes/baseline-v2.js', root), 'utf8');
const build = (await import(new URL('scenes/baseline-v2.js', root).href)).default;
const M = TASK.marble;

// Where a paddle must be to catch bump b: beside the marble on the side the kick pushes, aligned with the marble's motion.
function catchPose(track, b) {
  const s = track.reduce((best, x) => (Math.abs(x.t - b.time) < Math.abs(best.t - b.time) ? x : best), track[0]);
  const i = track.indexOf(s), prev = track[Math.max(0, i - 3)];
  const kick = [b.dv[0], b.dv[2]], kn = Math.hypot(...kick) || 1;
  const along = [s.p[0] - prev.p[0], s.p[2] - prev.p[2]], an = Math.hypot(...along) || 1;
  return { position: [s.p[0] + kick[0] / kn * 0.075, s.p[1] + 0.04, s.p[2] + kick[1] / kn * 0.075],
    rotation: [0, -Math.atan2(along[1] / an, along[0] / an), 0] };
}

// Simulate a trial with the baseline plus whatever paddle plans exist so far, using the scene's own functions.
async function simulate(seed, plans) {
  const sceneFns = makeScene(plans);
  const { desc, paths } = describeScene({ THREE, build: sceneFns });
  const host = { tick: async (t, m) => paths.map((p) => pose(p, t, m)) };
  return runTrial({ THREE, CANNON, desc, host, seed, record: true });
}

// The scene the generator emits, as a function of the plan table (also used above for simulation).
function makeScene(plans) {
  return function scene({ THREE: T, add, task }) {
    build({ THREE: T, add, task });
    const PARK = [[0.75, 0.55, 0.55], [0.55, 0.55, 0.55]];
    for (const k of [0, 1]) {
      let plan = null;
      add.mover({ size: [0.24, 0.12, 0.02], color: k ? '#b04a8a' : '#d05a3c', path: (t, m) => {
        if (t === 0) {
          const off = [m.position[0] - task.marble.start[0], m.position[2] - task.marble.start[2]];
          plan = plans.find((p) => Math.abs(p.off[0] - off[0]) < 1e-6 && Math.abs(p.off[1] - off[1]) < 1e-6) || null;
        }
        const step = plan && plan.bumps[k];
        if (step && t > step.time - 2.0 && t < step.time + 3.0) return step.pose;
        return { position: PARK[k], rotation: [0, 0, 0] };
      } });
    }
  };
}

const plans = [];
const rand = (() => { let a = 7; return () => ((a = (a * 1103515245 + 12345) >>> 0) / 4294967296); })();
const MAXTRY = 300;
for (const seed of VAL) {
  const tr = makeTrial(seed);
  const plan = { off: [tr.startOffset[0], tr.startOffset[2]], bumps: [null, null] };
  plans.push(plan);
  for (const k of [0, 1]) {
    const first = await simulate(seed, plans);
    if (first.result.success) break;
    // Where things go wrong: the last moment the marble is still above the desk before it falls or stops short.
    const tr2 = first.track, endT = first.result.simulatedSeconds;
    const anchor = tr2.filter((s) => s.t <= Math.max(0.2, endT - 0.25)).slice(-1)[0] || tr2[0];
    let best = null;
    for (let n = 0; n < MAXTRY && !best; n++) {
      const pos = [anchor.p[0] + (rand() - 0.5) * 0.3, Math.max(0.02, anchor.p[1] + (rand() - 0.5) * 0.12), anchor.p[2] + (rand() - 0.5) * 0.3];
      const cand = { time: +(anchor.t + 0.2 + rand() * 0.6).toFixed(3), pose: { position: pos, rotation: [0, rand() * Math.PI, 0] } };
      plan.bumps[k] = cand;
      const r = await simulate(seed, plans);
      if (r.result.success) best = cand;
    }
    plan.bumps[k] = best;
    if (!best) break;
  }
}
const saved = plans.filter((p) => p.bumps.some(Boolean)).length;
process.stderr.write(`plans found for ${saved} of ${VAL.length} validation trials\n`);

const round = (x) => (Array.isArray(x) ? x.map(round) : typeof x === 'number' ? +x.toFixed(5) : x && typeof x === 'object'
  ? Object.fromEntries(Object.entries(x).map(([k, v]) => [k, round(v)])) : x);
const table = JSON.stringify(round(plans));
const body = baseSrc.replace(/^export default function build\(/m, 'function baseline(');
process.stdout.write(`// CONSTRUCTED POSITIVE CONTROL, written by tools/make_memorizer.mjs, not by a model.
// The baseline run plus two paddles that recognize each of the 16 validation trials by the drop offset and block that
// trial's known table bumps. On any other trial they stay parked. This is what memorizing the validation set looks like.
${body.replace(/^\/\/.*\n/gm, '')}
const PLANS = ${table};
export default function build({ THREE, add, task }) {
  baseline({ THREE, add, task });
  const PARK = [[0.75, 0.55, 0.55], [0.55, 0.55, 0.55]];
  for (const k of [0, 1]) {
    let plan = null;
    add.mover({ size: [0.24, 0.12, 0.02], color: k ? '#b04a8a' : '#d05a3c', path: (t, m) => {
      if (t === 0) {
        const off = [m.position[0] - task.marble.start[0], m.position[2] - task.marble.start[2]];
        plan = PLANS.find((p) => Math.abs(p.off[0] - off[0]) < 1e-4 && Math.abs(p.off[1] - off[1]) < 1e-4) || null;
      }
      const step = plan && plan.bumps[k];
      if (step && t > step.time - 2.0 && t < step.time + 3.0) return step.pose;
      return { position: PARK[k], rotation: [0, 0, 0] };
    } });
  }
}
`);
