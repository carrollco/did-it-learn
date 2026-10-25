// The locked-down child process that runs a model's scene code for the judge. Started by sim.mjs with Node's
// permission model (read access to this file, world.mjs, task.mjs, three and the scene only; no writes, no child
// processes, no workers) and an empty environment. It never receives a seed, a trial number or the bumps: only the
// time and the marble's position and velocity, 60 times a second. Speaks JSON lines on stdin/stdout.
import * as THREE from 'three';
import { readFileSync } from 'node:fs';
import { pathToFileURL } from 'node:url';
import { createInterface } from 'node:readline';
import { describeScene, checkSource, pose, SceneError } from './world.mjs';

const file = process.argv[2];
let build = null, paths = null;
const send = (o) => process.stdout.write(JSON.stringify(o) + '\n');
for await (const line of createInterface({ input: process.stdin })) {
  let m;
  try { m = JSON.parse(line); } catch { send({ error: 'bad request' }); continue; }
  try {
    if (m.op === 'describe') {
      if (!build) {
        checkSource(readFileSync(file, 'utf8'));
        build = (await import(pathToFileURL(file).href)).default;
        if (typeof build !== 'function') throw new SceneError('scene must export a default build function');
      }
      const r = describeScene({ THREE, build });
      paths = r.paths;
      send({ desc: r.desc });
    } else if (m.op === 'tick') {
      send({ poses: paths.map((p) => pose(p, m.t, m.marble)) });
    } else send({ error: 'unknown op' });
  } catch (e) {
    send({ error: String((e && e.message) || e), scene: e instanceof SceneError });
  }
}
