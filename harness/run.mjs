#!/usr/bin/env node
// Command line for the trials: node run.mjs <scene.js> --split val|smoke|test | --seeds 5,6,7 [--json out.json] [--record]
// The test split needs HELDOUT_SECRET in the environment; sandboxes never get it.
import { writeFileSync } from 'node:fs';
import { SPLITS, heldoutSeeds } from './task.mjs';
import { loadScene, runSeeds } from './sim.mjs';
import { SceneError } from './world.mjs';

const args = process.argv.slice(2);
const opt = (name, dflt) => { const i = args.indexOf(name); return i >= 0 ? args[i + 1] : dflt; };
const file = args.find((a) => !a.startsWith('--') && !args[args.indexOf(a) - 1]?.startsWith('--'));
const split = opt('--seeds') ? 'custom' : opt('--split', 'val');
if (!file) { console.error('usage: node run.mjs <scene.js> --split val|smoke|test [--json out.json] [--record]'); process.exit(2); }

let seeds;
if (opt('--seeds')) seeds = opt('--seeds').split(',').map(Number).filter(Number.isFinite);
else if (split === 'test') {
  if (!process.env.HELDOUT_SECRET) { console.error('the test split is held out: HELDOUT_SECRET is not set'); process.exit(2); }
  seeds = heldoutSeeds(process.env.HELDOUT_SECRET);
} else if (SPLITS[split] && SPLITS[split].length) seeds = SPLITS[split];
else { console.error(`unknown split ${split}`); process.exit(2); }

const t0 = Date.now();
let out;
try {
  const build = await loadScene(file);
  out = { scene: file, split, valid: true, ...runSeeds(build, seeds, { record: args.includes('--record') }) };
} catch (e) {
  if (!(e instanceof SceneError)) throw e;
  out = { scene: file, split, valid: false, error: e.message, passed: 0, total: seeds.length, rate: 0, results: [] };
}
out.seconds = +((Date.now() - t0) / 1000).toFixed(1);
if (opt('--json')) writeFileSync(opt('--json'), JSON.stringify(out));
if (!out.valid) { console.log(`INVALID SCENE: ${out.error}`); process.exit(1); }
console.log(`${split}: ${out.passed}/${out.total} trials ended in the cup (${out.seconds}s)`);
if (split !== 'test') for (const r of out.results) {
  console.log(`  seed ${r.seed}: ${r.success ? 'IN CUP' : r.fell ? 'fell off the desk' : 'missed'}; final [${r.final}] ${r.distToCup} m from cup centre`);
}
