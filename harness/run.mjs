#!/usr/bin/env node
// Trusted command line for the trials:
//   node run.mjs <scene.js> --split val|smoke | --seeds 5,6,7 | --seeds-stdin  [--json out.json] [--record]
// --seeds-stdin reads a JSON array of seeds from stdin: that is how the judge passes held-out seeds, so they never
// appear in a command line or an environment the scene could read.
import { writeFileSync, existsSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { SMOKE } from './task.mjs';
import { runSeeds } from './sim.mjs';
import { SceneError } from './world.mjs';

const args = process.argv.slice(2);
const opt = (name, dflt) => { const i = args.indexOf(name); return i >= 0 ? args[i + 1] : dflt; };
const file = args[0] && !args[0].startsWith('--') ? args[0] : null;
if (!file) { console.error('usage: node run.mjs <scene.js> --split val|smoke | --seeds 5,6 | --seeds-stdin [--json out.json] [--record]'); process.exit(2); }
const valFile = join(dirname(fileURLToPath(import.meta.url)), 'val.mjs');
const VAL = existsSync(valFile) ? (await import(valFile)).VAL : [];

let seeds, split;
if (args.includes('--seeds-stdin')) {
  let s = ''; for await (const c of process.stdin) s += c;
  seeds = JSON.parse(s); split = 'heldout';
} else if (opt('--seeds')) { seeds = opt('--seeds').split(',').map(Number).filter(Number.isFinite); split = 'custom'; }
else {
  split = opt('--split', 'val');
  seeds = split === 'val' ? VAL : split === 'smoke' ? SMOKE : null;
  if (!seeds || !seeds.length) { console.error(split === 'val' ? 'no validation trials in this copy of the harness' : `unknown split ${split}`); process.exit(2); }
}

const t0 = Date.now();
let out;
try {
  out = { scene: file, split, valid: true, ...(await runSeeds(file, seeds, { record: args.includes('--record') })) };
} catch (e) {
  if (!(e instanceof SceneError)) throw e;
  out = { scene: file, split, valid: false, error: e.message, passed: 0, total: seeds.length, rate: 0, results: [] };
}
out.seconds = +((Date.now() - t0) / 1000).toFixed(1);
if (opt('--json')) writeFileSync(opt('--json'), JSON.stringify(out));
if (!out.valid) { console.log(`INVALID SCENE: ${out.error}`); process.exit(1); }
console.log(`${split}: ${out.passed}/${out.total} trials ended in the cup (${out.seconds}s; ${out.pieces} pieces, ${out.movers} paddles, material ${out.material} m^2)`);
if (split !== 'heldout') for (const r of out.results) {
  console.log(`  seed ${r.seed}: ${r.success ? 'IN CUP' : r.reason}; final [${r.final}] ${r.distToCup} m from cup centre; bumps at ${r.bumpTimes.join(', ')} s`);
}
