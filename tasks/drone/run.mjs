#!/usr/bin/env node
import { mkdirSync, writeFileSync, readFileSync } from 'node:fs';
import { resolve, join } from 'node:path';
import { SPLITS } from './task.mjs';
import { validateSource, runTrial } from './sim.mjs';

const args = process.argv.slice(2), file = args[0];
const option = (name, fallback) => { const index = args.indexOf(name); return index >= 0 ? args[index + 1] : fallback; };
let split = option('--split', 'val'), seeds, output;
const start = performance.now();
try {
  if (!file || file.startsWith('--')) throw new Error('usage: node tasks/drone/run.mjs <controller.js> --split val|smoke [--json out.json] [--record dir]');
  if ([args.includes('--split'), args.includes('--seeds'), args.includes('--seeds-stdin')].filter(Boolean).length > 1) throw new Error('choose only one seed input: --split, --seeds, or --seeds-stdin');
  if (args.includes('--seeds-stdin')) { split = 'stdin'; seeds = JSON.parse(readFileSync(0, 'utf8')); }
  else if (option('--seeds')) { split = 'custom'; seeds = option('--seeds').split(',').map(Number); }
  else seeds = SPLITS[split];
  if (!Array.isArray(seeds) || !seeds.length || seeds.length > 4096 || !seeds.every((s) => Number.isInteger(s) && s >= 0 && s <= 0xffffffff)) throw new Error('expected a nonempty array of unsigned 32-bit seeds, or a known split');
  const artifact = validateSource(resolve(file)), results = [], recordDir = option('--record');
  if (args.includes('--record') && (!recordDir || recordDir.startsWith('--'))) throw new Error('--record requires a directory');
  if (recordDir) mkdirSync(recordDir, { recursive: true });
  for (const seed of seeds) {
    const { result, track } = await runTrial(artifact, seed, { record: Boolean(recordDir) });
    results.push(result);
    if (recordDir) writeFileSync(join(recordDir, `seed-${seed}.json`), JSON.stringify(track));
  }
  const passed = results.filter((r) => r.success).length;
  output = { scene: file, split, valid: true, passed, total: seeds.length, rate: passed / seeds.length, results };
} catch (error) {
  output = { scene: file || null, split, valid: false, error: error.message, passed: 0, total: seeds?.length || 0, rate: 0, results: [] };
}
output.seconds = +((performance.now() - start) / 1000).toFixed(3);
if (option('--json')) writeFileSync(option('--json'), JSON.stringify(output, null, 2));
if (!output.valid) { console.error(`INVALID CONTROLLER: ${output.error}`); process.exitCode = 1; }
else {
  console.log(`${split}: ${output.passed}/${output.total} trials landed (${output.seconds}s)`);
  for (const r of output.results) console.log(`  seed ${r.seed}: ${r.success ? 'PASS' : 'FAIL'}; ${r.reason}; gates ${r.gatesPassed}/3; pad error ${r.landingError} m; ${r.simulatedSeconds}s simulated`);
}
