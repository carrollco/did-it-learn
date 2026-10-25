// Runs physics trials for one scene file in Node. No graphics, no browser.
import * as THREE from 'three';
import * as CANNON from 'cannon-es';
import { readFileSync } from 'node:fs';
import { pathToFileURL } from 'node:url';
import { checkSource, SceneError } from './world.mjs';
import { runTrial as sharedRun } from './trialrun.mjs';

export async function loadScene(file) {
  checkSource(readFileSync(file, 'utf8'));
  const mod = await import(pathToFileURL(file).href + `?v=${Math.random()}`);
  if (typeof mod.default !== 'function') throw new SceneError('scene must export a default build function');
  return mod.default;
}

export function runTrial(build, seed, { record = false } = {}) {
  const { result, track } = sharedRun({ THREE, CANNON, build, seed, record });
  return record ? { ...result, track } : result;
}

export function runSeeds(build, seeds, opts) {
  const results = seeds.map((s) => runTrial(build, s, opts));
  const passed = results.filter((r) => r.success).length;
  return { passed, total: seeds.length, rate: +(passed / seeds.length).toFixed(4), results };
}
