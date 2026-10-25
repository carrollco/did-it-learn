// Trusted judge side: starts the scene in a locked-down child process, gets its description, and runs trials.
// Scenes without paddles are described once (their pieces cannot react to anything); scenes with paddles get a
// fresh child per trial, so nothing carries over from one trial to the next.
import * as THREE from 'three';
import * as CANNON from 'cannon-es';
import { spawn } from 'node:child_process';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { validateDesc, SceneError } from './world.mjs';
import { runTrial } from './trialrun.mjs';

const HERE = dirname(fileURLToPath(import.meta.url));
const PERM = process.allowedNodeEnvironmentFlags.has('--permission') ? '--permission' : '--experimental-permission';
const READ = [join(HERE, 'host.mjs'), join(HERE, 'world.mjs'), join(HERE, 'task.mjs'), join(HERE, 'package.json'),
  join(HERE, 'node_modules', 'three'), join(HERE, 'node_modules', 'three', '*')];

export class ChildHost {
  constructor(sceneFile) {
    const scene = resolve(sceneFile);
    this.proc = spawn(process.execPath, [PERM, ...[...READ, scene].map((p) => `--allow-fs-read=${p}`), join(HERE, 'host.mjs'), scene],
      { env: {}, stdio: ['pipe', 'pipe', 'pipe'] });
    this.buf = ''; this.waiting = []; this.stderr = '';
    this.proc.stdout.on('data', (d) => {
      this.buf += d;
      let k;
      while ((k = this.buf.indexOf('\n')) >= 0) {
        const line = this.buf.slice(0, k); this.buf = this.buf.slice(k + 1);
        const w = this.waiting.shift();
        if (w) { clearTimeout(w.timer); try { w.ok(JSON.parse(line)); } catch { w.no(new SceneError('scene process wrote something that is not a reply')); } }
      }
    });
    this.proc.stderr.on('data', (d) => { this.stderr = (this.stderr + d).slice(-2000); });
    this.proc.on('exit', (code) => { for (const w of this.waiting.splice(0)) { clearTimeout(w.timer); w.no(new SceneError(`scene process exited (${code}): ${this.stderr.trim().split('\n').slice(-2).join(' ')}`)); } });
  }
  ask(msg, ms) {
    return new Promise((ok, no) => {
      const timer = setTimeout(() => no(new SceneError(`scene process did not answer within ${ms} ms`)), ms);
      this.waiting.push({ ok, no, timer });
      this.proc.stdin.write(JSON.stringify(msg) + '\n');
    });
  }
  async describe() {
    const r = await this.ask({ op: 'describe' }, 15000);
    if (r.error) throw new SceneError(r.error);
    return r.desc;
  }
  async tick(t, marble) {
    const r = await this.ask({ op: 'tick', t, marble }, 2000);
    if (r.error) throw new SceneError(r.error);
    return r.poses;
  }
  close() { try { this.proc.stdin.end(); this.proc.kill(); } catch { /* already gone */ } }
}

async function pool(items, n, fn) {
  const out = new Array(items.length); let next = 0;
  await Promise.all(Array.from({ length: Math.min(n, items.length) }, async () => {
    while (next < items.length) { const i = next++; out[i] = await fn(items[i], i); }
  }));
  return out;
}

export async function runSeeds(sceneFile, seeds, { record = false, concurrency = 8 } = {}) {
  const probe = new ChildHost(sceneFile);
  let desc;
  try { desc = await probe.describe(); } finally { probe.close(); }
  const usage = validateDesc(desc);
  const results = await pool(seeds, concurrency, async (seed) => {
    if (!desc.movers.length) {
      const { result, track } = await runTrial({ THREE, CANNON, desc, seed, record });
      return record ? { ...result, track } : result;
    }
    const host = new ChildHost(sceneFile);
    try {
      const fresh = await host.describe();
      const { result, track } = await runTrial({ THREE, CANNON, desc: fresh, host, seed, record });
      return record ? { ...result, track } : result;
    } finally { host.close(); }
  });
  const passed = results.filter((r) => r.success).length;
  return { passed, total: seeds.length, rate: +(passed / seeds.length).toFixed(4), material: usage.material,
    pieces: desc.pieces.length, movers: desc.movers.length, results };
}
