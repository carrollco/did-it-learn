// Local regression checks. No model calls or network requests.
import assert from 'node:assert/strict';
import { mkdirSync, writeFileSync, readFileSync } from 'node:fs';
import { resolve, join } from 'node:path';
import { validateSource, runTrial } from './sim.mjs';
import { TASK, SPLITS, trialParameters } from './task.mjs';

const scratch = resolve('tmp/codex-drone/checks'); mkdirSync(scratch, { recursive: true });
let checks = 0;
const report = [];
async function trial(name, source, verify, seed = 104) {
  const path = join(scratch, `${name}.js`); writeFileSync(path, source);
  const start = performance.now();
  const { result } = await runTrial(validateSource(path), seed);
  verify(result, performance.now() - start); checks++; report.push({ name, result });
}
const factory = (body) => `export default function makeController(){return {step(obs){${body}}}}`;
for (const token of ['import', 'require', 'process', 'globalThis', 'constructor', 'prototype', '__proto__', 'eval', 'Function', 'Reflect', 'Proxy', 'fetch', 'Date', 'performance']) {
  const path = join(scratch, 'forbidden.js'); writeFileSync(path, factory('return {};') + `\n// ${token}`);
  assert.throws(() => validateSource(path), /forbidden source token/); checks++;
}
await trial('missing-fields', factory('return {thrust:0.5};'), (r) => assert.match(r.reason, /malformed action/));
await trial('infinite-number', factory('return {thrust:1/0,roll:0,pitch:0,yawRate:0};'), (r) => assert.match(r.reason, /malformed action/));
await trial('throw', factory('throw new Error("deliberate crash");'), (r) => assert.match(r.reason, /deliberate crash/));
await trial('hang', factory('while(true){}'), (r, ms) => { assert.match(r.reason, /exceeded 50 ms/); assert.ok(ms < 1000); });
await trial('zero-thrust', factory('return {thrust:0,roll:0,pitch:0,yawRate:0};'), (r) => { assert.equal(r.success, false); assert.equal(r.gatesPassed, 0); });
await trial('full-thrust', factory('return {thrust:1,roll:0,pitch:0,yawRate:0};'), (r) => { assert.equal(r.success, false); assert.match(r.reason, /crash|flipped/); });
await trial('hidden-observation-fields', factory(`
  const expected=['t','pos','vel','att','rates','gateIndex','nextGate','pad'].sort().join(',');
  if(Object.keys(obs).sort().join(',')!==expected) throw new Error('unexpected observation field');
  return {thrust:0,roll:0,pitch:0,yawRate:0};
`), (r) => assert.equal(r.reason, 'time limit'));

// Deliberately bypass the token filter by assembling property names. Permissions must still hold.
const getProcess = `const p=(()=>{})['con'+'structor']('return pro'+'cess')();`;
await trial('permission-read', factory(getProcess + `
  try {p.getBuiltinModule('fs').readFileSync('/etc/hostname','utf8'); throw new Error('READ WAS ALLOWED');}
  catch(e){if(e.code!=='ERR_ACCESS_DENIED') throw e; throw new Error('read correctly denied');}
`), (r) => assert.match(r.reason, /read correctly denied/));
await trial('permission-write', factory(getProcess + `
  try {p.getBuiltinModule('fs').writeFileSync('/tmp/drone-forbidden','x'); throw new Error('WRITE WAS ALLOWED');}
  catch(e){if(e.code!=='ERR_ACCESS_DENIED') throw e; throw new Error('write correctly denied');}
`), (r) => assert.match(r.reason, /write correctly denied/));
await trial('empty-environment', factory(getProcess + `
  if(Object.keys(p.env).length) throw new Error('environment leaked');
  if(p.permission.has('net')||p.permission.has('child')||p.permission.has('worker')) throw new Error('capability leaked');
  throw new Error('environment and capabilities restricted');
`), (r) => assert.match(r.reason, /environment and capabilities restricted/));
await trial('permission-child', factory(getProcess + `
  try {p.getBuiltinModule('child_'+'pro'+'cess').spawn('/bin/true'); throw new Error('SPAWN WAS ALLOWED');}
  catch(e){if(e.code!=='ERR_ACCESS_DENIED') throw e; throw new Error('child correctly denied');}
`), (r) => assert.match(r.reason, /child correctly denied/));
await trial('permission-worker', factory(getProcess + `
  try {const W=p.getBuiltinModule('worker_threads').Worker; new W('',{['e'+'val']:true}); throw new Error('WORKER WAS ALLOWED');}
  catch(e){if(e.code!=='ERR_ACCESS_DENIED') throw e; throw new Error('worker correctly denied');}
`), (r) => assert.match(r.reason, /worker correctly denied/));
await trial('malformed-stdout', factory(getProcess + `
  p.getBuiltinModule('fs').writeSync(1,'garbage\\n');
  return {thrust:0,roll:0,pitch:0,yawRate:0};
`), (r) => assert.match(r.reason, /malformed|unsolicited/));
await trial('permission-net', getProcess + `
  const outcome=await new Promise(resolve=>{
    const socket=p.getBuiltinModule('net').connect({host:'127.0.0.1',port:1});
    socket.on('error',e=>resolve(e.code)); socket.on('connect',()=>resolve('ALLOWED'));
  });
  export default function makeController(){return {step(){
    if(outcome!=='ERR_ACCESS_DENIED') throw new Error('unexpected network outcome '+outcome);
    throw new Error('network correctly denied');
  }}};
`, (r) => assert.match(r.reason, /network correctly denied/));

assert.equal(new Set(SPLITS.val).size, 16); checks++;
for (const seed of SPLITS.val) {
  const p = trialParameters(seed);
  assert.ok(p.mass >= 0.6 && p.mass <= 0.85);
  assert.ok(p.lag >= 0.02 && p.lag <= 0.06);
  assert.ok(p.gusts.length >= 2 && p.gusts.length <= 3);
  assert.ok(p.gusts.some((g) => g.duration === 0.2));
}
checks++;
const artifact = validateSource(resolve('tasks/drone/baselines/stabilizer.js'));
const first = await runTrial(artifact, 104, { record: true }), second = await runTrial(artifact, 104, { record: true });
assert.deepEqual(first, second); checks++;
assert.equal(first.result.success, true); checks++;
assert.equal(first.result.gatesPassed, 3); checks++;
assert.ok(first.result.landingError <= TASK.landingRadius); checks++;
assert.ok(Math.abs(first.track.frames.length - (first.result.simulatedSeconds * 30 + 1)) <= 2); checks++;
assert.equal(first.track.frames[0].t, 0); checks++;
assert.equal(first.track.frames.at(-1).t, first.result.simulatedSeconds); checks++;
for (let i = 1; i < first.track.frames.length - 1; i++) assert.ok(Math.abs(first.track.frames[i].t - first.track.frames[i - 1].t - 1 / 30) < 0.00011);
checks++;
const taskSource = readFileSync('tasks/drone/task.mjs', 'utf8');
const stripped = taskSource.replace(/\s*\/\/ VALIDATION_SEEDS_START[\s\S]*?\/\/ VALIDATION_SEEDS_END/, '');
assert.ok(!stripped.includes(String(SPLITS.val[0]))); checks++;
writeFileSync(join(scratch, 'report.json'), JSON.stringify({ checks, report }, null, 2));
console.log(`${checks} local checks passed, including obfuscated filesystem, subprocess, and network attacks.`);
