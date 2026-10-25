import assert from 'node:assert/strict';
import * as Three from '../../harness/node_modules/three/build/three.module.js';
import { flightScene, setFlightCamera } from './scene.mjs';
import { readFileSync } from 'node:fs';
import { resolve } from 'node:path';
import { validateSource, runTrial } from './sim.mjs';
const { track } = await runTrial(validateSource(resolve('tasks/drone/baselines/stabilizer.js')), 1495601726, { record: true });
let renders = 0, scheduled;
class FakeRenderer {
  constructor() { this.shadowMap = {}; this.domElement = { setAttribute() {} }; }
  setSize() {} setPixelRatio() {} setClearColor() {}
  render(scene,camera) { scene.updateMatrixWorld(true); camera.updateMatrixWorld(true); renders++; }
}
globalThis.__three = { ...Three, WebGLRenderer: FakeRenderer };
globalThis.__flightScene = flightScene;
globalThis.__setFlightCamera = setFlightCamera;
globalThis.innerWidth=1280; globalThis.innerHeight=720; globalThis.devicePixelRatio=1;
globalThis.requestAnimationFrame=(callback)=>{scheduled=callback;};
globalThis.fetch=async()=>({ok:true,json:async()=>track});
const code = readFileSync('tasks/drone/view.mjs','utf8').replace(/^import .*\n/gm,'');
for(const cam of ['overview','chase','low','top','35,25,8']) {
  const nodes = new Map();
  globalThis.document = {body:{prepend(){}},createElement(){return {getContext(){return {fillText(){}};}};},querySelector(id){if(!nodes.has(id))nodes.set(id,{style:{}});return nodes.get(id);}};
  globalThis.window={}; globalThis.location={search:'?track=x&trail=1&cam='+encodeURIComponent(cam)};
  await import('data:text/javascript;base64,'+Buffer.from('const THREE=globalThis.__three; const flightScene=globalThis.__flightScene; const setFlightCamera=globalThis.__setFlightCamera;\n'+code+'\n// '+cam).toString('base64'));
  assert.equal(window.__error,undefined); assert.equal(window.__ready,true);
  assert.equal(window.__result.success,true); assert.equal(window.__frame.t,track.frames.at(-1).t);
  const scrub=nodes.get('#scrub');scrub.value='5';scrub.oninput();assert.equal(window.__frame.t,5);
  const select=nodes.get('#camera');select.value='top';select.onchange();
  nodes.get('#play').onclick();scheduled(performance.now()+50);assert.ok(window.__frame.t>=5);
}
console.log('5 replay camera paths, trail creation, scrubbing, and playback checked with real geometry and a stub WebGL renderer. Real browser graphics remain unverified. Renders:',renders);
