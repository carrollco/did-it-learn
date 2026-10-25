import { test, describe, before, after } from 'node:test';
import assert from 'node:assert/strict';
import { mkdtempSync, writeFileSync, rmSync, readFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, resolve } from 'node:path';
import { spawnSync } from 'node:child_process';
import net from 'node:net';

import { validateSource, runTrial, initialState } from '../../tasks/drone/sim.mjs';
import { TASK, SPLITS, trialParameters } from '../../tasks/drone/task.mjs';

const REPO_ROOT = resolve(import.meta.dirname, '../..');
const BASELINE = resolve(REPO_ROOT, 'tasks/drone/baselines/stabilizer.js');
const RUN_MJS = resolve(REPO_ROOT, 'tasks/drone/run.mjs');

let scratchDir;
before(() => {
  scratchDir = mkdtempSync(join(tmpdir(), 'drone-suite-'));
});

after(() => {
  if (scratchDir) rmSync(scratchDir, { recursive: true, force: true });
});

function writeController(name, source) {
  const file = join(scratchDir, `${name}.js`);
  writeFileSync(file, source, 'utf8');
  return file;
}

const factory = (body) => `export default function makeController() { return { step(obs) { ${body} } }; }`;
// Bypass the source regex filter by concatenating strings, matching tasks/drone/check.mjs:35
const getProcess = `const p = (() => {})['con' + 'structor']('return pro' + 'cess')();`;

describe('tasks/drone determinism and physics parameters', () => {
  test('same controller on the same seed gives identical results and tracks twice', async () => {
    const artifact = validateSource(BASELINE);
    const first = await runTrial(artifact, 104, { record: true });
    const second = await runTrial(artifact, 104, { record: true });

    assert.deepEqual(first, second);
    assert.equal(first.result.success, true);
    assert.equal(first.result.reason, 'landed');
    assert.equal(first.result.gatesPassed, 3);
    assert.ok(first.result.landingError <= TASK.landingRadius);
    assert.equal(first.track.frames[0].t, 0);
    assert.equal(first.track.frames.at(-1).t, first.result.simulatedSeconds);
  });

  test('failed flight is also completely deterministic across runs', async () => {
    const file = writeController('zero-thrust', factory('return { thrust: 0, roll: 0, pitch: 0, yawRate: 0 };'));
    const artifact = validateSource(file);
    const run1 = await runTrial(artifact, 101, { record: true });
    const run2 = await runTrial(artifact, 101, { record: true });

    assert.deepEqual(run1, run2);
    assert.equal(run1.result.success, false);
    assert.equal(run1.result.reason, 'time limit');
    assert.equal(run1.result.gatesPassed, 0);
  });

  test('trialParameters and PRNG are deterministic and within physical bounds', () => {
    assert.equal(new Set(SPLITS.val).size, 16);
    assert.deepEqual(SPLITS.smoke, [101, 102, 103, 104]);
    for (const seed of [...SPLITS.smoke, ...SPLITS.val]) {
      const p1 = trialParameters(seed);
      const p2 = trialParameters(seed);
      assert.deepEqual(p1, p2);
      assert.ok(p1.mass >= TASK.baseMass && p1.mass <= TASK.baseMass + TASK.payloadMax);
      assert.ok(p1.lag >= TASK.motorLagMin && p1.lag <= TASK.motorLagMax);
      assert.ok(p1.weakMotor >= 0 && p1.weakMotor < 4);
      assert.ok(p1.gusts.length >= 2 && p1.gusts.length <= 3);
      assert.equal(p1.gusts[0].duration, TASK.gustDurationMin);
      const s0 = initialState(p1);
      assert.equal(s0.t, 0);
      assert.equal(s0.pos.y, TASK.bodyHeight);
    }
  });
});

describe('tasks/drone source validation and forbidden tokens', () => {
  const forbiddenTokens = [
    'import', 'require', 'process', 'globalThis', 'constructor', 'prototype',
    '__proto__', 'eval', 'Function', 'Reflect', 'Proxy', 'fetch', 'Date', 'performance',
  ];

  test('validateSource rejects every forbidden token', () => {
    for (const token of forbiddenTokens) {
      const file = writeController(`tok-${token}`, `${factory('return { thrust: 0, roll: 0, pitch: 0, yawRate: 0 };')}\n// ${token}`);
      assert.throws(() => validateSource(file), /forbidden source token/, `token ${token} should be rejected`);
    }
  });

  test('validateSource rejects oversized files > 256 KiB', () => {
    const padding = 'x'.repeat(260 * 1024);
    const file = writeController('oversized', `${factory('return { thrust: 0, roll: 0, pitch: 0, yawRate: 0 };')}\n// ${padding}`);
    assert.throws(() => validateSource(file), /exceeds 256 KiB/);
  });

  test('non-function default export or missing step(obs) fails cleanly without crashing judge', async () => {
    const noDefault = writeController('no-default', 'export default 42;');
    const r1 = await runTrial(validateSource(noDefault), 101);
    assert.equal(r1.result.success, false);
    assert.match(r1.result.reason, /controller exited/);

    const noStep = writeController('no-step', 'export default function makeController() { return {}; }');
    const r2 = await runTrial(validateSource(noStep), 101);
    assert.equal(r2.result.success, false);
    assert.match(r2.result.reason, /controller exited/);
  });
});

describe('tasks/drone sandbox security and resource limits', () => {
  test('controller trying to read a file is stopped and scored as a failure', async () => {
    const file = writeController(
      'read-file',
      factory(`
        ${getProcess}
        const fs = p.getBuiltinModule('fs');
        const secret = fs.readFileSync(${JSON.stringify(resolve(REPO_ROOT, 'tasks/drone/task.mjs'))}, 'utf8');
        return { thrust: secret.length ? 0.5 : 0, roll: 0, pitch: 0, yawRate: 0 };
      `),
    );
    const { result } = await runTrial(validateSource(file), 104);
    assert.equal(result.success, false);
    assert.match(result.reason, /Access to this API has been restricted|ERR_ACCESS_DENIED|controller error/);
  });

  test('controller trying to write a file is stopped and scored as a failure', async () => {
    const target = join(scratchDir, 'should-not-exist.txt');
    const file = writeController(
      'write-file',
      factory(`
        ${getProcess}
        p.getBuiltinModule('fs').writeFileSync(${JSON.stringify(target)}, 'pwned');
        return { thrust: 0.5, roll: 0, pitch: 0, yawRate: 0 };
      `),
    );
    const { result } = await runTrial(validateSource(file), 104);
    assert.equal(result.success, false);
    assert.match(result.reason, /Access to this API has been restricted/);
  });

  test('controller trying to read process.env sees an empty environment and fails cleanly', async () => {
    process.env.HELDOUT_SECRET = 'parent-only-secret-value';
    const file = writeController(
      'read-env',
      factory(`
        ${getProcess}
        const keys = Object.keys(p.env);
        if (keys.length > 0 || p.env.HELDOUT_SECRET) {
          return { thrust: 0.5, roll: 0, pitch: 0, yawRate: 0 };
        }
        throw new Error('env is empty: ' + keys.length);
      `),
    );
    const { result } = await runTrial(validateSource(file), 104);
    assert.equal(result.success, false);
    assert.match(result.reason, /env is empty: 0/);
  });

  test('controller trying to start a child process is stopped and scored as a failure', async () => {
    const file = writeController(
      'spawn-proc',
      factory(`
        ${getProcess}
        p.getBuiltinModule('child_' + 'pro' + 'cess').spawnSync('/bin/echo', ['hi']);
        return { thrust: 0.5, roll: 0, pitch: 0, yawRate: 0 };
      `),
    );
    const { result } = await runTrial(validateSource(file), 104);
    assert.equal(result.success, false);
    assert.match(result.reason, /Access to this API has been restricted/);
  });

  test('controller trying to spawn a worker thread is stopped and scored as a failure', async () => {
    const file = writeController(
      'spawn-worker',
      factory(`
        ${getProcess}
        const W = p.getBuiltinModule('worker_threads').Worker;
        new W('', { ['e' + 'val']: true });
        return { thrust: 0.5, roll: 0, pitch: 0, yawRate: 0 };
      `),
    );
    const { result } = await runTrial(validateSource(file), 104);
    assert.equal(result.success, false);
    assert.match(result.reason, /Access to this API has been restricted/);
  });

  test('controller that loops forever in step() is stopped within 50ms timeout and scored as a failure', async () => {
    const file = writeController('infinite-loop', factory('while (true) {}'));
    const t0 = performance.now();
    const { result } = await runTrial(validateSource(file), 104);
    const elapsedMs = performance.now() - t0;

    assert.equal(result.success, false);
    assert.match(result.reason, /exceeded 50 ms/);
    assert.ok(elapsedMs < 2000, `expected fast kill, took ${elapsedMs}ms`);
  });

  test('controller that opens a network socket via obfuscated process.getBuiltinModule("net") is denied', {
    todo: 'BUG in tasks/drone/sim.mjs:10-13 & tasks/drone/host.mjs:4-9: Node 22 --experimental-permission does not restrict net sockets and host.mjs does not lock down process.getBuiltinModule("net")',
  }, async () => {
    let connected = false;
    const server = net.createServer((socket) => {
      connected = true;
      socket.on('error', () => {});
      socket.end('pong');
    });
    await new Promise((resolve) => server.listen(0, '127.0.0.1', resolve));
    const { port } = server.address();

    try {
      const file = writeController(
        'net-socket',
        `
        ${getProcess}
        const outcome = await new Promise((resolve) => {
          try {
            const sock = p.getBuiltinModule('net').connect({ host: '127.0.0.1', port: ${port} });
            sock.on('connect', () => { sock.end(); resolve('CONNECTED'); });
            sock.on('error', (e) => resolve(e.code || 'ERROR'));
          } catch (e) {
            resolve(e.code || 'THROWN');
          }
        });
        export default function makeController() {
          return {
            step() {
              if (outcome === 'CONNECTED') throw new Error('SOCKET_ALLOWED');
              throw new Error('network outcome: ' + outcome);
            }
          };
        }
        `,
      );
      const { result } = await runTrial(validateSource(file), 104);
      assert.equal(connected, false, 'child controller should not be permitted to connect to a TCP socket');
      assert.match(result.reason, /ERR_ACCESS_DENIED/);
    } finally {
      await new Promise((resolve) => server.close(resolve));
    }
  });
});

describe('tasks/drone malformed actions and protocol errors fail cleanly', () => {
  const malformedCases = [
    ['null-action', 'return null;'],
    ['undefined-action', 'return undefined;'],
    ['array-action', 'return [0.5, 0, 0, 0];'],
    ['primitive-action', 'return 0.5;'],
    ['missing-fields', 'return { thrust: 0.5 };'],
    ['nan-field', 'return { thrust: NaN, roll: 0, pitch: 0, yawRate: 0 };'],
    ['infinity-field', 'return { thrust: 1 / 0, roll: 0, pitch: 0, yawRate: 0 };'],
    ['neg-infinity-field', 'return { thrust: 0.5, roll: -1 / 0, pitch: 0, yawRate: 0 };'],
    ['string-number', 'return { thrust: "0.5", roll: 0, pitch: 0, yawRate: 0 };'],
  ];

  for (const [name, body] of malformedCases) {
    test(`malformed action (${name}) fails that flight cleanly`, async () => {
      const file = writeController(name, factory(body));
      const { result } = await runTrial(validateSource(file), 104);
      assert.equal(result.success, false);
      assert.equal(result.reason, 'malformed action: four finite numbers required');
    });
  }

  test('controller writing garbage to stdout fails cleanly without crashing judge', async () => {
    const file = writeController(
      'garbage-stdout',
      factory(`
        ${getProcess}
        p.getBuiltinModule('fs').writeSync(1, 'not-json-output\\n');
        return { thrust: 0.5, roll: 0, pitch: 0, yawRate: 0 };
      `),
    );
    const { result } = await runTrial(validateSource(file), 104);
    assert.equal(result.success, false);
    assert.match(result.reason, /malformed controller JSON|unsolicited controller output/);
  });

  test('run.mjs CLI handles --seeds, --seeds-stdin, --json, and invalid flags cleanly', () => {
    const jsonOut1 = join(scratchDir, 'out-seeds.json');
    const jsonOut2 = join(scratchDir, 'out-stdin.json');

    const res1 = spawnSync(process.execPath, [RUN_MJS, BASELINE, '--seeds', '101,104', '--json', jsonOut1], {
      cwd: REPO_ROOT,
      encoding: 'utf8',
    });
    assert.equal(res1.status, 0);
    const parsed1 = JSON.parse(readFileSync(jsonOut1, 'utf8'));
    assert.equal(parsed1.valid, true);
    assert.equal(parsed1.total, 2);

    const res2 = spawnSync(process.execPath, [RUN_MJS, BASELINE, '--seeds-stdin', '--json', jsonOut2], {
      cwd: REPO_ROOT,
      input: JSON.stringify([101, 104]),
      encoding: 'utf8',
    });
    assert.equal(res2.status, 0);
    const parsed2 = JSON.parse(readFileSync(jsonOut2, 'utf8'));
    assert.deepEqual(parsed1.results, parsed2.results);

    const jsonBad = join(scratchDir, 'out-bad.json');
    const resBad = spawnSync(process.execPath, [RUN_MJS, BASELINE, '--seeds', '-1', '--json', jsonBad], {
      cwd: REPO_ROOT,
      encoding: 'utf8',
    });
    assert.equal(resBad.status, 1);
    const parsedBad = JSON.parse(readFileSync(jsonBad, 'utf8'));
    assert.equal(parsedBad.valid, false);
  });
});
