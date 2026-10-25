#!/usr/bin/env node
import { homedir } from 'node:os';
// Record trusted flight poses, then render those poses with no artifact code in Chromium.
import { spawn } from 'node:child_process';
import { readFile, writeFile, mkdir, rename } from 'node:fs/promises';
import { resolve, relative, extname, join, dirname, sep } from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from '../../harness/node_modules/playwright/index.mjs';
import { softwareShot, softwareVideo } from './software.mjs';

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '../..');
const args = process.argv.slice(2), option = (n, d) => { const i = args.indexOf(n); return i >= 0 ? args[i + 1] : d; };
if (!args[0] || args[0].startsWith('--')) throw new Error('usage: node tasks/drone/shoot.mjs controller.js --out dir --seed 101 --cams overview,chase,low,top --trail [--video]');
const controller = resolve(args[0]), out = resolve(option('--out', 'tmp/codex-drone/shots'));
const seed = +option('--seed', '101'), camsText = option('--cams', 'overview,chase,low,top');
const cams = camsText.split(/[;|]/).flatMap((c) => c.split(',').length === 3 && c.split(',').every((s) => Number.isFinite(Number(s))) ? [c] : c.split(','));
const width = +option('--w', '1280'), height = +option('--h', '720');
await mkdir(out, { recursive: true });
// Use raw stdio in the runner: Node socket wrappers can be unavailable in restricted hosts.
const runner = fileURLToPath(new URL('./run.mjs', import.meta.url));
await new Promise((done, fail) => {
  const child = spawn(process.execPath, [runner, controller, '--seeds', String(seed), '--record', out, '--json', join(out, 'trial.json')], { stdio: 'inherit' });
  child.on('error', fail); child.on('exit', (code) => code === 0 ? done() : fail(new Error(`trial runner exited ${code}`)));
});
const trackFile = join(out, `seed-${seed}.json`);
const types = { '.html': 'text/html', '.mjs': 'text/javascript', '.js': 'text/javascript', '.json': 'application/json', '.png': 'image/png' };
// Fulfill browser requests locally. No listening socket or internet access is needed.
const base = 'http://drone.local/tasks/drone/view.html';
const routeFiles = async (context) => context.route('**/*', async (route) => {
  try {
    const url = new URL(route.request().url());
    if (url.origin !== 'http://drone.local') throw new Error('outside replay origin');
    const pathname = decodeURIComponent(url.pathname);
    const file = pathname === '/track.json' ? trackFile : resolve(ROOT, '.' + pathname);
    if (file !== trackFile && !file.startsWith(ROOT + sep)) throw new Error('outside root');
    await route.fulfill({ status: 200, contentType: types[extname(file)] || 'application/octet-stream', body: await readFile(file) });
  } catch { await route.fulfill({ status: 404, body: 'Not found' }); }
});
const report = { scene: args[0], seed, shots: [] };
let browser;
try {
  const localLibs = resolve(homedir(), '.cache/ms-playwright-local-libs/root/usr/lib/x86_64-linux-gnu');
  process.env.TMPDIR = resolve(ROOT, 'tmp/codex-drone/browser');
  await mkdir(process.env.TMPDIR, { recursive: true });
  if (!args.includes('--software')) {
    try {
      browser = await chromium.launch({ env: { ...process.env, LD_LIBRARY_PATH: [localLibs, process.env.LD_LIBRARY_PATH].filter(Boolean).join(':') },
        args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
    } catch (error) {
      report.browserError = error.message.split('\n').find((line) => line.includes('FATAL')) || error.message.split('\n')[0];
      console.error('Chromium unavailable; rendering the shared scene with local SVG/Cairo software.');
    }
  }
  if (!browser) {
    report.renderer = 'software';
    const track = JSON.parse(await readFile(trackFile, 'utf8'));
    for (const cam of cams) {
      const name = cam.replace(/[^a-z0-9]+/gi, '_'), file = join(out, `${name}.png`);
      await softwareShot(track, cam, join(out, `${name}.svg`), file, { width, height, trail: args.includes('--trail'), at: option('--t', 'end') });
      report.shots.push({ cam, file: relative(process.cwd(), file) });
    }
    report.result = track.result;
    if (args.includes('--video')) {
      const file = join(out, `replay_seed${seed}.webm`);
      report.videoInfo = await softwareVideo(track, cams[0], file, { width, height, trail: args.includes('--trail') });
      report.video = relative(process.cwd(), file);
    }
  } else {
  report.renderer = 'chromium';
  const qs = (extra) => new URLSearchParams({ track: '/track.json', w: width, h: height, ...(args.includes('--trail') ? { trail: 1 } : {}), ...extra }).toString();
  const waitReady = async (page, flag) => {
    await page.waitForFunction((name) => window[name] || window.__error, flag, { timeout: 60000 });
    const error = await page.evaluate(() => window.__error); if (error) throw new Error(error);
  };
  const stillContext = await browser.newContext({ viewport: { width, height } });
  await routeFiles(stillContext);
  const page = await stillContext.newPage();
  for (const cam of cams) {
    await page.goto(`${base}?${qs({ cam, t: option('--t', 'end') })}`);
    await waitReady(page, '__ready');
    const file = join(out, `${cam.replace(/[^a-z0-9]+/gi, '_')}.png`);
    await page.screenshot({ path: file });
    report.shots.push({ cam, file: relative(process.cwd(), file) });
    report.result = await page.evaluate(() => window.__result);
  }
  await stillContext.close();
  if (args.includes('--video')) {
    const context = await browser.newContext({ viewport: { width, height }, recordVideo: { dir: out, size: { width, height } } });
    await routeFiles(context);
    const vp = await context.newPage();
    await vp.goto(`${base}?${qs({ cam: cams[0], play: 1 })}`); await waitReady(vp, '__done');
    const video = vp.video(); await context.close();
    const file = join(out, `replay_seed${seed}.webm`); await rename(await video.path(), file);
    report.video = relative(process.cwd(), file);
  }
  }
  await writeFile(join(out, 'shots.json'), JSON.stringify(report, null, 2));
} finally { await browser?.close(); }
console.log(JSON.stringify(report));
