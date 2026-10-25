#!/usr/bin/env node
// Renders a scene in headless Chromium: still screenshots from camera angles, optionally a replay video.
// node shoot.mjs <scene.js> --out dir [--seed 1] [--cams overview,closeup,low,top | "az,el,dist;..."] [--t end] [--trail] [--video]
import { createServer } from 'node:http';
import { readFile, mkdir, rename } from 'node:fs/promises';
import { resolve, relative, extname, join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const ROOT = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const args = process.argv.slice(2);
const opt = (n, d) => { const i = args.indexOf(n); return i >= 0 ? args[i + 1] : d; };
const scene = resolve(args[0]);
const out = resolve(opt('--out', 'shots'));
const seed = opt('--seed', '1');
const cams = (opt('--cams', 'overview,closeup,low,top')).split(/[;|]/).flatMap((c) => (c.includes(',') && c.split(',').length === 3 ? [c] : c.split(',')));
const at = opt('--t', 'end');
const bump = opt('--bump', '');
const W = +opt('--w', 1280), H = +opt('--h', 720);

const TYPES = { '.html': 'text/html', '.mjs': 'text/javascript', '.js': 'text/javascript', '.json': 'application/json' };
// Serve the repository root plus the scene file itself under /scene/<name>, wherever it lives.
const server = createServer(async (req, res) => {
  const url = new URL(req.url, 'http://x');
  try {
    const file = url.pathname.startsWith('/scene/') ? scene : join(ROOT, decodeURIComponent(url.pathname));
    if (!file.startsWith(ROOT) && file !== scene) throw new Error('outside root');
    const body = await readFile(file);
    res.writeHead(200, { 'content-type': TYPES[extname(file)] || 'application/octet-stream', 'cache-control': 'no-store' });
    res.end(body);
  } catch { res.writeHead(404); res.end(); }
}).listen(0, '127.0.0.1');
await new Promise((r) => server.on('listening', r));
const base = `http://127.0.0.1:${server.address().port}/harness/view.html`;
const sceneParam = '/scene/' + encodeURIComponent(relative(ROOT, scene).replace(/[\\/]/g, '_'));

const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] });
await mkdir(out, { recursive: true });
const report = { scene: args[0], seed: +seed, shots: [] };
const trail = args.includes('--trail') ? { trail: 1 } : {};
const qs = (extra) => new URLSearchParams({ scene: sceneParam, seed, w: W, h: H, ...trail, ...(bump ? { bump } : {}), ...extra }).toString();
const waitReady = async (page, flag) => {
  await page.waitForFunction((f) => window[f] || window.__error, flag, { timeout: 180000 });
  const err = await page.evaluate(() => window.__error);
  if (err) throw new Error(err);
};
try {
  const page = await browser.newPage({ viewport: { width: W, height: H } });
  for (const cam of cams) {
    await page.goto(`${base}?${qs({ cam, t: at })}`);
    await waitReady(page, '__ready');
    const name = cam.replace(/[^a-z0-9]+/gi, '_');
    const file = join(out, `${name}.png`);
    await page.screenshot({ path: file });
    report.shots.push({ cam, file: relative(process.cwd(), file) });
    report.result = await page.evaluate(() => window.__result);
  }
  await page.close();
  if (args.includes('--video')) {
    const ctx = await browser.newContext({ viewport: { width: W, height: H }, recordVideo: { dir: out, size: { width: W, height: H } } });
    const vp = await ctx.newPage();
    await vp.goto(`${base}?${qs({ cam: cams[0], play: 1 })}`);
    await waitReady(vp, '__done');
    const v = vp.video();
    await ctx.close();
    const file = join(out, `replay_seed${seed}.webm`);
    await rename(await v.path(), file);
    report.video = relative(process.cwd(), file);
  }
} finally {
  await browser.close();
  server.close();
}
console.log(JSON.stringify(report));
