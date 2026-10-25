// Render one SVG file to a PNG with headless Chromium: node render.mjs in.svg out.png
import { chromium } from '/home/cagst/projects/argon-hackathon/harness/node_modules/playwright/index.mjs';
import { readFile, mkdir } from 'node:fs/promises';
import { resolve } from 'node:path';

const [svgPath, pngPath] = process.argv.slice(2);
const svg = await readFile(svgPath, 'utf8');
process.env.TMPDIR = resolve(process.cwd(), 'tmp/diagram-browser');
await mkdir(process.env.TMPDIR, { recursive: true });
const browser = await chromium.launch();
const page = await browser.newPage({ viewport: { width: 1200, height: 560 }, deviceScaleFactor: 2 });
await page.setContent(`<!doctype html><html><head><meta charset="utf-8"><style>html,body{margin:0;background:#fff}svg{display:block;width:1200px;height:560px}</style></head><body>${svg}</body></html>`);
await page.waitForTimeout(200);
await page.screenshot({ path: pngPath, clip: { x: 0, y: 0, width: 1200, height: 560 } });
await browser.close();
console.log(pngPath);
