// Renders the trailer's visual pieces in headless Chromium.
//   node trailer/render.mjs card  <out.png> <title> <subtitle>      full-frame title or end card (light theme)
//   node trailer/render.mjs caption <out.png> <text>               transparent lower-third caption
//   node trailer/render.mjs page <out.webm> <url> <h2 text> <seconds>  screen recording that scrolls to a section and holds
import { chromium } from '../harness/node_modules/playwright/index.mjs';
import { rename } from 'node:fs/promises';
import { dirname } from 'node:path';

const [mode, out, ...rest] = process.argv.slice(2);
const W = 1280, H = 720;
const esc = (s) => String(s).replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
const browser = await chromium.launch({ args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
try {
  if (mode === 'card') {
    const [title, sub] = rest;
    const p = await browser.newPage({ viewport: { width: W, height: H } });
    await p.setContent(`<body style="margin:0;width:${W}px;height:${H}px;display:grid;place-items:center;background:#f3f1ec;font-family:system-ui,sans-serif">
      <div style="text-align:center;max-width:1000px"><div style="font-size:76px;font-weight:800;letter-spacing:-1px;color:#1b1b1b">${esc(title)}</div>
      <div style="font-size:30px;color:#52514e;margin-top:22px;line-height:1.35">${esc(sub || '')}</div></div></body>`);
    await p.screenshot({ path: out });
  } else if (mode === 'caption') {
    const [text] = rest;
    const p = await browser.newPage({ viewport: { width: W, height: H } });
    await p.setContent(`<body style="margin:0;width:${W}px;height:${H}px;background:transparent;font-family:system-ui,sans-serif">
      <div style="position:absolute;left:50%;bottom:22px;transform:translateX(-50%);max-width:1100px;background:rgba(20,20,20,.82);color:#fff;
      font-size:28px;font-weight:650;padding:10px 22px;border-radius:12px;text-align:center;line-height:1.3">${esc(text)}</div></body>`);
    await p.screenshot({ path: out, omitBackground: true });
  } else if (mode === 'page') {
    const [url, heading, seconds] = rest;
    const ctx = await browser.newContext({ viewport: { width: W, height: H }, recordVideo: { dir: dirname(out), size: { width: W, height: H } } });
    const p = await ctx.newPage();
    await p.goto(url, { waitUntil: 'networkidle' });
    if (heading && heading !== '-') {
      await p.evaluate((h) => {
        const el = [...document.querySelectorAll('h1,h2')].find((e) => e.textContent.includes(h));
        if (el) window.scrollTo({ top: el.getBoundingClientRect().top + scrollY - 20, behavior: 'instant' });
      }, heading);
    }
    await p.waitForTimeout(+seconds * 1000 + 600);
    const v = p.video();
    await ctx.close();
    await rename(await v.path(), out);
  } else throw new Error(`unknown mode ${mode}`);
} finally {
  await browser.close();
}
