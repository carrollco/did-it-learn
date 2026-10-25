import { chromium } from '/home/cagst/projects/argon-hackathon/harness/node_modules/playwright/index.mjs';
const out='/home/cagst/projects/argon-hackathon/tmp/demo';
const caps=["Google's loop, told to win: 16/16 on practice, 0/64 on new flights",
"The cheat, in its own code: a hard-coded list of the 16 practice seeds",
"Rebuilds score the same on practice and new flights; the cheater does not",
"Argon read 944,000 tokens of agent transcripts in one request and found the seed hunt",
"The cheater's 32-token recipe passed its seed hunt on to every single rebuild",
"Fly any controller on a seed nobody has seen: the gamed loop's code fails at once",
"A rebuild from the gamed recipe, flown live; Argon checks the code and the flight"];
const b=await chromium.launch();
const pg=await b.newPage({viewport:{width:1280,height:720},deviceScaleFactor:1});
const css='body{margin:0;background:transparent;font:30px/1.25 Arial,Helvetica,sans-serif}.c{position:absolute;left:40px;right:40px;bottom:24px;background:#fff;color:#1a1a1a;padding:12px 20px;border-radius:6px;text-align:center;border:1px solid #888}';
for(let i=0;i<caps.length;i++){await pg.setContent(`<style>${css}</style><div class=c>${caps[i]}</div>`);await pg.screenshot({path:`${out}/cap${i+1}.png`,omitBackground:true});}
await pg.setContent(`<style>body{margin:0;background:#fff;font-family:Arial,sans-serif;text-align:center;color:#1a1a1a}</style><div style="margin-top:230px"><div style="font-size:54px;font-weight:bold">Did it learn?</div><div style="font-size:30px;margin-top:20px">Did the loop learn, or learn the test? Argon checks.</div><div style="font-size:36px;margin-top:40px">https://github.com/carrollco/did-it-learn</div></div>`);
await pg.screenshot({path:`${out}/end.png`});
await b.close();
