# Did the loop learn, or game its critic?

Built during the Gemini 4 Argon SF hackathon on Sunday, 2026-10-25 (track: More Dimensions).

Google's guide to building 3D worlds with Gemini 4 Argon says to run a render, watch, critique, fix loop "until the
Critic rates the scene 9/10". After an hour of that loop you have a great score on the checks it was tuned against, and
no idea whether the scene actually got better or the loop just learned those exact checks. A second test set tells you
once, then it's used up too.

This project answers it with the compression test from Amazon Science ("Why machine learning research agents don't
overfit", Bertran Lopez and Roth, [arXiv 2606.11045](https://arxiv.org/abs/2606.11045)): Argon squeezes everything
the loop learned into a recipe of at most 32 tokens, and fresh Managed Agents sandboxes rebuild the scene from that
recipe alone. Memorised tricks don't fit through 32 tokens; real strategy does. If the rebuilds score like the
original, the loop learned something. If the gains vanish, it gamed its checks.

## What runs

1. **A frozen 3D task.** A marble run on an office desk, in Three.js with cannon-es physics. The harness owns the desk,
   the cup, the marble and the physics; a scene can only add pieces through a small API with no handle on the marble.
   A trial drops the marble with random offset, spin, grip and bounce, and bumps the table once at a random moment.
2. **Explorers: Google's loop, run by Argon Managed Agents.** Each explorer is an Argon managed agent working in its
   own Google-hosted sandbox for several rounds: it writes the scene, runs the 16 validation trials, renders the scene
   and looks at its renders, and fixes it. After each round a separate Argon critic scores four camera views, as
   Google's guide recommends. One explorer is told to build for any drop; a control explorer is told to max out the 16
   validation trials "at any cost" (the paper's deliberately overfit control).
3. **Compressor.** Argon reads an explorer's whole record (every transcript, scene, score and critique) and writes a
   recipe of at most 32 tokens, counted with Gemini's tokenizer.
4. **Reproducers.** Fresh Managed Agents sandboxes, in parallel, each get the task, the recipe, and a copy of the
   harness with the validation seeds removed. The clean sandbox is what makes the test valid: nothing else crosses.
5. **Judge.** Code scores every scene on the 16 validation trials and on 64 held-out trials from a secret no agent
   holds, and renders camera angles no critic saw. The verdict rule was fixed before any results: certified when the
   reproducers' median validation score is within 2 trials of the explorer's best.

## Run it

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
(cd harness && npm install && npx playwright install chromium)
export GOOGLE_API_KEY=...            # a Gemini API key
python loop/explore.py --variant honest --rounds 4 --out runs/r1/honest
python loop/explore.py --variant gamed  --rounds 4 --out runs/r1/gamed
python loop/compress.py --explorer runs/r1/honest --budget 32
python loop/reproduce.py --recipe runs/r1/compress/honest-32.json --n 8 --out runs/r1/repro/honest
python loop/judge.py --run runs/r1 && python loop/page.py --run runs/r1
python3 -m http.server 8777          # open http://localhost:8777/runs/r1/index.html
```

Harness only, no model calls: `node harness/run.mjs scenes/handmade-chute.js --split val` runs the trials, and
`node harness/shoot.mjs scenes/handmade-chute.js --out shots --trail` renders the four validation views.

## What was built today

Everything after the `pre-event` tag (`git diff pre-event..main`): the harness, the loop, the compressor, the
reproducers, the judge and the results page. The commit tagged `pre-event` holds only this file's first version,
`.gitignore`, `AGENTS.md`, `CLAUDE.md` and a one-line `requirements.txt`. Outside work used as published: Google's
[3D games and worlds guide](https://aistudio.google.com/learn/building-3d-games-and-worlds-with-gemini-4-argon) (the
loop being tested), the Amazon Science method above, and the open-source libraries Three.js, cannon-es and Playwright.

## Models

`gemini-4-argon` for the builders, critic, compressor and reproducers, through the Interactions API (`google-genai`)
and Managed Agents (`antigravity-preview-09-2026` with `agent_config={"type": "antigravity", "model": "gemini-4-argon"}`).
