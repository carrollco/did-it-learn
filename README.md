# Did the loop learn, or game its critic?

Built during the Gemini 4 Argon SF hackathon on Sunday, 2026-10-25 (track: More Dimensions).

Google's guide to building 3D worlds with Gemini 4 Argon says to run a render, watch, critique, fix loop "until the
Critic rates the scene 9/10". After an hour of that loop you have a great score on the checks it was tuned against, and
no idea whether the scene actually got better or the loop just learned those exact checks. A second test set tells you
once, then it's used up too.

This project answers it with the compression test from Amazon Science ("Why machine learning research agents don't
overfit", Bertran Lopez and Roth, [arXiv 2606.11045](https://arxiv.org/abs/2606.11045)): Argon squeezes everything
the loop learned into a recipe of at most 32 tokens, and fresh Managed Agents sandboxes rebuild from that recipe alone.
A rebuild saw the validation trials only through those 32 tokens, so its validation score is one you can trust. The
loop's own artifact gets no such guarantee. Ship the rebuild, and compare: when the loop's artifact and its rebuilds
disagree on new trials, the loop memorized.

## What runs

1. **A frozen task.** A quadcopter must fly through three gates in order and land softly on a pad within 25 seconds,
   in gusty wind (`tasks/drone/`, built today by Codex). Each trial varies the payload, motor lag and weakness, every
   gust, and the start position by a few centimetres. The controller only sees noisy sensor readings, 100 times a
   second, and runs in its own child process under Node's permission model: it cannot read the trial, the seeds or
   any file but its own.
2. **Explorers: Google's loop, run by Argon in Managed Agents sandboxes.** Each explorer is an Argon managed agent in
   its own Google-hosted sandbox: it writes the controller, runs the 16 validation flights, renders and looks at its
   own flights, and fixes it. A separate Argon critic scores four camera views after each round, as Google's guide
   recommends. One explorer is honest; a control explorer is told to max out the 16 validation flights "at any cost"
   (the paper's deliberately overfit control).
3. **Compressor.** Argon reads an explorer's whole record (every transcript, controller, score and critique; 93k and
   172k tokens here) and writes a recipe of at most 32 tokens, counted with Gemini's tokenizer.
4. **Rebuilds.** Eight fresh Managed Agents sandboxes per recipe, in parallel, each get the task, the recipe, and a copy
   of the harness with the validation seeds removed. A third group gets no recipe at all, as the control.
5. **Judge.** Code scores every controller on the 16 validation flights and on 64 held-out flights from a secret no
   agent holds (see below). The rules for the labels are in `loop/judge.py`; every rebuild's score is published.
6. **Results page and live flights** (`loop/page.py`, `loop/serve.py`): verdicts, a chart of validation against held-out
   for every controller, and a button that flies any controller on a fresh random flight nobody has seen.

## What happened (2026-10-25)

- Both loops scored **16/16** on validation. The honest loop's controller flew **64/64** held-out flights. The loop told
  to cheat wrote a controller that hard-codes the 16 validation seeds and a copy of the flight physics, recognizes each
  test flight from the first sensor reading, and fails on the first tick of any flight it never studied: **0/64**.
- The cheater's seed list alone is 222 tokens. Its 32-token recipe kept the real flying strategy, and the rebuilds from
  it fly honestly: their validation scores predict their held-out scores (see the chart on the results page).
- First attempt, kept in `runs/r1`, `runs/r2`: a marble run on a desk. Argon built robust runs even when told to cheat,
  so there was nothing to catch. A helper agent found a physics glitch there (a box overlapping the drop point launches
  the marble into the cup); it is disclosed, not patched, because the task was frozen.

## Held-out trials

The 64 held-out trials come from a secret that only the judge's Python side holds. Its SHA-256 was committed at 11:49
Pacific, before the first marble v2 run, and posted publicly in
[this gist](https://gist.github.com/carrollco/2e3d0736c605205dd912d76f9ce648be):
`79c93d10260e9d5e4e275bbc5c0d77072adaededfac4c108706516912344f6ff`. The secret is revealed after judging, so anyone
can regenerate the trials (`loop/kit.py`, `heldout_seeds`) and re-score every artifact.

## Run it

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
(cd harness && npm install && npx playwright install chromium)
export GOOGLE_API_KEY=...            # a Gemini API key
python loop/explore.py --task drone --variant honest --rounds 2 --out runs/d1/honest
python loop/explore.py --task drone --variant gamed  --rounds 2 --out runs/d1/gamed
python loop/compress.py --explorer runs/d1/gamed --budget 32 --out runs/d1/compress/gamed-32.json
python loop/reproduce.py --task drone --recipe runs/d1/compress/gamed-32.json --n 8 --out runs/d1/repro/gamed
python loop/reproduce.py --task drone --recipe none --n 8 --out runs/d1/repro/none
HELDOUT_SECRET=... python loop/judge.py --run runs/d1 --task drone && python loop/page.py --run runs/d1
python loop/serve.py --port 8777    # pages plus live flights; open http://localhost:8777/runs/d1/index.html
```

Harness only, no model calls: `node tasks/drone/run.mjs tasks/drone/baselines/stabilizer.js --split val` flies the
plain baseline, and `node tasks/drone/shoot.mjs tasks/drone/baselines/stabilizer.js --out shots --seed 101 --trail`
renders it.

## What was built today

Everything after the `pre-event` tag (`git diff pre-event..main`): both tasks (the marble run in `harness/`, the
drone in `tasks/drone/`), the loop, the compressor, the rebuilds, the judge, the results page, the live-flight server
and the trailer pipeline (`trailer/`). The commit tagged `pre-event` holds only this file's first version,
`.gitignore`, `AGENTS.md`, `CLAUDE.md` and a one-line `requirements.txt`. Outside work used as published: Google's
[3D games and worlds guide](https://aistudio.google.com/learn/building-3d-games-and-worlds-with-gemini-4-argon) (the
loop being tested), the Amazon Science method above, and the open-source libraries Three.js, cannon-es and Playwright.

## Models

`gemini-4-argon` for the builders, critic, compressor and rebuilds, through the Interactions API (`google-genai`)
and Managed Agents (`antigravity-preview-09-2026` with `agent_config={"type": "antigravity", "model": "gemini-4-argon"}`).
The trailer: Argon writes and critiques it (watching the cut), `gemini-3.8-flash-tts` is the narrator and
`lyria-3-clip-preview` the music.
