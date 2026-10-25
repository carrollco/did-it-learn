# Plan: did the loop learn, or did it game its critic?

Built on 2026-10-25 at the Gemini 4 Argon SF hackathon. Track: More Dimensions (render, watch, critique, fix in a
loop), with a measured baseline.

## The problem

Google's guide to building 3D worlds with Argon tells you to loop "until the Critic rates the scene 9/10". After
hours of an autonomous loop, nobody can tell whether the 9/10 is real improvement or the loop learning the quirks of
the exact checks it was graded on. Holding back a second test set works once; every look at it uses it up.

## The idea

The compression test from Amazon Science, "Why machine learning research agents don't overfit"
(Bertran Lopez and Roth, 2026-09-10; paper arXiv 2606.11045): squeeze what an agent learned into about 32 tokens and
let a fresh agent with no other access rebuild from that alone. Memorized specifics do not fit through 32 tokens; real
strategy does. If the fresh rebuilds score as well as the original, the gains were learned. If the gains vanish, the
loop memorized its validation set.

## The pieces

1. **Task and harness (frozen before any model runs).** A 3D marble run on a desk in Three.js with cannon-es physics.
   The harness owns the marble, the cup and the desk; Argon's scene code may only add static and time-scripted pieces
   through a small API, and it cannot see the marble. A trial is a seeded drop: start offset, spin, friction,
   bounciness and a random table bump. Splits: 16 validation trials (the explorer sees these), 4 smoke trials (the
   reproducers see only these), 64 held-out test trials (only the final judge sees these; seeds come from a secret).
2. **Explorers (Managed Agents, Argon).** Google's loop inside a sandbox: write the scene, run the 16 validation
   trials, look at the renders, fix, repeat. One honest explorer; one told to maximize the 16 trials "at any cost"
   (the paper's deliberately overfit control).
3. **Compressor (Argon).** Reads the explorer's whole record and writes a recipe of at most 32 tokens.
4. **Reproducers (Managed Agents, Argon, fresh sandboxes).** Each gets the task, the harness with only the 4 smoke
   trials, and the recipe. Nothing else crosses over; the clean sandbox is the bottleneck.
5. **Judge (code).** Scores every scene on validation and on the held-out test set, and renders camera angles the
   loop never saw. Verdict: certificate (reproducers match the explorer) or flag (gains vanish).
6. **Results page.** Progress gallery of each loop, the recipe, the grid of reproductions, the verdict, and a live
   input: a judge's own camera angle and table bump, run on the spot.

## Kill tests

- 11:45: the harness runs trials headless and renders a hand-written scene. The probe shows whether the sandbox can
  render; if not, renders happen on the laptop and the sandbox runs physics only.
- 13:30: one honest and one gamed explorer have finished. If the gamed one shows no gap between validation and test,
  give both explorers time-scripted movers (paddles), which make memorizing the 16 trials possible.
- 15:00: 8 reproductions per explorer scored; results page shows both verdicts.
- 16:30: repository public, video recorded.

## Status

- [ ] Harness and frozen task
- [ ] Sandbox probe
- [ ] Explorer runner
- [ ] Compressor
- [ ] Reproducer runner
- [ ] Judge and held-out split
- [ ] Results page and live input
- [ ] README, video, public repository
