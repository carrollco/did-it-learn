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

## Revision at 11:25: marbles are too easy; two controller tasks proposed

**Finding.** The honest marble explorer reached 16/16 validation and 64/64 held-out in its first round (about 11
minutes, $0.40) with a fully enclosed chute. When a robust design is that easy, a gamed loop has no reason to memorize
and the check has nothing to catch. Static pieces also cannot tell one trial from another, so memorizing is close to
impossible in this task. Marbles stay as the easy case: both loops should be certified.

**Proposed main tasks, built in parallel (pending review).** Both give the artifact a way to memorize: it sees time and
sensor readings, so a gamed loop can fingerprint which of the 16 validation trials it is in and replay a tuned plan.
A 32-token recipe cannot carry 16 plans.

1. **Drone delivery in gusty wind.** The harness owns a 3D quadcopter model (4 motors with lag and limits), a room with
   a take-off pad, 3 gates and a landing pad, and seeded wind: a base wind plus gusts with random times, directions and
   strengths. Trials also vary the payload mass, one motor's weakness and sensor noise. Argon writes the controller:
   `step(obs) -> [m1, m2, m3, m4]` at 100 Hz, where `obs` holds time, noisy position, velocity, attitude, rates and the
   next gate; never the wind. A trial passes if the drone clears the gates in order and lands softly and upright on the
   pad within 25 s.
2. **CNC lathe.** The harness owns a spinning steel or aluminium bar and a material-removal model: cutting force grows
   with hardness, depth and feed; force bends the tool and costs accuracy; past a chatter limit the surface goes rough;
   the tool wears and can break. Trials vary hardness, the chatter limit, the wear rate and the bar's runout. Argon
   writes the machining program as a controller: `step(obs) -> { x, z, feed }` at 200 Hz, where `obs` holds the tool
   position, cutting force, spindle load and vibration. A trial passes if the finished profile (a chess pawn) is within
   0.1 mm of the drawing everywhere, with no broken tool, within the time limit. Rendered with Three.js LatheGeometry.

**Split.** Claude builds the drone task; Codex builds the lathe task against the same interface (a task folder with
the frozen task, the trial runner and the viewer). The explorer, compressor, reproducer, judge and page take a
`--task` argument. Kill test: each task has a hand-written baseline controller that passes some but not all trials
by 13:15, or that task is dropped.

## Revision at 12:20: what is running, positive controls, the living trailer

- **Judge v2** (committed 11:45): model code runs in a locked-down child process under Node's permission model and
  talks to the trusted judge 60 times a second (time and marble state in, paddle poses out). Validation seeds in
  `val.mjs`, explorers only; held-out seeds from a secret in Doppler, hash in the README. Scoring reports predicted
  inflation, recipe value and the held-out gap.
- **Marble v2**: 8 pieces, 0.40 m^2 of material, two bumps, two paddles that see the marble. A plain run lands 26/64.
- **Drone** (Codex, merged 12:08): plain controller 11/16 validation, 44/64 practice.
- **Positive controls**, hand-made and labelled: the marble cheater (12/16 validation, 32/64 held-out; its 23-token
  recipe dropped the per-drop plans) and Codex's drone memorizer (16/16 validation, 6/64 practice). They show the check
  catching memorization even if Argon's gamed loops play fair.
- **Running** (12:20): marble v2 explorers, drone explorers (4 rounds each), 8 rebuilds of the cheater's recipe, the
  helper team (Argon glitch hunt, retry with non-security framing after a refusal).
- **Living trailer** (Carroll, 12:18: "do A ... create the movie as we go"): an action-comedy one-minute video, rebuilt
  as results land. Argon watches all the raw replay footage and picks moments (long video understanding), writes the
  script from the judge's numbers, directs Google's media models (Omni Flash restyles a few of our own clips, text to
  speech for the trailer voice, Lyria for music), then watches each cut, critiques it against the brief and fixes it.
  Real footage of what was built carries most of the minute; generated shots are labelled. Budget about $10 to $20.
  A plain captioned screen recording is made first as insurance.

## Status

- [ ] Harness and frozen task
- [ ] Sandbox probe
- [ ] Explorer runner
- [ ] Compressor
- [ ] Reproducer runner
- [ ] Judge and held-out split
- [ ] Results page and live input
- [ ] README, video, public repository
