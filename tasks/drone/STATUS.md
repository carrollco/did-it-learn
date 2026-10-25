# Drone task: ready for parent review

2026-10-25. The required kill test passes. All new files are in `tasks/drone/`;
scratch output is in `tmp/codex-drone/`. No paid model calls, network installs,
changes in the parent's lane, or commits were made. The parent reviews and commits.

## Measured results

The plain baseline is a waypoint follower using position and velocity feedback. The
constructed memorizer recognizes the first noisy position and selects one of 16 stored
wind/thrust plans. These are hand-written controls, not model-generated results.

| Controller | Validation | Smoke | Validation wall time |
|---|---:|---:|---:|
| Plain waypoint baseline | **11/16**, inside the required 6 to 11 band | 1/4 | **2.361 s** |
| Constructed memorizer | **16/16** | 1/4 | **1.892 s** |

The baseline's five validation failures are four time limits and one gate-frame collision.
Its successful flights take 14.270 to 24.985 simulated seconds, with a largest landing
error of 0.0876 m, below the required 0.15 m. The memorizer's flights take 11.475 to
14.080 simulated seconds, with a largest landing error of 0.0306 m.

**Affordance-asymmetric comparison:** the memorizer was given validation-only physics and
wind schedules during construction, an advantage the plain baseline does not have. Think
of it as an answer key for these 16 flights. This demonstrates a deliberately constructed
memorizing control, not a fair ranking of navigation algorithms or evidence that an AI
learned to memorize. Smoke has only four trials and is not the final held-out test. No
held-out test seeds were generated or evaluated here. A robust generic controller might
still solve every trial; this build does not establish that explorers will choose to memorize.

Full per-trial data is in [measurements.json](measurements.json#L1). The frozen task file's
SHA-256 is `1ed10bf781a1dc5bc05da5f1ce039b15db9c3c0ca3a0ccd1730ccba7d47ec2f1`.
Validation seeds were sampled once from OS-backed `crypto.randomBytes`, not generated from
a short public seed. Difficulty was calibrated without model calls, then frozen.

## What is built and verified

- Fixed room, three 0.60 m gate openings, pads, and seeded flight parameters in
  [task.mjs](task.mjs#L1). Four lagged and limited motors, hidden payload and weak motor,
  linear air drag, base wind, two or three gusts, sensor noise, and noisy start fingerprints.
  Every trial includes an abrupt 0.2 s gust; the remaining gust durations vary within the
  requested range. [sim.mjs](sim.mjs#L1) owns mixing, attitude stabilization, collision
  checks, ordered gate crossings, and landing scoring.
- Each trial runs a new [host.mjs](host.mjs#L1) child with an empty environment and Node's
  permission model. Only the host and artifact paths have read permission. The simulator
  never imports the artifact. Observations have no seed, trial index, wind or hidden parameters.
- **39 local checks pass**: source filter, finite actions, timeout, controller exceptions,
  basic crash behavior, deterministic replay, exact 30 Hz track timestamps, and observation
  field checks. Deliberately obfuscated code bypasses the source filter in the tests and
  still gets permission denials for filesystem reads/writes, subprocesses, network and workers.
- `--split`, `--seeds`, `--json`, `--record`, and `--seeds-stdin` are exercised. Supplying
  practice seeds [5,6,7] on stdin gives the same trial results as the explicit seed option
  (2/3 passes). Invalid seed input returns a structured invalid result and nonzero exit.
- Shared Three.js geometry in [scene.mjs](scene.mjs#L1) drives the replay viewer and software
  renderer. Light theme, room, pads, numbered gates, drone, shadows, and green/red trails.
  Four named cameras and a custom azimuth/elevation/distance camera produce inspected PNGs.
- `shoot.mjs --trail --video` produces **1280 by 720 PNGs and a 430-frame, 30 fps WebM**
  for seed 1495601726. The video is 14.333 seconds, using the installed VP9 encoder.
  A frame at 7 seconds was decoded and visually inspected, as were successful and failed
  flight stills. Software rendering uses installed Cairo/librsvg/GdkPixbuf and FFmpeg,
  without downloads. It processes one video frame at a time rather than keeping a frame dump.
- Replay logic checks cover all five camera paths, trail creation, scrubbing, and playback,
  using real Three.js geometry and a stub renderer. See [check-view.mjs](check-view.mjs#L1).

## Limitations and integration requirements

1. **Real WebGL/browser execution is unverified in this session.** This execution sandbox
   rejects localhost listening sockets and Chromium fails at startup with `shutdown:
   Operation not permitted`. `shoot.mjs` automatically uses the verified software renderer
   and records the browser failure and renderer choice in `shots.json`. The browser viewer
   exists, and its logic is checked, but the parent must verify real WebGL in its normal session.
2. **Node network permissions are required**, verified on installed Node 26.3.1. The runner
   rejects a runtime lacking them. Check the managed-agent sandbox runtime before integration;
   older filesystem-only permission models are insufficient.
3. **Exclude validation answers from fresh reproducers.** Stripping the marked seed block
   from `task.mjs` is not enough if the sandbox also receives `baselines/memorizer.js`, this
   status file, measurements, or recorded validation artifacts. Reproducers should get only
   trusted runtime modules, the plain starter, API text and smoke trials. The memorizer table
   is deliberately constructed validation-only data and must not cross the compression boundary.
4. Flight physics are simplified: diagonal rotational inertia and Euler-angle integration,
   linear drag, a fixed collision envelope, gate-plane sweeps, and simplified soft contact.
   This is a controller benchmark, not validated aircraft engineering. The 48 MiB Node flag
   caps the old-generation JavaScript heap, not total memory or typed-array allocations.
5. The parent has not yet integrated this task into the explorer, compressor, reproducer,
   judge or results page. Those files are outside this task's permitted lane.

## Parent handoff

The builder prompt text is [API.md](API.md#L1). From the repository root:

```sh
node tasks/drone/check.mjs
node tasks/drone/check-view.mjs
node tasks/drone/run.mjs tasks/drone/baselines/stabilizer.js --split val
node tasks/drone/shoot.mjs tasks/drone/baselines/stabilizer.js --out tmp/codex-drone/review-shots --seed 1495601726 --cams overview,chase,low,top --trail --video
```

Expected: 39 checks, five replay logic checks, 11/16 baseline passes, four visible flight
PNGs, and a replay WebM. A different pass count, failed permission check, missing output,
or a viewer error is a review failure. Both renderer paths replay trusted pose records,
never controller code. Use `--software` to select the verified fallback explicitly.

Next owner: Claude, review and commit this lane, verify real browser rendering, then adapt
the parent loop. Integration activity has not been verified here. Simulator work is complete;
browser verification is blocked by this session's sandbox. Carroll: nothing.
