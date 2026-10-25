The artifact is one JavaScript file, `controller.js`, exporting:

```js
export default function makeController() {
  return {
    step(obs) {
      return { thrust: 0.45, roll: 0, pitch: 0, yawRate: 0 };
    },
  };
}
```

This example only illustrates the interface, it is not a flight controller. Start from
`tasks/drone/baselines/stabilizer.js`, a working plain waypoint follower. You can run it,
inspect results and pictures, change it, and repeat. Every trial creates a fresh controller.

Fly through three numbered square gate openings in order, then land on the landing pad,
within 25 seconds. The room spans x and z from -3 to 3 metres and y from 0 to 3 metres.
Gravity is 9.81 metres per second squared. Up is +y. Gate planes face +x, and only a
crossing from smaller x to larger x counts. Each opening is 0.60 metres wide and high;
the drone's horizontal collision half-width is 0.175 m and vertical half-height 0.07 m,
so its whole body must fit through. Gate frames are physical, the spaces outside them
are open. A missed gate is still the next gate until you return and cross it correctly.

The harness owns the inner attitude stabilizer and four motor mixer. You control navigation,
collective thrust, desired tilt, and yaw rate at 100 calls per simulated second. The simulation
integrates twice per call to resolve motor lag and gate crossings. It is a simplified benchmark,
not a validated model of a real aircraft.

`step(obs)` receives only these fields, all distances in metres, time in seconds, angles
in radians, speeds in metres per second, and angular rates in radians per second:

- `t`: elapsed simulated time, starting at 0, increasing by 0.01 seconds.
- `pos: {x, y, z}`: noisy drone centre position. A resting centre is at y = 0.07.
- `vel: {x, y, z}`: noisy velocity.
- `att: {roll, pitch, yaw}`: noisy attitude angles. At yaw zero, positive pitch tilts
  thrust toward +x, positive roll tilts it toward -z. Positive yaw rotates +x toward -z.
- `rates: {roll, pitch, yaw}`: noisy derivatives of the corresponding attitude angles.
- `gateIndex`: 0, 1, or 2 for the next gate; 3 after all three have been crossed.
- `nextGate: {centre: {x,y,z}, normal: {x,y,z}}`: the next gate, or `null` after the last.
  Gate centres are (-1.15, 1.05, -0.85), (0.15, 1.40, 0.25), (1.35, 1.05, 1.05).
  Normals are (1, 0, 0). Gate and pad coordinates are exact, not noisy.
- `pad: {centre: {x,y,z}}`: the landing-pad centre, (2.05, 0, 1.65).

Return a plain object with four finite numeric fields:

- `thrust`: collective motor demand, clamped to [0, 1]. Four motors can each produce
  up to 4 N. Demand is not altitude, and 0.5 is not guaranteed to hover.
- `roll` and `pitch`: desired attitude angles, each clamped to [-0.6, 0.6].
- `yawRate`: desired yaw rate, clamped to [-2, 2].

The mass is 0.6 kg plus a hidden 0 to 0.25 kg payload. Motors have a hidden 20 to 60 ms
first-order response lag, one has 0 to 25 percent less thrust, and air drag is linear.
Wind has a hidden seeded base component and two or three gusts. Every trial has an abrupt
0.2-second gust; other gusts last 0.2 to 1.5 seconds, with speeds of 1 to 4 m/s. The short
gust can move the drone before its tilted thrust catches up, so advance knowledge can help.
The starting pad position varies by a few centimetres and is visible with sensor noise in
the first observation. You never receive the seed, trial index, wind, mass, or motor parameters.

Success requires all three gates, then a centre within 0.15 m horizontally of the pad,
centre height at most 0.095 m, speed below 0.5 m/s, and roll/pitch each below 0.18 rad,
continuously for 0.25 s before the deadline. This short hold rejects a passing bounce.
Floor/wall contact faster than 0.85 m/s, a hard tilted floor landing, tilt beyond 1.2 rad,
or any gate-frame collision fails. Soft wall contact is constrained by the room boundary.

Rules: no imports, external access, or asynchronous actions. The source is rejected if it
contains any of `import`, `require`, `process`, `globalThis`, `constructor`, `prototype`,
`__proto__`, `eval`, `Function`, `Reflect`, `Proxy`, `fetch`, `Date`, `performance`, including
inside comments and strings. The child has an empty environment, permission to read only
its own source and host module, and no granted file writes, network, subprocesses, or workers.
The trusted simulator runs separately. Malformed output, a crash, or a step taking more
than 50 ms of wall time fails that trial. Keep source below 256 KiB, a source-size guard,
and the JavaScript old-generation heap below 48 MiB. This heap limit does not bound total
process memory or typed-array allocations. Do not write to stdout, which carries
the harness protocol. The supplied deterministic controller needs no randomness or clock.

Tools, from the repository root (dependencies are already installed, no installation needed).
The runner requires Node's filesystem and network permission support, verified here on
Node 26.3.1. It rejects older runtimes without network permissions rather than running
with a weaker boundary. The software renderer uses Python 3 and the installed system
Cairo, librsvg and GdkPixbuf libraries; video uses an already-installed FFmpeg encoder.

```sh
node tasks/drone/run.mjs controller.js --split smoke
node tasks/drone/run.mjs controller.js --seeds 5,6,7 --json tmp/practice.json
node tasks/drone/shoot.mjs controller.js --out tmp/drone-shots --seed 104 --cams overview,chase,low,top --trail
node tasks/drone/shoot.mjs controller.js --out tmp/drone-video --seed 104 --cams overview --trail --video
```

Explorers may also run `--split val`, the 16 validation trials. Fresh reproducers receive
only the smoke split. Held-out trials are supplied by the parent on standard input as a
JSON array using `--seeds-stdin`; this task never generates a held-out test split.

`--record tmp/tracks` writes `seed-N.json` files containing 30 fps poses and gate progress.
`--json` writes `{scene, split, valid, error?, passed, total, rate, seconds, results}`.
Each result has `{seed, success, reason, gatesPassed, landingError, simulatedSeconds}`.
`valid` means the source/CLI passed validation; execution failures appear in trial reasons.
`scene` is the controller path, retained for compatibility with the existing explorer loop.

`shoot.mjs` records the trial itself and writes four PNGs plus `shots.json`. Named cameras
are overview, chase, low, top. Custom cameras use degrees for azimuth/elevation and metres
for distance, for example `--cams '35,25,8;top'`. `--t 5` selects a still time, default `end`.
The viewer loads recorded poses only, never artifact code. Green trails indicate success,
red trails failure. The light-themed interactive page is `tasks/drone/view.html?track=...&trail=1`.
When Chromium cannot launch, still images use the same scene through local SVG/Cairo
software projection; `shots.json` identifies the renderer and any limitations.
