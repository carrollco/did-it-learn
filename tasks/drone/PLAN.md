# Drone implementation plan

Scope: a new, simplified drone benchmark, entirely within `tasks/drone/`. No paid calls,
installs, copies from other projects, or commits. The parent reviews and commits this landing.

1. Build fixed geometry and seeded flight parameters, a trusted flight simulator, and a
   controller child process with permissions and a strict JSON protocol.
2. Tune the plain waypoint controller to 6 through 11 successes on 16 validation trials by
   adjusting task difficulty. Freeze those numbers before any model exploration.
3. Build a track-only Three.js viewer and headless screenshot/video tool.
4. Check CLI formats, withheld seed input, failure handling, filesystem/network isolation,
   runtime, and visible flight renders. Record measured results and limitations in STATUS.md.
5. If time permits, add a clearly constructed controller that recognizes validation starts.

Status: steps 1 through 5 complete. The measured kill test, constructed memorizer,
permissions checks, screenshots and software video are recorded in [STATUS.md](STATUS.md#L1).
Real WebGL verification remains with the parent because this session's execution sandbox
prevents Chromium startup. Nothing was committed, per the task's explicit instruction.
