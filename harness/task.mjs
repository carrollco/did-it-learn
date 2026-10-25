// The frozen task, marble run v2: a marble run on a desk with a material budget, two table bumps, and moving paddles
// that may watch the marble. Units are metres, seconds and kilograms. The desk top is the plane y = 0.
// Validation seeds live in val.mjs; held-out seeds are supplied by the judge on stdin and never appear here.

export const TASK = {
  name: 'desk-marble-run-v2',
  goal: 'Build a marble run on an office desk that carries a dropped marble into the cup, even when the table gets bumped twice.',
  desk: { minX: -1.0, maxX: 1.0, minZ: -0.5, maxZ: 0.5 },
  bounds: { minX: -1.1, maxX: 1.1, minY: 0.0, maxY: 1.2, minZ: -0.6, maxZ: 0.6 },
  marble: { radius: 0.025, mass: 0.05, start: [-0.85, 0.8, -0.3] },
  cup: { center: [0.8, 0, 0.3], innerRadius: 0.07, wallHeight: 0.06, wallThickness: 0.012, segments: 20 },
  duration: 12.0,      // seconds simulated per trial; success is judged at the end
  dt: 1 / 480,         // fixed physics step
  controlHz: 60,       // how often paddles see the marble and choose a new pose
  budget: {
    pieces: 8, movers: 2, minSize: 0.012, maxSize: 1.1,
    material: 0.40,    // m^2: sum over static pieces of the area of each piece's largest face
    moverMaxSize: 0.25, moverMaxSpeed: 1.0, moverMaxSpin: 6.0,
  },
};

// Deterministic PRNG (mulberry32), so a seed always gives the same trial on every machine.
export function rng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// One trial: where the marble is dropped, how it moves, how grippy and bouncy the run is, and two table bumps.
export function trial(seed) {
  const r = rng(seed * 7919 + 17);
  const u = (lo, hi) => lo + (hi - lo) * r();
  const bump = (t0, t1, k0, k1) => {
    const a = u(0, 2 * Math.PI), k = u(k0, k1);
    return { time: u(t0, t1), dv: [k * Math.cos(a), u(0, 0.4), k * Math.sin(a)] };
  };
  return {
    seed,
    startOffset: [u(-0.03, 0.03), 0, u(-0.03, 0.03)],
    startVelocity: [u(-0.15, 0.15), 0, u(-0.15, 0.15)],
    friction: u(0.02, 0.3),
    restitution: u(0.05, 0.45),
    bumps: [bump(1.0, 5.0, 0.4, 1.2), bump(5.0, 9.0, 0.2, 0.8)],
  };
}

export const SMOKE = [101, 102, 103, 104];

// Camera angles the explorer's critic sees (validation views). Held-out views are chosen elsewhere.
export const VAL_CAMERAS = [
  { name: 'overview', position: [1.6, 1.3, 1.6], target: [0, 0.3, 0] },
  { name: 'closeup', position: [0.35, 0.45, 0.75], target: [0.6, 0.1, 0.2] },
  { name: 'low', position: [0.0, 0.08, 1.4], target: [0, 0.15, 0] },
  { name: 'top', position: [0.0, 2.4, 0.01], target: [0, 0, 0] },
];
