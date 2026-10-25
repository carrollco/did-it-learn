// Frozen benchmark definition. Change difficulty only before explorer/model runs.
export const TASK = Object.freeze({
  version: 'drone-1', dt: 0.01, substeps: 2, duration: 25, gravity: 9.81,
  room: Object.freeze({ minX: -3, maxX: 3, minZ: -3, maxZ: 3, height: 3 }),
  baseMass: 0.6, payloadMax: 0.25, maxMotorThrust: 4.0,
  motorLagMin: 0.02, motorLagMax: 0.06, motorWeaknessMax: 0.25,
  arm: 0.115, inertia: 0.006, yawCoefficient: 0.014, linearDrag: 0.42,
  bodyRadius: 0.175, bodyHeight: 0.07, gateOpening: 0.60, gateFrame: 0.07,
  baseWindMax: 0.9, gustMin: 1, gustMax: 4, gustDurationMin: 0.2,
  gustDurationMax: 1.5, gustStartMin: 2.0, gustStartMax: 12.0,
  positionNoise: 0.001, velocityNoise: 0.018, attitudeNoise: 0.003,
  rateNoise: 0.012, startOffset: 0.045,
  contactCrashSpeed: 0.85, flipAngle: 1.2, landingRadius: 0.15,
  landingSpeed: 0.5, landingTilt: 0.18, landingHold: 0.25,
});

const v = (x, y, z) => Object.freeze({ x, y, z });
export const TAKEOFF = v(-2.1, 0, -1.65);
export const PAD = Object.freeze({ centre: v(2.05, 0, 1.65) });
export const GATES = Object.freeze([
  Object.freeze({ centre: v(-1.15, 1.05, -0.85), normal: v(1, 0, 0) }),
  Object.freeze({ centre: v(0.15, 1.40, 0.25), normal: v(1, 0, 0) }),
  Object.freeze({ centre: v(1.35, 1.05, 1.05), normal: v(1, 0, 0) }),
]);

export const SPLITS = {
  smoke: [101, 102, 103, 104],
  // VALIDATION_SEEDS_START
  val: [2711538435, 3107713414, 2893621486, 575937923, 3575332085, 1495601726, 324394184, 3874630936, 2252910516, 1001788019, 1574008760, 4220499815, 2368392552, 1627644072, 2910676638, 4100957300],
  // VALIDATION_SEEDS_END
};

export function random(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export function trialParameters(seed) {
  const r = random(seed), between = (a, b) => a + (b - a) * r();
  const offset = { x: between(-TASK.startOffset, TASK.startOffset), z: between(-TASK.startOffset, TASK.startOffset) };
  const mass = TASK.baseMass + between(0, TASK.payloadMax);
  const lag = between(TASK.motorLagMin, TASK.motorLagMax);
  const weakMotor = Math.floor(r() * 4), weakness = between(0, TASK.motorWeaknessMax);
  const angle = between(0, 2 * Math.PI), strength = between(0, TASK.baseWindMax);
  const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
  const gusts = Array.from({ length: r() < 0.5 ? 2 : 3 }, (_, i) => {
    const start = between(TASK.gustStartMin, TASK.gustStartMax);
    // Every trial has one abrupt 200 ms gust, shorter than the navigation response.
    const duration = i === 0 ? TASK.gustDurationMin : between(TASK.gustDurationMin, TASK.gustDurationMax);
    const a = between(0, 2 * Math.PI), s = between(TASK.gustMin, TASK.gustMax);
    return { start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s };
  });
  return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
}
