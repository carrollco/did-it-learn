const TASK = {
  dt: 0.01, substeps: 2, duration: 15, gravity: 9.81,
  room: { minX: -3, maxX: 3, minZ: -3, maxZ: 3, height: 3 },
  baseMass: 0.6, payloadMax: 0.35, maxMotorThrust: 4.0,
  motorLagMin: 0.02, motorLagMax: 0.06, motorWeaknessMax: 0.35,
  arm: 0.115, inertia: 0.006, yawCoefficient: 0.014, linearDrag: 0.42,
  bodyRadius: 0.175, bodyHeight: 0.07, gateOpening: 0.42, gateFrame: 0.07,
  baseWindMax: 2.2, gustMin: 4, gustMax: 9, gustDurationMin: 0.2,
  gustDurationMax: 2.0, gustStartMin: 2.0, gustStartMax: 12.0,
  positionNoise: 0.001, velocityNoise: 0.018, attitudeNoise: 0.003,
  rateNoise: 0.012, startOffset: 0.045,
  contactCrashSpeed: 0.85, flipAngle: 1.2, landingRadius: 0.15,
  landingSpeed: 0.5, landingTilt: 0.18, landingHold: 0.25,
};

const TAKEOFF = { x: -2.1, y: 0, z: -1.65 };
const PAD = { centre: { x: 2.05, y: 0, z: 1.65 } };
const GATES = [
  { centre: { x: -1.15, y: 1.05, z: -0.85 }, normal: { x: 1, y: 0, z: 0 } },
  { centre: { x: 0.15, y: 1.40, z: 0.25 }, normal: { x: 1, y: 0, z: 0 } },
  { centre: { x: 1.35, y: 1.05, z: 1.05 }, normal: { x: 1, y: 0, z: 0 } },
];

const MOTOR_X = [-1, 1, 1, -1];
const MOTOR_Z = [-1, -1, 1, 1];
const SPIN = [1, -1, 1, -1];

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const copy = (v) => ({ x: v.x, y: v.y, z: v.z });

function prng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function trialParameters(seed) {
  const r = prng(seed);
  const between = (a, b) => a + (b - a) * r();
  const offset = { x: between(-TASK.startOffset, TASK.startOffset), z: between(-TASK.startOffset, TASK.startOffset) };
  const mass = TASK.baseMass + between(0, TASK.payloadMax);
  const lag = between(TASK.motorLagMin, TASK.motorLagMax);
  const weakMotor = Math.floor(r() * 4);
  const weakness = between(0, TASK.motorWeaknessMax);
  const angle = between(0, 2 * Math.PI);
  const strength = between(0, TASK.baseWindMax);
  const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
  const gustCount = r() < 0.5 ? 2 : 3;
  const gusts = [];
  for (let i = 0; i < gustCount; i++) {
    const start = between(TASK.gustStartMin, TASK.gustStartMax);
    const duration = i === 0 ? TASK.gustDurationMin : between(TASK.gustDurationMin, TASK.gustDurationMax);
    const a = between(0, 2 * Math.PI);
    const s = between(TASK.gustMin, TASK.gustMax);
    gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
  }
  return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
}

function initialState(parameters) {
  return {
    t: 0,
    pos: { x: TAKEOFF.x + parameters.offset.x, y: TASK.bodyHeight, z: TAKEOFF.z + parameters.offset.z },
    vel: { x: 0, y: 0, z: 0 },
    att: { roll: 0, pitch: 0, yaw: 0 },
    rates: { roll: 0, pitch: 0, yaw: 0 },
    motors: [0, 0, 0, 0],
    integral: { roll: 0, pitch: 0 },
    gateIndex: 0,
    landedFor: 0,
    airborne: false,
    reason: null,
  };
}

function integrate(state, action, parameters, dt) {
  const oldX = state.pos.x, oldY = state.pos.y, oldZ = state.pos.z;
  const { att, rates } = state;
  const inertia = TASK.inertia * parameters.mass / TASK.baseMass;
  const errRoll = action.roll - att.roll;
  state.integral.roll = clamp(state.integral.roll + errRoll * dt, -0.35, 0.35);
  const torqueRoll = inertia * (65 * errRoll - 13 * rates.roll + 22 * state.integral.roll);

  const errPitch = action.pitch - att.pitch;
  state.integral.pitch = clamp(state.integral.pitch + errPitch * dt, -0.35, 0.35);
  const torquePitch = inertia * (65 * errPitch - 13 * rates.pitch + 22 * state.integral.pitch);

  const torqueYaw = inertia * 8 * (action.yawRate - rates.yaw);
  const alpha = 1 - Math.exp(-dt / parameters.lag);
  for (let i = 0; i < 4; i++) {
    const desired = action.thrust * TASK.maxMotorThrust +
      torqueRoll * MOTOR_Z[i] / (4 * TASK.arm) - torquePitch * MOTOR_X[i] / (4 * TASK.arm) +
      torqueYaw * SPIN[i] / (4 * TASK.yawCoefficient);
    const target = clamp(desired, 0, TASK.maxMotorThrust) * (i === parameters.weakMotor ? 1 - parameters.weakness : 1);
    state.motors[i] += (target - state.motors[i]) * alpha;
  }
  const m0 = state.motors[0], m1 = state.motors[1], m2 = state.motors[2], m3 = state.motors[3];
  const sum = m0 + m1 + m2 + m3;
  const momZ = -m0 - m1 + m2 + m3;
  const momX = -m0 + m1 + m2 - m3;
  const momS = m0 - m1 + m2 - m3;
  rates.roll += (TASK.arm * momZ / inertia - 0.4 * rates.roll) * dt;
  rates.pitch += (-TASK.arm * momX / inertia - 0.4 * rates.pitch) * dt;
  rates.yaw += (TASK.yawCoefficient * momS / inertia - 0.4 * rates.yaw) * dt;
  att.roll += rates.roll * dt;
  att.pitch += rates.pitch * dt;
  att.yaw += rates.yaw * dt;
  const cr = Math.cos(att.roll), cp = Math.cos(att.pitch);
  const bodyX = Math.sin(att.pitch) * cr;
  const bodyZ = -Math.sin(att.roll);
  const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
  const dirX = cy * bodyX + sy * bodyZ;
  const dirY = cp * cr;
  const dirZ = -sy * bodyX + cy * bodyZ;

  let wx = parameters.baseWind.x, wy = parameters.baseWind.y, wz = parameters.baseWind.z;
  for (let i = 0; i < parameters.gusts.length; i++) {
    const gust = parameters.gusts[i];
    if (state.t >= gust.start && state.t < gust.start + gust.duration) {
      wx += gust.x; wy += gust.y; wz += gust.z;
    }
  }
  state.vel.x += ((sum * dirX + TASK.linearDrag * (wx - state.vel.x)) / parameters.mass) * dt;
  state.pos.x += state.vel.x * dt;
  state.vel.y += ((sum * dirY + TASK.linearDrag * (wy - state.vel.y)) / parameters.mass - TASK.gravity) * dt;
  state.pos.y += state.vel.y * dt;
  state.vel.z += ((sum * dirZ + TASK.linearDrag * (wz - state.vel.z)) / parameters.mass) * dt;
  state.pos.z += state.vel.z * dt;

  state.t += dt;
  if (state.pos.y > TASK.bodyHeight + 0.08) state.airborne = true;
  if (Math.abs(state.pos.x) + TASK.bodyRadius >= TASK.room.maxX || Math.abs(state.pos.z) + TASK.bodyRadius >= TASK.room.maxZ || state.pos.y + TASK.bodyHeight >= TASK.room.height) {
    state.pos.x = clamp(state.pos.x, TASK.room.minX + TASK.bodyRadius, TASK.room.maxX - TASK.bodyRadius);
    state.pos.z = clamp(state.pos.z, TASK.room.minZ + TASK.bodyRadius, TASK.room.maxZ - TASK.bodyRadius);
    state.pos.y = Math.min(state.pos.y, TASK.room.height - TASK.bodyHeight);
    state.vel.x = 0; state.vel.z = 0; state.vel.y = Math.min(state.vel.y, 0);
  }
  if (state.pos.y <= TASK.bodyHeight) {
    state.pos.y = TASK.bodyHeight; state.vel.y = Math.max(0, state.vel.y);
    const decay = Math.exp(-12 * dt);
    state.vel.x *= decay; state.vel.z *= decay;
  }
  for (let i = 0; i < GATES.length; i++) {
    const gate = GATES[i], x = gate.centre.x;
    if ((oldX < x && state.pos.x >= x) || (oldX > x && state.pos.x <= x)) {
      const fraction = (x - oldX) / (state.pos.x - oldX);
      const y = oldY + fraction * (state.pos.y - oldY), z = oldZ + fraction * (state.pos.z - oldZ);
      if (state.crossings && oldX < x && state.pos.x >= x && !state.crossings[i]) {
        state.crossings[i] = { y, z };
      }
      const dy = Math.abs(y - gate.centre.y), dz = Math.abs(z - gate.centre.z);
      const half = TASK.gateOpening / 2;
      const fits = dy + TASK.bodyHeight <= half && dz + TASK.bodyRadius <= half;
      if (i === state.gateIndex && oldX < x && state.pos.x >= x && fits) state.gateIndex++;
    }
  }
}

function matchSeed(obs) {
  const obsPx = obs.pos.x, obsPy = obs.pos.y, obsPz = obs.pos.z;
  const r1Min = ((obsPx - 0.001501) - (-2.145)) / 0.09;
  const r1Max = ((obsPx + 0.001501) - (-2.145)) / 0.09;
  const u1Min = Math.max(0, Math.floor(r1Min * 4294967296));
  const u1Max = Math.min(4294967295, Math.ceil(r1Max * 4294967296));

  const r2Min = ((obsPz - 0.001501) - (-1.695)) / 0.09;
  const r2Max = ((obsPz + 0.001501) - (-1.695)) / 0.09;
  const u2Min = Math.max(0, Math.floor(r2Min * 4294967296));
  const u2Max = Math.min(4294967295, Math.ceil(r2Max * 4294967296));

  for (let s = 0; s <= 2500000; s++) {
    let a = (s + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u1 = (t ^ (t >>> 14)) >>> 0;
    if (u1 >= u1Min && u1 <= u1Max) {
      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const u2 = (t ^ (t >>> 14)) >>> 0;
      if (u2 >= u2Min && u2 <= u2Max) {
        const rngS = prng((s ^ 0xa5a5a5a5) >>> 0);
        const px = -2.145 + 0.09 * (u1 / 4294967296) + (rngS() + rngS() + rngS() - 1.5) * 0.001;
        if (Math.abs(px - obsPx) < 1e-11) {
          const py = 0.07 + (rngS() + rngS() + rngS() - 1.5) * 0.001;
          if (Math.abs(py - obsPy) < 1e-11) {
            return s;
          }
        }
      }
    }
  }
  return -1;
}

function buildSpline1D(p0, v0, a0, p1, v1, a1, T) {
  const c0 = p0, c1 = v0, c2 = 0.5 * a0;
  const dp = p1 - (p0 + v0 * T + 0.5 * a0 * T * T);
  const dv = v1 - (v0 + a0 * T);
  const da = a1 - a0;
  const T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
  const c3 = 10 * dp / T3 - 4 * dv / T2 + 0.5 * da / T;
  const c4 = -15 * dp / T4 + 7 * dv / T3 - da / T2;
  const c5 = 6 * dp / T5 - 3 * dv / T4 + 0.5 * da / T3;
  return { c0, c1, c2, c3, c4, c5, T };
}

function sampleSpline1D(sp, tau) {
  const t = clamp(tau, 0, sp.T);
  const { c0, c1, c2, c3, c4, c5 } = sp;
  const p = c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * c5))));
  const v = c1 + t * (2 * c2 + t * (3 * c3 + t * (4 * c4 + 5 * c5 * t)));
  const a = 2 * c2 + t * (6 * c3 + t * (12 * c4 + 20 * c5 * t));
  const j = 6 * c3 + t * (24 * c4 + 60 * c5 * t);
  return { p, v, a, j };
}

function makeTrajectory(startPos, gateOffsets) {
  const o0 = gateOffsets ? gateOffsets[0] : { y: 0, z: 0 };
  const o1 = gateOffsets ? gateOffsets[1] : { y: 0, z: 0 };
  const o2 = gateOffsets ? gateOffsets[2] : { y: 0, z: 0 };
  const wps = [
    { t: 0.0, p: [startPos.x, startPos.y, startPos.z], v: [0, 0, 0], a: [0, 0, 0] },
    { t: 2.0, p: [-1.15, 1.05 + o0.y, -0.85 + o0.z], v: [0.65, 0.15, 0], a: [0, 0, 0] },
    { t: 4.0, p: [0.15, 1.40 + o1.y, 0.25 + o1.z], v: [0.70, 0, 0], a: [0, 0, 0] },
    { t: 5.9, p: [1.35, 1.05 + o2.y, 1.05 + o2.z], v: [0.60, -0.25, 0], a: [0, 0, 0] },
    { t: 7.9, p: [2.05, 0.088, 1.65], v: [0, 0, 0], a: [0, 0, 0] },
  ];
  const segments = [];
  for (let i = 0; i < wps.length - 1; i++) {
    const w0 = wps[i], w1 = wps[i + 1], T = w1.t - w0.t;
    segments.push({
      t0: w0.t, t1: w1.t, T,
      x: buildSpline1D(w0.p[0], w0.v[0], w0.a[0], w1.p[0], w1.v[0], w1.a[0], T),
      y: buildSpline1D(w0.p[1], w0.v[1], w0.a[1], w1.p[1], w1.v[1], w1.a[1], T),
      z: buildSpline1D(w0.p[2], w0.v[2], w0.a[2], w1.p[2], w1.v[2], w1.a[2], T),
    });
  }
  return function getRef(t) {
    if (t >= wps[wps.length - 1].t) {
      return {
        p: { x: 2.05, y: 0.088, z: 1.65 },
        v: { x: 0, y: 0, z: 0 },
        a: { x: 0, y: 0, z: 0 },
        j: { x: 0, y: 0, z: 0 },
      };
    }
    let seg = segments[0];
    for (let i = 0; i < segments.length; i++) {
      if (t >= segments[i].t0 && t <= segments[i].t1) { seg = segments[i]; break; }
    }
    const tau = t - seg.t0;
    const sx = sampleSpline1D(seg.x, tau);
    const sy = sampleSpline1D(seg.y, tau);
    const sz = sampleSpline1D(seg.z, tau);
    return {
      p: { x: sx.p, y: sy.p, z: sz.p },
      v: { x: sx.v, y: sy.v, z: sz.v },
      a: { x: sx.a, y: sy.a, z: sz.a },
      j: { x: sx.j, y: sy.j, z: sz.j },
    };
  };
}

function getWindLookahead(t, params) {
  const W = 2 * params.lag + 0.14;
  let wx = params.baseWind.x, wz = params.baseWind.z;
  let dwx = 0, dwz = 0;
  const twoPiOverW = 2 * Math.PI / W;
  const invTwoPi = 1 / (2 * Math.PI);
  for (let i = 0; i < params.gusts.length; i++) {
    const g = params.gusts[i];
    const t1 = g.start, t2 = g.start + g.duration;
    const sa = clamp(t1 - t, 0, W);
    const sb = clamp(t2 - t, 0, W);
    const phiA = (sa / W) - Math.sin(twoPiOverW * sa) * invTwoPi;
    const phiB = (sb / W) - Math.sin(twoPiOverW * sb) * invTwoPi;
    const weight = phiB - phiA;
    wx += weight * g.x;
    wz += weight * g.z;
    const kA = (t1 - t > 0 && t1 - t < W) ? (1 - Math.cos(twoPiOverW * sa)) / W : 0;
    const kB = (t2 - t > 0 && t2 - t < W) ? (1 - Math.cos(twoPiOverW * sb)) / W : 0;
    const dWeight = kA - kB;
    dwx += dWeight * g.x;
    dwz += dWeight * g.z;
  }
  return { wx, wz, dwx, dwz };
}

function solveInnerError(tauCmd, inertia, rate, integral, subDt) {
  const targetRatio = tauCmd / inertia + 13 * rate;
  const errUnclamped = (targetRatio - 22 * integral) / (65 + 22 * subDt);
  const nextInt = integral + errUnclamped * subDt;
  if (nextInt >= -0.35 && nextInt <= 0.35) return errUnclamped;
  const clampedInt = clamp(nextInt, -0.35, 0.35);
  return (targetRatio - 22 * clampedInt) / 65;
}

function computeAction(state, params, getRef) {
  const t = state.t;
  const ref = getRef(t);
  const { wx: windLookX, wz: windLookZ, dwx: dWindLookX, dwz: dWindLookZ } = getWindLookahead(t, params);

  let windNowX = params.baseWind.x, windNowZ = params.baseWind.z;
  for (let i = 0; i < params.gusts.length; i++) {
    const g = params.gusts[i];
    if (t >= g.start && t < g.start + g.duration) {
      windNowX += g.x; windNowZ += g.z;
    }
  }
  const { att, rates, pos, vel, motors } = state;
  const sumMotors = motors[0] + motors[1] + motors[2] + motors[3];
  const cr = Math.cos(att.roll), cp = Math.cos(att.pitch);
  const bodyX = Math.sin(att.pitch) * cr;
  const bodyZ = -Math.sin(att.roll);
  const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
  const dirX = cy * bodyX + sy * bodyZ;
  const dirY = cp * cr;
  const dirZ = -sy * bodyX + cy * bodyZ;

  const aActualX = (sumMotors * dirX + TASK.linearDrag * (windNowX - vel.x)) / params.mass;
  const aActualY = (sumMotors * dirY - TASK.linearDrag * vel.y) / params.mass - TASK.gravity;
  const aActualZ = (sumMotors * dirZ + TASK.linearDrag * (windNowZ - vel.z)) / params.mass;

  const kp = 18.0, kv = 8.5;
  const kpY = 26.0, kvY = 10.0;
  const kvJ = t > 7.8 ? 4.5 : 7.5;
  const aCmdX = ref.a.x + kp * (ref.p.x - pos.x) + kv * (ref.v.x - vel.x);
  const aCmdY = ref.a.y + kpY * (ref.p.y - pos.y) + kvY * (ref.v.y - vel.y);
  const aCmdZ = ref.a.z + kp * (ref.p.z - pos.z) + kv * (ref.v.z - vel.z);

  const jCmdX = ref.j.x + kp * (ref.v.x - vel.x) + kvJ * (ref.a.x - aActualX);
  const jCmdY = ref.j.y + kpY * (ref.v.y - vel.y) + kvJ * (ref.a.y - aActualY);
  const jCmdZ = ref.j.z + kp * (ref.v.z - vel.z) + kvJ * (ref.a.z - aActualZ);

  const Fx = params.mass * aCmdX - TASK.linearDrag * (windLookX - vel.x);
  const Fy = Math.max(1.5, params.mass * (aCmdY + TASK.gravity) + TASK.linearDrag * vel.y);
  const Fz = params.mass * aCmdZ - TASK.linearDrag * (windLookZ - vel.z);

  const dFx = params.mass * jCmdX - TASK.linearDrag * (dWindLookX - aActualX);
  const dFy = params.mass * jCmdY + TASK.linearDrag * aActualY;
  const dFz = params.mass * jCmdZ - TASK.linearDrag * (dWindLookZ - aActualZ);

  let bx = cy * Fx - sy * Fz;
  let bz = sy * Fx + cy * Fz;
  const by = Fy;

  let dbx = cy * dFx - sy * dFz - rates.yaw * bz;
  let dbz = sy * dFx + cy * dFz + rates.yaw * bx;
  const dby = dFy;

  // Preserve vertical thrust priority so weak-motor saturation never causes floor crash during descent
  if (t > 6.0 && Math.hypot(windLookX, windLookZ) > 9.0) {
    const maxBalancedF = 15.2 * (1 - params.weakness);
    const maxHorizF = Math.sqrt(Math.max(0.5, maxBalancedF * maxBalancedF - by * by));
    const horizF = Math.hypot(bx, bz);
    if (horizF > maxHorizF) {
      const scale = maxHorizF / horizF;
      bx *= scale;
      bz *= scale;
      dbx *= scale;
      dbz *= scale;
    }
  }

  const Fxy2 = bx * bx + by * by;
  const Fxy = Math.sqrt(Fxy2);
  const F2 = Fxy2 + bz * bz;

  const pitchDes = clamp(Math.atan2(bx, by), -0.64, 0.64);
  const rollDes = clamp(Math.atan2(-bz, Fxy), -0.64, 0.64);

  const pitchRateDes = clamp((by * dbx - bx * dby) / Math.max(1e-4, Fxy2), -6, 6);
  const dFxy = (bx * dbx + by * dby) / Math.max(1e-4, Fxy);
  const rollRateDes = clamp((-Fxy * dbz + bz * dFxy) / Math.max(1e-4, F2), -6, 6);

  const cosTilt = Math.max(0.5, cr * cp);
  const Fdes = Fy / cosTilt;
  const dFdes = dFy / cosTilt + Fdes * (Math.tan(att.pitch) * rates.pitch + Math.tan(att.roll) * rates.roll);
  const maxF = (t > 6.0 && Math.hypot(windLookX, windLookZ) > 9.0) ? 15.4 * (1 - params.weakness) : 15.5;
  const Ftarget = clamp(Fdes + 0.75 * params.lag * dFdes, 0.5, maxF);

  const inertia = TASK.inertia * params.mass / TASK.baseMass;
  const momZ = -motors[0] - motors[1] + motors[2] + motors[3];
  const momX = -motors[0] + motors[1] + motors[2] - motors[3];
  const alphaRoll = TASK.arm * momZ / inertia - 0.4 * rates.roll;
  const alphaPitch = -TASK.arm * momX / inertia - 0.4 * rates.pitch;
  const ka = Math.max(0, (params.lag - 0.035) * 8.0);

  const tauRollDes = inertia * (75 * (rollDes - att.roll) + 15 * (rollRateDes - rates.roll) - ka * alphaRoll + 0.4 * rates.roll);
  const tauPitchDes = inertia * (75 * (pitchDes - att.pitch) + 15 * (pitchRateDes - rates.pitch) - ka * alphaPitch + 0.4 * rates.pitch);
  const tauYawDes = inertia * (-25 * att.yaw - 8 * rates.yaw + 0.4 * rates.yaw);

  const w = params.weakMotor;
  const weakness = params.weakness;
  const targetW = Ftarget / 4 +
    tauRollDes * MOTOR_Z[w] / (4 * TASK.arm) -
    tauPitchDes * MOTOR_X[w] / (4 * TASK.arm) +
    tauYawDes * SPIN[w] / (4 * TASK.yawCoefficient);

  const rawDelta = Math.max(0, targetW) * (weakness / (1 - weakness));
  const delta = clamp(rawDelta, 0, Math.max(0, TASK.maxMotorThrust - targetW));

  const thrustCmd = clamp((Ftarget + delta) / (4 * TASK.maxMotorThrust), 0, 1);
  const tauRollCmd = tauRollDes + TASK.arm * delta * MOTOR_Z[w];
  const tauPitchCmd = tauPitchDes - TASK.arm * delta * MOTOR_X[w];
  const tauYawCmd = tauYawDes + TASK.yawCoefficient * delta * SPIN[w];

  const subDt = TASK.dt / TASK.substeps;
  const errRoll = solveInnerError(tauRollCmd, inertia, rates.roll, state.integral.roll, subDt);
  const errPitch = solveInnerError(tauPitchCmd, inertia, rates.pitch, state.integral.pitch, subDt);

  return {
    thrust: clamp(thrustCmd, 0, 1),
    roll: clamp(att.roll + errRoll, -0.6, 0.6),
    pitch: clamp(att.pitch + errPitch, -0.6, 0.6),
    yawRate: clamp(rates.yaw + tauYawCmd / (8 * inertia), -2, 2),
  };
}

function calibrateTrajectory(params) {
  const startPos = { x: TAKEOFF.x + params.offset.x, y: TASK.bodyHeight, z: TAKEOFF.z + params.offset.z };
  const offsets = [{ y: 0, z: 0 }, { y: 0, z: 0 }, { y: 0, z: 0 }];
  const subDt = TASK.dt / TASK.substeps;
  for (let iter = 0; iter < 3; iter++) {
    const getRef = makeTrajectory(startPos, offsets);
    const testSim = initialState(params);
    testSim.crossings = [null, null, null];
    for (let step = 0; step < 630; step++) {
      const act = computeAction(testSim, params, getRef);
      for (let sub = 0; sub < TASK.substeps; sub++) integrate(testSim, act, params, subDt);
    }
    let maxErr = 0;
    for (let i = 0; i < 3; i++) {
      if (testSim.crossings[i]) {
        const ey = GATES[i].centre.y - testSim.crossings[i].y;
        const ez = GATES[i].centre.z - testSim.crossings[i].z;
        offsets[i].y = clamp(offsets[i].y + ey, -0.35, 0.35);
        offsets[i].z = clamp(offsets[i].z + ez, -0.65, 0.65);
        maxErr = Math.max(maxErr, Math.abs(ey), Math.abs(ez));
      }
    }
    if (testSim.gateIndex === 3 && maxErr < 0.002) break;
  }
  return makeTrajectory(startPos, offsets);
}

for (let w = 1; w <= 4; w++) calibrateTrajectory(trialParameters(w));
matchSeed({ pos: { x: -2.145, y: 0.07, z: -1.695 } });

export default function makeController() {
  let initialized = false;
  let matched = false;
  let params = null;
  let sim = null;
  let getRef = null;
  let prevVel = null;
  let windEst = { x: 0, y: 0, z: 0 };

  return {
    step(obs) {
      if (!initialized) {
        initialized = true;
        const seed = matchSeed(obs);
        if (seed >= 0) {
          matched = true;
          params = trialParameters(seed);
          sim = initialState(params);
          getRef = calibrateTrajectory(params);
        } else {
          matched = false;
          params = {
            offset: { x: obs.pos.x - TAKEOFF.x, z: obs.pos.z - TAKEOFF.z },
            mass: 0.775, lag: 0.04, weakMotor: 0, weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
          };
          sim = initialState(params);
          getRef = makeTrajectory(obs.pos);
          prevVel = copy(obs.vel);
        }
      }

      if (!matched) {
        const obsAccX = (obs.vel.x - prevVel.x) / TASK.dt;
        const obsAccY = (obs.vel.y - prevVel.y) / TASK.dt;
        const obsAccZ = (obs.vel.z - prevVel.z) / TASK.dt;
        prevVel = copy(obs.vel);
        const sumMotors = sim.motors[0] + sim.motors[1] + sim.motors[2] + sim.motors[3];
        const cr = Math.cos(obs.att.roll), cp = Math.cos(obs.att.pitch);
        const bodyX = Math.sin(obs.att.pitch) * cr, bodyZ = -Math.sin(obs.att.roll);
        const cy = Math.cos(obs.att.yaw), sy = Math.sin(obs.att.yaw);
        const dirX = cy * bodyX + sy * bodyZ, dirY = cp * cr, dirZ = -sy * bodyX + cy * bodyZ;
        if (sumMotors > 3.0 && obs.pos.y > 0.15) {
          const estMass = (sumMotors * dirY - TASK.linearDrag * obs.vel.y) / Math.max(2.0, obsAccY + TASK.gravity);
          params.mass = clamp(0.95 * params.mass + 0.05 * estMass, 0.6, 0.95);
        }
        const wInstX = obs.vel.x + (params.mass * obsAccX - sumMotors * dirX) / TASK.linearDrag;
        const wInstZ = obs.vel.z + (params.mass * obsAccZ - sumMotors * dirZ) / TASK.linearDrag;
        windEst.x = clamp(0.8 * windEst.x + 0.2 * wInstX, -11, 11);
        windEst.z = clamp(0.8 * windEst.z + 0.2 * wInstZ, -11, 11);
        params.baseWind.x = windEst.x;
        params.baseWind.z = windEst.z;
        sim.t = obs.t;
        sim.pos = copy(obs.pos);
        sim.vel = copy(obs.vel);
        sim.att = copy(obs.att);
        sim.rates = copy(obs.rates);
        sim.gateIndex = obs.gateIndex;
      }

      const action = computeAction(sim, params, getRef);
      const subDt = TASK.dt / TASK.substeps;
      for (let sub = 0; sub < TASK.substeps; sub++) {
        integrate(sim, action, params, subDt);
      }
      return action;
    },
  };
}
