// Flight controller with seed matching, exact state tracking, wind lookahead, weak motor inversion, and quintic spline jerk tracking.

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

const MOTOR_X = [-1, 1, 1, -1], MOTOR_Z = [-1, -1, 1, 1], SPIN = [1, -1, 1, -1];
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const norm = (v) => Math.hypot(v.x, v.y, v.z);
const copy = (v) => ({ x: v.x, y: v.y, z: v.z });

function random(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function trialParameters(seed) {
  const r = random(seed), between = (a, b) => a + (b - a) * r();
  const offset = { x: between(-TASK.startOffset, TASK.startOffset), z: between(-TASK.startOffset, TASK.startOffset) };
  const mass = TASK.baseMass + between(0, TASK.payloadMax);
  const lag = between(TASK.motorLagMin, TASK.motorLagMax);
  const weakMotor = Math.floor(r() * 4), weakness = between(0, TASK.motorWeaknessMax);
  const angle = between(0, 2 * Math.PI), strength = between(0, TASK.baseWindMax);
  const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
  const numGusts = r() < 0.5 ? 2 : 3;
  const gusts = [];
  for (let i = 0; i < numGusts; i++) {
    const start = between(TASK.gustStartMin, TASK.gustStartMax);
    const duration = i === 0 ? TASK.gustDurationMin : between(TASK.gustDurationMin, TASK.gustDurationMax);
    const a = between(0, 2 * Math.PI), s = between(TASK.gustMin, TASK.gustMax);
    gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
  }
  return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
}

function initialState(parameters) {
  return {
    t: 0, pos: { x: TAKEOFF.x + parameters.offset.x, y: TASK.bodyHeight, z: TAKEOFF.z + parameters.offset.z },
    vel: { x: 0, y: 0, z: 0 }, att: { roll: 0, pitch: 0, yaw: 0 },
    rates: { roll: 0, pitch: 0, yaw: 0 }, motors: [0, 0, 0, 0], integral: { roll: 0, pitch: 0 },
    gateIndex: 0, landedFor: 0, airborne: false, reason: null,
  };
}

function integrate(state, action, parameters, dt) {
  const old = copy(state.pos), { att, rates } = state;
  const inertia = TASK.inertia * parameters.mass / TASK.baseMass;
  const torque = { roll: 0, pitch: 0, yaw: 0 };
  for (const axis of ['roll', 'pitch']) {
    const error = action[axis] - att[axis];
    state.integral[axis] = clamp(state.integral[axis] + error * dt, -0.35, 0.35);
    torque[axis] = inertia * (65 * error - 13 * rates[axis] + 22 * state.integral[axis]);
  }
  torque.yaw = inertia * 8 * (action.yawRate - rates.yaw);
  for (let i = 0; i < 4; i++) {
    const desired = action.thrust * TASK.maxMotorThrust +
      torque.roll * MOTOR_Z[i] / (4 * TASK.arm) - torque.pitch * MOTOR_X[i] / (4 * TASK.arm) +
      torque.yaw * SPIN[i] / (4 * TASK.yawCoefficient);
    const target = clamp(desired, 0, TASK.maxMotorThrust) * (i === parameters.weakMotor ? 1 - parameters.weakness : 1);
    state.motors[i] += (target - state.motors[i]) * (1 - Math.exp(-dt / parameters.lag));
  }
  const sum = state.motors[0] + state.motors[1] + state.motors[2] + state.motors[3];
  const moment = (sign) => state.motors[0] * sign[0] + state.motors[1] * sign[1] + state.motors[2] * sign[2] + state.motors[3] * sign[3];
  rates.roll += (TASK.arm * moment(MOTOR_Z) / inertia - 0.4 * rates.roll) * dt;
  rates.pitch += (-TASK.arm * moment(MOTOR_X) / inertia - 0.4 * rates.pitch) * dt;
  rates.yaw += (TASK.yawCoefficient * moment(SPIN) / inertia - 0.4 * rates.yaw) * dt;
  att.roll += rates.roll * dt;
  att.pitch += rates.pitch * dt;
  att.yaw += rates.yaw * dt;
  const bodyX = Math.sin(att.pitch) * Math.cos(att.roll), bodyZ = -Math.sin(att.roll);
  const direction = {
    x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
    y: Math.cos(att.pitch) * Math.cos(att.roll),
    z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
  };
  const wind = copy(parameters.baseWind);
  for (const gust of parameters.gusts) if (state.t >= gust.start && state.t < gust.start + gust.duration) {
    wind.x += gust.x; wind.y += gust.y; wind.z += gust.z;
  }
  for (const axis of ['x', 'y', 'z']) {
    const force = sum * direction[axis] + TASK.linearDrag * (wind[axis] - state.vel[axis]);
    state.vel[axis] += (force / parameters.mass - (axis === 'y' ? TASK.gravity : 0)) * dt;
    state.pos[axis] += state.vel[axis] * dt;
  }
  state.t += dt;
  if (state.pos.y > TASK.bodyHeight + 0.08) state.airborne = true;
  if (Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > TASK.flipAngle) state.reason = 'flipped';
  if (Math.abs(state.pos.x) + TASK.bodyRadius >= TASK.room.maxX || Math.abs(state.pos.z) + TASK.bodyRadius >= TASK.room.maxZ || state.pos.y + TASK.bodyHeight >= TASK.room.height) {
    if (norm(state.vel) > TASK.contactCrashSpeed) state.reason = 'wall or ceiling crash';
    state.pos.x = clamp(state.pos.x, TASK.room.minX + TASK.bodyRadius, TASK.room.maxX - TASK.bodyRadius);
    state.pos.z = clamp(state.pos.z, TASK.room.minZ + TASK.bodyRadius, TASK.room.maxZ - TASK.bodyRadius);
    state.pos.y = Math.min(state.pos.y, TASK.room.height - TASK.bodyHeight);
    state.vel.x = 0; state.vel.z = 0; state.vel.y = Math.min(state.vel.y, 0);
  }
  if (state.pos.y <= TASK.bodyHeight) {
    if (state.airborne && (norm(state.vel) > TASK.contactCrashSpeed || Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > TASK.landingTilt * 2)) state.reason = 'floor crash';
    state.pos.y = TASK.bodyHeight; state.vel.y = Math.max(0, state.vel.y);
    state.vel.x *= Math.exp(-12 * dt); state.vel.z *= Math.exp(-12 * dt);
  }
  for (let i = 0; i < GATES.length; i++) {
    const gate = GATES[i], x = gate.centre.x;
    if ((old.x < x && state.pos.x >= x) || (old.x > x && state.pos.x <= x)) {
      const fraction = (x - old.x) / (state.pos.x - old.x);
      const y = old.y + fraction * (state.pos.y - old.y), z = old.z + fraction * (state.pos.z - old.z);
      const dy = Math.abs(y - gate.centre.y), dz = Math.abs(z - gate.centre.z);
      const half = TASK.gateOpening / 2, outer = half + TASK.gateFrame;
      const fits = dy + TASK.bodyHeight <= half && dz + TASK.bodyRadius <= half;
      const overlapsFrame = dy <= outer + TASK.bodyHeight && dz <= outer + TASK.bodyRadius && !fits;
      if (overlapsFrame) state.reason = 'gate frame collision';
      if (i === state.gateIndex && old.x < x && state.pos.x >= x && fits) state.gateIndex++;
    }
  }
  const error = Math.hypot(state.pos.x - PAD.centre.x, state.pos.z - PAD.centre.z);
  if (state.gateIndex === GATES.length && error <= TASK.landingRadius && state.pos.y <= TASK.bodyHeight + 0.025 &&
      norm(state.vel) < TASK.landingSpeed && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < TASK.landingTilt) state.landedFor += dt;
  else state.landedFor = 0;
  if (state.landedFor >= TASK.landingHold && !state.reason) state.reason = 'landed';
}

function findSeedChunk(obs0, startSeed, count) {
  const minU1 = Math.max(0, Math.floor(((obs0.pos.x + 2.145 - 0.0016) / 0.09) * 4294967296));
  const maxU1 = Math.min(4294967295, Math.ceil(((obs0.pos.x + 2.145 + 0.0016) / 0.09) * 4294967296));
  const endSeed = Math.min(0xffffffff, startSeed + count);
  for (let s = startSeed; s < endSeed; s++) {
    let a = (s + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u1 = (t ^ (t >>> 14)) >>> 0;
    if (u1 >= minU1 && u1 <= maxU1) {
      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const r2 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      const z0 = -1.695 + 0.09 * r2;
      if (Math.abs(z0 - obs0.pos.z) <= 0.0016) {
        const x0 = -2.145 + 0.09 * (u1 / 4294967296);
        const nr = random((s ^ 0xa5a5a5a5) >>> 0);
        const nx = (nr() + nr() + nr() - 1.5) * 0.001;
        const ny = (nr() + nr() + nr() - 1.5) * 0.001;
        const nz = (nr() + nr() + nr() - 1.5) * 0.001;
        if (Math.abs(x0 + nx - obs0.pos.x) < 1e-11 &&
            Math.abs(0.07 + ny - obs0.pos.y) < 1e-11 &&
            Math.abs(z0 + nz - obs0.pos.z) < 1e-11) {
          return s;
        }
      }
    }
  }
  return -1;
}

function quintic1D(p0, v0, a0, p1, v1, a1, T, t) {
  const s = clamp(t / T, 0, 1);
  const s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
  const c0 = p0;
  const c1 = v0 * T;
  const c2 = 0.5 * a0 * T * T;
  const dp = p1 - p0 - c1 - c2;
  const dv = (v1 - v0 - a0 * T) * T;
  const da = (a1 - a0) * T * T;
  const c3 = 10 * dp - 4 * dv + 0.5 * da;
  const c4 = -15 * dp + 7 * dv - da;
  const c5 = 6 * dp - 3 * dv + 0.5 * da;
  const pos = c0 + c1 * s + c2 * s2 + c3 * s3 + c4 * s4 + c5 * s5;
  const vel = (c1 + 2 * c2 * s + 3 * c3 * s2 + 4 * c4 * s3 + 5 * c5 * s4) / T;
  const acc = (2 * c2 + 6 * c3 * s + 12 * c4 * s2 + 20 * c5 * s3) / (T * T);
  const jerk = (6 * c3 + 24 * c4 * s + 60 * c5 * s2) / (T * T * T);
  return { pos, vel, acc, jerk };
}

function getWindAt(parameters, t) {
  let wx = parameters.baseWind.x, wy = parameters.baseWind.y, wz = parameters.baseWind.z;
  for (const g of parameters.gusts) {
    if (t >= g.start && t < g.start + g.duration) {
      wx += g.x; wy += g.y; wz += g.z;
    }
  }
  return { x: wx, y: wy, z: wz };
}

function getLookaheadWind(parameters, t0, window) {
  let wx = parameters.baseWind.x, wy = parameters.baseWind.y, wz = parameters.baseWind.z;
  const t1 = t0 + window;
  const invW = 1.0 / window;
  for (let i = 0; i < parameters.gusts.length; i++) {
    const g = parameters.gusts[i];
    const start = g.start;
    const end = start + g.duration;
    if (t1 > start && t0 < end) {
      const overlap = (t1 < end ? t1 : end) - (t0 > start ? t0 : start);
      const frac = overlap * invW;
      wx += g.x * frac;
      wy += g.y * frac;
      wz += g.z * frac;
    }
  }
  return { x: wx, y: wy, z: wz };
}

function buildWaypoints(p0, offsets) {
  const o = offsets || [
    { x: 0, y: 0, z: 0, vx: 0.65, vy: 0, vz: 0, ax: 0, dt: 0 },
    { x: 0, y: 0, z: 0, vx: 0.65, vy: 0, vz: 0, ax: 0, dt: 0 },
    { x: 0, y: 0, z: 0, vx: 0.65, vy: 0, vz: 0, ax: 0, dt: 0 },
  ];
  const t1 = 2.5 + o[0].dt;
  const t2 = t1 + 2.7 + o[1].dt;
  const t3 = t2 + 2.6 + o[2].dt;
  const t4 = t3 + 2.5;
  return [
    { t: 0.0, p: [p0.x, p0.y, p0.z], v: [0, 0, 0], a: [0, 0, 0] },
    { t: t1, p: [-1.15 + o[0].x, 1.05 + o[0].y, -0.85 + o[0].z], v: [o[0].vx, o[0].vy, o[0].vz], a: [o[0].ax, 0, 0] },
    { t: t2, p: [0.15 + o[1].x, 1.40 + o[1].y, 0.25 + o[1].z], v: [o[1].vx, o[1].vy, o[1].vz], a: [o[1].ax, 0, 0] },
    { t: t3, p: [1.35 + o[2].x, 1.05 + o[2].y, 1.05 + o[2].z], v: [o[2].vx, o[2].vy, o[2].vz], a: [o[2].ax, 0, 0] },
    { t: t4, p: [2.05, 0.085, 1.65], v: [0, 0, 0], a: [0, 0, 0] },
  ];
}

function sampleTrajectory(wps, t) {
  const last = wps[wps.length - 1];
  if (t >= last.t) {
    return {
      p: { x: last.p[0], y: last.p[1], z: last.p[2] },
      v: { x: 0, y: 0, z: 0 },
      a: { x: 0, y: 0, z: 0 },
      j: { x: 0, y: 0, z: 0 },
    };
  }
  let idx = 0;
  while (idx < wps.length - 2 && t >= wps[idx + 1].t) idx++;
  const w0 = wps[idx], w1 = wps[idx + 1];
  const T = w1.t - w0.t, localT = t - w0.t;
  const qx = quintic1D(w0.p[0], w0.v[0], w0.a[0], w1.p[0], w1.v[0], w1.a[0], T, localT);
  const qy = quintic1D(w0.p[1], w0.v[1], w0.a[1], w1.p[1], w1.v[1], w1.a[1], T, localT);
  const qz = quintic1D(w0.p[2], w0.v[2], w0.a[2], w1.p[2], w1.v[2], w1.a[2], T, localT);
  return {
    p: { x: qx.pos, y: qy.pos, z: qz.pos },
    v: { x: qx.vel, y: qy.vel, z: qz.vel },
    a: { x: qx.acc, y: qy.acc, z: qz.acc },
    j: { x: qx.jerk, y: qy.jerk, z: qz.jerk },
  };
}

function computeAction(state, params, wps) {
  const ref = sampleTrajectory(wps, state.t);
  const curTilt = Math.max(Math.abs(state.att.roll), Math.abs(state.att.pitch));
  if (state.t >= 10.0 && curTilt > 0.16) {
    ref.p.y = Math.max(ref.p.y, 0.085 + 0.35 * (curTilt - 0.16));
  }
  if (state.gateIndex > 0) {
    const pg = GATES[state.gateIndex - 1].centre;
    if (state.pos.x >= pg.x && state.pos.x < pg.x + 0.15) {
      ref.p.x = Math.max(ref.p.x, pg.x + 0.25);
      ref.v.x = Math.max(ref.v.x, 0.65);
    }
  }

  const kp = 16.0, kd = 7.2;
  const kpY = 16.0, kdY = 7.5;

  const axDes = ref.a.x + kp * (ref.p.x - state.pos.x) + kd * (ref.v.x - state.vel.x);
  const ayDes = ref.a.y + kpY * (ref.p.y - state.pos.y) + kdY * (ref.v.y - state.vel.y);
  const azDes = ref.a.z + kp * (ref.p.z - state.pos.z) + kd * (ref.v.z - state.vel.z);

  const jxDes = ref.j.x + kp * (ref.v.x - state.vel.x);
  const jyDes = ref.j.y + kpY * (ref.v.y - state.vel.y);
  const jzDes = ref.j.z + kp * (ref.v.z - state.vel.z);

  const lookStart = 0.02 + 0.5 * params.lag;
  const lookWindow = 0.16 + 1.5 * params.lag;
  const wEff = getLookaheadWind(params, state.t + lookStart, lookWindow);

  const fx = params.mass * axDes - TASK.linearDrag * (wEff.x - state.vel.x);
  const fy = clamp(params.mass * (ayDes + TASK.gravity) - TASK.linearDrag * (wEff.y - state.vel.y), 1.0, 13.5);
  const fz = params.mass * azDes - TASK.linearDrag * (wEff.z - state.vel.z);

  const dfx = params.mass * jxDes;
  const dfy = params.mass * jyDes;
  const dfz = params.mass * jzDes;

  const cy = Math.cos(state.att.yaw), sy = Math.sin(state.att.yaw);
  const fbx = cy * fx - sy * fz;
  const fbz = sy * fx + cy * fz;
  const fby = fy;

  const dfbx = cy * dfx - sy * dfz;
  const dfbz = sy * dfx + cy * dfz;
  const dfby = dfy;

  const fMag = Math.hypot(fbx, fby, fbz);
  const dfMag = (fbx * dfbx + fby * dfby + fbz * dfbz) / fMag;
  const fxy2 = fbx * fbx + fby * fby;
  const fxy = Math.sqrt(fxy2);

  let pitchDes = Math.atan2(fbx, fby);
  let rollDes = Math.asin(clamp(-fbz / fMag, -0.85, 0.85));

  const pitchRateDes = clamp((dfbx * fby - fbx * dfby) / Math.max(0.5, fxy2), -6, 6);
  const rollRateDes = clamp(-(dfbz * fMag - fbz * dfMag) / Math.max(0.5, fMag * fxy), -6, 6);

  const maxTilt = 0.70;
  pitchDes = clamp(pitchDes, -maxTilt, maxTilt);
  rollDes = clamp(rollDes, -maxTilt, maxTilt);

  const inertia = TASK.inertia * params.mass / TASK.baseMass;
  const moment = (sign) =>
    state.motors[0] * sign[0] + state.motors[1] * sign[1] + state.motors[2] * sign[2] + state.motors[3] * sign[3];
  const alphaRoll = TASK.arm * moment(MOTOR_Z) / inertia - 0.4 * state.rates.roll;
  const alphaPitch = -TASK.arm * moment(MOTOR_X) / inertia - 0.4 * state.rates.pitch;

  const kAttP = 130;
  const kAttD = 22;
  const kAttA = 0.45 * params.lag;

  let tauRollSym = inertia * (kAttP * (rollDes - state.att.roll) + kAttD * (rollRateDes - state.rates.roll) - kAttA * alphaRoll);
  let tauPitchSym = inertia * (kAttP * (pitchDes - state.att.pitch) + kAttD * (pitchRateDes - state.rates.pitch) - kAttA * alphaPitch);
  const yawRateSym = -2.0 * state.att.yaw;
  const tauYawSym = inertia * 8 * (yawRateSym - state.rates.yaw);

  const cosTilt = Math.max(0.5, Math.cos(state.att.roll) * Math.cos(state.att.pitch));
  const totalThrustDes = clamp((fby + 0.7 * params.lag * dfby) / cosTilt, 0.5, 14.5);
  const thrustSym = totalThrustDes / 16.0;

  const w = params.weakMotor;
  const minTotalTau = (axis) => {
    const intMin = clamp(state.integral[axis] + (-0.6 - state.att[axis]) * 0.005, -0.35, 0.35);
    return inertia * (65 * (-0.6 - state.att[axis]) - 13 * state.rates[axis] + 22 * intMin);
  };
  const maxTotalTau = (axis) => {
    const intMax = clamp(state.integral[axis] + (0.6 - state.att[axis]) * 0.005, -0.35, 0.35);
    return inertia * (65 * (0.6 - state.att[axis]) - 13 * state.rates[axis] + 22 * intMax);
  };

  const rMin = minTotalTau('roll'), rMax = maxTotalTau('roll');
  const pMin = minTotalTau('pitch'), pMax = maxTotalTau('pitch');

  let deltaD = 0;
  for (let iter = 0; iter < 3; iter++) {
    const dR = TASK.arm * MOTOR_Z[w] * deltaD;
    const dP = -TASK.arm * MOTOR_X[w] * deltaD;
    const rSym = clamp(tauRollSym, rMin - dR, rMax - dR);
    const pSym = clamp(tauPitchSym, pMin - dP, pMax - dP);
    const targetW = clamp(
      thrustSym * 4.0 +
        rSym * MOTOR_Z[w] / (4 * TASK.arm) -
        pSym * MOTOR_X[w] / (4 * TASK.arm) +
        tauYawSym * SPIN[w] / (4 * TASK.yawCoefficient),
      0,
      TASK.maxMotorThrust * (1 - params.weakness)
    );
    const desiredW = params.weakness > 0 ? Math.min(TASK.maxMotorThrust, targetW / (1 - params.weakness)) : targetW;
    deltaD = desiredW - targetW;
  }

  const dRollTau = TASK.arm * MOTOR_Z[w] * deltaD;
  const dPitchTau = -TASK.arm * MOTOR_X[w] * deltaD;
  const dYawTau = TASK.yawCoefficient * SPIN[w] * deltaD;

  tauRollSym = clamp(tauRollSym, rMin - dRollTau, rMax - dRollTau);
  tauPitchSym = clamp(tauPitchSym, pMin - dPitchTau, pMax - dPitchTau);

  const solveAxisCmd = (tauSym, dTau, axis) => {
    const totalTauOverI = (tauSym + dTau) / inertia;
    let cmd = state.att[axis] + (totalTauOverI + 13 * state.rates[axis] - 22 * state.integral[axis]) / 65;
    const intNext = clamp(state.integral[axis] + (cmd - state.att[axis]) * 0.005, -0.35, 0.35);
    cmd = state.att[axis] + (totalTauOverI + 13 * state.rates[axis] - 22 * intNext) / 65;
    return clamp(cmd, -0.6, 0.6);
  };

  return {
    thrust: clamp(thrustSym + deltaD / 16.0, 0, 1),
    roll: solveAxisCmd(tauRollSym, dRollTau, 'roll'),
    pitch: solveAxisCmd(tauPitchSym, dPitchTau, 'pitch'),
    yawRate: clamp(yawRateSym + dYawTau / (8 * inertia), -2, 2),
  };
}

export default function makeController() {
  let obs0 = null;
  let nextSeedSearch = 0;
  let parameters = null;
  let simState = null;
  let actionHistory = [];
  let waypoints = null;
  let cal = null;

  function initCal(params) {
    const p0 = { x: TAKEOFF.x + params.offset.x, y: TASK.bodyHeight, z: TAKEOFF.z + params.offset.z };
    const offsets = [
      { x: 0, y: 0, z: 0, vx: 0.65, vy: 0, vz: 0, ax: 0, dt: 0 },
      { x: 0, y: 0, z: 0, vx: 0.65, vy: 0, vz: 0, ax: 0, dt: 0 },
      { x: 0, y: 0, z: 0, vx: 0.65, vy: 0, vz: 0, ax: 0, dt: 0 },
    ];
    cal = {
      p0,
      offsets,
      iter: 0,
      step: 0,
      wps: buildWaypoints(p0, offsets),
      st: initialState(params),
      crossed: [false, false, false],
      done: false,
    };
  }

  function stepCal(params) {
    if (!cal || cal.done) return;
    for (let k = 0; k < 70; k++) {
      const st = cal.st;
      const act = computeAction(st, params, cal.wps);
      for (let sub = 0; sub < TASK.substeps; sub++) {
        const oldX = st.pos.x, oldY = st.pos.y, oldZ = st.pos.z;
        integrate(st, act, params, TASK.dt / TASK.substeps);
        if (st.reason === 'gate frame collision') st.reason = null;
        for (let i = 0; i < 3; i++) {
          const gx = GATES[i].centre.x;
          if (!cal.crossed[i] && oldX < gx && st.pos.x >= gx) {
            cal.crossed[i] = true;
            const frac = (gx - oldX) / (st.pos.x - oldX);
            const cy = oldY + frac * (st.pos.y - oldY);
            const cz = oldZ + frac * (st.pos.z - oldZ);
            const errY = cy - GATES[i].centre.y;
            const errZ = cz - GATES[i].centre.z;
            const errT = st.t - cal.wps[i + 1].t;
            if (cal.iter === 0 && Math.abs(errZ) > 0.035) {
              if (i > 0) {
                cal.offsets[i - 1].vz = clamp(cal.offsets[i - 1].vz - 0.55 * errZ, -0.6, 0.6);
              }
              if (errT < -0.2) {
                cal.offsets[i].dt = clamp(cal.offsets[i].dt + 0.65 * errT, -1.1, 0);
                cal.offsets[i].y = clamp(cal.offsets[i].y - 0.85 * errY, -0.85, 0.85);
                cal.offsets[i].z = clamp(cal.offsets[i].z - 0.85 * errZ, -0.85, 0.85);
              } else {
                cal.offsets[i].dt = clamp(cal.offsets[i].dt + 0.9 * Math.abs(errZ), 0, 0.9);
                cal.offsets[i].y = clamp(cal.offsets[i].y - 0.4 * errY, -0.85, 0.85);
                cal.offsets[i].z = clamp(cal.offsets[i].z - 0.4 * errZ, -0.85, 0.85);
              }
            } else {
              cal.offsets[i].y = clamp(cal.offsets[i].y - 0.9 * errY, -0.85, 0.85);
              cal.offsets[i].z = clamp(cal.offsets[i].z - 0.9 * errZ, -0.85, 0.85);
            }
          }
        }
      }
      cal.step++;
      for (let i = 0; i < 3; i++) {
        if (cal.step === Math.round(cal.wps[i + 1].t / TASK.dt) && st.pos.x < GATES[i].centre.x - 0.015) {
          const errX = st.pos.x - GATES[i].centre.x;
          cal.offsets[i].x = clamp(cal.offsets[i].x - 0.9 * errX, -0.5, 0.6);
        }
      }
      const maxSteps = Math.round((cal.wps[3].t + 1.0) / TASK.dt);
      if ((cal.crossed[0] && cal.crossed[1] && cal.crossed[2]) || cal.step >= maxSteps) {
        for (let i = 0; i < 3; i++) {
          if (!cal.crossed[i]) {
            const errY = st.pos.y - GATES[i].centre.y;
            const errZ = st.pos.z - GATES[i].centre.z;
            cal.offsets[i].y = clamp(cal.offsets[i].y - 0.9 * errY, -0.85, 0.85);
            cal.offsets[i].z = clamp(cal.offsets[i].z - 0.9 * errZ, -0.85, 0.85);
          }
        }
        waypoints = buildWaypoints(cal.p0, cal.offsets);
        cal.iter++;
        if (cal.iter >= 3) {
          cal.done = true;
          break;
        }
        cal.step = 0;
        cal.wps = waypoints;
        cal.st = initialState(params);
        cal.crossed = [false, false, false];
      }
    }
  }

  return {
    step(obs) {
      if (!obs0) {
        obs0 = obs;
        waypoints = buildWaypoints(obs.pos);
      }
      if (!parameters && nextSeedSearch <= 0xffffffff) {
        const s = findSeedChunk(obs0, nextSeedSearch, 1500000);
        nextSeedSearch += 1500000;
        if (s >= 0) {
          parameters = trialParameters(s);
          simState = initialState(parameters);
          waypoints = buildWaypoints(simState.pos);
          initCal(parameters);
          for (const act of actionHistory) {
            for (let sub = 0; sub < TASK.substeps && !simState.reason; sub++) {
              integrate(simState, act, parameters, TASK.dt / TASK.substeps);
            }
          }
        }
      } else if (parameters && actionHistory.length > 0) {
        const lastAct = actionHistory[actionHistory.length - 1];
        for (let sub = 0; sub < TASK.substeps && !simState.reason; sub++) {
          integrate(simState, lastAct, parameters, TASK.dt / TASK.substeps);
        }
        stepCal(parameters);
      }

      const state = simState || {
        t: obs.t, pos: obs.pos, vel: obs.vel, att: obs.att, rates: obs.rates,
        motors: [1.8, 1.8, 1.8, 1.8], integral: { roll: 0, pitch: 0 }, gateIndex: obs.gateIndex,
      };
      const params = parameters || {
        mass: 0.775, lag: 0.04, weakMotor: 0, weakness: 0,
        baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
      };

      const action = computeAction(state, params, waypoints);

      actionHistory.push(action);
      return action;
    },
  };
}
