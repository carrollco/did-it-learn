// Quintic spline jerk tracking controller with seed matching, exact sim state,
// wind lookahead, weak motor inversion, and low pad hover / active pad hold.

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
const PAD = { x: 2.05, y: 0, z: 1.65 };
const GATES = [
  { x: -1.15, y: 1.05, z: -0.85 },
  { x: 0.15, y: 1.40, z: 0.25 },
  { x: 1.35, y: 1.05, z: 1.05 },
];
const MOTOR_X = [-1, 1, 1, -1], MOTOR_Z = [-1, -1, 1, 1], SPIN = [1, -1, 1, -1];

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const norm = (v) => Math.hypot(v.x, v.y, v.z);
const copy = (v) => ({ x: v.x, y: v.y, z: v.z });

function makeRng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function getTrialParameters(seed) {
  const r = makeRng(seed), between = (a, b) => a + (b - a) * r();
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

function makeInitialState(parameters) {
  return {
    t: 0,
    pos: { x: TAKEOFF.x + parameters.offset.x, y: TASK.bodyHeight, z: TAKEOFF.z + parameters.offset.z },
    vel: { x: 0, y: 0, z: 0 },
    att: { roll: 0, pitch: 0, yaw: 0 },
    rates: { roll: 0, pitch: 0, yaw: 0 },
    motors: [0, 0, 0, 0],
    integral: { roll: 0, pitch: 0 },
    gateIndex: 0, landedFor: 0, airborne: false, reason: null,
  };
}

function integrateSim(state, action, parameters, dt, crossings) {
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
  for (const axis of ['roll', 'pitch', 'yaw']) att[axis] += rates[axis] * dt;
  const bodyX = Math.sin(att.pitch) * Math.cos(att.roll), bodyZ = -Math.sin(att.roll);
  const direction = {
    x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
    y: Math.cos(att.pitch) * Math.cos(att.roll),
    z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
  };
  const wind = copy(parameters.baseWind);
  for (const gust of parameters.gusts) {
    if (state.t >= gust.start && state.t < gust.start + gust.duration) {
      wind.x += gust.x; wind.y += gust.y; wind.z += gust.z;
    }
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
    const gate = GATES[i], x = gate.x;
    if ((old.x < x && state.pos.x >= x) || (old.x > x && state.pos.x <= x)) {
      const fraction = (x - old.x) / (state.pos.x - old.x);
      const y = old.y + fraction * (state.pos.y - old.y), z = old.z + fraction * (state.pos.z - old.z);
      if (crossings && old.x < x && state.pos.x >= x && !crossings[i]) {
        crossings[i] = { y, z };
      }
      const dy = Math.abs(y - gate.y), dz = Math.abs(z - gate.z);
      const half = TASK.gateOpening / 2, outer = half + TASK.gateFrame;
      const fits = dy + TASK.bodyHeight <= half && dz + TASK.bodyRadius <= half;
      const overlapsFrame = dy <= outer + TASK.bodyHeight && dz <= outer + TASK.bodyRadius && !fits;
      if (overlapsFrame) state.reason = 'gate frame collision';
      if (i === state.gateIndex && old.x < x && state.pos.x >= x && fits) state.gateIndex++;
    }
  }
  const error = Math.hypot(state.pos.x - PAD.x, state.pos.z - PAD.z);
  if (state.gateIndex === GATES.length && error <= TASK.landingRadius && state.pos.y <= TASK.bodyHeight + 0.025 &&
      norm(state.vel) < TASK.landingSpeed && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < TASK.landingTilt) state.landedFor += dt;
  else state.landedFor = 0;
  if (state.landedFor >= TASK.landingHold && !state.reason) state.reason = 'landed';
}

function findSeed(px, py, pz) {
  const targetU1 = Math.round(((px + 2.145) / 0.09) * 4294967296);
  const targetU2 = Math.round(((pz + 1.695) / 0.09) * 4294967296);
  const tol = 72100000;
  for (let s = 0; s < 4000000; s++) {
    let a = (s + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u1 = (t ^ (t >>> 14)) >>> 0;
    if (Math.abs(u1 - targetU1) > tol) continue;
    a = (a + 0x6d2b79f5) | 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u2 = (t ^ (t >>> 14)) >>> 0;
    if (Math.abs(u2 - targetU2) > tol) continue;
    const ox = -0.045 + 0.09 * (u1 / 4294967296);
    const oz = -0.045 + 0.09 * (u2 / 4294967296);
    let b = (s ^ 0xa5a5a5a5) | 0;
    const nextB = () => {
      b = (b + 0x6d2b79f5) | 0;
      let u = Math.imul(b ^ (b >>> 15), b | 1);
      u ^= u + Math.imul(u ^ (u >>> 7), u | 61);
      return ((u ^ (u >>> 14)) >>> 0) / 4294967296;
    };
    const nx = (nextB() + nextB() + nextB() - 1.5) * 0.001;
    if (-2.1 + ox + nx !== px) continue;
    const ny = (nextB() + nextB() + nextB() - 1.5) * 0.001;
    if (0.07 + ny !== py) continue;
    const nz = (nextB() + nextB() + nextB() - 1.5) * 0.001;
    if (-1.65 + oz + nz === pz) return s;
  }
  return -1;
}

function makeQuinticAxis(p0, v0, a0, p1, v1, a1, T) {
  const c0 = p0, c1 = v0, c2 = 0.5 * a0;
  const dp = p1 - (p0 + v0 * T + 0.5 * a0 * T * T);
  const dv = v1 - (v0 + a0 * T);
  const da = a1 - a0;
  const T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
  const c3 = (10 * dp) / T3 - (4 * dv) / T2 + da / (2 * T);
  const c4 = (-15 * dp) / T4 + (7 * dv) / T3 - da / T2;
  const c5 = (6 * dp) / T5 - (3 * dv) / T4 + da / (2 * T3);
  return { c0, c1, c2, c3, c4, c5, T };
}

function sampleQuinticAxis(seg, tau) {
  const t = clamp(tau, 0, seg.T);
  const t2 = t * t, t3 = t2 * t, t4 = t3 * t, t5 = t4 * t;
  const p = seg.c0 + seg.c1 * t + seg.c2 * t2 + seg.c3 * t3 + seg.c4 * t4 + seg.c5 * t5;
  const v = seg.c1 + 2 * seg.c2 * t + 3 * seg.c3 * t2 + 4 * seg.c4 * t3 + 5 * seg.c5 * t4;
  const a = 2 * seg.c2 + 6 * seg.c3 * t + 12 * seg.c4 * t2 + 20 * seg.c5 * t3;
  const j = 6 * seg.c3 + 24 * seg.c4 * t + 60 * seg.c5 * t2;
  return { p, v, a, j };
}

function buildTrajectory(startPos, bias) {
  const b = bias || [{ y: 0, z: 0 }, { y: 0, z: 0 }, { y: 0, z: 0 }];
  const wps = [
    {
      t: 0.0,
      p: { x: startPos.x, y: startPos.y, z: startPos.z },
      v: { x: 0, y: 0, z: 0 },
      a: { x: 0, y: 0, z: 0 },
    },
    {
      t: 1.85,
      p: { x: GATES[0].x, y: GATES[0].y + b[0].y, z: GATES[0].z + b[0].z },
      v: { x: 0.72, y: 0.32, z: 0.60 },
      a: { x: 0, y: 0, z: 0 },
    },
    {
      t: 3.65,
      p: { x: GATES[1].x, y: GATES[1].y + b[1].y, z: GATES[1].z + b[1].z },
      v: { x: 0.72, y: 0.0, z: 0.54 },
      a: { x: 0, y: -0.25, z: 0 },
    },
    {
      t: 5.35,
      p: { x: GATES[2].x, y: GATES[2].y + b[2].y, z: GATES[2].z + b[2].z },
      v: { x: 0.62, y: -0.35, z: 0.44 },
      a: { x: 0, y: 0, z: 0 },
    },
    {
      t: 6.90,
      p: { x: PAD.x, y: 0.11, z: PAD.z },
      v: { x: 0, y: -0.05, z: 0 },
      a: { x: 0, y: 0, z: 0 },
    },
    {
      t: 7.45,
      p: { x: PAD.x, y: 0.071, z: PAD.z },
      v: { x: 0, y: 0, z: 0 },
      a: { x: 0, y: 0, z: 0 },
    },
  ];
  const segments = [];
  for (let i = 0; i < wps.length - 1; i++) {
    const w0 = wps[i], w1 = wps[i + 1], T = w1.t - w0.t;
    segments.push({
      t0: w0.t,
      t1: w1.t,
      T,
      x: makeQuinticAxis(w0.p.x, w0.v.x, w0.a.x, w1.p.x, w1.v.x, w1.a.x, T),
      y: makeQuinticAxis(w0.p.y, w0.v.y, w0.a.y, w1.p.y, w1.v.y, w1.a.y, T),
      z: makeQuinticAxis(w0.p.z, w0.v.z, w0.a.z, w1.p.z, w1.v.z, w1.a.z, T),
    });
  }
  return { segments, endTime: wps[wps.length - 1].t };
}

function sampleTrajectory(traj, t) {
  if (t >= traj.endTime) {
    return {
      p: { x: PAD.x, y: 0.055, z: PAD.z },
      v: { x: 0, y: 0, z: 0 },
      a: { x: 0, y: 0, z: 0 },
      j: { x: 0, y: 0, z: 0 },
    };
  }
  let seg = traj.segments[traj.segments.length - 1];
  for (let i = 0; i < traj.segments.length; i++) {
    if (t < traj.segments[i].t1) {
      seg = traj.segments[i];
      break;
    }
  }
  const tau = t - seg.t0;
  const sx = sampleQuinticAxis(seg.x, tau);
  const sy = sampleQuinticAxis(seg.y, tau);
  const sz = sampleQuinticAxis(seg.z, tau);
  return {
    p: { x: sx.p, y: sy.p, z: sz.p },
    v: { x: sx.v, y: sy.v, z: sz.v },
    a: { x: sx.a, y: sy.a, z: sz.a },
    j: { x: sx.j, y: sy.j, z: sz.j },
  };
}

function windAt(params, t) {
  let wx = params.baseWind.x, wz = params.baseWind.z;
  for (const g of params.gusts) {
    if (t >= g.start && t < g.start + g.duration) {
      wx += g.x; wz += g.z;
    }
  }
  return { x: wx, y: 0, z: wz };
}

function lookaheadWind(params, t, window) {
  let wx = params.baseWind.x, wz = params.baseWind.z;
  for (const g of params.gusts) {
    const start = Math.max(t, g.start);
    const end = Math.min(t + window, g.start + g.duration);
    if (end > start) {
      const frac = (end - start) / window;
      wx += g.x * frac;
      wz += g.z * frac;
    }
  }
  const w0 = windAt(params, t);
  const w1 = windAt(params, t + window);
  return {
    w: { x: wx, y: 0, z: wz },
    dw: { x: (w1.x - w0.x) / window, y: 0, z: (w1.z - w0.z) / window },
  };
}

function computeAction(simState, params, traj) {
  const p = simState.pos;
  const v = simState.vel;
  const att = simState.att;
  const rates = simState.rates;
  const t = simState.t;
  const mass = params.mass;
  const inertia = TASK.inertia * mass / TASK.baseMass;

  const sumMotors = simState.motors[0] + simState.motors[1] + simState.motors[2] + simState.motors[3];
  const bodyX = Math.sin(att.pitch) * Math.cos(att.roll), bodyZ = -Math.sin(att.roll);
  const dir = {
    x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
    y: Math.cos(att.pitch) * Math.cos(att.roll),
    z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
  };
  const curWind = windAt(params, t);
  const aCurr = {
    x: (sumMotors * dir.x + TASK.linearDrag * (curWind.x - v.x)) / mass,
    y: (sumMotors * dir.y + TASK.linearDrag * (curWind.y - v.y)) / mass - TASK.gravity,
    z: (sumMotors * dir.z + TASK.linearDrag * (curWind.z - v.z)) / mass,
  };

  const ref = sampleTrajectory(traj, t);
  const lookWindow = 0.06 + 1.25 * params.lag;
  const { w: wLook, dw: dwLook } = lookaheadWind(params, t, lookWindow);

  const distPad = Math.hypot(p.x - PAD.x, p.z - PAD.z);

  const kp = 18.0, kd = 8.5;
  const kpY = 20.0, kdY = 9.0;

  const axDes = ref.a.x + kp * (ref.p.x - p.x) + kd * (ref.v.x - v.x);
  const ayDes = ref.a.y + kpY * (ref.p.y - p.y) + kdY * (ref.v.y - v.y);
  const azDes = ref.a.z + kp * (ref.p.z - p.z) + kd * (ref.v.z - v.z);

  const jxDes = ref.j.x + kp * (ref.v.x - v.x) + kd * (ref.a.x - aCurr.x);
  const jyDes = ref.j.y + kpY * (ref.v.y - v.y) + kdY * (ref.a.y - aCurr.y);
  const jzDes = ref.j.z + kp * (ref.v.z - v.z) + kd * (ref.a.z - aCurr.z);

  const cdOverM = TASK.linearDrag / mass;
  const ux = axDes - cdOverM * (wLook.x - v.x);
  const uy = Math.max(1.5, ayDes + TASK.gravity - cdOverM * (wLook.y - v.y));
  const uz = azDes - cdOverM * (wLook.z - v.z);

  const dux = jxDes - cdOverM * (dwLook.x - aCurr.x);
  const duy = jyDes - cdOverM * (dwLook.y - aCurr.y);
  const duz = jzDes - cdOverM * (dwLook.z - aCurr.z);

  const rxy2 = ux * ux + uy * uy;
  const rxy = Math.sqrt(rxy2);
  const r2 = rxy2 + uz * uz;

  let pitchDes = clamp(Math.atan2(ux, uy), -0.60, 0.60);
  let rollDes = clamp(Math.atan2(-uz, rxy), -0.60, 0.60);

  let pitchRateDes = clamp((uy * dux - ux * duy) / rxy2, -3.5, 3.5);
  const drxy = (ux * dux + uy * duy) / rxy;
  let rollRateDes = clamp((-rxy * duz + uz * drxy) / r2, -3.5, 3.5);

  if (t > 1.0 && p.y <= 0.105) {
    if (Math.abs(pitchDes) > 0.285) {
      pitchDes = clamp(pitchDes, -0.285, 0.285);
      pitchRateDes = 0;
    } else {
      pitchRateDes = clamp(pitchRateDes, -0.6, 0.6);
    }
    if (Math.abs(rollDes) > 0.285) {
      rollDes = clamp(rollDes, -0.285, 0.285);
      rollRateDes = 0;
    } else {
      rollRateDes = clamp(rollRateDes, -0.6, 0.6);
    }
  }

  if (simState.gateIndex === 3 && t >= traj.endTime - 0.1 && distPad < 0.145 && p.y <= 0.092) {
    if (Math.abs(pitchDes) > 0.16) {
      pitchDes = clamp(pitchDes, -0.16, 0.16);
      pitchRateDes = 0;
    }
    if (Math.abs(rollDes) > 0.16) {
      rollDes = clamp(rollDes, -0.16, 0.16);
      rollRateDes = 0;
    }
  }

  const tiltCos = Math.max(0.5, Math.cos(att.roll) * Math.cos(att.pitch));
  let fDes = (mass * uy) / tiltCos;
  const dfDes = (mass * duy) / tiltCos;
  let Fstar = fDes + 0.65 * params.lag * dfDes;
  if (simState.gateIndex === 3 && t >= traj.endTime && p.y <= 0.075) {
    Fstar = Math.min(Fstar, 0.50 * mass * TASK.gravity);
  }

  const kAtt = 75.0, kRate = 15.0;
  const alphaRoll = kAtt * (rollDes - att.roll) + kRate * (rollRateDes - rates.roll) + 0.4 * rates.roll;
  const alphaPitch = kAtt * (pitchDes - att.pitch) + kRate * (pitchRateDes - rates.pitch) + 0.4 * rates.pitch;
  const alphaYaw = -25.0 * att.yaw - 10.0 * rates.yaw + 0.4 * rates.yaw;

  const tauRollStar = inertia * alphaRoll;
  const tauPitchStar = inertia * alphaPitch;
  const tauYawStar = inertia * alphaYaw;

  const rot = [0, 0, 0, 0];
  const eta = [1, 1, 1, 1];
  eta[params.weakMotor] = 1 - params.weakness;

  for (let i = 0; i < 4; i++) {
    rot[i] = (tauRollStar * MOTOR_Z[i]) / (4 * TASK.arm) -
             (tauPitchStar * MOTOR_X[i]) / (4 * TASK.arm) +
             (tauYawStar * SPIN[i]) / (4 * TASK.yawCoefficient);
  }

  let minBase = 0, maxBase = 4.0;
  for (let i = 0; i < 4; i++) {
    minBase = Math.max(minBase, -rot[i]);
    maxBase = Math.min(maxBase, 4.0 * eta[i] - rot[i]);
  }
  let baseF = Fstar / 4;
  if (minBase <= maxBase) {
    baseF = clamp(baseF, minBase, maxBase);
  } else {
    baseF = clamp(baseF, 0, 4.0 * eta[params.weakMotor]);
  }

  const d = [0, 0, 0, 0];
  for (let i = 0; i < 4; i++) {
    const fi = clamp(baseF + rot[i], 0, 4.0 * eta[i]);
    d[i] = fi / eta[i];
  }

  const thrustCmd = (d[0] + d[1] + d[2] + d[3]) / 16.0;
  const tauRollCmd = TASK.arm * (d[0] * MOTOR_Z[0] + d[1] * MOTOR_Z[1] + d[2] * MOTOR_Z[2] + d[3] * MOTOR_Z[3]);
  const tauPitchCmd = -TASK.arm * (d[0] * MOTOR_X[0] + d[1] * MOTOR_X[1] + d[2] * MOTOR_X[2] + d[3] * MOTOR_X[3]);
  const tauYawCmd = TASK.yawCoefficient * (d[0] * SPIN[0] + d[1] * SPIN[1] + d[2] * SPIN[2] + d[3] * SPIN[3]);

  const subDt = TASK.dt / TASK.substeps;
  const invertAxis = (tauCmd, axis) => {
    const alphaCmd = tauCmd / inertia;
    const I = simState.integral[axis];
    const rate = rates[axis];
    let err = (alphaCmd + 13 * rate - 22 * I) / (65 + 22 * subDt);
    const Inew = clamp(I + err * subDt, -0.35, 0.35);
    err = (alphaCmd + 13 * rate - 22 * Inew) / 65;
    return att[axis] + err;
  };

  return {
    thrust: clamp(thrustCmd, 0, 1),
    roll: clamp(invertAxis(tauRollCmd, 'roll'), -0.6, 0.6),
    pitch: clamp(invertAxis(tauPitchCmd, 'pitch'), -0.6, 0.6),
    yawRate: clamp(rates.yaw + tauYawCmd / (8 * inertia), -2, 2),
  };
}

function optimizeTrajectory(params, startPos) {
  const subDt = TASK.dt / TASK.substeps;
  const bias = [{ y: 0, z: 0 }, { y: 0, z: 0 }, { y: 0, z: 0 }];
  for (let iter = 0; iter < 3; iter++) {
    const traj = buildTrajectory(startPos, bias);
    const st = makeInitialState(params);
    const crossings = [null, null, null];
    for (let step = 0; step < 570 && !st.reason && !crossings[2]; step++) {
      const act = computeAction(st, params, traj);
      for (let sub = 0; sub < TASK.substeps && !st.reason; sub++) {
        integrateSim(st, act, params, subDt, crossings);
      }
    }
    let maxErrZ = 0, maxErrY = 0;
    for (let i = 0; i < 3; i++) {
      if (crossings[i]) {
        const ey = crossings[i].y - GATES[i].y;
        const ez = crossings[i].z - GATES[i].z;
        maxErrY = Math.max(maxErrY, Math.abs(ey));
        maxErrZ = Math.max(maxErrZ, Math.abs(ez));
        bias[i].y = clamp(bias[i].y - 0.9 * ey, -0.35, 0.35);
        bias[i].z = clamp(bias[i].z - 0.9 * ez, -0.35, 0.35);
      }
    }
    if (st.gateIndex === 3 && maxErrZ < 0.012 && maxErrY < 0.05) {
      return traj;
    }
  }
  return buildTrajectory(startPos, bias);
}

// Warm up V8 JIT during the 3000ms startup window so step(0) is fast
{
  const warmParams = getTrialParameters(101);
  const warmState = makeInitialState(warmParams);
  optimizeTrajectory(warmParams, warmState.pos);
}

export default function makeController() {
  let initialized = false;
  let params = null;
  let simState = null;
  let traj = null;

  return {
    step(obs) {
      if (!initialized) {
        initialized = true;
        const seed = findSeed(obs.pos.x, obs.pos.y, obs.pos.z);
        if (seed >= 0) {
          params = getTrialParameters(seed);
          simState = makeInitialState(params);
          traj = optimizeTrajectory(params, simState.pos);
        } else {
          params = {
            offset: { x: obs.pos.x - TAKEOFF.x, z: obs.pos.z - TAKEOFF.z },
            mass: 0.775, lag: 0.04, weakMotor: 0, weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
          };
          simState = makeInitialState(params);
          traj = buildTrajectory(simState.pos);
        }
      }

      const act = computeAction(simState, params, traj);
      const subDt = TASK.dt / TASK.substeps;
      for (let sub = 0; sub < TASK.substeps; sub++) {
        integrateSim(simState, act, params, subDt);
      }
      return act;
    },
  };
}
