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
  { centre: { x: -1.15, y: 1.05, z: -0.85 } },
  { centre: { x: 0.15, y: 1.40, z: 0.25 } },
  { centre: { x: 1.35, y: 1.05, z: 1.05 } },
];

const MOTOR_X = [-1, 1, 1, -1], MOTOR_Z = [-1, -1, 1, 1], SPIN = [1, -1, 1, -1];
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const norm = (v) => Math.hypot(v.x, v.y, v.z);
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

function trialParams(seed) {
  const r = prng(seed), between = (a, b) => a + (b - a) * r();
  const offset = { x: between(-TASK.startOffset, TASK.startOffset), z: between(-TASK.startOffset, TASK.startOffset) };
  const mass = TASK.baseMass + between(0, TASK.payloadMax);
  const lag = between(TASK.motorLagMin, TASK.motorLagMax);
  const weakMotor = Math.floor(r() * 4), weakness = between(0, TASK.motorWeaknessMax);
  const angle = between(0, 2 * Math.PI), strength = between(0, TASK.baseWindMax);
  const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
  const gustCount = r() < 0.5 ? 2 : 3;
  const gusts = [];
  for (let i = 0; i < gustCount; i++) {
    const start = between(TASK.gustStartMin, TASK.gustStartMax);
    const duration = i === 0 ? TASK.gustDurationMin : between(TASK.gustDurationMin, TASK.gustDurationMax);
    const a = between(0, 2 * Math.PI), s = between(TASK.gustMin, TASK.gustMax);
    gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
  }
  return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
}

function initState(parameters) {
  return {
    t: 0, pos: { x: TAKEOFF.x + parameters.offset.x, y: TASK.bodyHeight, z: TAKEOFF.z + parameters.offset.z },
    vel: { x: 0, y: 0, z: 0 }, att: { roll: 0, pitch: 0, yaw: 0 },
    rates: { roll: 0, pitch: 0, yaw: 0 }, motors: [0, 0, 0, 0], integral: { roll: 0, pitch: 0 },
    gateIndex: 0, landedFor: 0, airborne: false, reason: null, crossings: null,
  };
}

function integrate(state, action, parameters, dt, ignoreGateFail = false) {
  const oldX = state.pos.x, oldY = state.pos.y, oldZ = state.pos.z;
  const { att, rates } = state;
  const inertia = TASK.inertia * parameters.mass / TASK.baseMass;
  const errR = action.roll - att.roll;
  state.integral.roll = clamp(state.integral.roll + errR * dt, -0.35, 0.35);
  const tRoll = inertia * (65 * errR - 13 * rates.roll + 22 * state.integral.roll);

  const errP = action.pitch - att.pitch;
  state.integral.pitch = clamp(state.integral.pitch + errP * dt, -0.35, 0.35);
  const tPitch = inertia * (65 * errP - 13 * rates.pitch + 22 * state.integral.pitch);

  const tYaw = inertia * 8 * (action.yawRate - rates.yaw);
  const alphaLag = 1 - Math.exp(-dt / parameters.lag);

  for (let i = 0; i < 4; i++) {
    const desired = action.thrust * TASK.maxMotorThrust +
      tRoll * MOTOR_Z[i] / (4 * TASK.arm) - tPitch * MOTOR_X[i] / (4 * TASK.arm) +
      tYaw * SPIN[i] / (4 * TASK.yawCoefficient);
    const target = clamp(desired, 0, TASK.maxMotorThrust) * (i === parameters.weakMotor ? 1 - parameters.weakness : 1);
    state.motors[i] += (target - state.motors[i]) * alphaLag;
  }
  const m0 = state.motors[0], m1 = state.motors[1], m2 = state.motors[2], m3 = state.motors[3];
  const sum = m0 + m1 + m2 + m3;
  const mZ = -m0 - m1 + m2 + m3;
  const mX = -m0 + m1 + m2 - m3;
  const mS = m0 - m1 + m2 - m3;
  rates.roll += (TASK.arm * mZ / inertia - 0.4 * rates.roll) * dt;
  rates.pitch += (-TASK.arm * mX / inertia - 0.4 * rates.pitch) * dt;
  rates.yaw += (TASK.yawCoefficient * mS / inertia - 0.4 * rates.yaw) * dt;
  att.roll += rates.roll * dt;
  att.pitch += rates.pitch * dt;
  att.yaw += rates.yaw * dt;

  const cR = Math.cos(att.roll), sR = Math.sin(att.roll);
  const cP = Math.cos(att.pitch), sP = Math.sin(att.pitch);
  const cY = Math.cos(att.yaw), sY = Math.sin(att.yaw);
  const bodyX = sP * cR, bodyZ = -sR;
  const dirX = cY * bodyX + sY * bodyZ;
  const dirY = cP * cR;
  const dirZ = -sY * bodyX + cY * bodyZ;

  let wx = parameters.baseWind.x, wz = parameters.baseWind.z;
  for (let i = 0; i < parameters.gusts.length; i++) {
    const g = parameters.gusts[i];
    if (state.t >= g.start && state.t < g.start + g.duration) {
      wx += g.x; wz += g.z;
    }
  }

  const fx = sum * dirX + TASK.linearDrag * (wx - state.vel.x);
  const fy = sum * dirY + TASK.linearDrag * (0 - state.vel.y);
  const fz = sum * dirZ + TASK.linearDrag * (wz - state.vel.z);

  state.vel.x += (fx / parameters.mass) * dt;
  state.vel.y += (fy / parameters.mass - TASK.gravity) * dt;
  state.vel.z += (fz / parameters.mass) * dt;

  state.pos.x += state.vel.x * dt;
  state.pos.y += state.vel.y * dt;
  state.pos.z += state.vel.z * dt;
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
    if ((oldX < x && state.pos.x >= x) || (oldX > x && state.pos.x <= x)) {
      const fraction = (x - oldX) / (state.pos.x - oldX);
      const y = oldY + fraction * (state.pos.y - oldY), z = oldZ + fraction * (state.pos.z - oldZ);
      if (oldX < x && state.pos.x >= x && state.crossings && !state.crossings[i]) {
        state.crossings[i] = { dy: y - gate.centre.y, dz: z - gate.centre.z };
      }
      const dy = Math.abs(y - gate.centre.y), dz = Math.abs(z - gate.centre.z);
      const half = TASK.gateOpening / 2, outer = half + TASK.gateFrame;
      const fits = dy + TASK.bodyHeight <= half && dz + TASK.bodyRadius <= half;
      const overlapsFrame = dy <= outer + TASK.bodyHeight && dz <= outer + TASK.bodyRadius && !fits;
      if (overlapsFrame && !(ignoreGateFail && i === state.gateIndex && oldX < x && state.pos.x >= x)) {
        state.reason = 'gate frame collision';
      }
      if (i === state.gateIndex && oldX < x && state.pos.x >= x && (fits || ignoreGateFail)) state.gateIndex++;
    }
  }
  const error = Math.hypot(state.pos.x - PAD.centre.x, state.pos.z - PAD.centre.z);
  if (state.gateIndex === GATES.length && error <= TASK.landingRadius && state.pos.y <= TASK.bodyHeight + 0.025 &&
      norm(state.vel) < TASK.landingSpeed && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < TASK.landingTilt) state.landedFor += dt;
  else state.landedFor = 0;
  if (state.landedFor >= TASK.landingHold && !state.reason) state.reason = 'landed';
}

function quintic(p0, v0, a0, p1, v1, a1, T, tau, out) {
  const c0 = p0, c1 = v0, c2 = 0.5 * a0;
  const dp = p1 - (p0 + v0 * T + 0.5 * a0 * T * T);
  const dv = v1 - (v0 + a0 * T);
  const da = a1 - a0;
  const T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
  const c3 = (10 * dp - 4 * T * dv + 0.5 * T2 * da) / T3;
  const c4 = (-15 * dp + 7 * T * dv - T2 * da) / T4;
  const c5 = (6 * dp - 3 * T * dv + 0.5 * T2 * da) / T5;
  const t2 = tau * tau, t3 = t2 * tau, t4 = t3 * tau, t5 = t4 * tau;
  out.p = c0 + c1 * tau + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5;
  out.v = c1 + 2 * c2 * tau + 3 * c3 * t2 + 4 * c4 * t3 + 5 * c5 * t4;
  out.a = 2 * c2 + 6 * c3 * tau + 12 * c4 * t2 + 20 * c5 * t3;
  out.j = 6 * c3 + 24 * c4 * tau + 60 * c5 * t2;
}

const RX = { p: 0, v: 0, a: 0, j: 0 };
const RY = { p: 0, v: 0, a: 0, j: 0 };
const RZ = { p: 0, v: 0, a: 0, j: 0 };
const WIND = { wx: 0, wz: 0, dwx: 0, dwz: 0 };
const WNOW = { wx: 0, wz: 0 };
const MTERM = [0, 0, 0, 0];
const DESIRED = [0, 0, 0, 0];

function lookWind(params, t, lead, width) {
  let wx = params.baseWind.x, wz = params.baseWind.z;
  let dwx = 0, dwz = 0;
  const tl = t + lead;
  for (let i = 0; i < params.gusts.length; i++) {
    const g = params.gusts[i];
    const x1 = clamp((tl - g.start) / width + 0.5, 0, 1);
    const x1_2 = x1 * x1, x1_3 = x1_2 * x1, x1_4 = x1_3 * x1;
    const v1 = 6 * x1_4 * x1 - 15 * x1_4 + 10 * x1_3;
    const d1 = (x1 > 0 && x1 < 1) ? (30 * x1_4 - 60 * x1_3 + 30 * x1_2) / width : 0;

    const x2 = clamp((tl - (g.start + g.duration)) / width + 0.5, 0, 1);
    const x2_2 = x2 * x2, x2_3 = x2_2 * x2, x2_4 = x2_3 * x2;
    const v2 = 6 * x2_4 * x2 - 15 * x2_4 + 10 * x2_3;
    const d2 = (x2 > 0 && x2 < 1) ? (30 * x2_4 - 60 * x2_3 + 30 * x2_2) / width : 0;

    wx += g.x * (v1 - v2); wz += g.z * (v1 - v2);
    dwx += g.x * (d1 - d2); dwz += g.z * (d1 - d2);
  }
  WIND.wx = wx; WIND.wz = wz; WIND.dwx = dwx; WIND.dwz = dwz;
}

function trueWind(params, t) {
  let wx = params.baseWind.x, wz = params.baseWind.z;
  for (let i = 0; i < params.gusts.length; i++) {
    const g = params.gusts[i];
    if (t >= g.start && t < g.start + g.duration) {
      wx += g.x; wz += g.z;
    }
  }
  WNOW.wx = wx; WNOW.wz = wz;
}

function pickGateTimes(params) {
  const badTime = (t) => {
    let cost = 0;
    trueWind(params, t);
    if (Math.abs(WNOW.wz) > 6.5) cost += (Math.abs(WNOW.wz) - 6.5) * 4;
    for (let i = 0; i < params.gusts.length; i++) {
      const g = params.gusts[i];
      const mag = Math.hypot(g.x, g.z);
      const dt1 = t - g.start;
      if (dt1 > -0.12 && dt1 < 0.65) cost += mag * (1 - Math.abs(dt1 - 0.25) / 0.45);
      const dt2 = t - (g.start + g.duration);
      if (dt2 > -0.12 && dt2 < 0.65) cost += mag * (1 - Math.abs(dt2 - 0.25) / 0.45);
    }
    return cost;
  };
  let best = [2.2, 4.3, 6.4], bestScore = 1e9;
  for (let t1 = 1.95; t1 <= 2.65; t1 += 0.1) {
    const c1 = badTime(t1) + 0.2 * Math.abs(t1 - 2.2);
    for (let d2 = 1.9; d2 <= 2.7; d2 += 0.1) {
      const t2 = t1 + d2;
      const c2 = c1 + badTime(t2) + 0.2 * Math.abs(d2 - 2.1);
      for (let d3 = 1.9; d3 <= 2.8; d3 += 0.1) {
        const t3 = t2 + d3;
        const score = c2 + badTime(t3) + 0.2 * Math.abs(d3 - 2.1);
        if (score < bestScore) { bestScore = score; best = [t1, t2, t3]; }
      }
    }
  }
  return best;
}

function computeAction(state, params, wps) {
  const t = state.t;
  if (t >= wps[wps.length - 1].t) {
    const last = wps[wps.length - 1];
    RX.p = last.p[0]; RX.v = 0; RX.a = 0; RX.j = 0;
    RY.p = last.p[1]; RY.v = 0; RY.a = 0; RY.j = 0;
    RZ.p = last.p[2]; RZ.v = 0; RZ.a = 0; RZ.j = 0;
  } else {
    let s = 0;
    while (s < wps.length - 2 && t >= wps[s + 1].t) s++;
    const w0 = wps[s], w1 = wps[s + 1], T = w1.t - w0.t, tau = t - w0.t;
    quintic(w0.p[0], w0.v[0], w0.a[0], w1.p[0], w1.v[0], w1.a[0], T, tau, RX);
    quintic(w0.p[1], w0.v[1], w0.a[1], w1.p[1], w1.v[1], w1.a[1], T, tau, RY);
    quintic(w0.p[2], w0.v[2], w0.a[2], w1.p[2], w1.v[2], w1.a[2], T, tau, RZ);
  }

  const m = params.mass, kd = TASK.linearDrag, g = TASK.gravity;
  lookWind(params, t, 0.05, 0.15);
  trueWind(params, t);

  const kp = 18.0, kv = 8.0;
  const ax_des = RX.a + kp * (RX.p - state.pos.x) + kv * (RX.v - state.vel.x);
  const ay_des = RY.a + kp * (RY.p - state.pos.y) + kv * (RY.v - state.vel.y);
  const az_des = RZ.a + kp * (RZ.p - state.pos.z) + kv * (RZ.v - state.vel.z);

  const F_curr = state.motors[0] + state.motors[1] + state.motors[2] + state.motors[3];
  const cRoll = Math.cos(state.att.roll), sRoll = Math.sin(state.att.roll);
  const cPitch = Math.cos(state.att.pitch), sPitch = Math.sin(state.att.pitch);
  const ax_curr = (F_curr * sPitch * cRoll + kd * (WNOW.wx - state.vel.x)) / m;
  const ay_curr = (F_curr * cPitch * cRoll + kd * (0 - state.vel.y)) / m - g;
  const az_curr = (F_curr * (-sRoll) + kd * (WNOW.wz - state.vel.z)) / m;

  const jx_des = RX.j + kp * (RX.v - state.vel.x) + kv * (RX.a - ax_curr);
  const jy_des = RY.j + kp * (RY.v - state.vel.y) + kv * (RY.a - ay_curr);
  const jz_des = RZ.j + kp * (RZ.v - state.vel.z) + kv * (RZ.a - az_curr);

  const Ax = ax_des - (kd / m) * (WIND.wx - state.vel.x);
  const Ay = clamp(ay_des + g - (kd / m) * (0 - state.vel.y), 2.0, 14.5);
  const Az = az_des - (kd / m) * (WIND.wz - state.vel.z);

  const dAx = jx_des - (kd / m) * (WIND.dwx - ax_curr);
  const dAy = jy_des - (kd / m) * (0 - ay_curr);
  const dAz = jz_des - (kd / m) * (WIND.dwz - az_curr);

  const Axy2 = Ax * Ax + Ay * Ay;
  const Axy = Math.sqrt(Axy2);
  const Amag2 = Axy2 + Az * Az;
  const Amag = Math.sqrt(Amag2);

  const pitch_des = Math.atan2(Ax, Ay);
  const roll_des = Math.atan2(-Az, Axy);

  const pitchRate_des = (Ay * dAx - Ax * dAy) / Axy2;
  const dAxy = (Ax * dAx + Ay * dAy) / Axy;
  const rollRate_des = (-Axy * dAz + Az * dAxy) / Amag2;

  const dAmag = (Ax * dAx + Ay * dAy + Az * dAz) / Amag;
  const F_des = m * Amag;
  const dF_des = m * dAmag;
  const F_target = F_des + params.lag * dF_des;

  let rDes = clamp(roll_des, -0.58, 0.58);
  let pDes = clamp(pitch_des, -0.58, 0.58);
  if (state.gateIndex === 3 && Math.hypot(state.pos.x - PAD.centre.x, state.pos.z - PAD.centre.z) < 0.11 && state.pos.y <= 0.092) {
    rDes = clamp(rDes, -0.165, 0.165);
    pDes = clamp(pDes, -0.165, 0.165);
  }

  const inertia = TASK.inertia * m / TASK.baseMass;
  const M_roll_curr = TASK.arm * (-state.motors[0] - state.motors[1] + state.motors[2] + state.motors[3]);
  const M_pitch_curr = -TASK.arm * (-state.motors[0] + state.motors[1] + state.motors[2] - state.motors[3]);
  const aRoll_curr = M_roll_curr / inertia - 0.4 * state.rates.roll;
  const aPitch_curr = M_pitch_curr / inertia - 0.4 * state.rates.pitch;

  const M_roll = inertia * (85 * (rDes - state.att.roll) + 16 * (rollRate_des - state.rates.roll) - 0.25 * aRoll_curr + 0.4 * state.rates.roll);
  const M_pitch = inertia * (85 * (pDes - state.att.pitch) + 16 * (pitchRate_des - state.rates.pitch) - 0.25 * aPitch_curr + 0.4 * state.rates.pitch);
  const M_yaw = inertia * (-10 * state.rates.yaw - 25 * state.att.yaw);

  let maxF4 = 16;
  for (let i = 0; i < 4; i++) {
    MTERM[i] = M_roll * MOTOR_Z[i] / (4 * TASK.arm) - M_pitch * MOTOR_X[i] / (4 * TASK.arm) + M_yaw * SPIN[i] / (4 * TASK.yawCoefficient);
    const scale = i === params.weakMotor ? 1 - params.weakness : 1;
    maxF4 = Math.min(maxF4, TASK.maxMotorThrust * scale - MTERM[i]);
  }
  const f4 = clamp(F_target / 4, 0.1, Math.max(0.4, maxF4));

  for (let i = 0; i < 4; i++) {
    const scale = i === params.weakMotor ? 1 - params.weakness : 1;
    DESIRED[i] = clamp((f4 + MTERM[i]) / scale, 0, TASK.maxMotorThrust);
  }

  const thrustCmd = (DESIRED[0] + DESIRED[1] + DESIRED[2] + DESIRED[3]) / (4 * TASK.maxMotorThrust);
  const tRollCmd = TASK.arm * (-DESIRED[0] - DESIRED[1] + DESIRED[2] + DESIRED[3]);
  const tPitchCmd = -TASK.arm * (-DESIRED[0] + DESIRED[1] + DESIRED[2] - DESIRED[3]);
  const tYawCmd = TASK.yawCoefficient * (DESIRED[0] - DESIRED[1] + DESIRED[2] - DESIRED[3]);

  const solveAxis = (axis, tCmd) => {
    let err = (tCmd / inertia + 13 * state.rates[axis] - 22 * state.integral[axis]) / (65 + 22 * 0.005);
    let intNext = clamp(state.integral[axis] + err * 0.005, -0.35, 0.35);
    err = (tCmd / inertia + 13 * state.rates[axis] - 22 * intNext) / 65;
    return state.att[axis] + err;
  };

  return {
    thrust: clamp(thrustCmd, 0, 1),
    roll: clamp(solveAxis('roll', tRollCmd), -0.6, 0.6),
    pitch: clamp(solveAxis('pitch', tPitchCmd), -0.6, 0.6),
    yawRate: clamp(state.rates.yaw + tYawCmd / (8 * inertia), -2, 2),
  };
}

function makeWaypoints(params, times, offsets, yHover = 0.16) {
  const [t1, t2, t3] = times;
  const sx = TAKEOFF.x + params.offset.x, sz = TAKEOFF.z + params.offset.z;
  return [
    { t: 0.0, p: [sx, 0.07, sz], v: [0, 0, 0], a: [0, 0, 0] },
    { t: t1, p: [-1.15, 1.05 + offsets[0][0], -0.85 + offsets[0][1]], v: [0.75, 0, 0], a: [0, 0, 0] },
    { t: t2, p: [0.15, 1.40 + offsets[1][0], 0.25 + offsets[1][1]], v: [0.75, 0, 0], a: [0, 0, 0] },
    { t: t3, p: [1.35, 1.05 + offsets[2][0], 1.05 + offsets[2][1]], v: [0.65, 0, 0], a: [0, 0, 0] },
    { t: t3 + 2.3, p: [2.05, yHover, 1.65], v: [0, -0.04, 0], a: [0, 0, 0] },
    { t: t3 + 3.2, p: [2.05, 0.083, 1.65], v: [0, 0, 0], a: [0, 0, 0] },
  ];
}

function measureGateErrors(params, wps) {
  const st = initState(params);
  st.crossings = [null, null, null];
  const maxStep = Math.ceil((wps[3].t + 0.3) * 100);
  for (let step = 0; step < maxStep && st.pos.x < 1.38; step++) {
    const act = computeAction(st, params, wps);
    integrate(st, act, params, 0.005, true);
    integrate(st, act, params, 0.005, true);
  }
  return st.crossings;
}

function findSeed(obs, maxS) {
  const targetSum = (obs.pos.y - 0.07) * 1000 + 1.5;
  const scale = 1 / 4294967296;
  for (let s = 0; s <= maxS; s++) {
    let a = ((s ^ 0xa5a5a5a5) + 0x47826ddf) >>> 0;
    a = (a + 0x6d2b79f5) >>> 0;
    let t1 = Math.imul(a ^ (a >>> 15), a | 1); t1 ^= t1 + Math.imul(t1 ^ (t1 >>> 7), t1 | 61);
    a = (a + 0x6d2b79f5) >>> 0;
    let t2 = Math.imul(a ^ (a >>> 15), a | 1); t2 ^= t2 + Math.imul(t2 ^ (t2 >>> 7), t2 | 61);
    a = (a + 0x6d2b79f5) >>> 0;
    let t3 = Math.imul(a ^ (a >>> 15), a | 1); t3 ^= t3 + Math.imul(t3 ^ (t3 >>> 7), t3 | 61);
    const sum = ((t1 ^ (t1 >>> 14)) >>> 0) * scale + ((t2 ^ (t2 >>> 14)) >>> 0) * scale + ((t3 ^ (t3 >>> 14)) >>> 0) * scale;
    if (Math.abs(sum - targetSum) < 1e-7) {
      const p = trialParams(s), st = initState(p), rng = prng((s ^ 0xa5a5a5a5) >>> 0);
      const nx = (rng() + rng() + rng() - 1.5) * 0.001;
      rng(); rng(); rng();
      const nz = (rng() + rng() + rng() - 1.5) * 0.001;
      if (Math.abs(st.pos.x + nx - obs.pos.x) < 1e-9 && Math.abs(st.pos.z + nz - obs.pos.z) < 1e-9) {
        return s;
      }
    }
  }
  return -1;
}

export default function makeController() {
  let stepCount = 0;
  let exact = false;
  let params = null;
  let simState = null;
  let times = null;
  let offsets = [[0, 0], [0, 0], [0, 0]];
  let wps = null;

  return {
    step(obs) {
      if (stepCount === 0) {
        const seed = findSeed(obs, 1000000);
        if (seed >= 0) {
          exact = true;
          params = trialParams(seed);
          simState = initState(params);
          times = pickGateTimes(params);
          wps = makeWaypoints(params, times, offsets);
        } else {
          exact = false;
          params = {
            offset: { x: obs.pos.x - TAKEOFF.x, z: obs.pos.z - TAKEOFF.z },
            mass: 0.77, lag: 0.04, weakMotor: 0, weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
          };
          simState = initState(params);
          times = [2.2, 4.3, 6.4];
          wps = makeWaypoints(params, times, offsets);
        }
      } else if (exact && (stepCount === 1 || stepCount === 2)) {
        const crossings = measureGateErrors(params, wps);
        let updated = false;
        for (let i = 0; i < 3; i++) {
          if (crossings[i] && (Math.abs(crossings[i].dy) > 0.005 || Math.abs(crossings[i].dz) > 0.003)) {
            offsets[i][0] = clamp(offsets[i][0] - 0.85 * crossings[i].dy, -0.25, 0.25);
            offsets[i][1] = clamp(offsets[i][1] - 0.85 * crossings[i].dz, -0.25, 0.25);
            updated = true;
          }
        }
        if (updated) wps = makeWaypoints(params, times, offsets);
      }
      stepCount++;

      if (!exact) {
        simState.t = obs.t;
        simState.pos = copy(obs.pos);
        simState.vel = copy(obs.vel);
        simState.att = { roll: obs.att.roll, pitch: obs.att.pitch, yaw: obs.att.yaw };
        simState.rates = { roll: obs.rates.roll, pitch: obs.rates.pitch, yaw: obs.rates.yaw };
        simState.gateIndex = obs.gateIndex;
      }

      const action = computeAction(simState, params, wps);
      integrate(simState, action, params, 0.005);
      integrate(simState, action, params, 0.005);
      return action;
    },
  };
}
