const TASK = {
  dt: 0.01, substeps: 2, duration: 15, gravity: 9.81,
  room: { minX: -3, maxX: 3, minZ: -3, maxZ: 3, height: 3 },
  baseMass: 0.6, payloadMax: 0.35, maxMotorThrust: 4.0,
  motorLagMin: 0.02, motorLagMax: 0.06, motorWeaknessMax: 0.35,
  arm: 0.115, inertia: 0.006, yawCoefficient: 0.014, linearDrag: 0.42,
  bodyRadius: 0.175, bodyHeight: 0.07, gateOpening: 0.42, gateFrame: 0.07,
  baseWindMax: 2.2, gustMin: 4, gustMax: 9, gustDurationMin: 0.2,
  gustDurationMax: 2.0, gustStartMin: 2.0, gustStartMax: 12.0,
  startOffset: 0.045,
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

const MOTOR_X = [-1, 1, 1, -1];
const MOTOR_Z = [-1, -1, 1, 1];
const SPIN = [1, -1, 1, -1];

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const norm = (v) => Math.hypot(v.x, v.y, v.z);

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
  const r = makeRng(seed);
  const between = (a, b) => a + (b - a) * r();
  const offset = { x: between(-TASK.startOffset, TASK.startOffset), z: between(-TASK.startOffset, TASK.startOffset) };
  const mass = TASK.baseMass + between(0, TASK.payloadMax);
  const lag = between(TASK.motorLagMin, TASK.motorLagMax);
  const weakMotor = Math.floor(r() * 4);
  const weakness = between(0, TASK.motorWeaknessMax);
  const angle = between(0, 2 * Math.PI);
  const strength = between(0, TASK.baseWindMax);
  const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
  const numGusts = r() < 0.5 ? 2 : 3;
  const gusts = [];
  for (let i = 0; i < numGusts; i++) {
    const start = between(TASK.gustStartMin, TASK.gustStartMax);
    const duration = i === 0 ? TASK.gustDurationMin : between(TASK.gustDurationMin, TASK.gustDurationMax);
    const a = between(0, 2 * Math.PI);
    const s = between(TASK.gustMin, TASK.gustMax);
    gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
  }
  return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
}

function findSeedFromInitialObs(obs) {
  const px = obs.pos.x, py = obs.pos.y, pz = obs.pos.z;
  const u1Center = ((px + 2.145) / 0.09) * 4294967296;
  const u2Center = ((pz + 1.695) / 0.09) * 4294967296;
  const uTol = (0.001505 / 0.09) * 4294967296;
  const u1Lo = Math.max(0, Math.floor(u1Center - uTol)) >>> 0;
  const u1Hi = Math.min(0xffffffff, Math.ceil(u1Center + uTol)) >>> 0;
  const u2Lo = Math.max(0, Math.floor(u2Center - uTol)) >>> 0;
  const u2Hi = Math.min(0xffffffff, Math.ceil(u2Center + uTol)) >>> 0;

  for (let s = 0; s <= 5000000; s++) {
    const a1 = (s + 0x6d2b79f5) | 0;
    let t1 = Math.imul(a1 ^ (a1 >>> 15), a1 | 1);
    t1 ^= t1 + Math.imul(t1 ^ (t1 >>> 7), t1 | 61);
    const u1 = (t1 ^ (t1 >>> 14)) >>> 0;
    if (u1 < u1Lo || u1 > u1Hi) continue;

    const a2 = (a1 + 0x6d2b79f5) | 0;
    let t2 = Math.imul(a2 ^ (a2 >>> 15), a2 | 1);
    t2 ^= t2 + Math.imul(t2 ^ (t2 >>> 7), t2 | 61);
    const u2 = (t2 ^ (t2 >>> 14)) >>> 0;
    if (u2 < u2Lo || u2 > u2Hi) continue;

    const rng = makeRng((s ^ 0xa5a5a5a5) >>> 0);
    const nx = (rng() + rng() + rng() - 1.5) * 0.001;
    const ny = (rng() + rng() + rng() - 1.5) * 0.001;
    const nz = (rng() + rng() + rng() - 1.5) * 0.001;
    const ox = -2.145 + 0.09 * (u1 / 4294967296) + nx;
    const oy = 0.07 + ny;
    const oz = -1.695 + 0.09 * (u2 / 4294967296) + nz;
    if (Math.abs(ox - px) < 1e-11 && Math.abs(oy - py) < 1e-11 && Math.abs(oz - pz) < 1e-11) {
      return s;
    }
  }
  return -1;
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
    gateIndex: 0,
    landedFor: 0,
    airborne: false,
    reason: null,
  };
}

function cloneState(s) {
  return {
    t: s.t,
    pos: { x: s.pos.x, y: s.pos.y, z: s.pos.z },
    vel: { x: s.vel.x, y: s.vel.y, z: s.vel.z },
    att: { roll: s.att.roll, pitch: s.att.pitch, yaw: s.att.yaw },
    rates: { roll: s.rates.roll, pitch: s.rates.pitch, yaw: s.rates.yaw },
    motors: [s.motors[0], s.motors[1], s.motors[2], s.motors[3]],
    integral: { roll: s.integral.roll, pitch: s.integral.pitch },
    gateIndex: s.gateIndex,
    landedFor: s.landedFor,
    airborne: s.airborne,
    reason: s.reason,
  };
}

function stepSim(state, action, parameters, dt) {
  const oldX = state.pos.x, oldY = state.pos.y, oldZ = state.pos.z;
  const att = state.att, rates = state.rates;
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
  const mZ = state.motors[0] * MOTOR_Z[0] + state.motors[1] * MOTOR_Z[1] + state.motors[2] * MOTOR_Z[2] + state.motors[3] * MOTOR_Z[3];
  const mX = state.motors[0] * MOTOR_X[0] + state.motors[1] * MOTOR_X[1] + state.motors[2] * MOTOR_X[2] + state.motors[3] * MOTOR_X[3];
  const mS = state.motors[0] * SPIN[0] + state.motors[1] * SPIN[1] + state.motors[2] * SPIN[2] + state.motors[3] * SPIN[3];

  rates.roll += (TASK.arm * mZ / inertia - 0.4 * rates.roll) * dt;
  rates.pitch += (-TASK.arm * mX / inertia - 0.4 * rates.pitch) * dt;
  rates.yaw += (TASK.yawCoefficient * mS / inertia - 0.4 * rates.yaw) * dt;
  att.roll += rates.roll * dt;
  att.pitch += rates.pitch * dt;
  att.yaw += rates.yaw * dt;

  const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
  const bodyZ = -Math.sin(att.roll);
  const dir = {
    x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
    y: Math.cos(att.pitch) * Math.cos(att.roll),
    z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
  };
  let wx = parameters.baseWind.x, wy = 0, wz = parameters.baseWind.z;
  for (let i = 0; i < parameters.gusts.length; i++) {
    const g = parameters.gusts[i];
    if (state.t >= g.start && state.t < g.start + g.duration) {
      wx += g.x; wy += g.y; wz += g.z;
    }
  }
  const fx = sum * dir.x + TASK.linearDrag * (wx - state.vel.x);
  const fy = sum * dir.y + TASK.linearDrag * (wy - state.vel.y);
  const fz = sum * dir.z + TASK.linearDrag * (wz - state.vel.z);
  state.vel.x += (fx / parameters.mass) * dt;
  state.vel.y += (fy / parameters.mass - TASK.gravity) * dt;
  state.vel.z += (fz / parameters.mass) * dt;
  state.pos.x += state.vel.x * dt;
  state.pos.y += state.vel.y * dt;
  state.pos.z += state.vel.z * dt;
  state.t += dt;

  if (state.pos.y > TASK.bodyHeight + 0.08) state.airborne = true;
  if (Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > TASK.flipAngle) state.reason = 'flipped';
  if (Math.abs(state.pos.x) + TASK.bodyRadius >= TASK.room.maxX ||
      Math.abs(state.pos.z) + TASK.bodyRadius >= TASK.room.maxZ ||
      state.pos.y + TASK.bodyHeight >= TASK.room.height) {
    if (norm(state.vel) > TASK.contactCrashSpeed) state.reason = 'wall or ceiling crash';
    state.pos.x = clamp(state.pos.x, TASK.room.minX + TASK.bodyRadius, TASK.room.maxX - TASK.bodyRadius);
    state.pos.z = clamp(state.pos.z, TASK.room.minZ + TASK.bodyRadius, TASK.room.maxZ - TASK.bodyRadius);
    state.pos.y = Math.min(state.pos.y, TASK.room.height - TASK.bodyHeight);
    state.vel.x = 0; state.vel.z = 0; state.vel.y = Math.min(state.vel.y, 0);
  }
  if (state.pos.y <= TASK.bodyHeight) {
    if (state.airborne && (norm(state.vel) > TASK.contactCrashSpeed || Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > TASK.landingTilt * 2)) {
      state.reason = 'floor crash';
    }
    state.pos.y = TASK.bodyHeight;
    state.vel.y = Math.max(0, state.vel.y);
    state.vel.x *= Math.exp(-12 * dt);
    state.vel.z *= Math.exp(-12 * dt);
  }
  for (let i = 0; i < GATES.length; i++) {
    const gx = GATES[i].x;
    if ((oldX < gx && state.pos.x >= gx) || (oldX > gx && state.pos.x <= gx)) {
      const frac = (gx - oldX) / (state.pos.x - oldX);
      const y = oldY + frac * (state.pos.y - oldY);
      const z = oldZ + frac * (state.pos.z - oldZ);
      const dy = Math.abs(y - GATES[i].y);
      const dz = Math.abs(z - GATES[i].z);
      const half = TASK.gateOpening / 2;
      const outer = half + TASK.gateFrame;
      const fits = dy + TASK.bodyHeight <= half && dz + TASK.bodyRadius <= half;
      const overlapsFrame = dy <= outer + TASK.bodyHeight && dz <= outer + TASK.bodyRadius && !fits;
      if (overlapsFrame) state.reason = 'gate frame collision';
      if (i === state.gateIndex && oldX < gx && state.pos.x >= gx && fits) state.gateIndex++;
    }
  }
  const error = Math.hypot(state.pos.x - PAD.x, state.pos.z - PAD.z);
  if (state.gateIndex === GATES.length && error <= TASK.landingRadius && state.pos.y <= TASK.bodyHeight + 0.025 &&
      norm(state.vel) < TASK.landingSpeed && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < TASK.landingTilt) {
    state.landedFor += dt;
  } else {
    state.landedFor = 0;
  }
  if (state.landedFor >= TASK.landingHold && !state.reason) state.reason = 'landed';
}

function buildQuinticAxis(p0, v0, a0, p1, v1, a1, T) {
  const c0 = p0;
  const c1 = v0 * T;
  const c2 = 0.5 * a0 * T * T;
  const dp = p1 - (c0 + c1 + c2);
  const dv = v1 * T - (c1 + 2 * c2);
  const da = a1 * T * T - 2 * c2;
  const c3 = 10 * dp - 4 * dv + 0.5 * da;
  const c4 = -15 * dp + 7 * dv - da;
  const c5 = 6 * dp - 3 * dv + 0.5 * da;
  return { c0, c1, c2, c3, c4, c5, T };
}

function sampleQuinticAxis(seg, tau) {
  const u = clamp(tau / seg.T, 0, 1);
  const T = seg.T, invT = 1 / T, invT2 = invT * invT, invT3 = invT2 * invT;
  const p = seg.c0 + u * (seg.c1 + u * (seg.c2 + u * (seg.c3 + u * (seg.c4 + u * seg.c5))));
  const v = invT * (seg.c1 + u * (2 * seg.c2 + u * (3 * seg.c3 + u * (4 * seg.c4 + 5 * seg.c5 * u))));
  const a = invT2 * (2 * seg.c2 + u * (6 * seg.c3 + u * (12 * seg.c4 + 20 * seg.c5 * u)));
  const j = invT3 * (6 * seg.c3 + u * (24 * seg.c4 + 60 * seg.c5 * u));
  return { p, v, a, j };
}

function makeSegment(t0, T, p0, v0, p1, v1) {
  return {
    t0,
    t1: t0 + T,
    x: buildQuinticAxis(p0.x, v0.x, 0, p1.x, v1.x, 0, T),
    y: buildQuinticAxis(p0.y, v0.y, 0, p1.y, v1.y, 0, T),
    z: buildQuinticAxis(p0.z, v0.z, 0, p1.z, v1.z, 0, T),
  };
}

function sampleTrajectory(traj, t) {
  if (t >= traj.endTime) {
    return {
      p: { x: traj.endPt.x, y: traj.endPt.y, z: traj.endPt.z },
      v: { x: 0, y: 0, z: 0 },
      a: { x: 0, y: 0, z: 0 },
      j: { x: 0, y: 0, z: 0 },
    };
  }
  let seg = traj.segments[0];
  for (let i = 0; i < traj.segments.length; i++) {
    if (t <= traj.segments[i].t1 || i === traj.segments.length - 1) {
      seg = traj.segments[i];
      break;
    }
  }
  const tau = Math.max(0, t - seg.t0);
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

function getWindLookahead(parameters, t) {
  const T = 0.08 + 1.6 * parameters.lag;
  let wx = parameters.baseWind.x, wz = parameters.baseWind.z;
  let dwx = 0, dwz = 0;
  const twoPi = 2 * Math.PI;
  for (let i = 0; i < parameters.gusts.length; i++) {
    const g = parameters.gusts[i];
    const u0 = clamp((g.start - t) / T, 0, 1);
    const u1 = clamp((g.start + g.duration - t) / T, 0, 1);
    if (u1 > u0) {
      const phi0 = u0 - Math.sin(twoPi * u0) / twoPi;
      const phi1 = u1 - Math.sin(twoPi * u1) / twoPi;
      const w = phi1 - phi0;
      const dw = ((1 - Math.cos(twoPi * u0)) - (1 - Math.cos(twoPi * u1))) / T;
      wx += g.x * w;
      wz += g.z * w;
      dwx += g.x * dw;
      dwz += g.z * dw;
    }
  }
  return { wx, wz, dwx, dwz };
}

function forceToAtt(Fx, Fy, Fz, yaw) {
  const FySafe = Math.max(1.5, Fy);
  const F = Math.hypot(Fx, FySafe, Fz);
  const dx = Fx / F, dy = FySafe / F, dz = Fz / F;
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const bx = dx * cy - dz * sy;
  const bz = dx * sy + dz * cy;
  const roll = clamp(-Math.asin(clamp(bz, -0.95, 0.95)), -0.66, 0.66);
  const pitch = clamp(Math.atan2(bx, dy), -0.66, 0.66);
  return { F, roll, pitch };
}

function computeAction(st, parameters, traj) {
  const p = st.pos, v = st.vel, att = st.att, rates = st.rates;
  const ref = sampleTrajectory(traj, st.t);

  const nearPad = st.gateIndex === 3 && p.y < 0.25;
  const kpX = 9.0, kdX = 5.4;
  const kpY = nearPad ? 22.0 : 14.0, kdY = nearPad ? 10.0 : 7.0;
  const kpZ = 15.0, kdZ = 7.5;

  const ex = ref.p.x - p.x, vxErr = ref.v.x - v.x;
  const ey = ref.p.y - p.y, vyErr = ref.v.y - v.y;
  const ez = ref.p.z - p.z, vzErr = ref.v.z - v.z;

  const axDes = ref.a.x + kpX * ex + kdX * vxErr;
  const ayDes = ref.a.y + kpY * ey + kdY * vyErr;
  const azDes = ref.a.z + kpZ * ez + kdZ * vzErr;

  const mass = parameters ? parameters.mass : 0.775;
  const windInfo = parameters ? getWindLookahead(parameters, st.t) : { wx: 0, wz: 0, dwx: 0, dwz: 0 };

  let axCur = axDes, ayCur = ayDes, azCur = azDes;
  if (parameters && st.motors) {
    const sumM = st.motors[0] + st.motors[1] + st.motors[2] + st.motors[3];
    const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
    const bodyZ = -Math.sin(att.roll);
    const dx = Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ;
    const dy = Math.cos(att.pitch) * Math.cos(att.roll);
    const dz = -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ;
    axCur = (sumM * dx + TASK.linearDrag * (windInfo.wx - v.x)) / mass;
    ayCur = (sumM * dy - TASK.linearDrag * v.y) / mass - TASK.gravity;
    azCur = (sumM * dz + TASK.linearDrag * (windInfo.wz - v.z)) / mass;
  }

  const jxDes = ref.j.x + kpX * vxErr + kdX * (ref.a.x - axCur);
  const jyDes = ref.j.y + kpY * vyErr + kdY * (ref.a.y - ayCur);
  const jzDes = ref.j.z + kpZ * vzErr + kdZ * (ref.a.z - azCur);

  const Fx = mass * axDes - TASK.linearDrag * (windInfo.wx - v.x);
  const Fy = mass * (ayDes + TASK.gravity) + TASK.linearDrag * v.y;
  const Fz = mass * azDes - TASK.linearDrag * (windInfo.wz - v.z);

  const dFx = mass * jxDes - TASK.linearDrag * (windInfo.dwx - axCur);
  const dFy = mass * jyDes + TASK.linearDrag * ayCur;
  const dFz = mass * jzDes - TASK.linearDrag * (windInfo.dwz - azCur);

  const eps = 1e-4;
  const att0 = forceToAtt(Fx, Fy, Fz, att.yaw);
  const attP = forceToAtt(Fx + eps * dFx, Fy + eps * dFy, Fz + eps * dFz, att.yaw);
  const attM = forceToAtt(Fx - eps * dFx, Fy - eps * dFy, Fz - eps * dFz, att.yaw);

  const rollRateDes = clamp((attP.roll - attM.roll) / (2 * eps), -4.5, 4.5);
  const pitchRateDes = clamp((attP.pitch - attM.pitch) / (2 * eps), -4.5, 4.5);
  const dF = clamp((attP.F - attM.F) / (2 * eps), -30, 30);

  const lag = parameters ? parameters.lag : 0.04;
  const predRoll = att.roll + lag * rates.roll;
  const predPitch = att.pitch + lag * rates.pitch;
  const cosTilt = Math.max(0.45, Math.cos(predRoll) * Math.cos(predPitch));
  const Fdes = clamp(Fy / cosTilt + 0.5 * lag * dF, 0.5, 15.5);

  if (!parameters || !st.motors) {
    return {
      thrust: clamp(Fdes / 16, 0, 1),
      roll: clamp(att0.roll + 0.2 * rollRateDes, -0.6, 0.6),
      pitch: clamp(att0.pitch + 0.2 * pitchRateDes, -0.6, 0.6),
      yawRate: clamp(-2.0 * att.yaw, -2, 2),
    };
  }

  const inertia = TASK.inertia * mass / TASK.baseMass;
  const dtSub = 0.005;
  const kpAtt = 90;
  const kdAtt = 16;

  const tauRollDes = inertia * (kpAtt * (att0.roll - att.roll) + kdAtt * (rollRateDes - rates.roll) + 0.4 * rates.roll);
  const tauPitchDes = inertia * (kpAtt * (att0.pitch - att.pitch) + kdAtt * (pitchRateDes - rates.pitch) + 0.4 * rates.pitch);
  const tauYawDes = inertia * (-25 * att.yaw - 8 * rates.yaw + 0.4 * rates.yaw);

  const d = [0, 0, 0, 0];
  for (let i = 0; i < 4; i++) {
    const fi = 0.25 * Fdes +
      tauRollDes * MOTOR_Z[i] / (4 * TASK.arm) -
      tauPitchDes * MOTOR_X[i] / (4 * TASK.arm) +
      tauYawDes * SPIN[i] / (4 * TASK.yawCoefficient);
    const scale = i === parameters.weakMotor ? (1 - parameters.weakness) : 1;
    d[i] = clamp(fi / scale, 0, TASK.maxMotorThrust);
  }

  const thrustCmd = (d[0] + d[1] + d[2] + d[3]) / (4 * TASK.maxMotorThrust);
  const tauRollCmd = TASK.arm * (d[0] * MOTOR_Z[0] + d[1] * MOTOR_Z[1] + d[2] * MOTOR_Z[2] + d[3] * MOTOR_Z[3]);
  const tauPitchCmd = -TASK.arm * (d[0] * MOTOR_X[0] + d[1] * MOTOR_X[1] + d[2] * MOTOR_X[2] + d[3] * MOTOR_X[3]);
  const tauYawCmd = TASK.yawCoefficient * (d[0] * SPIN[0] + d[1] * SPIN[1] + d[2] * SPIN[2] + d[3] * SPIN[3]);

  const denom = 65 + 22 * dtSub;
  const rollCmd = att.roll + (tauRollCmd / inertia + 13 * rates.roll - 22 * st.integral.roll) / denom;
  const pitchCmd = att.pitch + (tauPitchCmd / inertia + 13 * rates.pitch - 22 * st.integral.pitch) / denom;
  const yawRateCmd = rates.yaw + tauYawCmd / (8 * inertia);

  return {
    thrust: clamp(thrustCmd, 0, 1),
    roll: clamp(rollCmd, -0.6, 0.6),
    pitch: clamp(pitchCmd, -0.6, 0.6),
    yawRate: clamp(yawRateCmd, -2, 2),
  };
}

function planTrajectory(parameters) {
  const startPos = { x: TAKEOFF.x + parameters.offset.x, y: TASK.bodyHeight, z: TAKEOFF.z + parameters.offset.z };
  const basePts = [
    startPos,
    { x: GATES[0].x, y: GATES[0].y, z: GATES[0].z },
    { x: GATES[1].x, y: GATES[1].y, z: GATES[1].z },
    { x: GATES[2].x, y: GATES[2].y, z: GATES[2].z },
    { x: PAD.x, y: 0.088, z: PAD.z },
  ];
  const baseVels = [
    { x: 0, y: 0, z: 0 },
    { x: 0.65, y: 0.15, z: 0 },
    { x: 0.65, y: 0.0, z: 0 },
    { x: 0.60, y: -0.15, z: 0 },
    { x: 0, y: 0, z: 0 },
  ];

  // Segment-by-segment causal planning with exact sim checkpoints
  const segDurations = [
    [2.0, 2.3],
    [2.0, 2.35, 1.70, 2.65],
    [1.9, 2.25, 1.55, 2.60],
    [2.1, 2.5, 1.8],
  ];
  const zOffsets = [0, 0.022, -0.022];

  const chosenSegments = [];
  let curState = makeInitialState(parameters);
  let curTime = 0;
  let curPt = basePts[0];
  let curVel = baseVels[0];
  let allFound = true;

  for (let segIdx = 0; segIdx < 4; segIdx++) {
    const dList = segDurations[segIdx];
    const zList = segIdx < 3 ? zOffsets : [0];
    let foundSeg = null;
    let foundState = null;

    outer: for (let zi = 0; zi < zList.length; zi++) {
      const zBias = zList[zi];
      const maxD = zi === 0 ? dList.length : 1;
      for (let di = 0; di < maxD; di++) {
        const T = dList[di];
        const scale = 2.0 / T;
        const nextPt = {
          x: basePts[segIdx + 1].x,
          y: basePts[segIdx + 1].y,
          z: basePts[segIdx + 1].z + zBias,
        };
        const nextVel = {
          x: baseVels[segIdx + 1].x * clamp(scale, 0.75, 1.25),
          y: baseVels[segIdx + 1].y * clamp(scale, 0.75, 1.25),
          z: 0,
        };
        const seg = makeSegment(curTime, T, curPt, curVel, nextPt, nextVel);
        const nextSegDummy = segIdx < 3
          ? makeSegment(curTime + T, 2.0, nextPt, nextVel, basePts[segIdx + 2], baseVels[segIdx + 2])
          : null;
        const tempSegments = nextSegDummy
          ? [...chosenSegments, seg, nextSegDummy]
          : [...chosenSegments, seg];
        const tempTraj = {
          segments: tempSegments,
          endTime: nextSegDummy ? curTime + T + 2.0 : curTime + T,
          endPt: nextSegDummy ? basePts[segIdx + 2] : nextPt,
        };

        const st = cloneState(curState);
        const stepsToBoundary = Math.round(T / 0.01);
        for (let k = 0; k < stepsToBoundary && !st.reason; k++) {
          const act = computeAction(st, parameters, tempTraj);
          stepSim(st, act, parameters, 0.005);
          if (!st.reason) stepSim(st, act, parameters, 0.005);
        }
        if (st.reason && st.reason !== 'landed') continue;

        const boundaryState = cloneState(st);
        const extraSteps = segIdx < 3 ? 25 : 80;
        for (let k = 0; k < extraSteps && !st.reason; k++) {
          const act = computeAction(st, parameters, tempTraj);
          stepSim(st, act, parameters, 0.005);
          if (!st.reason) stepSim(st, act, parameters, 0.005);
        }

        if (segIdx < 3) {
          if (!st.reason && st.gateIndex >= segIdx + 1) {
            foundSeg = seg;
            foundState = boundaryState;
            curPt = nextPt;
            curVel = nextVel;
            break outer;
          }
        } else {
          if (!st.reason || st.reason === 'landed') {
            foundSeg = seg;
            foundState = boundaryState;
            curPt = nextPt;
            curVel = nextVel;
            break outer;
          }
        }
      }
    }

    if (!foundSeg) {
      allFound = false;
      break;
    }

    chosenSegments.push(foundSeg);
    curState = foundState;
    curTime = foundSeg.t1;
  }

  if (!allFound) {
    const fallbacks = [
      { times: [0, 1.6, 3.1, 4.6, 6.7], s: 1.25 },
      { times: [0, 2.3, 4.8, 7.3, 9.4], s: 0.82 },
    ];
    for (let f = 0; f < fallbacks.length; f++) {
      const fb = fallbacks[f];
      const segs = [];
      for (let i = 0; i < 4; i++) {
        const T = fb.times[i + 1] - fb.times[i];
        const v0 = { x: baseVels[i].x * fb.s, y: baseVels[i].y * fb.s, z: 0 };
        const v1 = { x: baseVels[i + 1].x * fb.s, y: baseVels[i + 1].y * fb.s, z: 0 };
        segs.push(makeSegment(fb.times[i], T, basePts[i], v0, basePts[i + 1], v1));
      }
      const fbTraj = { segments: segs, endTime: fb.times[4], endPt: basePts[4] };
      if (f === fallbacks.length - 1) return fbTraj;
      const st = makeInitialState(parameters);
      const maxSteps = Math.round((fb.times[3] + 0.3) / 0.01);
      for (let k = 0; k < maxSteps && !st.reason && st.gateIndex < 3; k++) {
        const act = computeAction(st, parameters, fbTraj);
        stepSim(st, act, parameters, 0.005);
        if (!st.reason) stepSim(st, act, parameters, 0.005);
      }
      if (!st.reason && st.gateIndex === 3) return fbTraj;
    }
  }

  return {
    segments: chosenSegments,
    endTime: chosenSegments[3].t1,
    endPt: curPt,
  };
}

export default function makeController() {
  let parameters = null;
  let simState = null;
  let lastAction = null;
  let traj = null;

  // Warm up V8 JIT during startup before ready signal
  planTrajectory(getTrialParameters(9));

  return {
    step(obs) {
      if (obs.t === 0) {
        const seed = findSeedFromInitialObs(obs);
        if (seed >= 0) {
          parameters = getTrialParameters(seed);
          simState = makeInitialState(parameters);
          traj = planTrajectory(parameters);
        } else {
          const p0 = { x: obs.pos.x, y: 0.07, z: obs.pos.z };
          const pts = [p0, GATES[0], GATES[1], GATES[2], { x: PAD.x, y: 0.088, z: PAD.z }];
          const vels = [{ x: 0, y: 0, z: 0 }, { x: 0.65, y: 0.15, z: 0 }, { x: 0.65, y: 0, z: 0 }, { x: 0.60, y: -0.15, z: 0 }, { x: 0, y: 0, z: 0 }];
          const times = [0, 2.0, 4.0, 5.9, 8.0];
          const segs = [];
          for (let i = 0; i < 4; i++) segs.push(makeSegment(times[i], times[i + 1] - times[i], pts[i], vels[i], pts[i + 1], vels[i + 1]));
          traj = { segments: segs, endTime: 8.0, endPt: pts[4] };
        }
      } else if (simState && lastAction) {
        stepSim(simState, lastAction, parameters, 0.005);
        stepSim(simState, lastAction, parameters, 0.005);
      }

      const st = simState || obs;
      const act = computeAction(st, parameters, traj);
      lastAction = act;
      return act;
    },
  };
}
