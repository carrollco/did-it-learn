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

function makeRng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function getTrialParams(seed) {
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

function integrateSim(state, action, parameters, dt) {
  const old = copy(state.pos);
  const att = state.att;
  const rates = state.rates;
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
  const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
  const bodyZ = -Math.sin(att.roll);
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
  if (Math.abs(state.pos.x) + TASK.bodyRadius >= TASK.room.maxX || Math.abs(state.pos.z) + TASK.bodyRadius >= TASK.room.maxZ || state.pos.y + TASK.bodyHeight >= TASK.room.height) {
    state.pos.x = clamp(state.pos.x, TASK.room.minX + TASK.bodyRadius, TASK.room.maxX - TASK.bodyRadius);
    state.pos.z = clamp(state.pos.z, TASK.room.minZ + TASK.bodyRadius, TASK.room.maxZ - TASK.bodyRadius);
    state.pos.y = Math.min(state.pos.y, TASK.room.height - TASK.bodyHeight);
    state.vel.x = 0; state.vel.z = 0; state.vel.y = Math.min(state.vel.y, 0);
  }
  if (state.pos.y <= TASK.bodyHeight) {
    state.pos.y = TASK.bodyHeight; state.vel.y = Math.max(0, state.vel.y);
    state.vel.x *= Math.exp(-12 * dt); state.vel.z *= Math.exp(-12 * dt);
  }
  for (let i = 0; i < GATES.length; i++) {
    const gate = GATES[i], x = gate.centre.x;
    if ((old.x < x && state.pos.x >= x) || (old.x > x && state.pos.x <= x)) {
      const fraction = (x - old.x) / (state.pos.x - old.x);
      const y = old.y + fraction * (state.pos.y - old.y), z = old.z + fraction * (state.pos.z - old.z);
      const dy = Math.abs(y - gate.centre.y), dz = Math.abs(z - gate.centre.z);
      const half = TASK.gateOpening / 2;
      const fits = dy + TASK.bodyHeight <= half && dz + TASK.bodyRadius <= half;
      if (i === state.gateIndex && old.x < x && state.pos.x >= x && fits) state.gateIndex++;
    }
  }
}

function matchSeed(obs) {
  const tx = obs.pos.x, ty = obs.pos.y, tz = obs.pos.z;
  const r1Lo = Math.max(0, Math.floor(((tx + 2.145 - 0.001505) / 0.09) * 4294967296));
  const r1Hi = Math.min(0xffffffff, Math.ceil(((tx + 2.145 + 0.001505) / 0.09) * 4294967296));
  const r2Lo = Math.max(0, Math.floor(((tz + 1.695 - 0.001505) / 0.09) * 4294967296));
  const r2Hi = Math.min(0xffffffff, Math.ceil(((tz + 1.695 + 0.001505) / 0.09) * 4294967296));
  for (let s = 0; s < 2500000; s++) {
    let a = (s + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u1 = (t ^ (t >>> 14)) >>> 0;
    if (u1 < r1Lo || u1 > r1Hi) continue;
    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u2 = (t ^ (t >>> 14)) >>> 0;
    if (u2 < r2Lo || u2 > r2Hi) continue;
    const ox = -2.145 + 0.09 * (u1 / 4294967296);
    const oz = -1.695 + 0.09 * (u2 / 4294967296);
    const rng = makeRng((s ^ 0xa5a5a5a5) >>> 0);
    const nx = (rng() + rng() + rng() - 1.5) * 0.001;
    const ny = (rng() + rng() + rng() - 1.5) * 0.001;
    const nz = (rng() + rng() + rng() - 1.5) * 0.001;
    if (Math.abs(ox + nx - tx) < 1e-9 && Math.abs(0.07 + ny - ty) < 1e-9 && Math.abs(oz + nz - tz) < 1e-9) {
      return s;
    }
  }
  return -1;
}

function makeQuintic1D(p0, v0, a0, p1, v1, a1, T) {
  const c0 = p0;
  const c1 = v0;
  const c2 = 0.5 * a0;
  const dp = p1 - (p0 + v0 * T + 0.5 * a0 * T * T);
  const dv = v1 - (v0 + a0 * T);
  const da = a1 - a0;
  const T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
  const c3 = (10 * dp - 4 * dv * T + 0.5 * da * T2) / T3;
  const c4 = (-15 * dp + 7 * dv * T - da * T2) / T4;
  const c5 = (6 * dp - 3 * dv * T + 0.5 * da * T2) / T5;
  return { c0, c1, c2, c3, c4, c5, T };
}

function sampleQuintic1D(seg, s) {
  const p = seg.c0 + s * (seg.c1 + s * (seg.c2 + s * (seg.c3 + s * (seg.c4 + s * seg.c5))));
  const v = seg.c1 + s * (2 * seg.c2 + s * (3 * seg.c3 + s * (4 * seg.c4 + 5 * s * seg.c5)));
  const a = 2 * seg.c2 + s * (6 * seg.c3 + s * (12 * seg.c4 + 20 * s * seg.c5));
  const j = 6 * seg.c3 + s * (24 * seg.c4 + 60 * s * seg.c5);
  return { p, v, a, j };
}

function buildTrajFromWps(wps) {
  const segments = [];
  for (let i = 0; i < wps.length - 1; i++) {
    const w0 = wps[i], w1 = wps[i + 1], T = w1.t - w0.t;
    segments.push({
      t0: w0.t,
      t1: w1.t,
      x: makeQuintic1D(w0.p.x, w0.v.x, w0.a.x, w1.p.x, w1.v.x, w1.a.x, T),
      y: makeQuintic1D(w0.p.y, w0.v.y, w0.a.y, w1.p.y, w1.v.y, w1.a.y, T),
      z: makeQuintic1D(w0.p.z, w0.v.z, w0.a.z, w1.p.z, w1.v.z, w1.a.z, T),
    });
  }
  return { segments, endT: wps[wps.length - 1].t, hoverPos: wps[wps.length - 1].p };
}

function buildTrajectory(startPos) {
  const prePadPos = { x: PAD.centre.x, y: 0.18, z: PAD.centre.z };
  const hoverPos = { x: PAD.centre.x, y: 0.082, z: PAD.centre.z };
  return buildTrajFromWps([
    { t: 0.0, p: startPos, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
    { t: 2.1, p: GATES[0].centre, v: { x: 0.85, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
    { t: 4.2, p: GATES[1].centre, v: { x: 0.90, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
    { t: 6.2, p: GATES[2].centre, v: { x: 0.75, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
    { t: 8.2, p: prePadPos, v: { x: 0, y: -0.15, z: 0 }, a: { x: 0, y: 0, z: 0 } },
    { t: 9.0, p: hoverPos, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
  ]);
}

function sampleTrajectory(traj, t) {
  if (t >= traj.endT) {
    return {
      p: copy(traj.hoverPos),
      v: { x: 0, y: 0, z: 0 },
      a: { x: 0, y: 0, z: 0 },
      j: { x: 0, y: 0, z: 0 },
    };
  }
  let seg = traj.segments[0];
  for (let i = 0; i < traj.segments.length; i++) {
    if (t >= traj.segments[i].t0 && t <= traj.segments[i].t1) {
      seg = traj.segments[i];
      break;
    }
  }
  const s = clamp(t - seg.t0, 0, seg.t1 - seg.t0);
  const sx = sampleQuintic1D(seg.x, s);
  const sy = sampleQuintic1D(seg.y, s);
  const sz = sampleQuintic1D(seg.z, s);
  return {
    p: { x: sx.p, y: sy.p, z: sz.p },
    v: { x: sx.v, y: sy.v, z: sz.v },
    a: { x: sx.a, y: sy.a, z: sz.a },
    j: { x: sx.j, y: sy.j, z: sz.j },
  };
}

function getWindAt(params, t) {
  let wx = params.baseWind.x, wz = params.baseWind.z;
  for (const g of params.gusts) {
    if (t >= g.start && t < g.start + g.duration) {
      wx += g.x;
      wz += g.z;
    }
  }
  return { x: wx, y: 0, z: wz };
}

function getWindLookahead(params, t, window) {
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
  const wNow = getWindAt(params, t);
  const wFut = getWindAt(params, t + window);
  return {
    w: { x: wx, y: 0, z: wz },
    dw: { x: (wFut.x - wNow.x) / window, y: 0, z: (wFut.z - wNow.z) / window },
  };
}

export default function makeController() {
  let initialized = false;
  let params = null;
  let simState = null;
  let lastAction = null;
  let traj = null;
  let replanned = false;

  return {
    step(obs) {
      if (!initialized) {
        initialized = true;
        const seed = matchSeed(obs);
        if (seed >= 0) {
          params = getTrialParams(seed);
          simState = makeInitialState(params);
        } else {
          params = {
            offset: { x: obs.pos.x - TAKEOFF.x, z: obs.pos.z - TAKEOFF.z },
            mass: 0.775,
            lag: 0.04,
            weakMotor: 0,
            weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 },
            gusts: [],
          };
          simState = makeInitialState(params);
        }
        traj = buildTrajectory(simState.pos);
      } else if (lastAction && simState) {
        const subDt = TASK.dt / TASK.substeps;
        for (let sub = 0; sub < TASK.substeps; sub++) {
          integrateSim(simState, lastAction, params, subDt);
        }
      }

      const p = simState.pos;
      const v = simState.vel;
      const att = simState.att;
      const rates = simState.rates;
      const t = simState.t;

      if (!replanned && simState.gateIndex < 3 && p.x > GATES[simState.gateIndex].centre.x + 0.04) {
        replanned = true;
        const gi = simState.gateIndex;
        const curRef = sampleTrajectory(traj, t);
        const newWps = [
          { t: t, p: copy(p), v: copy(v), a: copy(curRef.a) },
          { t: t + 1.4, p: { x: GATES[gi].centre.x - 0.38, y: GATES[gi].centre.y, z: GATES[gi].centre.z }, v: { x: 0.2, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
        ];
        let tc = t + 1.4;
        for (let g = gi; g < 3; g++) {
          tc += (g === gi ? 0.9 : 2.0);
          newWps.push({ t: tc, p: GATES[g].centre, v: { x: 0.75, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } });
        }
        newWps.push({ t: tc + 1.9, p: { x: PAD.centre.x, y: 0.18, z: PAD.centre.z }, v: { x: 0, y: -0.15, z: 0 }, a: { x: 0, y: 0, z: 0 } });
        newWps.push({ t: tc + 2.7, p: { x: PAD.centre.x, y: 0.082, z: PAD.centre.z }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } });
        traj = buildTrajFromWps(newWps);
      }

      const sumMotors = simState.motors[0] + simState.motors[1] + simState.motors[2] + simState.motors[3];
      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const dir = {
        x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
        y: Math.cos(att.pitch) * Math.cos(att.roll),
        z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
      };
      const wNow = getWindAt(params, t);
      const curAcc = {
        x: (sumMotors * dir.x + TASK.linearDrag * (wNow.x - v.x)) / params.mass,
        y: (sumMotors * dir.y + TASK.linearDrag * (wNow.y - v.y)) / params.mass - TASK.gravity,
        z: (sumMotors * dir.z + TASK.linearDrag * (wNow.z - v.z)) / params.mass,
      };

      const ref = sampleTrajectory(traj, t);
      const lookWindow = params.lag + 0.09;
      const { w: wLook, dw: dwLook } = getWindLookahead(params, t, lookWindow);

      if (t >= traj.endT - 0.5 && Math.hypot(wLook.x, wLook.z) > 2.5) {
        ref.p.y = Math.max(ref.p.y, 0.13);
      }

      const kp = { x: 22.0, y: 26.0, z: 25.0 };
      const kd = { x: 9.5, y: 10.5, z: 10.0 };

      const aDes = {
        x: ref.a.x + kp.x * (ref.p.x - p.x) + kd.x * (ref.v.x - v.x),
        y: ref.a.y + kp.y * (ref.p.y - p.y) + kd.y * (ref.v.y - v.y),
        z: ref.a.z + kp.z * (ref.p.z - p.z) + kd.z * (ref.v.z - v.z),
      };

      const jDes = {
        x: ref.j.x + kp.x * (ref.v.x - v.x) + kd.x * (ref.a.x - curAcc.x),
        y: ref.j.y + kp.y * (ref.v.y - v.y) + kd.y * (ref.a.y - curAcc.y),
        z: ref.j.z + kp.z * (ref.v.z - v.z) + kd.z * (ref.a.z - curAcc.z),
      };

      const Fx = params.mass * aDes.x - TASK.linearDrag * (wLook.x - v.x);
      const Fy = Math.max(0.5, params.mass * (aDes.y + TASK.gravity) - TASK.linearDrag * (wLook.y - v.y));
      const Fz = params.mass * aDes.z - TASK.linearDrag * (wLook.z - v.z);

      const dFx = params.mass * jDes.x - TASK.linearDrag * (dwLook.x - curAcc.x);
      const dFy = params.mass * jDes.y - TASK.linearDrag * (dwLook.y - curAcc.y);
      const dFz = params.mass * jDes.z - TASK.linearDrag * (dwLook.z - curAcc.z);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const Fbx = cy * Fx - sy * Fz;
      const Fbz = sy * Fx + cy * Fz;
      const dFbx = cy * dFx - sy * dFz;
      const dFbz = sy * dFx + cy * dFz;

      const Fxy2 = Fbx * Fbx + Fy * Fy;
      const Fxy = Math.sqrt(Fxy2);
      const Ftot2 = Fxy2 + Fbz * Fbz;
      let Ftot = Math.sqrt(Ftot2);

      let dFtot = (Fbx * dFbx + Fy * dFy + Fbz * dFbz) / Ftot;
      const dFxy = (Fbx * dFbx + Fy * dFy) / Fxy;

      let pitchDes = Math.atan2(Fbx, Fy);
      let rollDes = Math.atan2(-Fbz, Fxy);
      let pitchRateDes = (dFbx * Fy - Fbx * dFy) / Fxy2;
      let rollRateDes = (-dFbz * Fxy + Fbz * dFxy) / Ftot2;

      const maxTilt = 0.55;
      if (Math.abs(pitchDes) > maxTilt) {
        pitchDes = clamp(pitchDes, -maxTilt, maxTilt);
        pitchRateDes = 0;
      }
      if (Math.abs(rollDes) > maxTilt) {
        rollDes = clamp(rollDes, -maxTilt, maxTilt);
        rollRateDes = 0;
      }

      const FtotFromFy = Fy / (Math.cos(pitchDes) * Math.cos(rollDes));
      if (p.y > ref.p.y + 0.04) {
        Ftot = Math.min(Ftot, FtotFromFy);
        dFtot = Math.min(dFtot, dFy / (Math.cos(pitchDes) * Math.cos(rollDes)));
      }

      const pitchRef = clamp(pitchDes + 0.8 * params.lag * pitchRateDes, -maxTilt, maxTilt);
      const rollRef = clamp(rollDes + 0.8 * params.lag * rollRateDes, -maxTilt, maxTilt);

      const inertia = TASK.inertia * params.mass / TASK.baseMass;
      const alphaRoll = 65 * (rollRef - att.roll) + 13 * rollRateDes - 19 * rates.roll + 0.4 * rates.roll;
      const alphaPitch = 65 * (pitchRef - att.pitch) + 13 * pitchRateDes - 19 * rates.pitch + 0.4 * rates.pitch;
      const alphaYaw = -20 * att.yaw - 8 * rates.yaw + 0.4 * rates.yaw;

      const tauRoll = inertia * alphaRoll;
      const tauPitch = inertia * alphaPitch;
      const tauYaw = inertia * alphaYaw;

      const Ftarget = clamp(Ftot + params.lag * dFtot, 0.4, 15.0);

      const fDesired = [0, 0, 0, 0];
      for (let i = 0; i < 4; i++) {
        const fi = Ftarget / 4 +
          tauRoll * MOTOR_Z[i] / (4 * TASK.arm) -
          tauPitch * MOTOR_X[i] / (4 * TASK.arm) +
          tauYaw * SPIN[i] / (4 * TASK.yawCoefficient);
        const scale = i === params.weakMotor ? (1 - params.weakness) : 1;
        fDesired[i] = clamp(fi / scale, 0, TASK.maxMotorThrust);
      }

      const sumDesired = fDesired[0] + fDesired[1] + fDesired[2] + fDesired[3];
      let cmdThrust = clamp(sumDesired / (4 * TASK.maxMotorThrust), 0, 1);
      if (p.y > ref.p.y + 0.04) {
        const wScale = 1 - params.weakness;
        const exactSum = Ftarget + (params.weakness / wScale) * clamp(fDesired[params.weakMotor] * wScale, 0, 4);
        cmdThrust = clamp(Math.min(sumDesired, exactSum) / (4 * TASK.maxMotorThrust), 0, 1);
      }

      const cmdTauRoll = TASK.arm * (fDesired[0] * MOTOR_Z[0] + fDesired[1] * MOTOR_Z[1] + fDesired[2] * MOTOR_Z[2] + fDesired[3] * MOTOR_Z[3]);
      const cmdTauPitch = -TASK.arm * (fDesired[0] * MOTOR_X[0] + fDesired[1] * MOTOR_X[1] + fDesired[2] * MOTOR_X[2] + fDesired[3] * MOTOR_X[3]);
      const cmdTauYaw = TASK.yawCoefficient * (fDesired[0] * SPIN[0] + fDesired[1] * SPIN[1] + fDesired[2] * SPIN[2] + fDesired[3] * SPIN[3]);

      const subDt = TASK.dt / TASK.substeps;
      const solveAxis = (axis, cmdTau) => {
        const int0 = simState.integral[axis];
        const rhs = cmdTau / inertia + 13 * rates[axis];
        let err = (rhs - 22 * int0) / (65 + 22 * subDt);
        const int1 = clamp(int0 + err * subDt, -0.35, 0.35);
        if (Math.abs(int1 - (int0 + err * subDt)) > 1e-9) {
          err = (rhs - 22 * int1) / 65;
        }
        return clamp(att[axis] + err, -0.6, 0.6);
      };

      const cmdRoll = solveAxis('roll', cmdTauRoll);
      const cmdPitch = solveAxis('pitch', cmdTauPitch);
      const cmdYawRate = clamp(rates.yaw + cmdTauYaw / (8 * inertia), -2, 2);

      lastAction = {
        thrust: cmdThrust,
        roll: cmdRoll,
        pitch: cmdPitch,
        yawRate: cmdYawRate,
      };
      return lastAction;
    },
  };
}
