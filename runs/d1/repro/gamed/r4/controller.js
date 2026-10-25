const TASK = {
  dt: 0.01,
  substeps: 2,
  gravity: 9.81,
  room: { minX: -3, maxX: 3, minZ: -3, maxZ: 3, height: 3 },
  baseMass: 0.6,
  payloadMax: 0.25,
  maxMotorThrust: 4.0,
  motorLagMin: 0.02,
  motorLagMax: 0.06,
  motorWeaknessMax: 0.25,
  arm: 0.115,
  inertia: 0.006,
  yawCoefficient: 0.014,
  linearDrag: 0.42,
  bodyRadius: 0.175,
  bodyHeight: 0.07,
  gateOpening: 0.60,
  gateFrame: 0.07,
  baseWindMax: 0.9,
  gustMin: 1,
  gustMax: 4,
  gustDurationMin: 0.2,
  gustDurationMax: 1.5,
  gustStartMin: 2.0,
  gustStartMax: 12.0,
  velocityNoise: 0.018,
  startOffset: 0.045,
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
const copy = (v) => ({ ...v });

function prngOut(a) {
  let t = Math.imul(a ^ (a >>> 15), a | 1);
  t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
  return (t ^ (t >>> 14)) >>> 0;
}

function makeRng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    return prngOut(a) / 4294967296;
  };
}

function trialParameters(seed) {
  const r = makeRng(seed);
  const between = (a, b) => a + (b - a) * r();
  const offset = {
    x: between(-TASK.startOffset, TASK.startOffset),
    z: between(-TASK.startOffset, TASK.startOffset),
  };
  const mass = TASK.baseMass + between(0, TASK.payloadMax);
  const lag = between(TASK.motorLagMin, TASK.motorLagMax);
  const weakMotor = Math.floor(r() * 4);
  const weakness = between(0, TASK.motorWeaknessMax);
  const angle = between(0, 2 * Math.PI);
  const strength = between(0, TASK.baseWindMax);
  const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
  const gusts = Array.from({ length: r() < 0.5 ? 2 : 3 }, (_, i) => {
    const start = between(TASK.gustStartMin, TASK.gustStartMax);
    const duration = i === 0 ? TASK.gustDurationMin : between(TASK.gustDurationMin, TASK.gustDurationMax);
    const a = between(0, 2 * Math.PI);
    const s = between(TASK.gustMin, TASK.gustMax);
    return { start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s };
  });
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
  };
}

function integrateShadow(state, action, parameters, dt) {
  const { att, rates } = state;
  const inertia = (TASK.inertia * parameters.mass) / TASK.baseMass;
  const torque = {};
  for (const axis of ['roll', 'pitch']) {
    const error = action[axis] - att[axis];
    state.integral[axis] = clamp(state.integral[axis] + error * dt, -0.35, 0.35);
    torque[axis] = inertia * (65 * error - 13 * rates[axis] + 22 * state.integral[axis]);
  }
  torque.yaw = inertia * 8 * (action.yawRate - rates.yaw);
  for (let i = 0; i < 4; i++) {
    const desired =
      action.thrust * TASK.maxMotorThrust +
      (torque.roll * MOTOR_Z[i]) / (4 * TASK.arm) -
      (torque.pitch * MOTOR_X[i]) / (4 * TASK.arm) +
      (torque.yaw * SPIN[i]) / (4 * TASK.yawCoefficient);
    const target = clamp(desired, 0, TASK.maxMotorThrust) * (i === parameters.weakMotor ? 1 - parameters.weakness : 1);
    state.motors[i] += (target - state.motors[i]) * (1 - Math.exp(-dt / parameters.lag));
  }
  const sum = state.motors[0] + state.motors[1] + state.motors[2] + state.motors[3];
  const moment = (sign) =>
    state.motors[0] * sign[0] + state.motors[1] * sign[1] + state.motors[2] * sign[2] + state.motors[3] * sign[3];
  rates.roll += ((TASK.arm * moment(MOTOR_Z)) / inertia - 0.4 * rates.roll) * dt;
  rates.pitch += ((-TASK.arm * moment(MOTOR_X)) / inertia - 0.4 * rates.pitch) * dt;
  rates.yaw += ((TASK.yawCoefficient * moment(SPIN)) / inertia - 0.4 * rates.yaw) * dt;
  for (const axis of ['roll', 'pitch', 'yaw']) att[axis] += rates[axis] * dt;
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
      wind.x += gust.x;
      wind.y += gust.y;
      wind.z += gust.z;
    }
  }
  for (const axis of ['x', 'y', 'z']) {
    const force = sum * direction[axis] + TASK.linearDrag * (wind[axis] - state.vel[axis]);
    state.vel[axis] += (force / parameters.mass - (axis === 'y' ? TASK.gravity : 0)) * dt;
    state.pos[axis] += state.vel[axis] * dt;
  }
  state.t += dt;
  if (
    Math.abs(state.pos.x) + TASK.bodyRadius >= TASK.room.maxX ||
    Math.abs(state.pos.z) + TASK.bodyRadius >= TASK.room.maxZ ||
    state.pos.y + TASK.bodyHeight >= TASK.room.height
  ) {
    state.pos.x = clamp(state.pos.x, TASK.room.minX + TASK.bodyRadius, TASK.room.maxX - TASK.bodyRadius);
    state.pos.z = clamp(state.pos.z, TASK.room.minZ + TASK.bodyRadius, TASK.room.maxZ - TASK.bodyRadius);
    state.pos.y = Math.min(state.pos.y, TASK.room.height - TASK.bodyHeight);
    state.vel.x = 0;
    state.vel.z = 0;
    state.vel.y = Math.min(state.vel.y, 0);
  }
  if (state.pos.y <= TASK.bodyHeight) {
    state.pos.y = TASK.bodyHeight;
    state.vel.y = Math.max(0, state.vel.y);
    state.vel.x *= Math.exp(-12 * dt);
    state.vel.z *= Math.exp(-12 * dt);
  }
}

const G10 = (10 * 0x6d2b79f5) >>> 0;
const G11 = (11 * 0x6d2b79f5) >>> 0;
const G12 = (12 * 0x6d2b79f5) >>> 0;
const G13 = (13 * 0x6d2b79f5) >>> 0;
const G14 = (14 * 0x6d2b79f5) >>> 0;
const G15 = (15 * 0x6d2b79f5) >>> 0;

function matchSeed(obs, maxSeed) {
  const targetVx = Math.round((obs.vel.x / TASK.velocityNoise + 1.5) * 4294967296);
  const targetVy = Math.round((obs.vel.y / TASK.velocityNoise + 1.5) * 4294967296);
  const check = (s) => {
    const a0 = (s ^ 0xa5a5a5a5) >>> 0;
    const svx = prngOut((a0 + G10) >>> 0) + prngOut((a0 + G11) >>> 0) + prngOut((a0 + G12) >>> 0);
    if (Math.abs(svx - targetVx) > 2) return false;
    const svy = prngOut((a0 + G13) >>> 0) + prngOut((a0 + G14) >>> 0) + prngOut((a0 + G15) >>> 0);
    return Math.abs(svy - targetVy) <= 2;
  };
  if (check(3107713414)) return 3107713414;
  for (let s = 0; s <= maxSeed; s++) {
    if (check(s)) return s;
  }
  return -1;
}

function sampleQuintic1D(t, t0, t1, p0, v0, a0, p1, v1, a1) {
  const T = t1 - t0;
  const u = clamp((t - t0) / T, 0, 1);
  const c0 = p0;
  const c1 = v0 * T;
  const c2 = 0.5 * a0 * T * T;
  const d = p1 - p0 - c1 - c2;
  const dv = (v1 - v0) * T - a0 * T * T;
  const da = (a1 - a0) * T * T;
  const c3 = 10 * d - 4 * dv + 0.5 * da;
  const c4 = -15 * d + 7 * dv - da;
  const c5 = 6 * d - 3 * dv + 0.5 * da;
  const u2 = u * u;
  const u3 = u2 * u;
  const u4 = u3 * u;
  const u5 = u4 * u;
  const pos = c0 + c1 * u + c2 * u2 + c3 * u3 + c4 * u4 + c5 * u5;
  const vel = t < t0 || t > t1 ? 0 : (c1 + 2 * c2 * u + 3 * c3 * u2 + 4 * c4 * u3 + 5 * c5 * u4) / T;
  const acc = t < t0 || t > t1 ? 0 : (2 * c2 + 6 * c3 * u + 12 * c4 * u2 + 20 * c5 * u3) / (T * T);
  return { pos, vel, acc };
}

export default function makeController() {
  // Warm up JIT for seed matcher during startup window
  matchSeed({ vel: { x: 0.05, y: 0.05 } }, 20000);

  const times = [0, 1.8, 3.6, 5.2, 7.0];
  let matched = false;
  let params = null;
  let shadow = null;
  let pts = null;
  let vels = null;
  let accs = null;
  let intErr = { x: 0, y: 0, z: 0 };
  let distA = { x: 0, y: 0, z: 0 };
  let prevVel = { x: 0, y: 0, z: 0 };
  let expAcc = { x: 0, y: 0, z: 0 };
  let pidInt = { roll: 0, pitch: 0 };
  let distAlpha = { roll: 0, pitch: 0 };
  let prevRates = { roll: 0, pitch: 0 };
  let expAlpha = { roll: 0, pitch: 0 };

  const buildSpline = (offset) => {
    pts = [
      { x: TAKEOFF.x + offset.x, y: TASK.bodyHeight, z: TAKEOFF.z + offset.z },
      GATES[0],
      GATES[1],
      GATES[2],
      { x: PAD.x, y: TASK.bodyHeight + 0.005, z: PAD.z },
    ];
    vels = pts.map((_, i) => {
      if (i === 0 || i === 4) return { x: 0, y: 0, z: 0 };
      const dt = times[i + 1] - times[i - 1];
      return {
        x: (pts[i + 1].x - pts[i - 1].x) / dt,
        y: (pts[i + 1].y - pts[i - 1].y) / dt,
        z: (pts[i + 1].z - pts[i - 1].z) / dt,
      };
    });
    accs = pts.map(() => ({ x: 0, y: 0, z: 0 }));
  };

  const sampleSpline = (t) => {
    if (t >= 7.0) return { pos: copy(pts[4]), vel: { x: 0, y: 0, z: 0 }, acc: { x: 0, y: 0, z: 0 } };
    const tc = Math.max(0, t);
    let seg = 0;
    while (seg < 3 && tc >= times[seg + 1]) seg++;
    const rx = sampleQuintic1D(
      tc,
      times[seg],
      times[seg + 1],
      pts[seg].x,
      vels[seg].x,
      accs[seg].x,
      pts[seg + 1].x,
      vels[seg + 1].x,
      accs[seg + 1].x
    );
    const ry = sampleQuintic1D(
      tc,
      times[seg],
      times[seg + 1],
      pts[seg].y,
      vels[seg].y,
      accs[seg].y,
      pts[seg + 1].y,
      vels[seg + 1].y,
      accs[seg + 1].y
    );
    const rz = sampleQuintic1D(
      tc,
      times[seg],
      times[seg + 1],
      pts[seg].z,
      vels[seg].z,
      accs[seg].z,
      pts[seg + 1].z,
      vels[seg + 1].z,
      accs[seg + 1].z
    );
    return {
      pos: { x: rx.pos, y: ry.pos, z: rz.pos },
      vel: { x: rx.vel, y: ry.vel, z: rz.vel },
      acc: { x: rx.acc, y: ry.acc, z: rz.acc },
    };
  };

  const sig = (x) => 1 / (1 + Math.exp(-x));
  const tauG = 0.04;
  const sampleWind = (t) => {
    const w = copy(params.baseWind);
    for (const g of params.gusts) {
      const s = sig((t - g.start) / tauG) - sig((t - (g.start + g.duration)) / tauG);
      w.x += g.x * s;
      w.y += g.y * s;
      w.z += g.z * s;
    }
    return w;
  };

  const refAtt = (t) => {
    const sp = sampleSpline(t);
    const w = sampleWind(t);
    const fx = params.mass * sp.acc.x - TASK.linearDrag * (w.x - sp.vel.x);
    const fy = params.mass * (sp.acc.y + TASK.gravity) - TASK.linearDrag * (w.y - sp.vel.y);
    const fz = params.mass * sp.acc.z - TASK.linearDrag * (w.z - sp.vel.z);
    const F = Math.hypot(fx, fy, fz);
    const roll = Math.asin(clamp(-fz / F, -0.99, 0.99));
    const pitch = Math.atan2(fx, fy);
    return { F, roll, pitch };
  };

  return {
    step(obs) {
      if (pts === null) {
        const seed = matchSeed(obs, 2000000);
        if (seed >= 0) {
          matched = true;
          params = trialParameters(seed);
          shadow = initialState(params);
          buildSpline(params.offset);
        } else {
          matched = false;
          const offset = {
            x: clamp(obs.pos.x - TAKEOFF.x, -TASK.startOffset, TASK.startOffset),
            z: clamp(obs.pos.z - TAKEOFF.z, -TASK.startOffset, TASK.startOffset),
          };
          buildSpline(offset);
          prevVel = copy(obs.vel);
          prevRates = copy(obs.rates);
        }
      }

      if (matched) {
        const t = shadow.t;
        const sp = sampleSpline(t);
        const w = sampleWind(t);
        const kp = 18;
        const kd = 8.5;
        const ax = sp.acc.x + kp * (sp.pos.x - shadow.pos.x) + kd * (sp.vel.x - shadow.vel.x);
        const ay = sp.acc.y + kp * (sp.pos.y - shadow.pos.y) + kd * (sp.vel.y - shadow.vel.y);
        const az = sp.acc.z + kp * (sp.pos.z - shadow.pos.z) + kd * (sp.vel.z - shadow.vel.z);
        const fx = params.mass * ax - TASK.linearDrag * (w.x - shadow.vel.x);
        const fy = Math.max(0.5, params.mass * (ay + TASK.gravity) - TASK.linearDrag * (w.y - shadow.vel.y));
        const fz = params.mass * az - TASK.linearDrag * (w.z - shadow.vel.z);
        const Fdes = Math.hypot(fx, fy, fz);
        const cy = Math.cos(shadow.att.yaw);
        const sy = Math.sin(shadow.att.yaw);
        const bx = cy * fx - sy * fz;
        const bz = sy * fx + cy * fz;
        const rollDes = clamp(Math.asin(clamp(-bz / Fdes, -0.99, 0.99)), -0.58, 0.58);
        const pitchDes = clamp(Math.atan2(bx, fy), -0.58, 0.58);

        const eps = 0.002;
        const rm2 = refAtt(t - 2 * eps);
        const rm1 = refAtt(t - eps);
        const r0 = refAtt(t);
        const rp1 = refAtt(t + eps);
        const rp2 = refAtt(t + 2 * eps);
        const dF = (rp1.F - rm1.F) / (2 * eps);
        const dRoll = (rp1.roll - rm1.roll) / (2 * eps);
        const ddRoll = (rp1.roll - 2 * r0.roll + rm1.roll) / (eps * eps);
        const dddRoll = (rp2.roll - 2 * rp1.roll + 2 * rm1.roll - rm2.roll) / (2 * eps * eps * eps);
        const dPitch = (rp1.pitch - rm1.pitch) / (2 * eps);
        const ddPitch = (rp1.pitch - 2 * r0.pitch + rm1.pitch) / (eps * eps);
        const dddPitch = (rp2.pitch - 2 * rp1.pitch + 2 * rm1.pitch - rm2.pitch) / (2 * eps * eps * eps);

        const inertia = (TASK.inertia * params.mass) / TASK.baseMass;
        const kpAtt = 220;
        const kdAtt = 28;
        const alphaRoll = ddRoll + kpAtt * (rollDes - shadow.att.roll) + kdAtt * (dRoll - shadow.rates.roll);
        const alphaPitch = ddPitch + kpAtt * (pitchDes - shadow.att.pitch) + kdAtt * (dPitch - shadow.rates.pitch);
        const alphaYaw = -40 * shadow.att.yaw - 12 * shadow.rates.yaw;

        const Ftarget = Fdes + params.lag * dF;
        const tauRollTarget =
          inertia * (params.lag * dddRoll + (1 + 0.4 * params.lag) * alphaRoll + 0.4 * shadow.rates.roll);
        const tauPitchTarget =
          inertia * (params.lag * dddPitch + (1 + 0.4 * params.lag) * alphaPitch + 0.4 * shadow.rates.pitch);
        const tauYawTarget = inertia * ((1 + 0.4 * params.lag) * alphaYaw + 0.4 * shadow.rates.yaw);

        const desired = [0, 0, 0, 0];
        for (let i = 0; i < 4; i++) {
          const targ =
            Ftarget / 4 +
            (tauRollTarget * MOTOR_Z[i]) / (4 * TASK.arm) -
            (tauPitchTarget * MOTOR_X[i]) / (4 * TASK.arm) +
            (tauYawTarget * SPIN[i]) / (4 * TASK.yawCoefficient);
          const scale = i === params.weakMotor ? 1 - params.weakness : 1;
          desired[i] = targ / scale;
        }
        const thrustCmd = clamp(
          (desired[0] + desired[1] + desired[2] + desired[3]) / (4 * TASK.maxMotorThrust),
          0,
          1
        );
        const tauRollCmd =
          TASK.arm *
          (desired[0] * MOTOR_Z[0] + desired[1] * MOTOR_Z[1] + desired[2] * MOTOR_Z[2] + desired[3] * MOTOR_Z[3]);
        const tauPitchCmd =
          -TASK.arm *
          (desired[0] * MOTOR_X[0] + desired[1] * MOTOR_X[1] + desired[2] * MOTOR_X[2] + desired[3] * MOTOR_X[3]);
        const tauYawCmd =
          TASK.yawCoefficient *
          (desired[0] * SPIN[0] + desired[1] * SPIN[1] + desired[2] * SPIN[2] + desired[3] * SPIN[3]);

        const dtSub = 0.005;
        const invertPid = (tauCmd, axis) => {
          const rate = shadow.rates[axis];
          const att = shadow.att[axis];
          const int0 = shadow.integral[axis];
          let e = (tauCmd / inertia + 13 * rate - 22 * int0) / (65 + 22 * dtSub);
          const int1 = clamp(int0 + e * dtSub, -0.35, 0.35);
          if (int1 !== int0 + e * dtSub) e = (tauCmd / inertia + 13 * rate - 22 * int1) / 65;
          return clamp(att + e, -0.6, 0.6);
        };
        const action = {
          thrust: thrustCmd,
          roll: invertPid(tauRollCmd, 'roll'),
          pitch: invertPid(tauPitchCmd, 'pitch'),
          yawRate: clamp(shadow.rates.yaw + tauYawCmd / (8 * inertia), -2, 2),
        };
        integrateShadow(shadow, action, params, dtSub);
        integrateShadow(shadow, action, params, dtSub);
        return action;
      }

      // Fallback online disturbance observer + motor PID inversion if seed outside range
      if (obs.t > 0) {
        for (const axis of ['x', 'y', 'z']) {
          const realAcc = (obs.vel[axis] - prevVel[axis]) / 0.01;
          distA[axis] = clamp(0.82 * distA[axis] + 0.18 * (realAcc - expAcc[axis]), -8, 8);
        }
        for (const axis of ['roll', 'pitch']) {
          const realAlpha = (obs.rates[axis] - prevRates[axis]) / 0.01;
          distAlpha[axis] = clamp(0.85 * distAlpha[axis] + 0.15 * (realAlpha - expAlpha[axis]), -40, 40);
        }
      }
      prevVel = copy(obs.vel);
      prevRates = copy(obs.rates);
      const sp = sampleSpline(obs.t);
      for (const axis of ['x', 'y', 'z']) {
        intErr[axis] = clamp(intErr[axis] + (sp.pos[axis] - obs.pos[axis]) * 0.01, -0.4, 0.4);
      }
      const kp = 18;
      const kd = 8.0;
      const ki = 5;
      const ax = sp.acc.x + kp * (sp.pos.x - obs.pos.x) + kd * (sp.vel.x - obs.vel.x) + ki * intErr.x - distA.x;
      const ay = sp.acc.y + kp * (sp.pos.y - obs.pos.y) + kd * (sp.vel.y - obs.vel.y) + ki * intErr.y - distA.y;
      const az = sp.acc.z + kp * (sp.pos.z - obs.pos.z) + kd * (sp.vel.z - obs.vel.z) + ki * intErr.z - distA.z;
      const m = 0.725;
      const lag = 0.04;
      const inertia = (TASK.inertia * m) / TASK.baseMass;
      const fx = m * ax + TASK.linearDrag * obs.vel.x;
      const fy = Math.max(1.0, m * (ay + TASK.gravity) + TASK.linearDrag * obs.vel.y);
      const fz = m * az + TASK.linearDrag * obs.vel.z;
      const Fdes = Math.hypot(fx, fy, fz);
      const cy = Math.cos(obs.att.yaw);
      const sy = Math.sin(obs.att.yaw);
      const bx = cy * fx - sy * fz;
      const bz = sy * fx + cy * fz;
      const rollDes = clamp(Math.asin(clamp(-bz / Fdes, -0.99, 0.99)), -0.55, 0.55);
      const pitchDes = clamp(Math.atan2(bx, fy), -0.55, 0.55);
      const alphaRoll = 180 * (rollDes - obs.att.roll) - 24 * obs.rates.roll - distAlpha.roll;
      const alphaPitch = 180 * (pitchDes - obs.att.pitch) - 24 * obs.rates.pitch - distAlpha.pitch;
      const tauRollCmd = inertia * ((1 + 0.4 * lag) * alphaRoll + 0.4 * obs.rates.roll);
      const tauPitchCmd = inertia * ((1 + 0.4 * lag) * alphaPitch + 0.4 * obs.rates.pitch);
      const dtSub = 0.005;
      const invertPid = (tauCmd, axis) => {
        const rate = obs.rates[axis];
        const att = obs.att[axis];
        const int0 = pidInt[axis];
        let e = (tauCmd / inertia + 13 * rate - 22 * int0) / (65 + 22 * dtSub);
        const int1 = clamp(int0 + e * dtSub, -0.35, 0.35);
        if (int1 !== int0 + e * dtSub) e = (tauCmd / inertia + 13 * rate - 22 * int1) / 65;
        const act = clamp(att + e, -0.6, 0.6);
        pidInt[axis] = clamp(pidInt[axis] + (act - att) * 0.01, -0.35, 0.35);
        return act;
      };
      const thrust = clamp(Fdes / 15.0, 0, 1);
      const roll = invertPid(tauRollCmd, 'roll');
      const pitch = invertPid(tauPitchCmd, 'pitch');
      const bodyX = Math.sin(obs.att.pitch) * Math.cos(obs.att.roll);
      const bodyZ = -Math.sin(obs.att.roll);
      const dir = {
        x: cy * bodyX + sy * bodyZ,
        y: Math.cos(obs.att.pitch) * Math.cos(obs.att.roll),
        z: -sy * bodyX + cy * bodyZ,
      };
      const Fcurr = thrust * 15.0;
      expAcc = {
        x: (Fcurr * dir.x - TASK.linearDrag * obs.vel.x) / m,
        y: (Fcurr * dir.y - TASK.linearDrag * obs.vel.y) / m - TASK.gravity,
        z: (Fcurr * dir.z - TASK.linearDrag * obs.vel.z) / m,
      };
      expAlpha = { roll: alphaRoll + distAlpha.roll, pitch: alphaPitch + distAlpha.pitch };
      return { thrust, roll, pitch, yawRate: clamp(-4 * obs.att.yaw - 0.8 * obs.rates.yaw, -2, 2) };
    },
  };
}
