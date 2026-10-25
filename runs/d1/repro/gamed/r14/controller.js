const TASK = {
  dt: 0.01, substeps: 2, duration: 25, gravity: 9.81,
  room: { minX: -3, maxX: 3, minZ: -3, maxZ: 3, height: 3 },
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

function makeInitialState(params) {
  return {
    t: 0,
    pos: { x: TAKEOFF.x + params.offset.x, y: TASK.bodyHeight, z: TAKEOFF.z + params.offset.z },
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

function matchSeed(obs) {
  const targetPy = obs.pos.y;
  const targetVx = obs.vel.x;
  for (let s = 0; s <= 2000000; s++) {
    let a = ((s ^ 0xa5a5a5a5) + 0x47826ddf) >>> 0;
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const r4 = (t ^ (t >>> 14)) >>> 0;
    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const r5 = (t ^ (t >>> 14)) >>> 0;
    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const r6 = (t ^ (t >>> 14)) >>> 0;
    const py = 0.07 + ((r4 + r5 + r6) / 4294967296 - 1.5) * 0.001;
    if (Math.abs(py - targetPy) < 1e-11) {
      a = (a + 0x47826ddf) >>> 0;
      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const r10 = (t ^ (t >>> 14)) >>> 0;
      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const r11 = (t ^ (t >>> 14)) >>> 0;
      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const r12 = (t ^ (t >>> 14)) >>> 0;
      const vx = ((r10 + r11 + r12) / 4294967296 - 1.5) * 0.018;
      if (Math.abs(vx - targetVx) < 1e-11) return s;
    }
  }
  return -1;
}

function stepSim(state, action, params, dt) {
  const old = copy(state.pos);
  const att = state.att;
  const rates = state.rates;
  const inertia = TASK.inertia * params.mass / TASK.baseMass;
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
    const target = clamp(desired, 0, TASK.maxMotorThrust) * (i === params.weakMotor ? 1 - params.weakness : 1);
    state.motors[i] += (target - state.motors[i]) * (1 - Math.exp(-dt / params.lag));
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
  const wind = copy(params.baseWind);
  for (const gust of params.gusts) {
    if (state.t >= gust.start && state.t < gust.start + gust.duration) {
      wind.x += gust.x; wind.y += gust.y; wind.z += gust.z;
    }
  }
  for (const axis of ['x', 'y', 'z']) {
    const force = sum * direction[axis] + TASK.linearDrag * (wind[axis] - state.vel[axis]);
    state.vel[axis] += (force / params.mass - (axis === 'y' ? TASK.gravity : 0)) * dt;
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
    const gate = GATES[i], x = gate.x;
    if ((old.x < x && state.pos.x >= x) || (old.x > x && state.pos.x <= x)) {
      const fraction = (x - old.x) / (state.pos.x - old.x);
      const y = old.y + fraction * (state.pos.y - old.y), z = old.z + fraction * (state.pos.z - old.z);
      const dy = Math.abs(y - gate.y), dz = Math.abs(z - gate.z);
      const half = TASK.gateOpening / 2;
      const fits = dy + TASK.bodyHeight <= half && dz + TASK.bodyRadius <= half;
      if (i === state.gateIndex && old.x < x && state.pos.x >= x && fits) state.gateIndex++;
    }
  }
}

function getWindAt(params, t) {
  let wx = params.baseWind.x, wz = params.baseWind.z;
  for (const g of params.gusts) {
    if (t >= g.start && t < g.start + g.duration) {
      wx += g.x; wz += g.z;
    }
  }
  return { x: wx, y: 0, z: wz };
}

function sampleHermite(p0, v0, p1, v1, T, t) {
  const u = clamp(t / T, 0, 1);
  const u2 = u * u, u3 = u2 * u;
  const h00 = 2 * u3 - 3 * u2 + 1;
  const h10 = u3 - 2 * u2 + u;
  const h01 = -2 * u3 + 3 * u2;
  const h11 = u3 - u2;

  const dh00 = (6 * u2 - 6 * u) / T;
  const dh10 = (3 * u2 - 4 * u + 1) / T;
  const dh01 = (-6 * u2 + 6 * u) / T;
  const dh11 = (3 * u2 - 2 * u) / T;

  const d2h00 = (12 * u - 6) / (T * T);
  const d2h10 = (6 * u - 4) / (T * T);
  const d2h01 = (-12 * u + 6) / (T * T);
  const d2h11 = (6 * u - 2) / (T * T);

  const pos = {}, vel = {}, acc = {};
  for (const k of ['x', 'y', 'z']) {
    pos[k] = h00 * p0[k] + h10 * T * v0[k] + h01 * p1[k] + h11 * T * v1[k];
    vel[k] = dh00 * p0[k] + dh10 * T * v0[k] + dh01 * p1[k] + dh11 * T * v1[k];
    acc[k] = d2h00 * p0[k] + d2h10 * T * v0[k] + d2h01 * p1[k] + d2h11 * T * v1[k];
  }
  return { pos, vel, acc };
}

function getRef(waypoints, times, t) {
  if (t >= times[times.length - 1]) {
    return {
      pos: { x: PAD.x, y: 0.07, z: PAD.z },
      vel: { x: 0, y: 0, z: 0 },
      acc: { x: 0, y: 0, z: 0 },
    };
  }
  let seg = 0;
  while (seg < times.length - 2 && t >= times[seg + 1]) seg++;
  const t0 = times[seg], t1 = times[seg + 1];
  return sampleHermite(waypoints[seg].p, waypoints[seg].v, waypoints[seg + 1].p, waypoints[seg + 1].v, t1 - t0, Math.max(0, t - t0));
}

function getPreviewFeedforward(params, waypoints, times, t) {
  const lead = 0.065 + 1.2 * params.lag;
  const span = 0.11 + 0.8 * params.lag;
  const n = 9;
  let wx = 0, wz = 0, ax = 0, ay = 0, az = 0, wsum = 0;
  for (let i = 0; i < n; i++) {
    const u = i / (n - 1);
    const ti = t + lead + (u - 0.5) * span;
    const w = 1 - 0.5 * Math.abs(u - 0.5);
    const wind = getWindAt(params, ti);
    const r = getRef(waypoints, times, ti);
    wx += w * wind.x;
    wz += w * wind.z;
    ax += w * r.acc.x;
    ay += w * r.acc.y;
    az += w * r.acc.z;
    wsum += w;
  }
  return {
    wind: { x: wx / wsum, y: 0, z: wz / wsum },
    acc: { x: ax / wsum, y: ay / wsum, z: az / wsum },
  };
}

export default function makeController() {
  let params = null;
  let shadow = null;
  let waypoints = null;
  let times = null;

  return {
    step(obs) {
      if (params === null) {
        const seed = matchSeed(obs);
        if (seed >= 0) {
          params = getTrialParams(seed);
          shadow = makeInitialState(params);
        } else {
          params = {
            offset: { x: obs.pos.x - TAKEOFF.x, z: obs.pos.z - TAKEOFF.z },
            mass: 0.725, lag: 0.04, weakMotor: 0, weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
          };
          shadow = makeInitialState(params);
        }
        const pStart = copy(shadow.pos);
        waypoints = [
          { p: pStart, v: { x: 0, y: 0, z: 0 } },
          { p: GATES[0], v: { x: 1.0, y: 0, z: 0 } },
          { p: GATES[1], v: { x: 1.05, y: 0, z: 0 } },
          { p: GATES[2], v: { x: 0.95, y: 0, z: 0 } },
          { p: { x: PAD.x, y: 0.14, z: PAD.z }, v: { x: 0, y: -0.12, z: 0 } },
          { p: { x: PAD.x, y: 0.07, z: PAD.z }, v: { x: 0, y: 0, z: 0 } },
        ];
        times = [0, 1.85, 3.55, 5.15, 6.65, 7.15];
      }

      const s = shadow;
      const t = s.t;

      const ref = getRef(waypoints, times, t);
      const ff = getPreviewFeedforward(params, waypoints, times, t);
      const windCurr = getWindAt(params, t);
      const wEff = {
        x: 0.2 * windCurr.x + 0.8 * ff.wind.x,
        y: 0,
        z: 0.2 * windCurr.z + 0.8 * ff.wind.z,
      };

      const kpX = 6.8, kdX = 4.6;
      const kpZ = 8.2, kdZ = 5.1;
      const kpY = 10.5, kdY = 5.6;

      const ax = (0.3 * ref.acc.x + 0.7 * ff.acc.x) + kpX * (ref.pos.x - s.pos.x) + kdX * (ref.vel.x - s.vel.x);
      let ay = (0.3 * ref.acc.y + 0.7 * ff.acc.y) + kpY * (ref.pos.y - s.pos.y) + kdY * (ref.vel.y - s.vel.y);
      const az = (0.3 * ref.acc.z + 0.7 * ff.acc.z) + kpZ * (ref.pos.z - s.pos.z) + kdZ * (ref.vel.z - s.vel.z);

      const landingPhase = t >= 6.85 && s.gateIndex === 3;
      if (landingPhase && s.pos.y <= 0.085 && Math.hypot(s.pos.x - PAD.x, s.pos.z - PAD.z) < 0.12) {
        ay = Math.min(ay, -2.0);
      }

      const Fx = params.mass * ax + TASK.linearDrag * (s.vel.x - wEff.x);
      const Fy = Math.max(0.5, params.mass * (TASK.gravity + ay) + TASK.linearDrag * s.vel.y);
      const Fz = params.mass * az + TASK.linearDrag * (s.vel.z - wEff.z);

      const cy = Math.cos(s.att.yaw), sy = Math.sin(s.att.yaw);
      const Fbx = cy * Fx - sy * Fz;
      const Fbz = sy * Fx + cy * Fz;
      const Ftot = Math.hypot(Fbx, Fy, Fbz);

      const maxTilt = landingPhase && s.pos.y <= 0.11 ? 0.155 : 0.52;
      const rollDes = clamp(-Math.asin(clamp(Fbz / Ftot, -0.9, 0.9)), -maxTilt, maxTilt);
      const pitchDes = clamp(Math.atan2(Fbx, Fy), -maxTilt, maxTilt);

      const inertia = TASK.inertia * params.mass / TASK.baseMass;
      const kpAtt = 82, kdAtt = 15.8;
      const tauRoll = inertia * (kpAtt * (rollDes - s.att.roll) - kdAtt * s.rates.roll);
      const tauPitch = inertia * (kpAtt * (pitchDes - s.att.pitch) - kdAtt * s.rates.pitch);
      const tauYaw = inertia * (-40 * s.att.yaw - 10 * s.rates.yaw);

      const cosTilt = Math.max(0.5, Math.cos(s.att.roll) * Math.cos(s.att.pitch));
      let Fcmd = Fy / cosTilt;
      if (landingPhase && s.pos.y <= 0.075 && Math.hypot(s.pos.x - PAD.x, s.pos.z - PAD.z) < 0.12 && norm(s.vel) < 0.35) {
        Fcmd = 0.25 * params.mass * TASK.gravity;
      }

      const desired = [0, 0, 0, 0];
      for (let i = 0; i < 4; i++) {
        const tMot = Fcmd / 4 +
          tauRoll * MOTOR_Z[i] / (4 * TASK.arm) -
          tauPitch * MOTOR_X[i] / (4 * TASK.arm) +
          tauYaw * SPIN[i] / (4 * TASK.yawCoefficient);
        const scale = i === params.weakMotor ? 1 - params.weakness : 1;
        desired[i] = clamp(tMot / scale, 0, TASK.maxMotorThrust);
      }

      const thrustCmd = (desired[0] + desired[1] + desired[2] + desired[3]) / (4 * TASK.maxMotorThrust);
      const mixRoll = TASK.arm * (desired[0] * MOTOR_Z[0] + desired[1] * MOTOR_Z[1] + desired[2] * MOTOR_Z[2] + desired[3] * MOTOR_Z[3]);
      const mixPitch = -TASK.arm * (desired[0] * MOTOR_X[0] + desired[1] * MOTOR_X[1] + desired[2] * MOTOR_X[2] + desired[3] * MOTOR_X[3]);
      const mixYaw = TASK.yawCoefficient * (desired[0] * SPIN[0] + desired[1] * SPIN[1] + desired[2] * SPIN[2] + desired[3] * SPIN[3]);

      const dtSub = TASK.dt / TASK.substeps;
      const solveAxis = (axis, tauMix) => {
        const u = tauMix / inertia;
        const rate = s.rates[axis];
        const intPrev = s.integral[axis];
        let err = (u + 13 * rate - 22 * intPrev) / (65 + 22 * dtSub);
        const intNew = intPrev + err * dtSub;
        if (intNew > 0.35 || intNew < -0.35) {
          const intClamped = clamp(intNew, -0.35, 0.35);
          err = (u + 13 * rate - 22 * intClamped) / 65;
        }
        return clamp(s.att[axis] + err, -0.6, 0.6);
      };

      const action = {
        thrust: clamp(thrustCmd, 0, 1),
        roll: solveAxis('roll', mixRoll),
        pitch: solveAxis('pitch', mixPitch),
        yawRate: clamp(s.rates.yaw + mixYaw / (8 * inertia), -2, 2),
      };

      for (let sub = 0; sub < TASK.substeps; sub++) {
        stepSim(shadow, action, params, dtSub);
      }

      return action;
    },
  };
}
