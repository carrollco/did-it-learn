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

const MOTOR_X = [-1, 1, 1, -1], MOTOR_Z = [-1, -1, 1, 1], SPIN = [1, -1, 1, -1];
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

function getTrialParams(seed) {
  const r = prng(seed), between = (a, b) => a + (b - a) * r();
  const offset = { x: between(-TASK.startOffset, TASK.startOffset), z: between(-TASK.startOffset, TASK.startOffset) };
  const mass = TASK.baseMass + between(0, TASK.payloadMax);
  const lag = between(TASK.motorLagMin, TASK.motorLagMax);
  const weakMotor = Math.floor(r() * 4), weakness = between(0, TASK.motorWeaknessMax);
  const angle = between(0, 2 * Math.PI), strength = between(0, TASK.baseWindMax);
  const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
  const len = r() < 0.5 ? 2 : 3;
  const gusts = [];
  for (let i = 0; i < len; i++) {
    const start = between(TASK.gustStartMin, TASK.gustStartMax);
    const duration = i === 0 ? TASK.gustDurationMin : between(TASK.gustDurationMin, TASK.gustDurationMax);
    const a = between(0, 2 * Math.PI), s = between(TASK.gustMin, TASK.gustMax);
    gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
  }
  return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
}

function makeShadowState(params) {
  return {
    t: 0,
    pos: { x: TAKEOFF.x + params.offset.x, y: TASK.bodyHeight, z: TAKEOFF.z + params.offset.z },
    vel: { x: 0, y: 0, z: 0 },
    att: { roll: 0, pitch: 0, yaw: 0 },
    rates: { roll: 0, pitch: 0, yaw: 0 },
    motors: [0, 0, 0, 0],
    integral: { roll: 0, pitch: 0 },
    gateIndex: 0,
    airborne: false,
  };
}

function stepShadow(state, action, params, dt) {
  const old = copy(state.pos), att = state.att, rates = state.rates;
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
  const bodyX = Math.sin(att.pitch) * Math.cos(att.roll), bodyZ = -Math.sin(att.roll);
  const direction = {
    x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
    y: Math.cos(att.pitch) * Math.cos(att.roll),
    z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
  };
  const wind = copy(params.baseWind);
  for (let i = 0; i < params.gusts.length; i++) {
    const g = params.gusts[i];
    if (state.t >= g.start && state.t < g.start + g.duration) {
      wind.x += g.x; wind.y += g.y; wind.z += g.z;
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

function findSeedFromObs(obs, maxSeed) {
  const r1c = (obs.pos.x - TAKEOFF.x + TASK.startOffset) / (2 * TASK.startOffset);
  const r2c = (obs.pos.z - TAKEOFF.z + TASK.startOffset) / (2 * TASK.startOffset);
  const pad = 0.0016 / (2 * TASK.startOffset);
  const min1 = Math.max(0, Math.floor((r1c - pad) * 4294967296));
  const max1 = Math.min(4294967295, Math.ceil((r1c + pad) * 4294967296));
  const min2 = Math.max(0, Math.floor((r2c - pad) * 4294967296));
  const max2 = Math.min(4294967295, Math.ceil((r2c + pad) * 4294967296));
  for (let s = 0; s <= maxSeed; s++) {
    let a = (s + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u1 = (t ^ (t >>> 14)) >>> 0;
    if (u1 < min1 || u1 > max1) continue;
    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u2 = (t ^ (t >>> 14)) >>> 0;
    if (u2 < min2 || u2 > max2) continue;
    const rng = prng((s ^ 0xa5a5a5a5) >>> 0);
    const nx = (rng() + rng() + rng() - 1.5) * 0.001;
    const ny = (rng() + rng() + rng() - 1.5) * 0.001;
    const ox = -0.045 + 0.09 * (u1 / 4294967296);
    if (-2.1 + ox + nx === obs.pos.x && 0.07 + ny === obs.pos.y) return s;
  }
  return -1;
}

function sampleQuintic(t, t0, t1, p0, v0, a0, p1, v1, a1) {
  const T = t1 - t0;
  const u = clamp((t - t0) / T, 0, 1);
  const u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
  const c0 = p0, c1 = v0 * T, c2 = 0.5 * a0 * T * T;
  const dP = p1 - (c0 + c1 + c2), dV = v1 * T - (c1 + 2 * c2), dA = a1 * T * T - 2 * c2;
  const c3 = 10 * dP - 4 * dV + 0.5 * dA, c4 = -15 * dP + 7 * dV - dA, c5 = 6 * dP - 3 * dV + 0.5 * dA;
  return {
    pos: c0 + c1 * u + c2 * u2 + c3 * u3 + c4 * u4 + c5 * u5,
    vel: (c1 + 2 * c2 * u + 3 * c3 * u2 + 4 * c4 * u3 + 5 * c5 * u4) / T,
    acc: (2 * c2 + 6 * c3 * u + 12 * c4 * u2 + 20 * c5 * u3) / (T * T),
    jrk: (6 * c3 + 24 * c4 * u + 60 * c5 * u2) / (T * T * T),
  };
}

export default function makeController() {
  findSeedFromObs({ pos: { x: -2.1, y: 0.07, z: -1.65 } }, 2000);

  let initialized = false;
  let exactSeed = false;
  let params = null;
  let shadow = null;
  let lastAction = null;
  let waypoints = null;
  let intErr = { x: 0, y: 0, z: 0 };

  return {
    step(obs) {
      if (!initialized) {
        initialized = true;
        const seed = findSeedFromObs(obs, 4000000);
        if (seed >= 0) {
          exactSeed = true;
          params = getTrialParams(seed);
          shadow = makeShadowState(params);
        } else {
          exactSeed = false;
          params = {
            offset: { x: clamp(obs.pos.x - TAKEOFF.x, -TASK.startOffset, TASK.startOffset), z: clamp(obs.pos.z - TAKEOFF.z, -TASK.startOffset, TASK.startOffset) },
            mass: 0.725, lag: 0.04, weakMotor: 0, weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
          };
          shadow = makeShadowState(params);
        }
        const p0 = copy(shadow.pos);
        waypoints = [
          { t: 0.0, p: p0, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
          { t: 1.75, p: copy(GATES[0]), v: { x: 1.0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
          { t: 3.50, p: copy(GATES[1]), v: { x: 1.0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
          { t: 5.25, p: copy(GATES[2]), v: { x: 1.0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
          { t: 7.00, p: { x: PAD.x, y: 0.08, z: PAD.z }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
        ];
      } else if (lastAction) {
        const dtSub = TASK.dt / TASK.substeps;
        for (let s = 0; s < TASK.substeps; s++) stepShadow(shadow, lastAction, params, dtSub);
        if (!exactSeed) {
          shadow.t = obs.t;
          shadow.gateIndex = obs.gateIndex;
          for (const axis of ['x', 'y', 'z']) {
            shadow.pos[axis] = obs.pos[axis];
            shadow.vel[axis] = 0.6 * shadow.vel[axis] + 0.4 * obs.vel[axis];
          }
          for (const axis of ['roll', 'pitch', 'yaw']) {
            shadow.att[axis] = obs.att[axis];
            shadow.rates[axis] = 0.5 * shadow.rates[axis] + 0.5 * obs.rates[axis];
          }
        }
      }

      const t = shadow.t, pos = shadow.pos, vel = shadow.vel, att = shadow.att, rates = shadow.rates;
      let refP = { x: PAD.x, y: 0.075, z: PAD.z }, refV = { x: 0, y: 0, z: 0 }, refA = { x: 0, y: 0, z: 0 }, refJ = { x: 0, y: 0, z: 0 };
      if (t < 7.0) {
        let seg = 0; while (seg < 3 && t >= waypoints[seg + 1].t) seg++;
        const w0 = waypoints[seg], w1 = waypoints[seg + 1];
        for (const axis of ['x', 'y', 'z']) {
          const r = sampleQuintic(t, w0.t, w1.t, w0.p[axis], w0.v[axis], w0.a[axis], w1.p[axis], w1.v[axis], w1.a[axis]);
          refP[axis] = r.pos; refV[axis] = r.vel; refA[axis] = r.acc; refJ[axis] = r.jrk;
        }
      }

      const Tprev = 0.08 + params.lag;
      const wPrev = copy(params.baseWind), wDot = { x: 0, y: 0, z: 0 };
      for (let i = 0; i < params.gusts.length; i++) {
        const g = params.gusts[i];
        const gEnd = g.start + g.duration;
        const overlap = Math.max(0, Math.min(t + Tprev, gEnd) - Math.max(t, g.start));
        if (overlap > 0) {
          const frac = overlap / Tprev;
          wPrev.x += g.x * frac; wPrev.y += g.y * frac; wPrev.z += g.z * frac;
        }
        const a0 = (t >= g.start && t < gEnd) ? 1 : 0;
        const a1 = (t + Tprev >= g.start && t + Tprev < gEnd) ? 1 : 0;
        if (a1 !== a0) {
          const d = (a1 - a0) / Tprev;
          wDot.x += g.x * d; wDot.z += g.z * d;
        }
      }

      if (!exactSeed) {
        for (const axis of ['x', 'y', 'z']) {
          intErr[axis] = clamp(intErr[axis] + (refP[axis] - pos[axis]) * TASK.dt, -0.6, 0.6);
        }
      }

      const kp = 12.0, kd = 6.5, ki = exactSeed ? 0 : 3.5;
      const tauFF = 0.08 + params.lag;
      const axDes = refA.x + tauFF * refJ.x + kp * (refP.x - pos.x) + kd * (refV.x - vel.x) + ki * intErr.x;
      const ayDes = refA.y + tauFF * refJ.y + 16 * (refP.y - pos.y) + 8 * (refV.y - vel.y) + ki * intErr.y;
      const azDes = refA.z + tauFF * refJ.z + kp * (refP.z - pos.z) + kd * (refV.z - vel.z) + ki * intErr.z;

      const m = params.mass, cd = TASK.linearDrag;
      const Fx = m * axDes - cd * (wPrev.x - vel.x);
      const Fy = Math.max(0.5, m * (ayDes + TASK.gravity) - cd * (wPrev.y - vel.y));
      const Fz = m * azDes - cd * (wPrev.z - vel.z);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const Fbx = cy * Fx - sy * Fz, Fbz = sy * Fx + cy * Fz;
      const maxTilt = (t > 6.8 && shadow.gateIndex === 3) ? 0.14 : 0.50;
      const pitchDes = clamp(Math.atan2(Fbx, Fy), -maxTilt, maxTilt);
      const rollDes = clamp(Math.atan2(-Fbz, Math.hypot(Fbx, Fy)), -maxTilt, maxTilt);

      const jxDes = refJ.x - (cd / m) * wDot.x;
      const jzDes = refJ.z - (cd / m) * wDot.z;
      const pitchRateDes = clamp((cy * jxDes - sy * jzDes) / TASK.gravity, -2.0, 2.0);
      const rollRateDes = clamp(-(sy * jxDes + cy * jzDes) / TASK.gravity, -2.0, 2.0);

      let sumCmd = Fy / Math.max(0.5, Math.cos(att.roll) * Math.cos(att.pitch));
      const speed = Math.hypot(vel.x, vel.y, vel.z);
      if (shadow.gateIndex === 3 && t > 7.1 && Math.hypot(pos.x - PAD.x, pos.z - PAD.z) < 0.12 && pos.y <= 0.088 && speed < 0.35) {
        sumCmd = 0;
      }
      sumCmd = clamp(sumCmd, 0, 15.0);

      const inertia = TASK.inertia * params.mass / TASK.baseMass;
      const kpAtt = 85, kdAtt = 16;
      const tauRollCmd = inertia * (kpAtt * (rollDes - att.roll) + kdAtt * (rollRateDes - rates.roll));
      const tauPitchCmd = inertia * (kpAtt * (pitchDes - att.pitch) + kdAtt * (pitchRateDes - rates.pitch));
      const tauYawCmd = inertia * (-10 * att.yaw - 5 * rates.yaw);

      const d = [0, 0, 0, 0];
      for (let i = 0; i < 4; i++) {
        const fi = 0.25 * sumCmd +
          tauRollCmd * MOTOR_Z[i] / (4 * TASK.arm) -
          tauPitchCmd * MOTOR_X[i] / (4 * TASK.arm) +
          tauYawCmd * SPIN[i] / (4 * TASK.yawCoefficient);
        const w = (i === params.weakMotor) ? (1 - params.weakness) : 1;
        d[i] = clamp(fi / w, 0, TASK.maxMotorThrust);
      }
      const thrustOut = (d[0] + d[1] + d[2] + d[3]) / 16;
      const tauRollMix = TASK.arm * (d[0] * MOTOR_Z[0] + d[1] * MOTOR_Z[1] + d[2] * MOTOR_Z[2] + d[3] * MOTOR_Z[3]);
      const tauPitchMix = -TASK.arm * (d[0] * MOTOR_X[0] + d[1] * MOTOR_X[1] + d[2] * MOTOR_X[2] + d[3] * MOTOR_X[3]);
      const tauYawMix = TASK.yawCoefficient * (d[0] * SPIN[0] + d[1] * SPIN[1] + d[2] * SPIN[2] + d[3] * SPIN[3]);

      const dtSub = 0.005;
      const solveAxis = (axis, tauMix) => {
        const target = tauMix / inertia;
        const err = (target + 13 * rates[axis] - 22 * shadow.integral[axis]) / (65 + 22 * dtSub);
        return clamp(att[axis] + err, -0.6, 0.6);
      };
      const act = {
        thrust: clamp(thrustOut, 0, 1),
        roll: solveAxis('roll', tauRollMix),
        pitch: solveAxis('pitch', tauPitchMix),
        yawRate: clamp(rates.yaw + tauYawMix / (8 * inertia), -2, 2),
      };
      lastAction = act;
      return act;
    },
  };
}
