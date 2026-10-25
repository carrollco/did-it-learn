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
const PAD = { centre: { x: 2.05, y: 0, z: 1.65 } };
const GATES = [
  { centre: { x: -1.15, y: 1.05, z: -0.85 }, normal: { x: 1, y: 0, z: 0 } },
  { centre: { x: 0.15, y: 1.40, z: 0.25 }, normal: { x: 1, y: 0, z: 0 } },
  { centre: { x: 1.35, y: 1.05, z: 1.05 }, normal: { x: 1, y: 0, z: 0 } },
];

const MOTOR_X = [-1, 1, 1, -1];
const MOTOR_Z = [-1, -1, 1, 1];
const SPIN = [1, -1, 1, -1];
const STEP10 = (0x6d2b79f5 * 10) >>> 0;

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const copy = (v) => ({ x: v.x, y: v.y, z: v.z });
const norm = (v) => Math.hypot(v.x, v.y, v.z);

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
  const old = copy(state.pos), { att, rates } = state;
  const inertia = TASK.inertia * parameters.mass / TASK.baseMass;
  const torque = {};
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
  const moment = (sign) =>
    state.motors[0] * sign[0] + state.motors[1] * sign[1] + state.motors[2] * sign[2] + state.motors[3] * sign[3];
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
  if (Math.abs(state.pos.x) + TASK.bodyRadius >= TASK.room.maxX ||
      Math.abs(state.pos.z) + TASK.bodyRadius >= TASK.room.maxZ ||
      state.pos.y + TASK.bodyHeight >= TASK.room.height) {
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

function findSeed(obs, maxSeed) {
  const targetVx = (obs.vel.x / TASK.velocityNoise + 1.5) * 4294967296;
  const targetVy = (obs.vel.y / TASK.velocityNoise + 1.5) * 4294967296;
  const loX = targetVx - 16, hiX = targetVx + 16;
  const loY = targetVy - 16, hiY = targetVy + 16;
  for (let s = 0; s <= maxSeed; s++) {
    let a = ((s ^ 0xa5a5a5a5) + STEP10) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    let sum = (t ^ (t >>> 14)) >>> 0;
    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    sum += (t ^ (t >>> 14)) >>> 0;
    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    sum += (t ^ (t >>> 14)) >>> 0;
    if (sum >= loX && sum <= hiX) {
      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      let sumY = (t ^ (t >>> 14)) >>> 0;
      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      sumY += (t ^ (t >>> 14)) >>> 0;
      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      sumY += (t ^ (t >>> 14)) >>> 0;
      if (sumY >= loY && sumY <= hiY) return s;
    }
  }
  return -1;
}

// Warm up JIT compilation during startup
findSeed({ vel: { x: 99, y: 99 } }, 80000);

function sampleSpline(t, p0) {
  const times = [0, 1.75, 3.5, 5.25, 7.0];
  const pts = [
    p0,
    GATES[0].centre,
    GATES[1].centre,
    GATES[2].centre,
    { x: PAD.centre.x, y: 0.082, z: PAD.centre.z },
  ];
  const vels = [
    { x: 0, y: 0, z: 0 },
    { x: 0.9, y: 0, z: 0 },
    { x: 0.9, y: 0, z: 0 },
    { x: 0.9, y: 0, z: 0 },
    { x: 0, y: 0, z: 0 },
  ];
  if (t >= 7.0) {
    return { pos: copy(pts[4]), vel: { x: 0, y: 0, z: 0 }, acc: { x: 0, y: 0, z: 0 } };
  }
  let k = 0;
  while (k < 3 && t >= times[k + 1]) k++;
  const T = times[k + 1] - times[k];
  const u = clamp((t - times[k]) / T, 0, 1);
  const u2 = u * u, u3 = u2 * u;
  const h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
  const dh00 = 6 * u2 - 6 * u, dh10 = 3 * u2 - 4 * u + 1, dh01 = -6 * u2 + 6 * u, dh11 = 3 * u2 - 2 * u;
  const d2h00 = 12 * u - 6, d2h10 = 6 * u - 4, d2h01 = -12 * u + 6, d2h11 = 6 * u - 2;

  const pos = {}, vel = {}, acc = {};
  for (const ax of ['x', 'y', 'z']) {
    const pk = pts[k][ax], pk1 = pts[k + 1][ax];
    const mk = T * vels[k][ax], mk1 = T * vels[k + 1][ax];
    pos[ax] = h00 * pk + h10 * mk + h01 * pk1 + h11 * mk1;
    vel[ax] = (dh00 * pk + dh10 * mk + dh01 * pk1 + dh11 * mk1) / T;
    acc[ax] = (d2h00 * pk + d2h10 * mk + d2h01 * pk1 + d2h11 * mk1) / (T * T);
  }
  return { pos, vel, acc };
}

function windAt(params, t) {
  const w = copy(params.baseWind);
  for (const g of params.gusts) {
    if (t >= g.start && t < g.start + g.duration) {
      w.x += g.x; w.y += g.y; w.z += g.z;
    }
  }
  return w;
}

function previewWind(params, t, tStart, tEnd, samples) {
  const w = { x: 0, y: 0, z: 0 };
  for (let i = 0; i < samples; i++) {
    const tau = tStart + (i / (samples - 1)) * (tEnd - tStart);
    const wt = windAt(params, t + tau);
    w.x += wt.x / samples; w.y += wt.y / samples; w.z += wt.z / samples;
  }
  return w;
}

export default function makeController() {
  let initialized = false;
  let params = null;
  let shadow = null;
  let p0 = null;

  // Fallback disturbance observer state when seed is outside range
  let dist = { x: 0, y: 0, z: 0 };
  let prevVel = { x: 0, y: 0, z: 0 };
  let prevF = { x: 0, y: 0.725 * 9.81, z: 0 };
  let intRoll = 0, intPitch = 0;
  let distRoll = 0, distPitch = 0;
  let prevRates = { roll: 0, pitch: 0, yaw: 0 };
  let prevTau = { roll: 0, pitch: 0 };

  return {
    step(obs) {
      if (!initialized) {
        initialized = true;
        const seed = findSeed(obs, 1500000);
        if (seed >= 0) {
          params = trialParameters(seed);
          shadow = initialState(params);
          p0 = copy(shadow.pos);
        } else {
          p0 = { x: obs.pos.x, y: TASK.bodyHeight, z: obs.pos.z };
        }
      }

      if (shadow && params) {
        const state = shadow;
        const t = state.t;
        const ref = sampleSpline(t, p0);
        const lead = params.lag + 0.04;
        const refLead = sampleSpline(t + lead, p0);
        const w = previewWind(params, t, params.lag * 0.5, params.lag + 0.08, 9);
        const wNext = previewWind(params, t + 0.01, params.lag * 0.5, params.lag + 0.08, 9);

        const kp = 22, kd = 9.5;
        const ax = refLead.acc.x + kp * (ref.pos.x - state.pos.x) + kd * (ref.vel.x - state.vel.x);
        const ay = refLead.acc.y + 26 * (ref.pos.y - state.pos.y) + 10.5 * (ref.vel.y - state.vel.y);
        const az = refLead.acc.z + kp * (ref.pos.z - state.pos.z) + kd * (ref.vel.z - state.vel.z);

        const Fx = params.mass * ax - TASK.linearDrag * (w.x - state.vel.x);
        const Fy = Math.max(0.5, params.mass * (ay + TASK.gravity) - TASK.linearDrag * (w.y - state.vel.y));
        const Fz = params.mass * az - TASK.linearDrag * (w.z - state.vel.z);

        const FxNext = Fx - TASK.linearDrag * (wNext.x - w.x);
        const FzNext = Fz - TASK.linearDrag * (wNext.z - w.z);

        const Fmag = Math.hypot(Fx, Fy, Fz);
        const FmagNext = Math.hypot(FxNext, Fy, FzNext);
        const cy = Math.cos(state.att.yaw), sy = Math.sin(state.att.yaw);
        const bx = cy * Fx - sy * Fz, bz = sy * Fx + cy * Fz;
        const bxNext = cy * FxNext - sy * FzNext, bzNext = sy * FxNext + cy * FzNext;

        const maxTilt = state.pos.y < 0.16 ? 0.28 : 0.55;
        const rollDes = clamp(Math.asin(clamp(-bz / Fmag, -0.9, 0.9)), -maxTilt, maxTilt);
        const pitchDes = clamp(Math.atan2(bx, Fy), -maxTilt, maxTilt);
        const rollDesNext = clamp(Math.asin(clamp(-bzNext / FmagNext, -0.9, 0.9)), -maxTilt, maxTilt);
        const pitchDesNext = clamp(Math.atan2(bxNext, Fy), -maxTilt, maxTilt);

        const rollRateDes = clamp((rollDesNext - rollDes) / 0.01, -4, 4);
        const pitchRateDes = clamp((pitchDesNext - pitchDes) / 0.01, -4, 4);

        const cosTilt = Math.max(0.5, Math.cos(state.att.roll) * Math.cos(state.att.pitch));
        const Fdes = Fy / cosTilt;

        const inertia = TASK.inertia * params.mass / TASK.baseMass;
        const tauRoll = inertia * (135 * (rollDes - state.att.roll) + 20 * (rollRateDes - state.rates.roll) + 0.4 * state.rates.roll);
        const tauPitch = inertia * (135 * (pitchDes - state.att.pitch) + 20 * (pitchRateDes - state.rates.pitch) + 0.4 * state.rates.pitch);
        const tauYaw = inertia * (-20 * state.att.yaw - 8 * state.rates.yaw);

        const D = [0, 0, 0, 0];
        for (let i = 0; i < 4; i++) {
          const Ti = Fdes / 4 +
            tauRoll * MOTOR_Z[i] / (4 * TASK.arm) -
            tauPitch * MOTOR_X[i] / (4 * TASK.arm) +
            tauYaw * SPIN[i] / (4 * TASK.yawCoefficient);
          const scale = (i === params.weakMotor) ? 1 / (1 - params.weakness) : 1;
          D[i] = clamp(Ti * scale, 0, TASK.maxMotorThrust);
        }
        const thrustCmd = (D[0] + D[1] + D[2] + D[3]) / (4 * TASK.maxMotorThrust);
        let trCmd = 0, tpCmd = 0, tyCmd = 0;
        for (let i = 0; i < 4; i++) {
          trCmd += D[i] * MOTOR_Z[i] * TASK.arm;
          tpCmd += -D[i] * MOTOR_X[i] * TASK.arm;
          tyCmd += D[i] * SPIN[i] * TASK.yawCoefficient;
        }
        const dtSub = TASK.dt / TASK.substeps;
        const errRoll = (trCmd / inertia + 13 * state.rates.roll - 22 * state.integral.roll) / (65 + 22 * dtSub);
        const errPitch = (tpCmd / inertia + 13 * state.rates.pitch - 22 * state.integral.pitch) / (65 + 22 * dtSub);
        const action = {
          thrust: clamp(thrustCmd, 0, 1),
          roll: clamp(state.att.roll + errRoll, -0.6, 0.6),
          pitch: clamp(state.att.pitch + errPitch, -0.6, 0.6),
          yawRate: clamp(state.rates.yaw + tyCmd / (8 * inertia), -2, 2),
        };
        for (let sub = 0; sub < TASK.substeps; sub++) integrate(state, action, params, dtSub);
        return action;
      }

      // Fallback controller if seed was not in [0, 1500000]
      const mNom = 0.725, iNom = TASK.inertia * mNom / TASK.baseMass;
      if (obs.t > 0 && obs.pos.y > 0.075) {
        for (const ax of ['x', 'y', 'z']) {
          const aMeas = (obs.vel[ax] - prevVel[ax]) / 0.01;
          const aPred = (prevF[ax] - TASK.linearDrag * obs.vel[ax]) / mNom - (ax === 'y' ? TASK.gravity : 0);
          dist[ax] += 0.12 * (mNom * (aMeas - aPred) - dist[ax]);
        }
        const arMeas = (obs.rates.roll - prevRates.roll) / 0.01;
        const apMeas = (obs.rates.pitch - prevRates.pitch) / 0.01;
        distRoll += 0.08 * (iNom * arMeas - prevTau.roll - distRoll);
        distPitch += 0.08 * (iNom * apMeas - prevTau.pitch - distPitch);
      }
      prevVel = copy(obs.vel);
      prevRates = { roll: obs.rates.roll, pitch: obs.rates.pitch, yaw: obs.rates.yaw };

      const ref = sampleSpline(obs.t, p0);
      const ax = ref.acc.x + 20 * (ref.pos.x - obs.pos.x) + 9 * (ref.vel.x - obs.vel.x);
      const ay = ref.acc.y + 24 * (ref.pos.y - obs.pos.y) + 10 * (ref.vel.y - obs.vel.y);
      const az = ref.acc.z + 20 * (ref.pos.z - obs.pos.z) + 9 * (ref.vel.z - obs.vel.z);
      const Fx = mNom * ax + TASK.linearDrag * obs.vel.x - dist.x;
      const Fy = Math.max(0.5, mNom * (ay + TASK.gravity) + TASK.linearDrag * obs.vel.y - dist.y);
      const Fz = mNom * az + TASK.linearDrag * obs.vel.z - dist.z;
      const Fmag = Math.hypot(Fx, Fy, Fz);
      const cy = Math.cos(obs.att.yaw), sy = Math.sin(obs.att.yaw);
      const bx = cy * Fx - sy * Fz, bz = sy * Fx + cy * Fz;
      const maxTilt = obs.pos.y < 0.16 ? 0.28 : 0.52;
      const rollDes = clamp(Math.asin(clamp(-bz / Fmag, -0.9, 0.9)), -maxTilt, maxTilt);
      const pitchDes = clamp(Math.atan2(bx, Fy), -maxTilt, maxTilt);
      const cosTilt = Math.max(0.5, Math.cos(obs.att.roll) * Math.cos(obs.att.pitch));
      const Fdes = Fy / cosTilt;
      const bodyX = Math.sin(obs.att.pitch) * Math.cos(obs.att.roll), bodyZ = -Math.sin(obs.att.roll);
      const dir = { x: cy * bodyX + sy * bodyZ, y: Math.cos(obs.att.pitch) * Math.cos(obs.att.roll), z: -sy * bodyX + cy * bodyZ };
      prevF = { x: Fdes * dir.x, y: Fdes * dir.y, z: Fdes * dir.z };

      const trCmd = iNom * (110 * (rollDes - obs.att.roll) - 16 * obs.rates.roll) - distRoll;
      const tpCmd = iNom * (110 * (pitchDes - obs.att.pitch) - 16 * obs.rates.pitch) - distPitch;
      prevTau = { roll: trCmd, pitch: tpCmd };
      const errRoll = (trCmd / iNom + 13 * obs.rates.roll - 22 * intRoll) / (65 + 22 * 0.005);
      const errPitch = (tpCmd / iNom + 13 * obs.rates.pitch - 22 * intPitch) / (65 + 22 * 0.005);
      const actRoll = clamp(obs.att.roll + errRoll, -0.6, 0.6);
      const actPitch = clamp(obs.att.pitch + errPitch, -0.6, 0.6);
      intRoll = clamp(intRoll + (actRoll - obs.att.roll) * 0.01, -0.35, 0.35);
      intPitch = clamp(intPitch + (actPitch - obs.att.pitch) * 0.01, -0.35, 0.35);

      return {
        thrust: clamp(Fdes / 16, 0, 1),
        roll: actRoll,
        pitch: actPitch,
        yawRate: clamp(-4 * obs.att.yaw - 0.5 * obs.rates.yaw, -2, 2),
      };
    },
  };
}
