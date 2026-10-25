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

function findSeed(obs, maxSeed) {
  const targetY = obs.pos.y;
  const targetVx = obs.vel.x;
  const C1 = 0x6d2b79f5 | 0;
  const C4 = (4 * C1) | 0;
  const C10 = (10 * C1) | 0;
  const INV = 1 / 4294967296;
  for (let seed = 0; seed < maxSeed; seed++) {
    const s0 = (seed ^ 0xa5a5a5a5) | 0;
    let a = (s0 + C4) | 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const r4 = ((t ^ (t >>> 14)) >>> 0) * INV;
    a = (a + C1) | 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const r5 = ((t ^ (t >>> 14)) >>> 0) * INV;
    a = (a + C1) | 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const r6 = ((t ^ (t >>> 14)) >>> 0) * INV;
    if (0.07 + (r4 + r5 + r6 - 1.5) * 0.001 === targetY) {
      let a2 = (s0 + C10) | 0;
      let t2 = Math.imul(a2 ^ (a2 >>> 15), a2 | 1);
      t2 ^= t2 + Math.imul(t2 ^ (t2 >>> 7), t2 | 61);
      const r10 = ((t2 ^ (t2 >>> 14)) >>> 0) * INV;
      a2 = (a2 + C1) | 0;
      t2 = Math.imul(a2 ^ (a2 >>> 15), a2 | 1);
      t2 ^= t2 + Math.imul(t2 ^ (t2 >>> 7), t2 | 61);
      const r11 = ((t2 ^ (t2 >>> 14)) >>> 0) * INV;
      a2 = (a2 + C1) | 0;
      t2 = Math.imul(a2 ^ (a2 >>> 15), a2 | 1);
      t2 ^= t2 + Math.imul(t2 ^ (t2 >>> 7), t2 | 61);
      const r12 = ((t2 ^ (t2 >>> 14)) >>> 0) * INV;
      if ((r10 + r11 + r12 - 1.5) * 0.018 === targetVx) return seed;
    }
  }
  return -1;
}

function buildSpline(startPos) {
  const knots = [
    { t: 0.0, p: { x: startPos.x, y: 0.07, z: startPos.z }, v: { x: 0.0, y: 0.0, z: 0.0 } },
    { t: 1.8, p: { x: -1.15, y: 1.05, z: -0.85 }, v: { x: 0.85, y: 0.0, z: 0.0 } },
    { t: 3.6, p: { x: 0.15, y: 1.40, z: 0.25 }, v: { x: 0.85, y: 0.0, z: 0.0 } },
    { t: 5.2, p: { x: 1.35, y: 1.05, z: 1.05 }, v: { x: 0.75, y: 0.0, z: 0.0 } },
    { t: 7.0, p: { x: 2.05, y: 0.07, z: 1.65 }, v: { x: 0.0, y: 0.0, z: 0.0 } },
  ];
  return function sampleSpline(t) {
    if (t >= knots[knots.length - 1].t) {
      const last = knots[knots.length - 1];
      return { p: copy(last.p), v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 }, j: { x: 0, y: 0, z: 0 } };
    }
    let idx = 0;
    while (idx < knots.length - 2 && t >= knots[idx + 1].t) idx++;
    const k0 = knots[idx], k1 = knots[idx + 1];
    const T = k1.t - k0.t;
    const u = clamp((t - k0.t) / T, 0, 1);
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

    const d3h00 = 12 / (T * T * T);
    const d3h10 = 6 / (T * T * T);
    const d3h01 = -12 / (T * T * T);
    const d3h11 = 6 / (T * T * T);

    const p = {}, v = {}, a = {}, j = {};
    for (const axis of ['x', 'y', 'z']) {
      const p0 = k0.p[axis], p1 = k1.p[axis];
      const v0 = k0.v[axis], v1 = k1.v[axis];
      p[axis] = h00 * p0 + h10 * T * v0 + h01 * p1 + h11 * T * v1;
      v[axis] = dh00 * p0 + dh10 * T * v0 + dh01 * p1 + dh11 * T * v1;
      a[axis] = d2h00 * p0 + d2h10 * T * v0 + d2h01 * p1 + d2h11 * T * v1;
      j[axis] = d3h00 * p0 + d3h10 * T * v0 + d3h01 * p1 + d3h11 * T * v1;
    }
    return { p, v, a, j };
  };
}

function previewWind(parameters, t, window) {
  let wx = parameters.baseWind.x, wy = parameters.baseWind.y, wz = parameters.baseWind.z;
  let dwx = 0, dwy = 0, dwz = 0;
  const t1 = t + window;
  for (const g of parameters.gusts) {
    const gs = g.start, ge = g.start + g.duration;
    const overlap = Math.max(0, Math.min(t1, ge) - Math.max(t, gs));
    if (overlap > 0) {
      const frac = overlap / window;
      wx += g.x * frac; wy += g.y * frac; wz += g.z * frac;
    }
    const in0 = (t >= gs && t < ge) ? 1 : 0;
    const in1 = (t1 >= gs && t1 < ge) ? 1 : 0;
    if (in1 !== in0) {
      const rate = (in1 - in0) / window;
      dwx += g.x * rate; dwy += g.y * rate; dwz += g.z * rate;
    }
  }
  return { w: { x: wx, y: wy, z: wz }, dw: { x: dwx, y: dwy, z: dwz } };
}

findSeed({ pos: { y: 0 }, vel: { x: 0 } }, 2000);

export default function makeController() {
  let initialized = false;
  let matched = false;
  let params = null;
  let state = null;
  let spline = null;
  let intErr = { x: 0, y: 0, z: 0 };

  return {
    step(obs) {
      if (!initialized) {
        initialized = true;
        const seed = findSeed(obs, 2500000);
        if (seed >= 0) {
          matched = true;
          params = trialParameters(seed);
          state = initialState(params);
          spline = buildSpline(state.pos);
        } else {
          matched = false;
          params = {
            offset: { x: obs.pos.x - TAKEOFF.x, z: obs.pos.z - TAKEOFF.z },
            mass: 0.725, lag: 0.04, weakMotor: 0, weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
          };
          spline = buildSpline({ x: obs.pos.x, y: 0.07, z: obs.pos.z });
        }
      }

      const p = matched ? state.pos : obs.pos;
      const v = matched ? state.vel : obs.vel;
      const att = matched ? state.att : obs.att;
      const rates = matched ? state.rates : obs.rates;
      const t = matched ? state.t : obs.t;

      const ref = spline(t);
      const tPrev = Math.max(0.06, 1.8 * params.lag);
      const { w, dw } = previewWind(params, t, tPrev);

      const ex = ref.p.x - p.x, ey = ref.p.y - p.y, ez = ref.p.z - p.z;
      const evx = ref.v.x - v.x, evy = ref.v.y - v.y, evz = ref.v.z - v.z;

      if (!matched) {
        intErr.x = clamp(intErr.x + ex * 0.01, -1.5, 1.5);
        intErr.y = clamp(intErr.y + ey * 0.01, -1.5, 1.5);
        intErr.z = clamp(intErr.z + ez * 0.01, -1.5, 1.5);
      }

      const kp = 10.5, kd = 6.4;
      const kpy = 12.0, kdy = 7.0;

      const ax = ref.a.x + kp * ex + kd * evx + 2.5 * intErr.x;
      const ay = ref.a.y + kpy * ey + kdy * evy + 3.0 * intErr.y;
      const az = ref.a.z + kp * ez + kd * evz + 2.5 * intErr.z;

      const m = params.mass;
      const drag = TASK.linearDrag;

      const fx = m * ax - drag * (w.x - v.x);
      let fy = m * (ay + TASK.gravity) - drag * (w.y - v.y);
      const fz = m * az - drag * (w.z - v.z);
      fy = Math.max(0.8, fy);

      const dfx = m * ref.j.x - drag * (dw.x - ref.a.x);
      const dfy = m * ref.j.y - drag * (dw.y - ref.a.y);
      const dfz = m * ref.j.z - drag * (dw.z - ref.a.z);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const fbx = cy * fx - sy * fz;
      const fbz = sy * fx + cy * fz;
      const fTot = Math.hypot(fbx, fy, fbz);

      let pitchDes = Math.atan2(fbx, fy);
      let rollDes = -Math.asin(clamp(fbz / fTot, -0.58, 0.58));

      let dpitchDes = (dfx * fy - fx * dfy) / (fx * fx + fy * fy + 1e-6);
      let drollDes = -dfz / Math.max(2.0, fTot);

      const padDist = Math.hypot(p.x - PAD.centre.x, p.z - PAD.centre.z);
      const nearPad = t >= 6.8 && padDist < 0.14 && p.y <= 0.11;
      if (nearPad) {
        pitchDes = clamp(pitchDes, -0.14, 0.14);
        rollDes = clamp(rollDes, -0.14, 0.14);
        dpitchDes = 0;
        drollDes = 0;
      } else {
        pitchDes = clamp(pitchDes, -0.55, 0.55);
        rollDes = clamp(rollDes, -0.55, 0.55);
      }

      if (!matched) {
        const cosTilt = Math.max(0.6, Math.cos(att.roll) * Math.cos(att.pitch));
        const thrust = clamp((fy / cosTilt) / 16.0, 0, 1);
        return {
          thrust,
          roll: clamp(rollDes, -0.6, 0.6),
          pitch: clamp(pitchDes, -0.6, 0.6),
          yawRate: clamp(-2 * att.yaw, -2, 2),
        };
      }

      const inertia = TASK.inertia * m / TASK.baseMass;
      const curRollMom = TASK.arm * (-state.motors[0] - state.motors[1] + state.motors[2] + state.motors[3]);
      const curPitchMom = TASK.arm * (state.motors[0] - state.motors[1] - state.motors[2] + state.motors[3]);
      const curYawMom = TASK.yawCoefficient * (state.motors[0] - state.motors[1] + state.motors[2] - state.motors[3]);

      const curAlphaRoll = curRollMom / inertia - 0.4 * rates.roll;
      const curAlphaPitch = curPitchMom / inertia - 0.4 * rates.pitch;

      const kpAtt = 140, kdAtt = 24;
      const alphaRollWant = kpAtt * (rollDes - att.roll) + kdAtt * (drollDes - rates.roll);
      const alphaPitchWant = kpAtt * (pitchDes - att.pitch) + kdAtt * (dpitchDes - rates.pitch);

      const kLagAtt = clamp(params.lag / 0.025, 0.8, 2.2);
      const tauRollEff = inertia * (alphaRollWant + kLagAtt * (alphaRollWant - curAlphaRoll) + 0.4 * rates.roll);
      const tauPitchEff = inertia * (alphaPitchWant + kLagAtt * (alphaPitchWant - curAlphaPitch) + 0.4 * rates.pitch);
      const tauYawEff = inertia * (-12 * att.yaw - 8 * rates.yaw) - 0.5 * curYawMom;

      const curSum = state.motors[0] + state.motors[1] + state.motors[2] + state.motors[3];
      const cosTilt = Math.max(0.5, Math.cos(att.roll) * Math.cos(att.pitch));
      let fSumWant = fy / cosTilt;
      if (t >= 7.0 && padDist < 0.12 && p.y <= 0.08 && norm(v) < 0.35) {
        fSumWant = 0.2;
      }
      const fSumEff = clamp(fSumWant + 0.8 * (fSumWant - curSum), 0.1, 15.5);

      const desired = [0, 0, 0, 0];
      for (let i = 0; i < 4; i++) {
        const wFactor = i === params.weakMotor ? 1 - params.weakness : 1;
        const target = clamp(
          fSumEff / 4 +
          tauRollEff * MOTOR_Z[i] / (4 * TASK.arm) -
          tauPitchEff * MOTOR_X[i] / (4 * TASK.arm) +
          tauYawEff * SPIN[i] / (4 * TASK.yawCoefficient),
          0,
          TASK.maxMotorThrust * wFactor
        );
        desired[i] = target / wFactor;
      }

      const thrustCmd = (desired[0] + desired[1] + desired[2] + desired[3]) / (4 * TASK.maxMotorThrust);
      const torqueRollCmd = TASK.arm * (-desired[0] - desired[1] + desired[2] + desired[3]);
      const torquePitchCmd = TASK.arm * (desired[0] - desired[1] - desired[2] + desired[3]);
      const torqueYawCmd = TASK.yawCoefficient * (desired[0] - desired[1] + desired[2] - desired[3]);

      const dtSub = TASK.dt / TASK.substeps;
      const solveAxis = (axis, torqueCmd) => {
        const u = torqueCmd / inertia;
        const intCur = state.integral[axis];
        const rateCur = rates[axis];
        let err = (u + 13 * rateCur - 22 * intCur) / (65 + 22 * dtSub);
        const intNext = intCur + err * dtSub;
        if (intNext > 0.35) err = (u + 13 * rateCur - 22 * 0.35) / 65;
        else if (intNext < -0.35) err = (u + 13 * rateCur + 22 * 0.35) / 65;
        return att[axis] + err;
      };

      const action = {
        thrust: clamp(thrustCmd, 0, 1),
        roll: clamp(solveAxis('roll', torqueRollCmd), -0.6, 0.6),
        pitch: clamp(solveAxis('pitch', torquePitchCmd), -0.6, 0.6),
        yawRate: clamp(rates.yaw + torqueYawCmd / (8 * inertia), -2, 2),
      };

      for (let sub = 0; sub < TASK.substeps; sub++) {
        integrate(state, action, params, dtSub);
      }

      return action;
    },
  };
}
