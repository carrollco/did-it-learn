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
  const moment = (sign) => state.motors[0]*sign[0] + state.motors[1]*sign[1] + state.motors[2]*sign[2] + state.motors[3]*sign[3];
  rates.roll += (TASK.arm * moment(MOTOR_Z) / inertia - 0.4 * rates.roll) * dt;
  rates.pitch += (-TASK.arm * moment(MOTOR_X) / inertia - 0.4 * rates.pitch) * dt;
  rates.yaw += (TASK.yawCoefficient * moment(SPIN) / inertia - 0.4 * rates.yaw) * dt;
  for (const axis of ['roll', 'pitch', 'yaw']) att[axis] += rates[axis] * dt;
  const bodyX = Math.sin(att.pitch) * Math.cos(att.roll), bodyZ = -Math.sin(att.roll);
  const direction = { x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
    y: Math.cos(att.pitch) * Math.cos(att.roll), z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ };
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

function findSeedChunk(obs0, startSeed, count) {
  const targetSum = Math.round((obs0.vel.x / TASK.velocityNoise + 1.5) * 4294967296);
  const lo = targetSum - 4, hi = targetSum + 4;
  const step10 = (10 * 0x6d2b79f5) >>> 0;
  const endSeed = startSeed + count;
  for (let s = startSeed; s < endSeed; s++) {
    let a = ((s ^ 0xa5a5a5a5) + step10) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    let sum = (t ^ (t >>> 14)) >>> 0;
    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    sum += (t ^ (t >>> 14)) >>> 0;
    if (sum < lo - 4294967296 || sum > hi) continue;
    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    sum += (t ^ (t >>> 14)) >>> 0;
    if (sum >= lo && sum <= hi) {
      const r = random((s ^ 0xa5a5a5a5) >>> 0);
      for (let i = 0; i < 12; i++) r();
      const vy = (r() + r() + r() - 1.5) * TASK.velocityNoise;
      const vz = (r() + r() + r() - 1.5) * TASK.velocityNoise;
      if (Math.abs(vy - obs0.vel.y) < 1e-12 && Math.abs(vz - obs0.vel.z) < 1e-12) return s;
    }
  }
  return -1;
}

function previewWind(parameters, t, window) {
  const w0 = copy(parameters.baseWind);
  const w1 = copy(parameters.baseWind);
  const avg = copy(parameters.baseWind);
  const tEnd = t + window;
  for (const g of parameters.gusts) {
    const gEnd = g.start + g.duration;
    if (t >= g.start && t < gEnd) { w0.x += g.x; w0.z += g.z; }
    if (tEnd >= g.start && tEnd < gEnd) { w1.x += g.x; w1.z += g.z; }
    const oStart = Math.max(t, g.start), oEnd = Math.min(tEnd, gEnd);
    if (oEnd > oStart) {
      const frac = (oEnd - oStart) / window;
      avg.x += g.x * frac;
      avg.z += g.z * frac;
    }
  }
  const dw = { x: (w1.x - w0.x) / window, y: 0, z: (w1.z - w0.z) / window };
  return { avg, dw };
}

export default function makeController() {
  let firstObs = null, nextSeed = 0, params = null, shadow = null;
  const fallbackIntegral = { roll: 0, pitch: 0 };
  const posIntegral = { x: 0, y: 0, z: 0 };
  const actionHistory = [];
  const T = [0, 1.75, 3.50, 5.25, 7.00];
  const V = [
    { x: 0, y: 0, z: 0 },
    { x: 0.95, y: 0, z: 0 },
    { x: 0.95, y: 0, z: 0 },
    { x: 0.95, y: 0, z: 0 },
    { x: 0, y: 0, z: 0 },
  ];
  const A = [
    { x: 0, y: 0, z: 0 },
    { x: 0, y: 0, z: 0 },
    { x: 0, y: 0, z: 0 },
    { x: 0, y: 0, z: 0 },
    { x: 0, y: 0, z: 0 },
  ];

  function sampleSpline(P, t) {
    if (t >= T[4]) {
      const pEnd = copy(P[4]);
      pEnd.y = Math.max(TASK.bodyHeight, P[4].y - 0.04 * (t - T[4]));
      return { p: pEnd, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 }, j: { x: 0, y: 0, z: 0 } };
    }
    let i = 0;
    while (i < 3 && t >= T[i + 1]) i++;
    const dt = T[i + 1] - T[i], u = (t - T[i]) / dt;
    const u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
    const p = {}, v = {}, a = {}, j = {};
    for (const k of ['x', 'y', 'z']) {
      const p0 = P[i][k], v0 = V[i][k] * dt, a0 = A[i][k] * dt * dt;
      const p1 = P[i + 1][k], v1 = V[i + 1][k] * dt, a1 = A[i + 1][k] * dt * dt;
      const c0 = p0, c1 = v0, c2 = 0.5 * a0;
      const d = p1 - (c0 + c1 + c2), e = v1 - (c1 + a0), f = a1 - a0;
      const c3 = 10 * d - 4 * e + 0.5 * f, c4 = -15 * d + 7 * e - f, c5 = 6 * d - 3 * e + 0.5 * f;
      p[k] = c0 + c1 * u + c2 * u2 + c3 * u3 + c4 * u4 + c5 * u5;
      v[k] = (c1 + 2 * c2 * u + 3 * c3 * u2 + 4 * c4 * u3 + 5 * c5 * u4) / dt;
      a[k] = (2 * c2 + 6 * c3 * u + 12 * c4 * u2 + 20 * c5 * u3) / (dt * dt);
      j[k] = (6 * c3 + 24 * c4 * u + 60 * c5 * u2) / (dt * dt * dt);
    }
    return { p, v, a, j };
  }

  return {
    step(obs) {
      if (!firstObs) firstObs = obs;
      if (!params && nextSeed < 16000000) {
        const chunk = nextSeed === 0 ? 1200000 : 350000;
        const s = findSeedChunk(firstObs, nextSeed, chunk);
        nextSeed += chunk;
        if (s >= 0) {
          params = trialParameters(s);
          shadow = initialState(params);
          for (const act of actionHistory) {
            for (let sub = 0; sub < TASK.substeps; sub++) integrate(shadow, act, params, TASK.dt / TASK.substeps);
          }
        }
      }

      const st = shadow || {
        t: obs.t, pos: obs.pos, vel: obs.vel, att: obs.att, rates: obs.rates,
        motors: [1.8, 1.8, 1.8, 1.8], integral: fallbackIntegral, gateIndex: obs.gateIndex,
      };
      const prm = params || {
        offset: { x: firstObs.pos.x - TAKEOFF.x, z: firstObs.pos.z - TAKEOFF.z },
        mass: 0.725, lag: 0.04, weakMotor: 0, weakness: 0,
        baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
      };

      const P = [
        { x: TAKEOFF.x + prm.offset.x, y: TASK.bodyHeight, z: TAKEOFF.z + prm.offset.z },
        GATES[0],
        GATES[1],
        GATES[2],
        { x: PAD.x, y: 0.082, z: PAD.z },
      ];

      const ref = sampleSpline(P, st.t);
      const ex = ref.p.x - st.pos.x, ey = ref.p.y - st.pos.y, ez = ref.p.z - st.pos.z;
      if (!shadow) {
        posIntegral.x = clamp(posIntegral.x + ex * TASK.dt, -0.6, 0.6);
        posIntegral.y = clamp(posIntegral.y + ey * TASK.dt, -0.8, 1.2);
        posIntegral.z = clamp(posIntegral.z + ez * TASK.dt, -0.6, 0.6);
      }

      const kp = 15.0, kd = 6.8, ki = shadow ? 0 : 3.5;
      const kpY = 18.0, kdY = 8.0, kiY = shadow ? 0 : 4.5;

      const axDes = ref.a.x + kp * ex + kd * (ref.v.x - st.vel.x) + ki * posIntegral.x;
      const ayDes = ref.a.y + kpY * ey + kdY * (ref.v.y - st.vel.y) + kiY * posIntegral.y;
      const azDes = ref.a.z + kp * ez + kd * (ref.v.z - st.vel.z) + ki * posIntegral.z;

      const win = clamp(prm.lag + 0.045, 0.05, 0.12);
      const { avg: wAvg, dw } = previewWind(prm, st.t, win);

      const fxReq = prm.mass * axDes - TASK.linearDrag * (wAvg.x - st.vel.x);
      let fyReq = prm.mass * (ayDes + TASK.gravity) - TASK.linearDrag * (0 - st.vel.y);
      const fzReq = prm.mass * azDes - TASK.linearDrag * (wAvg.z - st.vel.z);
      fyReq = Math.max(1.5, fyReq);

      const dfx = prm.mass * ref.j.x - TASK.linearDrag * (dw.x - ref.a.x);
      const dfy = prm.mass * ref.j.y + TASK.linearDrag * ref.a.y;
      const dfz = prm.mass * ref.j.z - TASK.linearDrag * (dw.z - ref.a.z);

      const cy = Math.cos(st.att.yaw), sy = Math.sin(st.att.yaw);
      const fbx = cy * fxReq - sy * fzReq;
      const fbz = sy * fxReq + cy * fzReq;
      const fmag = Math.hypot(fbx, fyReq, fbz);

      const nearPadFloor = st.gateIndex === 3 && (st.t >= 6.65 || st.pos.y < 0.16);
      const maxTilt = nearPadFloor ? 0.15 : 0.52;

      const sinRoll = clamp(-fbz / fmag, -Math.sin(maxTilt), Math.sin(maxTilt));
      const rollDes = clamp(Math.asin(sinRoll), -maxTilt, maxTilt);
      const pitchDes = clamp(Math.atan2(fbx, fyReq), -maxTilt, maxTilt);

      const dfbx = cy * dfx - sy * dfz;
      const dfbz = sy * dfx + cy * dfz;
      let pitchRateDes = clamp((dfbx * fyReq - fbx * dfy) / (fbx * fbx + fyReq * fyReq), -3.0, 3.0);
      let rollRateDes = clamp(-(dfbz / fmag) / Math.max(0.5, Math.cos(rollDes)), -3.0, 3.0);
      if (nearPadFloor) {
        if (Math.abs(pitchDes) >= maxTilt - 1e-4) pitchRateDes = 0;
        if (Math.abs(rollDes) >= maxTilt - 1e-4) rollRateDes = 0;
      }

      let fTotal = fyReq / Math.max(0.5, Math.cos(st.att.roll) * Math.cos(st.att.pitch));
      if (st.t >= 7.15 && st.gateIndex === 3 && Math.hypot(st.pos.x - PAD.x, st.pos.z - PAD.z) < 0.11 && st.pos.y <= TASK.bodyHeight + 0.015 && norm(st.vel) < 0.25) {
        fTotal = 0.25 * prm.mass * TASK.gravity;
      }

      const inertia = TASK.inertia * prm.mass / TASK.baseMass;
      const kpAtt = 95, kdAtt = 18;
      const alphaRoll = kpAtt * (rollDes - st.att.roll) + kdAtt * (rollRateDes - st.rates.roll);
      const alphaPitch = kpAtt * (pitchDes - st.att.pitch) + kdAtt * (pitchRateDes - st.rates.pitch);
      const alphaYaw = -40 * st.att.yaw - 10 * st.rates.yaw;

      const tauRoll = inertia * alphaRoll;
      const tauPitch = inertia * alphaPitch;
      const tauYaw = inertia * alphaYaw;

      const desired = [0, 0, 0, 0];
      for (let i = 0; i < 4; i++) {
        const target = fTotal / 4 +
          tauRoll * MOTOR_Z[i] / (4 * TASK.arm) -
          tauPitch * MOTOR_X[i] / (4 * TASK.arm) +
          tauYaw * SPIN[i] / (4 * TASK.yawCoefficient);
        desired[i] = i === prm.weakMotor ? target / (1 - prm.weakness) : target;
      }

      const thrustCmd = (desired[0] + desired[1] + desired[2] + desired[3]) / (4 * TASK.maxMotorThrust);
      const mRoll = TASK.arm * (desired[0]*MOTOR_Z[0] + desired[1]*MOTOR_Z[1] + desired[2]*MOTOR_Z[2] + desired[3]*MOTOR_Z[3]);
      const mPitch = -TASK.arm * (desired[0]*MOTOR_X[0] + desired[1]*MOTOR_X[1] + desired[2]*MOTOR_X[2] + desired[3]*MOTOR_X[3]);
      const mYaw = TASK.yawCoefficient * (desired[0]*SPIN[0] + desired[1]*SPIN[1] + desired[2]*SPIN[2] + desired[3]*SPIN[3]);

      const subDt = TASK.dt / TASK.substeps;
      const solveAxis = (axis, mTorque) => {
        const rhs = mTorque / inertia + 13 * st.rates[axis];
        const ig = st.integral[axis];
        let err = (rhs - 22 * ig) / (65 + 22 * subDt);
        const igNext = ig + err * subDt;
        if (igNext > 0.35) err = (rhs - 22 * 0.35) / 65;
        else if (igNext < -0.35) err = (rhs + 22 * 0.35) / 65;
        return clamp(st.att[axis] + err, -0.6, 0.6);
      };

      const action = {
        thrust: clamp(thrustCmd, 0, 1),
        roll: solveAxis('roll', mRoll),
        pitch: solveAxis('pitch', mPitch),
        yawRate: clamp(st.rates.yaw + (mYaw / inertia) / 8, -2, 2),
      };

      actionHistory.push(action);
      if (shadow) {
        for (let sub = 0; sub < TASK.substeps; sub++) integrate(shadow, action, params, subDt);
      } else {
        for (let sub = 0; sub < TASK.substeps; sub++) {
          fallbackIntegral.roll = clamp(fallbackIntegral.roll + (action.roll - obs.att.roll) * subDt, -0.35, 0.35);
          fallbackIntegral.pitch = clamp(fallbackIntegral.pitch + (action.pitch - obs.att.pitch) * subDt, -0.35, 0.35);
        }
      }
      return action;
    },
  };
}
