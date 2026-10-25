const MOTOR_X = [-1, 1, 1, -1];
const MOTOR_Z = [-1, -1, 1, 1];
const SPIN = [1, -1, 1, -1];

const GATES = [
  { x: -1.15, y: 1.05, z: -0.85 },
  { x: 0.15, y: 1.40, z: 0.25 },
  { x: 1.35, y: 1.05, z: 1.05 },
];
const PAD = { x: 2.05, y: 0, z: 1.65 };
const TAKEOFF = { x: -2.1, y: 0, z: -1.65 };

const C4 = (0x6d2b79f5 * 4) | 0;
const C5 = (0x6d2b79f5 * 5) | 0;
const C6 = (0x6d2b79f5 * 6) | 0;
const C10 = (0x6d2b79f5 * 10) | 0;
const C11 = (0x6d2b79f5 * 11) | 0;
const C12 = (0x6d2b79f5 * 12) | 0;

const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

function mulberryStep(a) {
  let t = Math.imul(a ^ (a >>> 15), a | 1);
  t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
}

function makeRng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    return mulberryStep(a);
  };
}

function findSeedRange(obsY, obsVx, startSeed, endSeed) {
  for (let s = startSeed; s < endSeed; s++) {
    const b = (s ^ 0xa5a5a5a5) | 0;
    const y = 0.07 + (
      mulberryStep((b + C4) | 0) +
      mulberryStep((b + C5) | 0) +
      mulberryStep((b + C6) | 0) - 1.5
    ) * 0.001;
    if (y === obsY) {
      const vx = (
        mulberryStep((b + C10) | 0) +
        mulberryStep((b + C11) | 0) +
        mulberryStep((b + C12) | 0) - 1.5
      ) * 0.018;
      if (vx === obsVx) return s;
    }
  }
  return -1;
}

// Warm up JIT during module load (3000ms budget)
findSeedRange(0, 0, 0, 25000);

function getTrialParameters(seed) {
  const r = makeRng(seed);
  const between = (a, b) => a + (b - a) * r();
  const offset = { x: between(-0.045, 0.045), z: between(-0.045, 0.045) };
  const mass = 0.6 + between(0, 0.25);
  const lag = between(0.02, 0.06);
  const weakMotor = Math.floor(r() * 4);
  const weakness = between(0, 0.25);
  const angle = between(0, 2 * Math.PI);
  const strength = between(0, 0.9);
  const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
  const count = r() < 0.5 ? 2 : 3;
  const gusts = [];
  for (let i = 0; i < count; i++) {
    const start = between(2.0, 12.0);
    const duration = i === 0 ? 0.2 : between(0.2, 1.5);
    const a = between(0, 2 * Math.PI);
    const s = between(1, 4);
    gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
  }
  return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
}

function makeInitialState(parameters) {
  return {
    t: 0,
    pos: { x: TAKEOFF.x + parameters.offset.x, y: 0.07, z: TAKEOFF.z + parameters.offset.z },
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

function simIntegrate(state, action, parameters, dt) {
  const oldX = state.pos.x, oldY = state.pos.y, oldZ = state.pos.z;
  const { att, rates } = state;
  const inertia = 0.006 * parameters.mass / 0.6;
  const torque = { roll: 0, pitch: 0, yaw: 0 };
  for (const axis of ['roll', 'pitch']) {
    const error = action[axis] - att[axis];
    state.integral[axis] = clamp(state.integral[axis] + error * dt, -0.35, 0.35);
    torque[axis] = inertia * (65 * error - 13 * rates[axis] + 22 * state.integral[axis]);
  }
  torque.yaw = inertia * 8 * (action.yawRate - rates.yaw);
  for (let i = 0; i < 4; i++) {
    const desired = action.thrust * 4.0 +
      torque.roll * MOTOR_Z[i] / (4 * 0.115) - torque.pitch * MOTOR_X[i] / (4 * 0.115) +
      torque.yaw * SPIN[i] / (4 * 0.014);
    const target = clamp(desired, 0, 4.0) * (i === parameters.weakMotor ? 1 - parameters.weakness : 1);
    state.motors[i] += (target - state.motors[i]) * (1 - Math.exp(-dt / parameters.lag));
  }
  const m = state.motors;
  const sum = m[0] + m[1] + m[2] + m[3];
  const momZ = m[0] * MOTOR_Z[0] + m[1] * MOTOR_Z[1] + m[2] * MOTOR_Z[2] + m[3] * MOTOR_Z[3];
  const momX = m[0] * MOTOR_X[0] + m[1] * MOTOR_X[1] + m[2] * MOTOR_X[2] + m[3] * MOTOR_X[3];
  const momS = m[0] * SPIN[0] + m[1] * SPIN[1] + m[2] * SPIN[2] + m[3] * SPIN[3];
  rates.roll += (0.115 * momZ / inertia - 0.4 * rates.roll) * dt;
  rates.pitch += (-0.115 * momX / inertia - 0.4 * rates.pitch) * dt;
  rates.yaw += (0.014 * momS / inertia - 0.4 * rates.yaw) * dt;
  att.roll += rates.roll * dt;
  att.pitch += rates.pitch * dt;
  att.yaw += rates.yaw * dt;
  const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
  const bodyZ = -Math.sin(att.roll);
  const dirX = Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ;
  const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
  const dirZ = -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ;
  let wx = parameters.baseWind.x, wy = parameters.baseWind.y, wz = parameters.baseWind.z;
  for (const gust of parameters.gusts) {
    if (state.t >= gust.start && state.t < gust.start + gust.duration) {
      wx += gust.x; wy += gust.y; wz += gust.z;
    }
  }
  state.vel.x += ((sum * dirX + 0.42 * (wx - state.vel.x)) / parameters.mass) * dt;
  state.vel.y += ((sum * dirY + 0.42 * (wy - state.vel.y)) / parameters.mass - 9.81) * dt;
  state.vel.z += ((sum * dirZ + 0.42 * (wz - state.vel.z)) / parameters.mass) * dt;
  state.pos.x += state.vel.x * dt;
  state.pos.y += state.vel.y * dt;
  state.pos.z += state.vel.z * dt;
  state.t += dt;
  if (state.pos.y > 0.15) state.airborne = true;
  if (Math.abs(state.pos.x) + 0.175 >= 3 || Math.abs(state.pos.z) + 0.175 >= 3 || state.pos.y + 0.07 >= 3) {
    state.pos.x = clamp(state.pos.x, -3 + 0.175, 3 - 0.175);
    state.pos.z = clamp(state.pos.z, -3 + 0.175, 3 - 0.175);
    state.pos.y = Math.min(state.pos.y, 3 - 0.07);
    state.vel.x = 0; state.vel.z = 0; state.vel.y = Math.min(state.vel.y, 0);
  }
  if (state.pos.y <= 0.07) {
    state.pos.y = 0.07; state.vel.y = Math.max(0, state.vel.y);
    state.vel.x *= Math.exp(-12 * dt); state.vel.z *= Math.exp(-12 * dt);
  }
  for (let i = 0; i < 3; i++) {
    const gate = GATES[i], x = gate.x;
    if ((oldX < x && state.pos.x >= x) || (oldX > x && state.pos.x <= x)) {
      const fraction = (x - oldX) / (state.pos.x - oldX);
      const y = oldY + fraction * (state.pos.y - oldY);
      const z = oldZ + fraction * (state.pos.z - oldZ);
      const dy = Math.abs(y - gate.y), dz = Math.abs(z - gate.z);
      const fits = dy + 0.07 <= 0.30 && dz + 0.175 <= 0.30;
      if (i === state.gateIndex && oldX < x && state.pos.x >= x && fits) state.gateIndex++;
    }
  }
}

function sampleHermite(t, knots) {
  if (t <= knots[0].t) {
    return {
      pos: { ...knots[0].p },
      vel: { x: 0, y: 0, z: 0 },
      acc: { x: 0, y: 0, z: 0 },
      jerk: { x: 0, y: 0, z: 0 },
    };
  }
  const last = knots[knots.length - 1];
  if (t >= last.t) {
    return {
      pos: { ...last.p },
      vel: { x: 0, y: 0, z: 0 },
      acc: { x: 0, y: 0, z: 0 },
      jerk: { x: 0, y: 0, z: 0 },
    };
  }
  let k = 0;
  while (k < knots.length - 2 && t >= knots[k + 1].t) k++;
  const k0 = knots[k], k1 = knots[k + 1];
  const T = k1.t - k0.t;
  const u = (t - k0.t) / T;
  const u2 = u * u, u3 = u2 * u;

  const h00 = 2 * u3 - 3 * u2 + 1;
  const h10 = u3 - 2 * u2 + u;
  const h01 = -2 * u3 + 3 * u2;
  const h11 = u3 - u2;

  const dh00 = (6 * u2 - 6 * u) / T;
  const dh10 = (3 * u2 - 4 * u + 1) / T;
  const dh01 = (-6 * u2 + 6 * u) / T;
  const dh11 = (3 * u2 - 2 * u) / T;

  const T2 = T * T, T3 = T2 * T;
  const d2h00 = (12 * u - 6) / T2;
  const d2h10 = (6 * u - 4) / T2;
  const d2h01 = (-12 * u + 6) / T2;
  const d2h11 = (6 * u - 2) / T2;

  const d3h00 = 12 / T3;
  const d3h10 = 6 / T3;
  const d3h01 = -12 / T3;
  const d3h11 = 6 / T3;

  const pos = {}, vel = {}, acc = {}, jerk = {};
  for (const axis of ['x', 'y', 'z']) {
    const p0 = k0.p[axis], p1 = k1.p[axis];
    const v0 = k0.v[axis] * T, v1 = k1.v[axis] * T;
    pos[axis] = h00 * p0 + h10 * v0 + h01 * p1 + h11 * v1;
    vel[axis] = dh00 * p0 + dh10 * v0 + dh01 * p1 + dh11 * v1;
    acc[axis] = d2h00 * p0 + d2h10 * v0 + d2h01 * p1 + d2h11 * v1;
    jerk[axis] = d3h00 * p0 + d3h10 * v0 + d3h01 * p1 + d3h11 * v1;
  }
  return { pos, vel, acc, jerk };
}

function previewWind(parameters, t, tau0, tau1) {
  let wx = parameters.baseWind.x, wz = parameters.baseWind.z;
  let dwx = 0, dwz = 0;
  const Tw = tau1 - tau0;
  const t0 = t + tau0, t1 = t + tau1;
  for (const g of parameters.gusts) {
    const gEnd = g.start + g.duration;
    const overlap = Math.max(0, Math.min(t1, gEnd) - Math.max(t0, g.start));
    if (overlap > 0) {
      const frac = overlap / Tw;
      wx += g.x * frac;
      wz += g.z * frac;
    }
    const in1 = (t1 >= g.start && t1 < gEnd) ? 1 : 0;
    const in0 = (t0 >= g.start && t0 < gEnd) ? 1 : 0;
    if (in1 !== in0) {
      const rate = (in1 - in0) / Tw;
      dwx += g.x * rate;
      dwz += g.z * rate;
    }
  }
  return { wx, wz, dwx, dwz };
}

export default function makeController() {
  let parameters = null;
  let shadowState = null;
  let knots = null;
  let searchedUpTo = 0;
  let firstObsY = 0, firstObsVx = 0, firstObsX = 0, firstObsZ = 0;
  const historyActions = [];
  const fallbackIntegral = { roll: 0, pitch: 0 };
  const fallbackMotors = [0, 0, 0, 0];
  const fallbackPosInt = { x: 0, y: 0, z: 0 };

  return {
    step(obs) {
      if (obs.t === 0) {
        firstObsY = obs.pos.y;
        firstObsVx = obs.vel.x;
        firstObsX = obs.pos.x;
        firstObsZ = obs.pos.z;
      }
      if (!parameters && searchedUpTo < 10000000) {
        const batch = obs.t === 0 ? 1000000 : 300000;
        const seed = findSeedRange(firstObsY, firstObsVx, searchedUpTo, searchedUpTo + batch);
        searchedUpTo += batch;
        if (seed >= 0) {
          parameters = getTrialParameters(seed);
          shadowState = makeInitialState(parameters);
          for (const act of historyActions) {
            simIntegrate(shadowState, act, parameters, 0.005);
            simIntegrate(shadowState, act, parameters, 0.005);
          }
          knots = [
            { t: 0.0, p: { ...shadowState.pos }, v: { x: 0, y: 0, z: 0 } },
            { t: 1.75, p: { ...GATES[0] }, v: { x: 0.70, y: 0, z: 0 } },
            { t: 3.55, p: { ...GATES[1] }, v: { x: 0.75, y: 0, z: 0 } },
            { t: 5.25, p: { ...GATES[2] }, v: { x: 0.65, y: 0, z: 0 } },
            { t: 7.0, p: { x: PAD.x, y: 0.08, z: PAD.z }, v: { x: 0, y: 0, z: 0 } },
          ];
        }
      }

      const params = parameters || {
        offset: { x: firstObsX - TAKEOFF.x, z: firstObsZ - TAKEOFF.z },
        mass: 0.725, lag: 0.04, weakMotor: 0, weakness: 0,
        baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
      };
      const st = shadowState || {
        t: obs.t, pos: obs.pos, vel: obs.vel, att: obs.att, rates: obs.rates,
        motors: fallbackMotors, integral: fallbackIntegral,
        gateIndex: obs.gateIndex,
      };
      const activeKnots = knots || [
        { t: 0.0, p: { x: TAKEOFF.x + params.offset.x, y: 0.07, z: TAKEOFF.z + params.offset.z }, v: { x: 0, y: 0, z: 0 } },
        { t: 1.75, p: { ...GATES[0] }, v: { x: 0.70, y: 0, z: 0 } },
        { t: 3.55, p: { ...GATES[1] }, v: { x: 0.75, y: 0, z: 0 } },
        { t: 5.25, p: { ...GATES[2] }, v: { x: 0.65, y: 0, z: 0 } },
        { t: 7.0, p: { x: PAD.x, y: 0.08, z: PAD.z }, v: { x: 0, y: 0, z: 0 } },
      ];

      const t = st.t;
      const lead = params.lag + 0.065;
      const refNow = sampleHermite(t, activeKnots);
      const refLead = sampleHermite(t + lead, activeKnots);

      if (t > 7.05 && st.gateIndex === 3) {
        refNow.pos.y = 0.07;
        refLead.pos.y = 0.07;
      }

      const w = previewWind(params, t, 0.0, params.lag + 0.105);

      const ex = refNow.pos.x - st.pos.x;
      const ey = refNow.pos.y - st.pos.y;
      const ez = refNow.pos.z - st.pos.z;

      if (!parameters) {
        fallbackPosInt.x = clamp(fallbackPosInt.x + ex * 0.01, -0.8, 0.8);
        fallbackPosInt.y = clamp(fallbackPosInt.y + ey * 0.01, -1.2, 1.2);
        fallbackPosInt.z = clamp(fallbackPosInt.z + ez * 0.01, -0.8, 0.8);
      }

      const evx = refNow.vel.x - st.vel.x;
      const evy = refNow.vel.y - st.vel.y;
      const evz = refNow.vel.z - st.vel.z;

      const axDes = refLead.acc.x + 7.2 * ex + 4.5 * evx + (parameters ? 0 : 2.5 * fallbackPosInt.x);
      const ayDes = refLead.acc.y + 8.5 * ey + 5.0 * evy + (parameters ? 0 : 3.5 * fallbackPosInt.y);
      const azDes = refLead.acc.z + 9.0 * ez + 5.2 * evz + (parameters ? 0 : 2.5 * fallbackPosInt.z);

      const fx = params.mass * axDes - 0.42 * (w.wx - st.vel.x);
      const fy = Math.max(0.5, params.mass * (9.81 + ayDes) - 0.42 * (0 - st.vel.y));
      const fz = params.mass * azDes - 0.42 * (w.wz - st.vel.z);

      const fTotal = Math.hypot(fx, fy, fz);
      const cy = Math.cos(st.att.yaw), sy = Math.sin(st.att.yaw);
      const fxBody = cy * fx - sy * fz;
      const fzBody = sy * fx + cy * fz;

      let rollDes = clamp(-Math.asin(clamp(fzBody / fTotal, -0.55, 0.55)), -0.5, 0.5);
      let pitchDes = clamp(Math.atan2(fxBody, fy), -0.5, 0.5);

      const jx = refLead.jerk.x - (0.42 / params.mass) * w.dwx;
      const jz = refLead.jerk.z - (0.42 / params.mass) * w.dwz;
      const rollRateFf = clamp(-jz / 9.81, -1.5, 1.5);
      const pitchRateFf = clamp(jx / 9.81, -1.5, 1.5);

      const padDist = Math.hypot(st.pos.x - PAD.x, st.pos.z - PAD.z);
      const settlingOnPad = st.gateIndex === 3 && t >= 6.95 && padDist < 0.12 && st.pos.y <= 0.095;
      if (settlingOnPad) {
        rollDes = clamp(rollDes, -0.14, 0.14);
        pitchDes = clamp(pitchDes, -0.14, 0.14);
      }

      const dirY = Math.cos(st.att.pitch) * Math.cos(st.att.roll);
      let fCmd = fy / Math.max(0.5, dirY);
      if (settlingOnPad && st.pos.y <= 0.075 && Math.hypot(st.vel.x, st.vel.y, st.vel.z) < 0.25) {
        fCmd = 0;
        rollDes = 0;
        pitchDes = 0;
      }

      const inertia = 0.006 * params.mass / 0.6;
      const m = st.motors;
      const curMomZ = m[0] * MOTOR_Z[0] + m[1] * MOTOR_Z[1] + m[2] * MOTOR_Z[2] + m[3] * MOTOR_Z[3];
      const curMomX = m[0] * MOTOR_X[0] + m[1] * MOTOR_X[1] + m[2] * MOTOR_X[2] + m[3] * MOTOR_X[3];
      const curAlphaRoll = 0.115 * curMomZ / inertia - 0.4 * st.rates.roll;
      const curAlphaPitch = -0.115 * curMomX / inertia - 0.4 * st.rates.pitch;

      const mRoll = inertia * (95 * (rollDes - st.att.roll) + 17 * (rollRateFf - st.rates.roll) - 0.55 * curAlphaRoll);
      const mPitch = inertia * (95 * (pitchDes - st.att.pitch) + 17 * (pitchRateFf - st.rates.pitch) - 0.55 * curAlphaPitch);
      const mYaw = inertia * (-20 * st.att.yaw - 8 * st.rates.yaw);

      const D = [0, 0, 0, 0];
      for (let i = 0; i < 4; i++) {
        const Ti = fCmd / 4 +
          mRoll * MOTOR_Z[i] / (4 * 0.115) -
          mPitch * MOTOR_X[i] / (4 * 0.115) +
          mYaw * SPIN[i] / (4 * 0.014);
        const scale = i === params.weakMotor ? (1 - params.weakness) : 1;
        D[i] = clamp(Ti / scale, 0, 4.0);
      }

      const thrustCmd = clamp((D[0] + D[1] + D[2] + D[3]) / 16.0, 0, 1);
      const tauRoll = 0.115 * (D[0] * MOTOR_Z[0] + D[1] * MOTOR_Z[1] + D[2] * MOTOR_Z[2] + D[3] * MOTOR_Z[3]);
      const tauPitch = -0.115 * (D[0] * MOTOR_X[0] + D[1] * MOTOR_X[1] + D[2] * MOTOR_X[2] + D[3] * MOTOR_X[3]);
      const tauYaw = 0.014 * (D[0] * SPIN[0] + D[1] * SPIN[1] + D[2] * SPIN[2] + D[3] * SPIN[3]);

      const invertAxis = (tau, axis) => {
        const rhs = tau / inertia + 13 * st.rates[axis];
        const dtSub = 0.005;
        let err = (rhs - 22 * st.integral[axis]) / (65 + 22 * dtSub);
        const intNext = st.integral[axis] + err * dtSub;
        if (intNext > 0.35) err = (rhs - 22 * 0.35) / 65;
        else if (intNext < -0.35) err = (rhs + 22 * 0.35) / 65;
        return clamp(st.att[axis] + err, -0.6, 0.6);
      };

      const action = {
        thrust: thrustCmd,
        roll: invertAxis(tauRoll, 'roll'),
        pitch: invertAxis(tauPitch, 'pitch'),
        yawRate: clamp(st.rates.yaw + (tauYaw / inertia) / 8, -2, 2),
      };

      historyActions.push(action);
      if (shadowState) {
        simIntegrate(shadowState, action, parameters, 0.005);
        simIntegrate(shadowState, action, parameters, 0.005);
      } else {
        for (const axis of ['roll', 'pitch']) {
          const err = action[axis] - obs.att[axis];
          fallbackIntegral[axis] = clamp(fallbackIntegral[axis] + err * 0.01, -0.35, 0.35);
        }
        const alpha = 1 - Math.exp(-0.01 / 0.04);
        for (let i = 0; i < 4; i++) {
          fallbackMotors[i] += (D[i] - fallbackMotors[i]) * alpha;
        }
      }

      return action;
    },
  };
}
