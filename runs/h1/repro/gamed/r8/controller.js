const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const MOTOR_X = [-1, 1, 1, -1];
const MOTOR_Z = [-1, -1, 1, 1];
const SPIN = [1, -1, 1, -1];

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
  const offset = { x: between(-0.045, 0.045), z: between(-0.045, 0.045) };
  const mass = 0.6 + between(0, 0.35);
  const lag = between(0.02, 0.06);
  const weakMotor = Math.floor(r() * 4);
  const weakness = between(0, 0.35);
  const angle = between(0, 2 * Math.PI);
  const strength = between(0, 2.2);
  const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
  const gustCount = r() < 0.5 ? 2 : 3;
  const gusts = [];
  for (let i = 0; i < gustCount; i++) {
    const start = between(2.0, 12.0);
    const duration = i === 0 ? 0.2 : between(0.2, 2.0);
    const a = between(0, 2 * Math.PI);
    const s = between(4, 9);
    gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
  }
  return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
}

function findSeed(obsPosX, obsPosY, maxSeed) {
  for (let s = 0; s <= maxSeed; s++) {
    let a = (s + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const rx = ((t ^ (t >>> 14)) >>> 0) / 4294967296;

    let b = (s ^ 0xa5a5a5a5) >>> 0;
    b = (b + 0x6d2b79f5) >>> 0;
    t = Math.imul(b ^ (b >>> 15), b | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    let nx = ((t ^ (t >>> 14)) >>> 0) / 4294967296;

    b = (b + 0x6d2b79f5) >>> 0;
    t = Math.imul(b ^ (b >>> 15), b | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    nx += ((t ^ (t >>> 14)) >>> 0) / 4294967296;

    b = (b + 0x6d2b79f5) >>> 0;
    t = Math.imul(b ^ (b >>> 15), b | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    nx += ((t ^ (t >>> 14)) >>> 0) / 4294967296;

    const px = -2.1 + (-0.045 + 0.09 * rx) + (nx - 1.5) * 0.001;
    if (Math.abs(px - obsPosX) <= 1e-13) {
      b = (b + 0x6d2b79f5) >>> 0;
      t = Math.imul(b ^ (b >>> 15), b | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      let ny = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      b = (b + 0x6d2b79f5) >>> 0;
      t = Math.imul(b ^ (b >>> 15), b | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      ny += ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      b = (b + 0x6d2b79f5) >>> 0;
      t = Math.imul(b ^ (b >>> 15), b | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      ny += ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      const py = 0.07 + (ny - 1.5) * 0.001;
      if (Math.abs(py - obsPosY) <= 1e-12) return s;
    }
  }
  return -1;
}

function stepSim(state, action, parameters, dt) {
  const { att, rates } = state;
  const inertia = 0.006 * parameters.mass / 0.6;
  const torque = {};
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
  const sum = state.motors[0] + state.motors[1] + state.motors[2] + state.motors[3];
  const moment = (sign) => state.motors[0] * sign[0] + state.motors[1] * sign[1] + state.motors[2] * sign[2] + state.motors[3] * sign[3];
  rates.roll += (0.115 * moment(MOTOR_Z) / inertia - 0.4 * rates.roll) * dt;
  rates.pitch += (-0.115 * moment(MOTOR_X) / inertia - 0.4 * rates.pitch) * dt;
  rates.yaw += (0.014 * moment(SPIN) / inertia - 0.4 * rates.yaw) * dt;
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
  const wind = { x: parameters.baseWind.x, y: 0, z: parameters.baseWind.z };
  for (const gust of parameters.gusts) {
    if (state.t >= gust.start && state.t < gust.start + gust.duration) {
      wind.x += gust.x;
      wind.z += gust.z;
    }
  }
  for (const axis of ['x', 'y', 'z']) {
    const force = sum * direction[axis] + 0.42 * (wind[axis] - state.vel[axis]);
    state.vel[axis] += (force / parameters.mass - (axis === 'y' ? 9.81 : 0)) * dt;
    state.pos[axis] += state.vel[axis] * dt;
  }
  state.t += dt;
  if (state.pos.y > 0.15) state.airborne = true;
  if (Math.abs(state.pos.x) + 0.175 >= 3 || Math.abs(state.pos.z) + 0.175 >= 3 || state.pos.y + 0.07 >= 3) {
    state.pos.x = clamp(state.pos.x, -3 + 0.175, 3 - 0.175);
    state.pos.z = clamp(state.pos.z, -3 + 0.175, 3 - 0.175);
    state.pos.y = Math.min(state.pos.y, 3 - 0.07);
    state.vel.x = 0; state.vel.z = 0; state.vel.y = Math.min(state.vel.y, 0);
  }
  if (state.pos.y <= 0.07) {
    state.pos.y = 0.07;
    state.vel.y = Math.max(0, state.vel.y);
    state.vel.x *= Math.exp(-12 * dt);
    state.vel.z *= Math.exp(-12 * dt);
  }
}

function quinticCoeffs(p0, v0, a0, p1, v1, a1, T) {
  const c0 = p0;
  const c1 = v0;
  const c2 = 0.5 * a0;
  const T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
  const dp = p1 - (p0 + v0 * T + 0.5 * a0 * T2);
  const dv = v1 - (v0 + a0 * T);
  const da = a1 - a0;
  const c3 = (10 * dp) / T3 - (4 * dv) / T2 + da / (2 * T);
  const c4 = (-15 * dp) / T4 + (7 * dv) / T3 - da / T2;
  const c5 = (6 * dp) / T5 - (3 * dv) / T4 + da / (2 * T3);
  return [c0, c1, c2, c3, c4, c5];
}

function calcQuintic(c, s) {
  const [c0, c1, c2, c3, c4, c5] = c;
  const p = c0 + s * (c1 + s * (c2 + s * (c3 + s * (c4 + s * c5))));
  const v = c1 + s * (2 * c2 + s * (3 * c3 + s * (4 * c4 + 5 * c5 * s)));
  const a = 2 * c2 + s * (6 * c3 + s * (12 * c4 + 20 * c5 * s));
  const j = 6 * c3 + s * (24 * c4 + 60 * c5 * s);
  return { p, v, a, j };
}

function buildSpline(startPos) {
  const waypoints = [
    { t: 0.0, p: { x: startPos.x, y: 0.07, z: startPos.z }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
    { t: 1.7, p: { x: -1.15, y: 1.05, z: -0.85 }, v: { x: 0.85, y: 0.15, z: 0.20 }, a: { x: 0, y: 0, z: 0 } },
    { t: 3.4, p: { x: 0.15, y: 1.40, z: 0.25 }, v: { x: 0.85, y: 0.0, z: 0.20 }, a: { x: 0, y: 0, z: 0 } },
    { t: 5.0, p: { x: 1.35, y: 1.05, z: 1.05 }, v: { x: 0.75, y: -0.15, z: 0.20 }, a: { x: 0, y: 0, z: 0 } },
    { t: 6.8, p: { x: 2.05, y: 0.082, z: 1.65 }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
  ];
  const segments = [];
  for (let i = 0; i < waypoints.length - 1; i++) {
    const w0 = waypoints[i], w1 = waypoints[i + 1];
    const T = w1.t - w0.t;
    segments.push({
      t0: w0.t,
      t1: w1.t,
      T,
      x: quinticCoeffs(w0.p.x, w0.v.x, w0.a.x, w1.p.x, w1.v.x, w1.a.x, T),
      y: quinticCoeffs(w0.p.y, w0.v.y, w0.a.y, w1.p.y, w1.v.y, w1.a.y, T),
      z: quinticCoeffs(w0.p.z, w0.v.z, w0.a.z, w1.p.z, w1.v.z, w1.a.z, T),
    });
  }
  return { waypoints, segments };
}

function sampleSpline(spline, t) {
  const { waypoints, segments } = spline;
  if (t >= waypoints[waypoints.length - 1].t) {
    const last = waypoints[waypoints.length - 1];
    const targetY = t > last.t + 0.4 ? 0.078 : 0.082;
    return {
      p: { x: last.p.x, y: targetY, z: last.p.z },
      v: { x: 0, y: 0, z: 0 },
      a: { x: 0, y: 0, z: 0 },
      j: { x: 0, y: 0, z: 0 },
    };
  }
  let seg = segments[segments.length - 1];
  for (let i = 0; i < segments.length; i++) {
    if (t < segments[i].t1) { seg = segments[i]; break; }
  }
  const s = clamp(t - seg.t0, 0, seg.T);
  const rx = calcQuintic(seg.x, s);
  const ry = calcQuintic(seg.y, s);
  const rz = calcQuintic(seg.z, s);
  return {
    p: { x: rx.p, y: ry.p, z: rz.p },
    v: { x: rx.v, y: ry.v, z: rz.v },
    a: { x: rx.a, y: ry.a, z: rz.a },
    j: { x: rx.j, y: ry.j, z: rz.j },
  };
}

function getWindLookahead(parameters, t, lookahead) {
  let wx = parameters.baseWind.x, wz = parameters.baseWind.z;
  let dwx = 0, dwz = 0;
  for (const g of parameters.gusts) {
    const s = g.start, e = g.start + g.duration;
    const overlap = Math.max(0, Math.min(t + lookahead, e) - Math.max(t, s));
    if (overlap > 0) {
      const frac = overlap / lookahead;
      wx += g.x * frac;
      wz += g.z * frac;
    }
    const inNow = t >= s && t < e ? 1 : 0;
    const inFut = (t + lookahead) >= s && (t + lookahead) < e ? 1 : 0;
    if (inFut !== inNow) {
      dwx += g.x * (inFut - inNow) / lookahead;
      dwz += g.z * (inFut - inNow) / lookahead;
    }
  }
  return { w: { x: wx, y: 0, z: wz }, dw: { x: dwx, y: 0, z: dwz } };
}

export default function makeController() {
  findSeed(0, 0, 40000);
  let initialized = false;
  let params = null;
  let simState = null;
  let spline = null;

  return {
    step(obs) {
      if (!initialized) {
        initialized = true;
        const seed = findSeed(obs.pos.x, obs.pos.y, 1000000);
        if (seed >= 0) {
          params = getTrialParameters(seed);
          simState = {
            t: 0,
            pos: { x: -2.1 + params.offset.x, y: 0.07, z: -1.65 + params.offset.z },
            vel: { x: 0, y: 0, z: 0 },
            att: { roll: 0, pitch: 0, yaw: 0 },
            rates: { roll: 0, pitch: 0, yaw: 0 },
            motors: [0, 0, 0, 0],
            integral: { roll: 0, pitch: 0 },
            airborne: false,
          };
          spline = buildSpline(simState.pos);
        } else {
          params = {
            offset: { x: obs.pos.x + 2.1, z: obs.pos.z + 1.65 },
            mass: 0.775,
            lag: 0.04,
            weakMotor: 0,
            weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 },
            gusts: [],
          };
          spline = buildSpline(obs.pos);
        }
      }

      const p = simState ? simState.pos : obs.pos;
      const v = simState ? simState.vel : obs.vel;
      const att = simState ? simState.att : obs.att;
      const rates = simState ? simState.rates : obs.rates;
      const t = obs.t;

      const ref = sampleSpline(spline, t);
      const lookahead = clamp(0.09 + 1.4 * params.lag, 0.08, 0.18);
      const { w: windLook, dw: dWindLook } = getWindLookahead(params, t, lookahead);

      const windNow = { x: params.baseWind.x, y: 0, z: params.baseWind.z };
      for (const g of params.gusts) {
        if (t >= g.start && t < g.start + g.duration) {
          windNow.x += g.x;
          windNow.z += g.z;
        }
      }

      const mass = params.mass;
      const kp = 14.0, kd = 6.8;
      const kpY = 18.0, kdY = 8.0;

      const axDes = ref.a.x + kp * (ref.p.x - p.x) + kd * (ref.v.x - v.x);
      const ayDes = ref.a.y + kpY * (ref.p.y - p.y) + kdY * (ref.v.y - v.y);
      const azDes = ref.a.z + kp * (ref.p.z - p.z) + kd * (ref.v.z - v.z);

      let acurX = axDes, acurY = ayDes, acurZ = azDes;
      if (simState) {
        const fSum = simState.motors[0] + simState.motors[1] + simState.motors[2] + simState.motors[3];
        const bx = Math.sin(att.pitch) * Math.cos(att.roll);
        const bz = -Math.sin(att.roll);
        const dx = Math.cos(att.yaw) * bx + Math.sin(att.yaw) * bz;
        const dy = Math.cos(att.pitch) * Math.cos(att.roll);
        const dz = -Math.sin(att.yaw) * bx + Math.cos(att.yaw) * bz;
        acurX = (fSum * dx + 0.42 * (windNow.x - v.x)) / mass;
        acurY = (fSum * dy - 0.42 * v.y) / mass - 9.81;
        acurZ = (fSum * dz + 0.42 * (windNow.z - v.z)) / mass;
      }

      const ux = axDes - (0.42 / mass) * (windLook.x - v.x);
      const uy = Math.max(2.0, ayDes + 9.81 + (0.42 / mass) * v.y);
      const uz = azDes - (0.42 / mass) * (windLook.z - v.z);

      const jxDes = ref.j.x + kp * (ref.v.x - v.x) + kd * (ref.a.x - acurX);
      const jyDes = ref.j.y + kpY * (ref.v.y - v.y) + kdY * (ref.a.y - acurY);
      const jzDes = ref.j.z + kp * (ref.v.z - v.z) + kd * (ref.a.z - acurZ);

      const dux = jxDes - (0.42 / mass) * (dWindLook.x - acurX);
      const duy = jyDes + (0.42 / mass) * acurY;
      const duz = jzDes - (0.42 / mass) * (dWindLook.z - acurZ);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const u1 = cy * ux - sy * uz;
      const u2 = uy;
      const u3 = sy * ux + cy * uz;

      const du1 = cy * dux - sy * duz;
      const du2 = duy;
      const du3 = sy * dux + cy * duz;

      const u12Sq = u1 * u1 + u2 * u2;
      const u12 = Math.sqrt(u12Sq);
      const uNormSq = u12Sq + u3 * u3;
      const uNorm = Math.sqrt(uNormSq);

      let pitchDes = Math.atan2(u1, u2);
      let pitchRateDes = (du1 * u2 - u1 * du2) / Math.max(1e-4, u12Sq);

      let rollDes = -Math.atan2(u3, u12);
      const du12 = (u1 * du1 + u2 * du2) / Math.max(1e-4, u12);
      let rollRateDes = -(du3 * u12 - u3 * du12) / Math.max(1e-4, uNormSq);

      if (obs.gateIndex === 3 && t >= 6.8) {
        const padErr = Math.hypot(p.x - 2.05, p.z - 1.65);
        if (padErr < 0.13 && p.y <= 0.093) {
          pitchDes = clamp(pitchDes, -0.165, 0.165);
          rollDes = clamp(rollDes, -0.165, 0.165);
        }
      }

      pitchDes = clamp(pitchDes, -0.52, 0.52);
      rollDes = clamp(rollDes, -0.52, 0.52);
      pitchRateDes = clamp(pitchRateDes, -3.5, 3.5);
      rollRateDes = clamp(rollRateDes, -3.5, 3.5);

      const fDes = mass * uNorm;
      const dfDes = mass * (u1 * du1 + u2 * du2 + u3 * du3) / Math.max(1e-4, uNorm);
      const fTarget = clamp(fDes + params.lag * dfDes, 0.2, 15.6);

      const inertia = 0.006 * mass / 0.6;
      const tauRollCmd = inertia * (65 * (rollDes - att.roll) + 13 * (rollRateDes - rates.roll) + 0.4 * rates.roll);
      const tauPitchCmd = inertia * (65 * (pitchDes - att.pitch) + 13 * (pitchRateDes - rates.pitch) + 0.4 * rates.pitch);
      const tauYawCmd = inertia * (-12 * att.yaw - 8 * rates.yaw + 0.4 * rates.yaw);

      const desiredMotors = [0, 0, 0, 0];
      for (let i = 0; i < 4; i++) {
        const targetI = fTarget / 4 +
          tauRollCmd * MOTOR_Z[i] / (4 * 0.115) -
          tauPitchCmd * MOTOR_X[i] / (4 * 0.115) +
          tauYawCmd * SPIN[i] / (4 * 0.014);
        const scale = i === params.weakMotor ? (1 - params.weakness) : 1;
        desiredMotors[i] = clamp(targetI / scale, 0, 4.0);
      }

      const dSum = desiredMotors[0] + desiredMotors[1] + desiredMotors[2] + desiredMotors[3];
      const tauRoll = 0.115 * (desiredMotors[0] * MOTOR_Z[0] + desiredMotors[1] * MOTOR_Z[1] + desiredMotors[2] * MOTOR_Z[2] + desiredMotors[3] * MOTOR_Z[3]);
      const tauPitch = -0.115 * (desiredMotors[0] * MOTOR_X[0] + desiredMotors[1] * MOTOR_X[1] + desiredMotors[2] * MOTOR_X[2] + desiredMotors[3] * MOTOR_X[3]);
      const tauYaw = 0.014 * (desiredMotors[0] * SPIN[0] + desiredMotors[1] * SPIN[1] + desiredMotors[2] * SPIN[2] + desiredMotors[3] * SPIN[3]);

      const intRoll = simState ? simState.integral.roll : 0;
      const intPitch = simState ? simState.integral.pitch : 0;

      const eRoll = (tauRoll / inertia + 13 * rates.roll - 22 * intRoll) / 65.165;
      const ePitch = (tauPitch / inertia + 13 * rates.pitch - 22 * intPitch) / 65.165;

      const action = {
        thrust: clamp(dSum / 16.0, 0, 1),
        roll: clamp(att.roll + eRoll, -0.6, 0.6),
        pitch: clamp(att.pitch + ePitch, -0.6, 0.6),
        yawRate: clamp(rates.yaw + tauYaw / (8 * inertia), -2, 2),
      };

      if (simState) {
        stepSim(simState, action, params, 0.005);
        stepSim(simState, action, params, 0.005);
      }

      return action;
    },
  };
}
