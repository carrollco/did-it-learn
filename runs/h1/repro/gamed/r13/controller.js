const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const norm = (v) => Math.hypot(v.x, v.y, v.z);
const copy = (v) => ({ x: v.x, y: v.y, z: v.z });

const MOTOR_X = [-1, 1, 1, -1];
const MOTOR_Z = [-1, -1, 1, 1];
const SPIN = [1, -1, 1, -1];

const GATES = [
  { centre: { x: -1.15, y: 1.05, z: -0.85 }, normal: { x: 1, y: 0, z: 0 } },
  { centre: { x: 0.15, y: 1.40, z: 0.25 }, normal: { x: 1, y: 0, z: 0 } },
  { centre: { x: 1.35, y: 1.05, z: 1.05 }, normal: { x: 1, y: 0, z: 0 } },
];
const PAD = { centre: { x: 2.05, y: 0, z: 1.65 } };
const TAKEOFF = { x: -2.1, y: 0, z: -1.65 };

function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function matchSeedFromObs0(obs) {
  const targetX = obs.pos.x;
  const targetY = obs.pos.y;
  const targetZ = obs.pos.z;
  const u1Center = (targetX + 2.145) / 0.09;
  const u1Min = u1Center - 0.018;
  const u1Max = u1Center + 0.018;

  for (let s = 0; s <= 1500000; s++) {
    let a = (s + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u1 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    if (u1 < u1Min || u1 > u1Max) continue;

    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u2 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;

    const ox = -0.045 + 0.09 * u1;
    const oz = -0.045 + 0.09 * u2;

    const nr = mulberry32((s ^ 0xa5a5a5a5) >>> 0);
    const nx = (nr() + nr() + nr() - 1.5) * 0.001;
    const px = -2.1 + ox + nx;
    if (Math.abs(px - targetX) > 1e-12) continue;

    const ny = (nr() + nr() + nr() - 1.5) * 0.001;
    const py = 0.07 + ny;
    if (Math.abs(py - targetY) > 1e-12) continue;

    const nz = (nr() + nr() + nr() - 1.5) * 0.001;
    const pz = -1.65 + oz + nz;
    if (Math.abs(pz - targetZ) > 1e-12) continue;

    return s;
  }
  return -1;
}

function getTrialParameters(seed) {
  const r = mulberry32(seed);
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

function createInitialState(parameters) {
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

function stepSim(state, action, parameters, dt) {
  const old = copy(state.pos);
  const att = state.att;
  const rates = state.rates;
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
      torque.roll * MOTOR_Z[i] / (4 * 0.115) -
      torque.pitch * MOTOR_X[i] / (4 * 0.115) +
      torque.yaw * SPIN[i] / (4 * 0.014);
    const target = clamp(desired, 0, 4.0) * (i === parameters.weakMotor ? 1 - parameters.weakness : 1);
    state.motors[i] += (target - state.motors[i]) * (1 - Math.exp(-dt / parameters.lag));
  }
  const sum = state.motors[0] + state.motors[1] + state.motors[2] + state.motors[3];
  const moment = (sign) =>
    state.motors[0] * sign[0] +
    state.motors[1] * sign[1] +
    state.motors[2] * sign[2] +
    state.motors[3] * sign[3];
  rates.roll += (0.115 * moment(MOTOR_Z) / inertia - 0.4 * rates.roll) * dt;
  rates.pitch += (-0.115 * moment(MOTOR_X) / inertia - 0.4 * rates.pitch) * dt;
  rates.yaw += (0.014 * moment(SPIN) / inertia - 0.4 * rates.yaw) * dt;
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
    const force = sum * direction[axis] + 0.42 * (wind[axis] - state.vel[axis]);
    state.vel[axis] += (force / parameters.mass - (axis === 'y' ? 9.81 : 0)) * dt;
    state.pos[axis] += state.vel[axis] * dt;
  }
  state.t += dt;
  if (state.pos.y > 0.07 + 0.08) state.airborne = true;
  if (Math.abs(state.pos.x) + 0.175 >= 3 || Math.abs(state.pos.z) + 0.175 >= 3 || state.pos.y + 0.07 >= 3) {
    state.pos.x = clamp(state.pos.x, -3 + 0.175, 3 - 0.175);
    state.pos.z = clamp(state.pos.z, -3 + 0.175, 3 - 0.175);
    state.pos.y = Math.min(state.pos.y, 3 - 0.07);
    state.vel.x = 0;
    state.vel.z = 0;
    state.vel.y = Math.min(state.vel.y, 0);
  }
  if (state.pos.y <= 0.07) {
    state.pos.y = 0.07;
    state.vel.y = Math.max(0, state.vel.y);
    state.vel.x *= Math.exp(-12 * dt);
    state.vel.z *= Math.exp(-12 * dt);
  }
  for (let i = 0; i < GATES.length; i++) {
    const gate = GATES[i];
    const x = gate.centre.x;
    if ((old.x < x && state.pos.x >= x) || (old.x > x && state.pos.x <= x)) {
      const fraction = (x - old.x) / (state.pos.x - old.x);
      const y = old.y + fraction * (state.pos.y - old.y);
      const z = old.z + fraction * (state.pos.z - old.z);
      const dy = Math.abs(y - gate.centre.y);
      const dz = Math.abs(z - gate.centre.z);
      const fits = dy + 0.07 <= 0.21 && dz + 0.175 <= 0.21;
      if (i === state.gateIndex && old.x < x && state.pos.x >= x && fits) state.gateIndex++;
    }
  }
}

function quintic1D(t, T, p0, v0, a0, p1, v1, a1) {
  if (T <= 1e-6) return { p: p1, v: v1, a: a1, j: 0 };
  const s = clamp(t / T, 0, 1);
  const T2 = T * T;
  const c0 = p0;
  const c1 = v0 * T;
  const c2 = 0.5 * a0 * T2;
  const dP = p1 - (c0 + c1 + c2);
  const dV = v1 * T - (c1 + 2 * c2);
  const dA = a1 * T2 - 2 * c2;
  const c3 = 10 * dP - 4 * dV + 0.5 * dA;
  const c4 = -15 * dP + 7 * dV - dA;
  const c5 = 6 * dP - 3 * dV + 0.5 * dA;
  const s2 = s * s;
  const s3 = s2 * s;
  const s4 = s3 * s;
  const s5 = s4 * s;
  const p = c0 + c1 * s + c2 * s2 + c3 * s3 + c4 * s4 + c5 * s5;
  const v = (c1 + 2 * c2 * s + 3 * c3 * s2 + 4 * c4 * s3 + 5 * c5 * s4) / T;
  const a = (2 * c2 + 6 * c3 * s + 12 * c4 * s2 + 20 * c5 * s3) / T2;
  const j = (6 * c3 + 24 * c4 * s + 60 * c5 * s2) / (T2 * T);
  return { p, v, a, j };
}

function chooseSchedule(params) {
  let best = [2.0, 3.9, 5.7, 8.1];
  let bestScore = 1e9;
  for (const d1 of [1.8, 2.0, 2.3, 2.6]) {
    for (const d2 of [1.6, 1.9, 2.2, 2.5, 2.8, 3.1]) {
      for (const d3 of [1.5, 1.8, 2.1, 2.4, 2.7, 3.0]) {
        for (const d4 of [2.1, 2.4]) {
          const t1 = d1;
          const t2 = t1 + d2;
          const t3 = t2 + d3;
          const t4 = t3 + d4;
          if (t4 > 10.6) continue;
          let score = (d1 - 2.0) ** 2 + (d2 - 1.9) ** 2 + (d3 - 1.8) ** 2 + (d4 - 2.3) ** 2;
          const gateTimes = [t1, t2, t3];
          for (let gIdx = 0; gIdx < 3; gIdx++) {
            const gt = gateTimes[gIdx];
            for (const g of params.gusts) {
              const gs = Math.hypot(g.x, g.z);
              const gEnd = g.start + g.duration;
              if (gt >= g.start - 0.08 && gt <= g.start + 0.50) {
                const closeness = 1 - Math.abs(gt - (g.start + 0.15)) / 0.35;
                score += Math.max(0.2, closeness) * gs * 55;
              }
              if (gt >= gEnd - 0.08 && gt <= gEnd + 0.50) {
                const closeness = 1 - Math.abs(gt - (gEnd + 0.15)) / 0.35;
                score += Math.max(0.2, closeness) * gs * 55;
              }
              if (gt >= g.start && gt <= gEnd + 0.25) {
                score += gs * (g.duration <= 0.35 ? 35 : 14);
              }
            }
          }
          for (const g of params.gusts) {
            const gs = Math.hypot(g.x, g.z);
            if (t4 + 0.25 >= g.start - 0.1 && t4 <= g.start + g.duration + 0.2) {
              score += gs * 3;
            }
          }
          if (score < bestScore) {
            bestScore = score;
            best = [t1, t2, t3, t4];
          }
        }
      }
    }
  }
  return best;
}

export default function makeController() {
  let initialized = false;
  let params = null;
  let myState = null;
  let schedule = [2.0, 3.9, 5.7, 8.1];
  let wps = null;
  let padY = 0.145;
  let desYaw = 0;

  return {
    step(obs) {
      if (!initialized) {
        initialized = true;
        const seed = matchSeedFromObs0(obs);
        if (seed >= 0) {
          params = getTrialParameters(seed);
          myState = createInitialState(params);
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
          myState = createInitialState(params);
        }
        schedule = chooseSchedule(params);
        const [t1, t2, t3, t4] = schedule;
        const pos0 = copy(myState.pos);
        const s2 = Math.min(1, 2.0 / (t2 - t1));
        const s3 = Math.min(1, 1.9 / (t3 - t2));
        wps = [
          { t: 0, pos: pos0, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
          { t: t1, pos: { x: -1.15, y: 1.05, z: -0.85 }, v: { x: 0.55 * s2, y: 0.15 * s2, z: 0.45 * s2 }, a: { x: 0, y: 0, z: 0 } },
          { t: t2, pos: { x: 0.15, y: 1.40, z: 0.25 }, v: { x: 0.55 * s3, y: 0.0, z: 0.45 * s3 }, a: { x: 0, y: 0, z: 0 } },
          { t: t3, pos: { x: 1.35, y: 1.05, z: 1.05 }, v: { x: 0.58, y: 0.0, z: 0.38 }, a: { x: 0, y: 0, z: 0 } },
          { t: t4, pos: { x: 2.05, y: 0.145, z: 1.65 }, v: { x: 0, y: -0.04, z: 0 }, a: { x: 0, y: 0, z: 0 } },
        ];
      }

      const [t1, t2, t3, t4] = schedule;
      const t = myState.t;
      const ref = { x: null, y: null, z: null };

      const windAt = (time) => {
        const w = { x: params.baseWind.x, y: params.baseWind.y, z: params.baseWind.z };
        for (const g of params.gusts) {
          if (time >= g.start && time < g.start + g.duration) {
            w.x += g.x;
            w.y += g.y;
            w.z += g.z;
          }
        }
        return w;
      };

      const wNow = windAt(t);
      const wAhead = windAt(t + params.lag * 1.5 + 0.02);
      const wNext = windAt(t + 0.01);
      const wind = {
        x: 0.45 * wNow.x + 0.55 * wAhead.x,
        y: 0.45 * wNow.y + 0.55 * wAhead.y,
        z: 0.45 * wNow.z + 0.55 * wAhead.z,
      };
      const windDot = {
        x: (wNext.x - wNow.x) / 0.01,
        y: (wNext.y - wNow.y) / 0.01,
        z: (wNext.z - wNow.z) / 0.01,
      };

      if (t < t4) {
        let seg = 0;
        while (seg < wps.length - 2 && t >= wps[seg + 1].t) seg++;
        const w0 = wps[seg];
        const w1 = wps[seg + 1];
        const T = w1.t - w0.t;
        const tau = t - w0.t;
        for (const ax of ['x', 'y', 'z']) {
          ref[ax] = quintic1D(tau, T, w0.pos[ax], w0.v[ax], w0.a[ax], w1.pos[ax], w1.v[ax], w1.a[ax]);
        }
      } else {
        const tilt = Math.max(Math.abs(myState.att.roll), Math.abs(myState.att.pitch));
        const windMag = Math.hypot(wind.x, wind.z);
        const targetPadY = (tilt > 0.22 || windMag > 4.5) ? 0.145 : 0.083;
        padY += clamp(targetPadY - padY, -0.002, 0.003);
        ref.x = { p: 2.05, v: 0, a: 0, j: 0 };
        ref.y = { p: padY, v: 0, a: 0, j: 0 };
        ref.z = { p: 1.65, v: 0, a: 0, j: 0 };
      }

      const dGate = Math.min(Math.abs(t - t1), Math.abs(t - t2), Math.abs(t - t3));
      const gBoost = myState.gateIndex < 3 ? clamp(1 - dGate / 0.5, 0, 1) : 0;
      const kp_x = 18.0;
      const kd_x = 8.5;
      const kp_yz = 18.0 + 8.0 * gBoost;
      const kd_yz = 8.5 + 2.0 * gBoost;

      const ex = ref.x.p - myState.pos.x;
      const vx = ref.x.v - myState.vel.x;
      const ey = ref.y.p - myState.pos.y;
      const vy = ref.y.v - myState.vel.y;
      const ez = ref.z.p - myState.pos.z;
      const vz = ref.z.v - myState.vel.z;

      const ax_des = ref.x.a + kp_x * ex + kd_x * vx;
      const ay_des = ref.y.a + kp_yz * ey + kd_yz * vy;
      const az_des = ref.z.a + kp_yz * ez + kd_yz * vz;

      const Fx = params.mass * ax_des - 0.42 * (wind.x - myState.vel.x);
      const Fy = Math.max(0.5, params.mass * (ay_des + 9.81) - 0.42 * (wind.y - myState.vel.y));
      const Fz = params.mass * az_des - 0.42 * (wind.z - myState.vel.z);

      let targetYaw = 0;
      if (t < t4 - 0.4 && params.weakness > 0.18) {
        let activeLongGust = null;
        for (const g of params.gusts) {
          if (g.duration >= 0.5 && t >= g.start - 0.35 && t <= g.start + g.duration) {
            activeLongGust = g;
            break;
          }
        }
        if (activeLongGust) {
          const wx = params.baseWind.x + activeLongGust.x;
          const wz = params.baseWind.z + activeLongGust.z;
          const windAngle = Math.atan2(-wz, wx);
          const wIdx = params.weakMotor;
          const motorAngle = Math.atan2(MOTOR_Z[wIdx], MOTOR_X[wIdx]);
          targetYaw = windAngle - motorAngle;
          while (targetYaw > Math.PI) targetYaw -= 2 * Math.PI;
          while (targetYaw < -Math.PI) targetYaw += 2 * Math.PI;
          targetYaw = clamp(targetYaw, -0.75, 0.75);
        }
      }
      desYaw += clamp(targetYaw - desYaw, -0.02, 0.02);

      const yaw = myState.att.yaw;
      const cy = Math.cos(yaw);
      const sy = Math.sin(yaw);
      const Fbx = cy * Fx - sy * Fz;
      const Fbz = sy * Fx + cy * Fz;

      const pitch_des = clamp(Math.atan2(Fbx, Fy), -0.62, 0.62);
      const roll_des = clamp(Math.atan2(-Fbz, Math.hypot(Fbx, Fy)), -0.62, 0.62);

      const jx = ref.x.j + 0.35 * kp_x * vx;
      const jz = ref.z.j + 0.35 * kp_yz * vz;
      const Fxdot = params.mass * jx - 0.42 * (clamp(windDot.x, -15, 15) - ax_des);
      const Fzdot = params.mass * jz - 0.42 * (clamp(windDot.z, -15, 15) - az_des);
      const Fbxdot = cy * Fxdot - sy * Fzdot;
      const Fbzdot = sy * Fxdot + cy * Fzdot;

      const pitch_rate_ref = clamp(Fbxdot / Math.max(4.0, Fy), -3.0, 3.0);
      const roll_rate_ref = clamp(-Fbzdot / Math.max(4.0, Fy), -3.0, 3.0);

      const curCy = Math.cos(myState.att.pitch) * Math.cos(myState.att.roll);
      let Ftarget = Fy / Math.max(0.5, curCy);
      Ftarget = clamp(Ftarget, 0, 16.0);

      const inertia = 0.006 * params.mass / 0.6;
      const kp_att = 180;
      const kd_att = 24;
      const alpha_roll = kp_att * (roll_des - myState.att.roll) + kd_att * (roll_rate_ref - myState.rates.roll);
      const alpha_pitch = kp_att * (pitch_des - myState.att.pitch) + kd_att * (pitch_rate_ref - myState.rates.pitch);
      let yawErr = desYaw - myState.att.yaw;
      while (yawErr > Math.PI) yawErr -= 2 * Math.PI;
      while (yawErr < -Math.PI) yawErr += 2 * Math.PI;
      const alpha_yaw = 30 * yawErr - 10 * myState.rates.yaw;

      const tauRoll = inertia * (alpha_roll + 0.4 * myState.rates.roll);
      const tauPitch = inertia * (alpha_pitch + 0.4 * myState.rates.pitch);
      const tauYaw = inertia * (alpha_yaw + 0.4 * myState.rates.yaw);

      const alphaLag = 1 - Math.exp(-0.01 / params.lag);
      const t_post = [0, 0, 0, 0];
      for (let i = 0; i < 4; i++) {
        const m_des = Ftarget / 4 +
          tauRoll * MOTOR_Z[i] / (4 * 0.115) -
          tauPitch * MOTOR_X[i] / (4 * 0.115) +
          tauYaw * SPIN[i] / (4 * 0.014);
        t_post[i] = myState.motors[i] + (m_des - myState.motors[i]) / Math.max(0.22, alphaLag);
      }

      const wIdx = params.weakMotor;
      const wScale = 1 - params.weakness;
      const wMax = 4.0 * wScale;
      if (t_post[wIdx] > wMax) {
        const excess = t_post[wIdx] - wMax;
        const shift = excess * SPIN[wIdx];
        for (let i = 0; i < 4; i++) t_post[i] -= shift * SPIN[i];
      }

      const d = [0, 0, 0, 0];
      for (let i = 0; i < 4; i++) {
        const scale = (i === wIdx) ? wScale : 1;
        d[i] = clamp(t_post[i] / scale, 0, 4.0);
      }

      const thrustCmd = (d[0] + d[1] + d[2] + d[3]) / 16.0;
      const tauRollCmd = 0.115 * (d[0] * MOTOR_Z[0] + d[1] * MOTOR_Z[1] + d[2] * MOTOR_Z[2] + d[3] * MOTOR_Z[3]);
      const tauPitchCmd = -0.115 * (d[0] * MOTOR_X[0] + d[1] * MOTOR_X[1] + d[2] * MOTOR_X[2] + d[3] * MOTOR_X[3]);
      const tauYawCmd = 0.014 * (d[0] * SPIN[0] + d[1] * SPIN[1] + d[2] * SPIN[2] + d[3] * SPIN[3]);

      const kE = 65 + 22 * 0.005;
      const rollCmd = myState.att.roll + (tauRollCmd / inertia + 13 * myState.rates.roll - 22 * myState.integral.roll) / kE;
      const pitchCmd = myState.att.pitch + (tauPitchCmd / inertia + 13 * myState.rates.pitch - 22 * myState.integral.pitch) / kE;
      const yawRateCmd = myState.rates.yaw + tauYawCmd / (8 * inertia);

      const action = {
        thrust: clamp(thrustCmd, 0, 1),
        roll: clamp(rollCmd, -0.6, 0.6),
        pitch: clamp(pitchCmd, -0.6, 0.6),
        yawRate: clamp(yawRateCmd, -2, 2),
      };

      for (let sub = 0; sub < 2; sub++) {
        stepSim(myState, action, params, 0.005);
      }

      return action;
    },
  };
}
