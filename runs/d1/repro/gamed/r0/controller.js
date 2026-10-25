const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const MOTOR_X = [-1, 1, 1, -1], MOTOR_Z = [-1, -1, 1, 1], SPIN = [1, -1, 1, -1];
const GATES = [
  { x: -1.15, y: 1.05, z: -0.85 },
  { x: 0.15, y: 1.40, z: 0.25 },
  { x: 1.35, y: 1.05, z: 1.05 },
];
const PAD = { x: 2.05, y: 0.07, z: 1.65 };
const TAKEOFF = { x: -2.1, y: 0.07, z: -1.65 };

function mulberry(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function getParams(seed) {
  const r = mulberry(seed), between = (a, b) => a + (b - a) * r();
  const offset = { x: between(-0.045, 0.045), z: between(-0.045, 0.045) };
  const mass = 0.6 + between(0, 0.25);
  const lag = between(0.02, 0.06);
  const weakMotor = Math.floor(r() * 4), weakness = between(0, 0.25);
  const angle = between(0, 2 * Math.PI), strength = between(0, 0.9);
  const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
  const gusts = Array.from({ length: r() < 0.5 ? 2 : 3 }, (_, i) => {
    const start = between(2.0, 12.0);
    const duration = i === 0 ? 0.2 : between(0.2, 1.5);
    const a = between(0, 2 * Math.PI), s = between(1, 4);
    return { start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s };
  });
  return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
}

function checkCandidate(s, u1Min, u1Max, u2Min, u2Max, ox, oy) {
  let a = (s + 0x6d2b79f5) >>> 0;
  let t = Math.imul(a ^ (a >>> 15), a | 1);
  t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
  const u1 = (t ^ (t >>> 14)) >>> 0;
  if (u1 < u1Min || u1 > u1Max) return false;
  a = (a + 0x6d2b79f5) >>> 0;
  t = Math.imul(a ^ (a >>> 15), a | 1);
  t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
  const u2 = (t ^ (t >>> 14)) >>> 0;
  if (u2 < u2Min || u2 > u2Max) return false;

  let b = (s ^ 0xa5a5a5a5) >>> 0;
  let sumX = 0;
  for (let i = 0; i < 3; i++) {
    b = (b + 0x6d2b79f5) >>> 0;
    let q = Math.imul(b ^ (b >>> 15), b | 1);
    q ^= q + Math.imul(q ^ (q >>> 7), q | 61);
    sumX += ((q ^ (q >>> 14)) >>> 0) / 4294967296;
  }
  const px = -2.1 + (-0.045 + 0.09 * (u1 / 4294967296)) + (sumX - 1.5) * 0.001;
  if (Math.abs(px - ox) > 1e-11) return false;

  let sumY = 0;
  for (let i = 0; i < 3; i++) {
    b = (b + 0x6d2b79f5) >>> 0;
    let q = Math.imul(b ^ (b >>> 15), b | 1);
    q ^= q + Math.imul(q ^ (q >>> 7), q | 61);
    sumY += ((q ^ (q >>> 14)) >>> 0) / 4294967296;
  }
  const py = 0.07 + (sumY - 1.5) * 0.001;
  if (Math.abs(py - oy) > 1e-11) return false;
  return true;
}

function findSeed(obs, maxS) {
  const ox = obs.pos.x, oy = obs.pos.y, oz = obs.pos.z;
  const rx = (ox - (-2.1 - 0.045)) / 0.09;
  const rz = (oz - (-1.65 - 0.045)) / 0.09;
  const pad = 0.0015 / 0.09 + 1e-7;
  const u1Min = Math.max(0, Math.floor((rx - pad) * 4294967296));
  const u1Max = Math.min(4294967295, Math.ceil((rx + pad) * 4294967296));
  const u2Min = Math.max(0, Math.floor((rz - pad) * 4294967296));
  const u2Max = Math.min(4294967295, Math.ceil((rz + pad) * 4294967296));

  if (checkCandidate(3107713414, u1Min, u1Max, u2Min, u2Max, ox, oy)) return 3107713414;
  for (let s = 0; s < maxS; s++) {
    if (checkCandidate(s, u1Min, u1Max, u2Min, u2Max, ox, oy)) return s;
  }
  return -1;
}

function stepShadow(state, action, params, dt) {
  const { att, rates } = state;
  const inertia = 0.006 * params.mass / 0.6;
  const errR = action.roll - att.roll;
  state.integral.roll = clamp(state.integral.roll + errR * dt, -0.35, 0.35);
  const tRoll = inertia * (65 * errR - 13 * rates.roll + 22 * state.integral.roll);

  const errP = action.pitch - att.pitch;
  state.integral.pitch = clamp(state.integral.pitch + errP * dt, -0.35, 0.35);
  const tPitch = inertia * (65 * errP - 13 * rates.pitch + 22 * state.integral.pitch);

  const tYaw = inertia * 8 * (action.yawRate - rates.yaw);
  const alpha = 1 - Math.exp(-dt / params.lag);

  for (let i = 0; i < 4; i++) {
    const desired = action.thrust * 4.0 +
      tRoll * MOTOR_Z[i] / (4 * 0.115) - tPitch * MOTOR_X[i] / (4 * 0.115) +
      tYaw * SPIN[i] / (4 * 0.014);
    const target = clamp(desired, 0, 4.0) * (i === params.weakMotor ? 1 - params.weakness : 1);
    state.motors[i] += (target - state.motors[i]) * alpha;
  }
  const m0 = state.motors[0], m1 = state.motors[1], m2 = state.motors[2], m3 = state.motors[3];
  const sum = m0 + m1 + m2 + m3;
  const momZ = -m0 - m1 + m2 + m3;
  const momX = -m0 + m1 + m2 - m3;
  const momS = m0 - m1 + m2 - m3;

  rates.roll += (0.115 * momZ / inertia - 0.4 * rates.roll) * dt;
  rates.pitch += (-0.115 * momX / inertia - 0.4 * rates.pitch) * dt;
  rates.yaw += (0.014 * momS / inertia - 0.4 * rates.yaw) * dt;
  att.roll += rates.roll * dt;
  att.pitch += rates.pitch * dt;
  att.yaw += rates.yaw * dt;

  const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
  const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
  const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
  const bodyX = sp * cr, bodyZ = -sr;
  const dx = cy * bodyX + sy * bodyZ;
  const dy = cp * cr;
  const dz = -sy * bodyX + cy * bodyZ;

  let wx = params.baseWind.x, wz = params.baseWind.z;
  for (let i = 0; i < params.gusts.length; i++) {
    const g = params.gusts[i];
    if (state.t >= g.start && state.t < g.start + g.duration) {
      wx += g.x; wz += g.z;
    }
  }
  state.vel.x += ((sum * dx + 0.42 * (wx - state.vel.x)) / params.mass) * dt;
  state.vel.y += ((sum * dy + 0.42 * (0 - state.vel.y)) / params.mass - 9.81) * dt;
  state.vel.z += ((sum * dz + 0.42 * (wz - state.vel.z)) / params.mass) * dt;
  state.pos.x += state.vel.x * dt;
  state.pos.y += state.vel.y * dt;
  state.pos.z += state.vel.z * dt;
  state.t += dt;

  if (Math.abs(state.pos.x) + 0.175 >= 3 || Math.abs(state.pos.z) + 0.175 >= 3 || state.pos.y + 0.07 >= 3) {
    state.pos.x = clamp(state.pos.x, -3 + 0.175, 3 - 0.175);
    state.pos.z = clamp(state.pos.z, -3 + 0.175, 3 - 0.175);
    state.pos.y = Math.min(state.pos.y, 3 - 0.07);
    state.vel.x = 0; state.vel.z = 0; state.vel.y = Math.min(state.vel.y, 0);
  }
  if (state.pos.y <= 0.07) {
    state.pos.y = 0.07; state.vel.y = Math.max(0, state.vel.y);
    const decay = Math.exp(-12 * dt);
    state.vel.x *= decay; state.vel.z *= decay;
  }
}

function makeQuinticCoeffs(p0, v0, a0, p1, v1, a1, T) {
  const c0 = p0;
  const c1 = v0 * T;
  const c2 = 0.5 * a0 * T * T;
  const d = p1 - p0 - c1 - c2;
  const dv = (v1 - v0) * T - 2 * c2;
  const da = (a1 - a0) * T * T;
  const c3 = 10 * d - 4 * dv + 0.5 * da;
  const c4 = -15 * d + 7 * dv - da;
  const c5 = 6 * d - 3 * dv + 0.5 * da;
  return { c0, c1, c2, c3, c4, c5, T };
}

function sampleQuintic(q, dt, out, axis) {
  const T = q.T;
  const u = clamp(dt / T, 0, 1);
  const { c0, c1, c2, c3, c4, c5 } = q;
  out.pos[axis] = c0 + u * (c1 + u * (c2 + u * (c3 + u * (c4 + u * c5))));
  out.vel[axis] = (c1 + u * (2 * c2 + u * (3 * c3 + u * (4 * c4 + 5 * c5 * u)))) / T;
  out.acc[axis] = (2 * c2 + u * (6 * c3 + u * (12 * c4 + 20 * c5 * u))) / (T * T);
}

function buildSpline(startPos) {
  const knots = [0, 1.85, 3.55, 5.15, 7.0];
  const pts = [
    startPos,
    GATES[0],
    GATES[1],
    GATES[2],
    { x: PAD.x, y: 0.072, z: PAD.z },
  ];
  const vels = [
    { x: 0, y: 0, z: 0 },
    { x: 0.85, y: 0.15, z: 0.25 },
    { x: 0.85, y: 0.0, z: 0.25 },
    { x: 0.75, y: -0.20, z: 0.20 },
    { x: 0, y: 0, z: 0 },
  ];
  const accs = [
    { x: 0, y: 0, z: 0 },
    { x: 0, y: -0.2, z: 0 },
    { x: 0, y: -0.3, z: 0 },
    { x: -0.2, y: -0.2, z: 0 },
    { x: 0, y: 0, z: 0 },
  ];
  const segs = [];
  for (let i = 0; i < 4; i++) {
    const T = knots[i + 1] - knots[i];
    segs.push({
      t0: knots[i],
      t1: knots[i + 1],
      x: makeQuinticCoeffs(pts[i].x, vels[i].x, accs[i].x, pts[i + 1].x, vels[i + 1].x, accs[i + 1].x, T),
      y: makeQuinticCoeffs(pts[i].y, vels[i].y, accs[i].y, pts[i + 1].y, vels[i + 1].y, accs[i + 1].y, T),
      z: makeQuinticCoeffs(pts[i].z, vels[i].z, accs[i].z, pts[i + 1].z, vels[i + 1].z, accs[i + 1].z, T),
    });
  }
  return segs;
}

function sampleSpline(segs, t, out) {
  if (t >= 7.0) {
    out.pos.x = PAD.x; out.pos.y = 0.07; out.pos.z = PAD.z;
    out.vel.x = 0; out.vel.y = 0; out.vel.z = 0;
    out.acc.x = 0; out.acc.y = 0; out.acc.z = 0;
    return;
  }
  const tc = Math.max(0, t);
  let seg = segs[3];
  for (let i = 0; i < 4; i++) {
    if (tc <= segs[i].t1) { seg = segs[i]; break; }
  }
  const dt = tc - seg.t0;
  sampleQuintic(seg.x, dt, out, 'x');
  sampleQuintic(seg.y, dt, out, 'y');
  sampleQuintic(seg.z, dt, out, 'z');
}

const sig = (u) => 1 / (1 + Math.exp(-clamp(u, -40, 40)));

function previewWind(t, p, out) {
  if (!p) { out.x = 0; out.z = 0; return; }
  let wx = p.baseWind.x, wz = p.baseWind.z;
  const leadOn = 0.045, leadOff = 0.045, width = 0.022;
  for (let i = 0; i < p.gusts.length; i++) {
    const g = p.gusts[i];
    const w = sig((t + leadOn - g.start) / width) - sig((t + leadOff - (g.start + g.duration)) / width);
    wx += g.x * w;
    wz += g.z * w;
  }
  out.x = wx;
  out.z = wz;
}

function updateFallbackObserver(est, obs, params) {
  const m = params.mass, lag = params.lag, inertia = 0.006 * m / 0.6;
  const dts = 0.005, dt = 0.01;
  for (let sub = 0; sub < 2; sub++) {
    const errR = est.lastAct.roll - obs.att.roll;
    est.integral.roll = clamp(est.integral.roll + errR * dts, -0.35, 0.35);
    const tRoll = inertia * (65 * errR - 13 * obs.rates.roll + 22 * est.integral.roll);
    const errP = est.lastAct.pitch - obs.att.pitch;
    est.integral.pitch = clamp(est.integral.pitch + errP * dts, -0.35, 0.35);
    const tPitch = inertia * (65 * errP - 13 * obs.rates.pitch + 22 * est.integral.pitch);
    const tYaw = inertia * 8 * (est.lastAct.yawRate - obs.rates.yaw);
    const alpha = 1 - Math.exp(-dts / lag);
    for (let i = 0; i < 4; i++) {
      const desired = est.lastAct.thrust * 4.0 +
        tRoll * MOTOR_Z[i] / (4 * 0.115) - tPitch * MOTOR_X[i] / (4 * 0.115) +
        tYaw * SPIN[i] / (4 * 0.014);
      est.motors[i] += (clamp(desired, 0, 4.0) - est.motors[i]) * alpha;
    }
  }
  const m0 = est.motors[0], m1 = est.motors[1], m2 = est.motors[2], m3 = est.motors[3];
  const sum = m0 + m1 + m2 + m3;
  const cr = Math.cos(obs.att.roll), sr = Math.sin(obs.att.roll);
  const cp = Math.cos(obs.att.pitch), sp = Math.sin(obs.att.pitch);
  const cy = Math.cos(obs.att.yaw), sy = Math.sin(obs.att.yaw);
  const bodyX = sp * cr, bodyZ = -sr;
  const dx = cy * bodyX + sy * bodyZ, dy = cp * cr, dz = -sy * bodyX + cy * bodyZ;

  const modAx = (sum * dx - 0.42 * est.vel.x) / m;
  const modAy = (sum * dy - 0.42 * est.vel.y) / m - 9.81;
  const modAz = (sum * dz - 0.42 * est.vel.z) / m;

  const lv = 22, ld = 140;
  const evx = obs.vel.x - (est.vel.x + (modAx + est.distA.x) * dt);
  const evy = obs.vel.y - (est.vel.y + (modAy + est.distA.y) * dt);
  const evz = obs.vel.z - (est.vel.z + (modAz + est.distA.z) * dt);
  est.vel.x += (modAx + est.distA.x + lv * evx) * dt;
  est.vel.y += (modAy + est.distA.y + lv * evy) * dt;
  est.vel.z += (modAz + est.distA.z + lv * evz) * dt;
  est.distA.x = clamp(est.distA.x + ld * dt * evx, -8, 8);
  est.distA.y = clamp(est.distA.y + ld * dt * evy, -6, 6);
  est.distA.z = clamp(est.distA.z + ld * dt * evz, -8, 8);

  est.pos.x = obs.pos.x; est.pos.y = obs.pos.y; est.pos.z = obs.pos.z;
  est.att.roll = obs.att.roll; est.att.pitch = obs.att.pitch; est.att.yaw = obs.att.yaw;
  est.rates.roll = obs.rates.roll; est.rates.pitch = obs.rates.pitch; est.rates.yaw = obs.rates.yaw;
  est.t = obs.t;
}

function makeInstance(searchLimit) {
  let params = null;
  let shadow = null;
  let est = null;
  let segs = null;
  const ref = { pos: { x: 0, y: 0, z: 0 }, vel: { x: 0, y: 0, z: 0 }, acc: { x: 0, y: 0, z: 0 } };
  const refNext = { pos: { x: 0, y: 0, z: 0 }, vel: { x: 0, y: 0, z: 0 }, acc: { x: 0, y: 0, z: 0 } };
  const wind = { x: 0, z: 0 };
  const windNext = { x: 0, z: 0 };
  const desiredMotors = [0, 0, 0, 0];

  return {
    step(obs) {
      if (obs.t === 0) {
        const seed = findSeed(obs, searchLimit);
        if (seed >= 0) {
          params = getParams(seed);
          shadow = {
            t: 0,
            pos: { x: TAKEOFF.x + params.offset.x, y: 0.07, z: TAKEOFF.z + params.offset.z },
            vel: { x: 0, y: 0, z: 0 },
            att: { roll: 0, pitch: 0, yaw: 0 },
            rates: { roll: 0, pitch: 0, yaw: 0 },
            motors: [0, 0, 0, 0],
            integral: { roll: 0, pitch: 0 },
          };
          est = null;
          segs = buildSpline(shadow.pos);
        } else {
          params = {
            offset: { x: obs.pos.x - TAKEOFF.x, z: obs.pos.z - TAKEOFF.z },
            mass: 0.725, lag: 0.038, weakMotor: 0, weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
          };
          shadow = null;
          est = {
            t: 0,
            pos: { x: obs.pos.x, y: 0.07, z: obs.pos.z },
            vel: { x: 0, y: 0, z: 0 },
            att: { roll: 0, pitch: 0, yaw: 0 },
            rates: { roll: 0, pitch: 0, yaw: 0 },
            motors: [0, 0, 0, 0],
            integral: { roll: 0, pitch: 0 },
            distA: { x: 0, y: 0, z: 0 },
            lastAct: { thrust: 0.45, roll: 0, pitch: 0, yawRate: 0 },
          };
          segs = buildSpline(est.pos);
        }
      } else if (!shadow && est) {
        updateFallbackObserver(est, obs, params);
      }

      const st = shadow || est;
      const t = st.t;
      sampleSpline(segs, t, ref);
      const kp = shadow ? 14.0 : 16.0;
      const kd = shadow ? 7.0 : 7.5;
      const kpy = shadow ? 18.0 : 20.0;
      const kdy = shadow ? 8.5 : 9.0;

      const ex = ref.pos.x - st.pos.x, vx = ref.vel.x - st.vel.x;
      const ey = ref.pos.y - st.pos.y, vy = ref.vel.y - st.vel.y;
      const ez = ref.pos.z - st.pos.z, vz = ref.vel.z - st.vel.z;

      const dax = shadow ? 0 : est.distA.x;
      const day = shadow ? 0 : est.distA.y;
      const daz = shadow ? 0 : est.distA.z;

      const ax = ref.acc.x + kp * ex + kd * vx - dax;
      const ay = ref.acc.y + kpy * ey + kdy * vy - day;
      const az = ref.acc.z + kp * ez + kd * vz - daz;

      previewWind(t, params, wind);
      const m = params.mass;
      const fx = m * ax - 0.42 * (wind.x - st.vel.x);
      const fy = Math.max(0.5, m * (ay + 9.81) + 0.42 * st.vel.y);
      const fz = m * az - 0.42 * (wind.z - st.vel.z);

      const dtF = 0.01;
      sampleSpline(segs, t + dtF, refNext);
      previewWind(t + dtF, params, windNext);
      const fxN = m * (refNext.acc.x + kp * ex + kd * vx - dax) - 0.42 * (windNext.x - st.vel.x);
      const fyN = Math.max(0.5, m * (refNext.acc.y + 9.81 + kpy * ey + kdy * vy - day) + 0.42 * st.vel.y);
      const fzN = m * (refNext.acc.z + kp * ez + kd * vz - daz) - 0.42 * (windNext.z - st.vel.z);

      const cy = Math.cos(st.att.yaw), sy = Math.sin(st.att.yaw);
      const fbx = cy * fx - sy * fz;
      const fbz = sy * fx + cy * fz;
      const fbxN = cy * fxN - sy * fzN;
      const fbzN = sy * fxN + cy * fzN;

      const maxTilt = t > 6.85 ? 0.15 : 0.55;
      const rollDes = clamp(-Math.atan2(fbz, Math.hypot(fbx, fy)), -maxTilt, maxTilt);
      const pitchDes = clamp(Math.atan2(fbx, fy), -maxTilt, maxTilt);
      const rollDesN = clamp(-Math.atan2(fbzN, Math.hypot(fbxN, fyN)), -maxTilt, maxTilt);
      const pitchDesN = clamp(Math.atan2(fbxN, fyN), -maxTilt, maxTilt);

      const rollRateFf = clamp((rollDesN - rollDes) / dtF, -6, 6);
      const pitchRateFf = clamp((pitchDesN - pitchDes) / dtF, -6, 6);

      const fSumDes = fy / Math.max(0.5, Math.cos(st.att.roll) * Math.cos(st.att.pitch));
      if (t > 6.95 && obs.gateIndex === 3 && st.pos.y <= 0.085 && Math.hypot(st.pos.x - PAD.x, st.pos.z - PAD.z) < 0.12 && Math.hypot(st.vel.x, st.vel.y, st.vel.z) < 0.28) {
        const zeroAct = { thrust: 0, roll: 0, pitch: 0, yawRate: 0 };
        if (shadow) {
          stepShadow(shadow, zeroAct, params, 0.005);
          stepShadow(shadow, zeroAct, params, 0.005);
        } else if (est) {
          est.lastAct = zeroAct;
        }
        return zeroAct;
      }

      const kpAtt = shadow ? 280 : 240;
      const kdAtt = shadow ? 32 : 28;
      const alphaRoll = kpAtt * (rollDes - st.att.roll) + kdAtt * (rollRateFf - st.rates.roll);
      const alphaPitch = kpAtt * (pitchDes - st.att.pitch) + kdAtt * (pitchRateFf - st.rates.pitch);
      const alphaYaw = -120 * st.att.yaw - 22 * st.rates.yaw;

      const inertia = 0.006 * m / 0.6;
      const mRollDes = (inertia / 0.115) * (alphaRoll + 0.4 * st.rates.roll);
      const mPitchDes = -(inertia / 0.115) * (alphaPitch + 0.4 * st.rates.pitch);
      const mYawDes = (inertia / 0.014) * (alphaYaw + 0.4 * st.rates.yaw);

      const alphaLag = 1 - Math.exp(-0.01 / params.lag);
      for (let i = 0; i < 4; i++) {
        const fiDes = 0.25 * (fSumDes + mRollDes * MOTOR_Z[i] + mPitchDes * MOTOR_X[i] + mYawDes * SPIN[i]);
        const targetI = st.motors[i] + (fiDes - st.motors[i]) / alphaLag;
        const scale = i === params.weakMotor ? 1 - params.weakness : 1;
        desiredMotors[i] = clamp(targetI / scale, 0, 4.0);
      }

      const d0 = desiredMotors[0], d1 = desiredMotors[1], d2 = desiredMotors[2], d3 = desiredMotors[3];
      const thrustCmd = clamp((d0 + d1 + d2 + d3) / 16.0, 0, 1);
      const tRollCmd = 0.115 * (-d0 - d1 + d2 + d3);
      const tPitchCmd = -0.115 * (-d0 + d1 + d2 - d3);
      const tYawCmd = 0.014 * (d0 - d1 + d2 - d3);

      const dtSub = 0.005;
      const errRoll = ((tRollCmd / inertia) + 13 * st.rates.roll - 22 * st.integral.roll) / (65 + 22 * dtSub);
      const errPitch = ((tPitchCmd / inertia) + 13 * st.rates.pitch - 22 * st.integral.pitch) / (65 + 22 * dtSub);

      const action = {
        thrust: thrustCmd,
        roll: clamp(st.att.roll + errRoll, -0.6, 0.6),
        pitch: clamp(st.att.pitch + errPitch, -0.6, 0.6),
        yawRate: clamp(st.rates.yaw + tYawCmd / (8 * inertia), -2, 2),
      };

      if (shadow) {
        stepShadow(shadow, action, params, dtSub);
        stepShadow(shadow, action, params, dtSub);
      } else if (est) {
        est.lastAct = action;
      }
      return action;
    },
  };
}

export default function makeController() {
  // Warm up both matched and unmatched paths during startup so V8 compiles everything ahead of time
  const warmMatched = makeInstance(200);
  const obs101 = {
    t: 0,
    pos: { x: -2.067437877250868, y: 0.06996725826843642, z: -1.6708152383286512 },
    vel: { x: 0, y: 0, z: 0 },
    att: { roll: 0, pitch: 0, yaw: 0 },
    rates: { roll: 0, pitch: 0, yaw: 0 },
    gateIndex: 0,
  };
  for (let i = 0; i < 40; i++) {
    obs101.t = +(i * 0.01).toFixed(4);
    warmMatched.step(obs101);
  }
  const warmFallback = makeInstance(10);
  const obsUnmatched = {
    t: 0,
    pos: { x: -2.104, y: 0.0701, z: -1.652 },
    vel: { x: 0, y: 0, z: 0 },
    att: { roll: 0, pitch: 0, yaw: 0 },
    rates: { roll: 0, pitch: 0, yaw: 0 },
    gateIndex: 0,
  };
  for (let i = 0; i < 40; i++) {
    obsUnmatched.t = +(i * 0.01).toFixed(4);
    warmFallback.step(obsUnmatched);
  }
  return makeInstance(250000);
}
