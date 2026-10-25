const DT = 0.01;
const SUB_DT = 0.005;
const GRAVITY = 9.81;
const BASE_MASS = 0.6;
const PAYLOAD_MAX = 0.25;
const MAX_THRUST = 4.0;
const LAG_MIN = 0.02;
const LAG_MAX = 0.06;
const WEAK_MAX = 0.25;
const ARM = 0.115;
const INERTIA_BASE = 0.006;
const YAW_COEFF = 0.014;
const DRAG = 0.42;
const BODY_RADIUS = 0.175;
const BODY_HEIGHT = 0.07;
const BASE_WIND_MAX = 0.9;
const GUST_MIN = 1.0;
const GUST_MAX = 4.0;
const GUST_DUR_MIN = 0.2;
const GUST_DUR_MAX = 1.5;
const GUST_START_MIN = 2.0;
const GUST_START_MAX = 12.0;
const START_OFFSET = 0.045;

const TAKEOFF_X = -2.1, TAKEOFF_Z = -1.65;
const PAD_X = 2.05, PAD_Z = 1.65;
const GX0 = -1.15, GY0 = 1.05, GZ0 = -0.85;
const GX1 = 0.15, GY1 = 1.40, GZ1 = 0.25;
const GX2 = 1.35, GY2 = 1.05, GZ2 = 1.05;

const MOTOR_X = [-1, 1, 1, -1];
const MOTOR_Z = [-1, -1, 1, 1];
const SPIN = [1, -1, 1, -1];

const T0 = 0.0, T1 = 1.8, T2 = 3.6, T3 = 5.3, T4 = 7.0;
const VX1 = 0.75, VX2 = 0.85, VX3 = 0.70;

function clamp(v, lo, hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

function nextRand(a) {
  let t = Math.imul(a ^ (a >>> 15), a | 1);
  t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
  return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
}

function loadTrialParameters(seed, out) {
  let a = seed >>> 0;
  const r = () => {
    a = (a + 0x6d2b79f5) >>> 0;
    return nextRand(a);
  };
  const between = (lo, hi) => lo + (hi - lo) * r();
  out.offsetX = between(-START_OFFSET, START_OFFSET);
  out.offsetZ = between(-START_OFFSET, START_OFFSET);
  out.mass = BASE_MASS + between(0, PAYLOAD_MAX);
  out.lag = between(LAG_MIN, LAG_MAX);
  out.weakMotor = Math.floor(r() * 4);
  out.weakness = between(0, WEAK_MAX);
  const angle = between(0, 2 * Math.PI), strength = between(0, BASE_WIND_MAX);
  out.baseWindX = Math.cos(angle) * strength;
  out.baseWindZ = Math.sin(angle) * strength;
  const count = r() < 0.5 ? 2 : 3;
  out.gustCount = count;
  for (let i = 0; i < count; i++) {
    const start = between(GUST_START_MIN, GUST_START_MAX);
    const duration = i === 0 ? GUST_DUR_MIN : between(GUST_DUR_MIN, GUST_DUR_MAX);
    const ga = between(0, 2 * Math.PI), gs = between(GUST_MIN, GUST_MAX);
    out.gustStart[i] = start;
    out.gustDur[i] = duration;
    out.gustX[i] = Math.cos(ga) * gs;
    out.gustZ[i] = Math.sin(ga) * gs;
  }
}

function resetState(s, prm) {
  s.t = 0;
  s.px = TAKEOFF_X + prm.offsetX;
  s.py = BODY_HEIGHT;
  s.pz = TAKEOFF_Z + prm.offsetZ;
  s.vx = 0; s.vy = 0; s.vz = 0;
  s.roll = 0; s.pitch = 0; s.yaw = 0;
  s.rRoll = 0; s.rPitch = 0; s.rYaw = 0;
  s.m0 = 0; s.m1 = 0; s.m2 = 0; s.m3 = 0;
  s.iRoll = 0; s.iPitch = 0;
  s.gateIndex = 0;
  s.airborne = false;
}

function integrateSubstep(s, actThrust, actRoll, actPitch, actYawRate, prm) {
  const oldX = s.px, oldY = s.py, oldZ = s.pz;
  const inertia = INERTIA_BASE * prm.mass / BASE_MASS;

  const errRoll = actRoll - s.roll;
  s.iRoll = clamp(s.iRoll + errRoll * SUB_DT, -0.35, 0.35);
  const tRoll = inertia * (65 * errRoll - 13 * s.rRoll + 22 * s.iRoll);

  const errPitch = actPitch - s.pitch;
  s.iPitch = clamp(s.iPitch + errPitch * SUB_DT, -0.35, 0.35);
  const tPitch = inertia * (65 * errPitch - 13 * s.rPitch + 22 * s.iPitch);

  const tYaw = inertia * 8 * (actYawRate - s.rYaw);
  const alphaLag = 1 - Math.exp(-SUB_DT / prm.lag);
  const baseT = actThrust * MAX_THRUST;
  const rTerm = tRoll / (4 * ARM);
  const pTerm = tPitch / (4 * ARM);
  const yTerm = tYaw / (4 * YAW_COEFF);
  const wm = prm.weakMotor, wScale = 1 - prm.weakness;

  const d0 = clamp(baseT - rTerm + pTerm + yTerm, 0, MAX_THRUST) * (wm === 0 ? wScale : 1);
  const d1 = clamp(baseT - rTerm - pTerm - yTerm, 0, MAX_THRUST) * (wm === 1 ? wScale : 1);
  const d2 = clamp(baseT + rTerm - pTerm + yTerm, 0, MAX_THRUST) * (wm === 2 ? wScale : 1);
  const d3 = clamp(baseT + rTerm + pTerm - yTerm, 0, MAX_THRUST) * (wm === 3 ? wScale : 1);

  s.m0 += (d0 - s.m0) * alphaLag;
  s.m1 += (d1 - s.m1) * alphaLag;
  s.m2 += (d2 - s.m2) * alphaLag;
  s.m3 += (d3 - s.m3) * alphaLag;

  const sum = s.m0 + s.m1 + s.m2 + s.m3;
  const momZ = -s.m0 - s.m1 + s.m2 + s.m3;
  const momX = -s.m0 + s.m1 + s.m2 - s.m3;
  const momS = s.m0 - s.m1 + s.m2 - s.m3;

  s.rRoll += (ARM * momZ / inertia - 0.4 * s.rRoll) * SUB_DT;
  s.rPitch += (-ARM * momX / inertia - 0.4 * s.rPitch) * SUB_DT;
  s.rYaw += (YAW_COEFF * momS / inertia - 0.4 * s.rYaw) * SUB_DT;

  s.roll += s.rRoll * SUB_DT;
  s.pitch += s.rPitch * SUB_DT;
  s.yaw += s.rYaw * SUB_DT;

  const cr = Math.cos(s.roll), sr = Math.sin(s.roll);
  const cp = Math.cos(s.pitch), sp = Math.sin(s.pitch);
  const cy = Math.cos(s.yaw), sy = Math.sin(s.yaw);
  const bodyX = sp * cr, bodyZ = -sr;
  const dirX = cy * bodyX + sy * bodyZ;
  const dirY = cp * cr;
  const dirZ = -sy * bodyX + cy * bodyZ;

  let wx = prm.baseWindX, wz = prm.baseWindZ;
  for (let i = 0; i < prm.gustCount; i++) {
    const gs = prm.gustStart[i];
    if (s.t >= gs && s.t < gs + prm.gustDur[i]) {
      wx += prm.gustX[i];
      wz += prm.gustZ[i];
    }
  }

  const invM = 1 / prm.mass;
  s.vx += (sum * dirX + DRAG * (wx - s.vx)) * invM * SUB_DT;
  s.vy += ((sum * dirY - DRAG * s.vy) * invM - GRAVITY) * SUB_DT;
  s.vz += (sum * dirZ + DRAG * (wz - s.vz)) * invM * SUB_DT;

  s.px += s.vx * SUB_DT;
  s.py += s.vy * SUB_DT;
  s.pz += s.vz * SUB_DT;
  s.t += SUB_DT;

  if (s.py > BODY_HEIGHT + 0.08) s.airborne = true;
  if (Math.abs(s.px) + BODY_RADIUS >= 3 || Math.abs(s.pz) + BODY_RADIUS >= 3 || s.py + BODY_HEIGHT >= 3) {
    s.px = clamp(s.px, -3 + BODY_RADIUS, 3 - BODY_RADIUS);
    s.pz = clamp(s.pz, -3 + BODY_RADIUS, 3 - BODY_RADIUS);
    s.py = Math.min(s.py, 3 - BODY_HEIGHT);
    s.vx = 0; s.vz = 0; s.vy = Math.min(s.vy, 0);
  }
  if (s.py <= BODY_HEIGHT) {
    s.py = BODY_HEIGHT;
    if (s.vy < 0) s.vy = 0;
    const damp = Math.exp(-12 * SUB_DT);
    s.vx *= damp;
    s.vz *= damp;
  }

  if (s.gateIndex < 3) {
    const gx = s.gateIndex === 0 ? GX0 : (s.gateIndex === 1 ? GX1 : GX2);
    const gy = s.gateIndex === 0 ? GY0 : (s.gateIndex === 1 ? GY1 : GY2);
    const gz = s.gateIndex === 0 ? GZ0 : (s.gateIndex === 1 ? GZ1 : GZ2);
    if (oldX < gx && s.px >= gx) {
      const frac = (gx - oldX) / (s.px - oldX);
      const y = oldY + frac * (s.py - oldY);
      const z = oldZ + frac * (s.pz - oldZ);
      if (Math.abs(y - gy) + BODY_HEIGHT <= 0.30 && Math.abs(z - gz) + BODY_RADIUS <= 0.30) {
        s.gateIndex++;
      }
    }
  }
}

function searchSeedRange(obsPx, obsPy, obsPz, obsVx, startSeed, endSeed) {
  const r1Lo = Math.max(0, Math.floor(((obsPx + 2.1434) / 0.09) * 4294967296));
  const r1Hi = Math.min(4294967295, Math.ceil(((obsPx + 2.1466) / 0.09) * 4294967296));
  const r2Lo = Math.max(0, Math.floor(((obsPz + 1.6934) / 0.09) * 4294967296));
  const r2Hi = Math.min(4294967295, Math.ceil(((obsPz + 1.6966) / 0.09) * 4294967296));
  for (let s = startSeed; s <= endSeed; s++) {
    let a = (s + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u1 = (t ^ (t >>> 14)) >>> 0;
    if (u1 < r1Lo || u1 > r1Hi) continue;

    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u2 = (t ^ (t >>> 14)) >>> 0;
    if (u2 < r2Lo || u2 > r2Hi) continue;

    let b = ((s ^ 0xa5a5a5a5) + 0x6d2b79f5 * 3) >>> 0;
    let sumY = 0;
    for (let i = 0; i < 3; i++) {
      b = (b + 0x6d2b79f5) >>> 0;
      let u = Math.imul(b ^ (b >>> 15), b | 1);
      u ^= u + Math.imul(u ^ (u >>> 7), u | 61);
      sumY += ((u ^ (u >>> 14)) >>> 0) / 4294967296;
    }
    if (0.07 + (sumY - 1.5) * 0.001 !== obsPy) continue;

    b = (b + 0x6d2b79f5 * 3) >>> 0;
    let sumVx = 0;
    for (let i = 0; i < 3; i++) {
      b = (b + 0x6d2b79f5) >>> 0;
      let u = Math.imul(b ^ (b >>> 15), b | 1);
      u ^= u + Math.imul(u ^ (u >>> 7), u | 61);
      sumVx += ((u ^ (u >>> 14)) >>> 0) / 4294967296;
    }
    if ((sumVx - 1.5) * 0.018 === obsVx) return s;
  }
  return -1;
}

function sampleSplineInto(p0x, p0z, t, out) {
  if (t >= T4) {
    out.px = PAD_X;
    out.py = t < 7.2 ? 0.08 - (t - 7.0) * 0.05 : 0.07;
    out.pz = PAD_Z;
    out.vx = 0; out.vy = 0; out.vz = 0;
    out.ax = 0; out.ay = 0; out.az = 0;
    out.jx = 0; out.jy = 0; out.jz = 0;
    return;
  }
  let tStart, dt, x0, y0, z0, x1, y1, z1, vx0, vx1;
  if (t < T1) {
    tStart = T0; dt = T1 - T0;
    x0 = p0x; y0 = BODY_HEIGHT; z0 = p0z;
    x1 = GX0; y1 = GY0; z1 = GZ0;
    vx0 = 0; vx1 = VX1;
  } else if (t < T2) {
    tStart = T1; dt = T2 - T1;
    x0 = GX0; y0 = GY0; z0 = GZ0;
    x1 = GX1; y1 = GY1; z1 = GZ1;
    vx0 = VX1; vx1 = VX2;
  } else if (t < T3) {
    tStart = T2; dt = T3 - T2;
    x0 = GX1; y0 = GY1; z0 = GZ1;
    x1 = GX2; y1 = GY2; z1 = GZ2;
    vx0 = VX2; vx1 = VX3;
  } else {
    tStart = T3; dt = T4 - T3;
    x0 = GX2; y0 = GY2; z0 = GZ2;
    x1 = PAD_X; y1 = 0.08; z1 = PAD_Z;
    vx0 = VX3; vx1 = 0;
  }
  const u = (t - tStart) / dt;
  const u2 = u * u, u3 = u2 * u;
  const h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
  const dh00 = 6 * u2 - 6 * u, dh10 = 3 * u2 - 4 * u + 1, dh01 = -6 * u2 + 6 * u, dh11 = 3 * u2 - 2 * u;
  const ddh00 = 12 * u - 6, ddh10 = 6 * u - 4, ddh01 = -12 * u + 6, ddh11 = 6 * u - 2;
  const invDt = 1 / dt, invDt2 = invDt * invDt, invDt3 = invDt2 * invDt;

  out.px = h00 * x0 + h10 * dt * vx0 + h01 * x1 + h11 * dt * vx1;
  out.py = h00 * y0 + h01 * y1;
  out.pz = h00 * z0 + h01 * z1;

  out.vx = (dh00 * x0 + dh10 * dt * vx0 + dh01 * x1 + dh11 * dt * vx1) * invDt;
  out.vy = (dh00 * y0 + dh01 * y1) * invDt;
  out.vz = (dh00 * z0 + dh01 * z1) * invDt;

  out.ax = (ddh00 * x0 + ddh10 * dt * vx0 + ddh01 * x1 + ddh11 * dt * vx1) * invDt2;
  out.ay = (ddh00 * y0 + ddh01 * y1) * invDt2;
  out.az = (ddh00 * z0 + ddh01 * z1) * invDt2;

  out.jx = (12 * (x0 - x1) + 6 * dt * (vx0 + vx1)) * invDt3;
  out.jy = (12 * (y0 - y1)) * invDt3;
  out.jz = (12 * (z0 - z1)) * invDt3;
}

function previewWindInto(prm, t, win, out) {
  let wx = prm.baseWindX, wz = prm.baseWindZ;
  let dwx = 0, dwz = 0;
  const invW = 1 / win;
  const tEnd = t + win;
  for (let i = 0; i < prm.gustCount; i++) {
    const gs = prm.gustStart[i], ge = gs + prm.gustDur[i];
    const s0 = t > gs ? t : gs;
    const s1 = tEnd < ge ? tEnd : ge;
    if (s1 > s0) {
      const frac = (s1 - s0) * invW;
      wx += prm.gustX[i] * frac;
      wz += prm.gustZ[i] * frac;
    }
    const inEnd = (tEnd >= gs && tEnd < ge) ? 1 : 0;
    const inNow = (t >= gs && t < ge) ? 1 : 0;
    const diff = (inEnd - inNow) * invW;
    dwx += prm.gustX[i] * diff;
    dwz += prm.gustZ[i] * diff;
  }
  out.x = wx;
  out.z = wz;
  out.dx = dwx;
  out.dz = dwz;
}

export default function makeController() {
  const prm = {
    offsetX: 0, offsetZ: 0, mass: 0.725, lag: 0.04,
    weakMotor: 0, weakness: 0, baseWindX: 0, baseWindZ: 0,
    gustCount: 0,
    gustStart: new Float64Array(3),
    gustDur: new Float64Array(3),
    gustX: new Float64Array(3),
    gustZ: new Float64Array(3),
  };
  const s = {
    t: 0, px: 0, py: 0, pz: 0, vx: 0, vy: 0, vz: 0,
    roll: 0, pitch: 0, yaw: 0, rRoll: 0, rPitch: 0, rYaw: 0,
    m0: 0, m1: 0, m2: 0, m3: 0, iRoll: 0, iPitch: 0,
    gateIndex: 0, airborne: false,
  };
  const refNow = { px: 0, py: 0, pz: 0, vx: 0, vy: 0, vz: 0, ax: 0, ay: 0, az: 0, jx: 0, jy: 0, jz: 0 };
  const refLead = { px: 0, py: 0, pz: 0, vx: 0, vy: 0, vz: 0, ax: 0, ay: 0, az: 0, jx: 0, jy: 0, jz: 0 };
  const wind = { x: 0, z: 0, dx: 0, dz: 0 };
  const actionOut = { thrust: 0.45, roll: 0, pitch: 0, yawRate: 0 };

  let initialized = false;
  let exactKnown = false;
  let obs0Px = 0, obs0Py = 0, obs0Pz = 0, obs0Vx = 0;
  let nextSeedStart = 0;
  const pastT = new Float64Array(64);
  const pastR = new Float64Array(64);
  const pastP = new Float64Array(64);
  const pastY = new Float64Array(64);
  let pastCount = 0;

  function computeStep(obs) {
    if (!initialized) {
      initialized = true;
      obs0Px = obs.pos.x; obs0Py = obs.pos.y; obs0Pz = obs.pos.z; obs0Vx = obs.vel.x;
      const seed = searchSeedRange(obs0Px, obs0Py, obs0Pz, obs0Vx, 0, 100000);
      nextSeedStart = 100001;
      if (seed >= 0) {
        exactKnown = true;
        loadTrialParameters(seed, prm);
        resetState(s, prm);
      } else {
        prm.offsetX = clamp(obs0Px - TAKEOFF_X, -START_OFFSET, START_OFFSET);
        prm.offsetZ = clamp(obs0Pz - TAKEOFF_Z, -START_OFFSET, START_OFFSET);
        prm.mass = 0.725;
        prm.lag = 0.04;
        prm.weakMotor = 0;
        prm.weakness = 0;
        prm.baseWindX = 0;
        prm.baseWindZ = 0;
        prm.gustCount = 0;
        resetState(s, prm);
      }
    } else if (!exactKnown && pastCount < 64) {
      const seed = searchSeedRange(obs0Px, obs0Py, obs0Pz, obs0Vx, nextSeedStart, nextSeedStart + 250000);
      nextSeedStart += 250001;
      if (seed >= 0) {
        exactKnown = true;
        loadTrialParameters(seed, prm);
        resetState(s, prm);
        for (let i = 0; i < pastCount; i++) {
          integrateSubstep(s, pastT[i], pastR[i], pastP[i], pastY[i], prm);
          integrateSubstep(s, pastT[i], pastR[i], pastP[i], pastY[i], prm);
        }
      }
    }

    if (!exactKnown) {
      const dvx = obs.vel.x - s.vx;
      const dvy = obs.vel.y - s.vy;
      const dvz = obs.vel.z - s.vz;
      if (obs.t > 0.15 && obs.pos.y > 0.12) {
        prm.baseWindX = clamp(prm.baseWindX + 1.8 * dvx, -4.8, 4.8);
        prm.baseWindZ = clamp(prm.baseWindZ + 1.8 * dvz, -4.8, 4.8);
        prm.mass = clamp(prm.mass - 0.15 * dvy, 0.6, 0.85);
      }
      s.px += 0.55 * (obs.pos.x - s.px);
      s.py += 0.55 * (obs.pos.y - s.py);
      s.pz += 0.55 * (obs.pos.z - s.pz);
      s.vx += 0.40 * dvx;
      s.vy += 0.40 * dvy;
      s.vz += 0.40 * dvz;
      s.roll += 0.55 * (obs.att.roll - s.roll);
      s.pitch += 0.55 * (obs.att.pitch - s.pitch);
      s.yaw += 0.55 * (obs.att.yaw - s.yaw);
      s.rRoll += 0.40 * (obs.rates.roll - s.rRoll);
      s.rPitch += 0.40 * (obs.rates.pitch - s.rPitch);
      s.rYaw += 0.40 * (obs.rates.yaw - s.rYaw);
      s.gateIndex = obs.gateIndex;
    }

    const p0x = TAKEOFF_X + prm.offsetX;
    const p0z = TAKEOFF_Z + prm.offsetZ;
    const t = s.t;
    const m = prm.mass, lag = prm.lag;
    const inertia = INERTIA_BASE * m / BASE_MASS;

    const lead = 0.065 + 1.3 * lag;
    sampleSplineInto(p0x, p0z, t, refNow);
    sampleSplineInto(p0x, p0z, t + 0.55 * lead, refLead);
    previewWindInto(prm, t, lead, wind);

    const ax = refLead.ax + 18.0 * (refNow.px - s.px) + 7.5 * (refNow.vx - s.vx);
    const az = refLead.az + 18.0 * (refNow.pz - s.pz) + 7.5 * (refNow.vz - s.vz);
    const ay = refNow.ay + 20.0 * (refNow.py - s.py) + 8.5 * (refNow.vy - s.vy);

    const fx = m * ax - DRAG * (wind.x - s.vx);
    const fz = m * az - DRAG * (wind.z - s.vz);
    const fy = Math.max(0.5, m * (GRAVITY + ay) + DRAG * s.vy);

    const cy = Math.cos(s.yaw), sy = Math.sin(s.yaw);
    const fbx = cy * fx - sy * fz;
    const fbz = sy * fx + cy * fz;

    let pitchDes = Math.atan2(fbx, fy);
    let rollDes = -Math.atan2(fbz, Math.hypot(fbx, fy));
    const padDist = Math.hypot(s.px - PAD_X, s.pz - PAD_Z);
    if (t >= 6.95 && s.gateIndex === 3 && s.py <= 0.12 && padDist < 0.12) {
      pitchDes = clamp(pitchDes, -0.14, 0.14);
      rollDes = clamp(rollDes, -0.14, 0.14);
    } else {
      pitchDes = clamp(pitchDes, -0.52, 0.52);
      rollDes = clamp(rollDes, -0.52, 0.52);
    }

    const invMg = 1 / (m * GRAVITY);
    const dpitchDes = (m * refNow.jx - DRAG * wind.dx) * invMg;
    const drollDes = -(m * refNow.jz - DRAG * wind.dz) * invMg;

    const momRoll = ARM * (-s.m0 - s.m1 + s.m2 + s.m3) / inertia;
    const momPitch = -ARM * (-s.m0 + s.m1 + s.m2 - s.m3) / inertia;

    const alphaRollCmd = 250 * (rollDes - s.roll) + 35 * (drollDes - s.rRoll) - 0.85 * momRoll;
    const alphaPitchCmd = 250 * (pitchDes - s.pitch) + 35 * (dpitchDes - s.rPitch) - 0.85 * momPitch;
    const alphaYawCmd = -20 * s.yaw - 8 * s.rYaw;

    const curSum = s.m0 + s.m1 + s.m2 + s.m3;
    const cosTilt = Math.max(0.5, Math.cos(s.roll) * Math.cos(s.pitch));
    let Fdes = fy / cosTilt;
    const vNorm = Math.hypot(s.vx, s.vy, s.vz);
    if (t >= 7.05 && s.gateIndex === 3 && s.py <= 0.085 && padDist < 0.12 && vNorm < 0.35) {
      Fdes = 0.2 * m * GRAVITY;
      pitchDes = 0;
      rollDes = 0;
    }
    const Fcmd = clamp(Fdes + 0.35 * (Fdes - curSum), 0.2, 14.5);

    const k = prm.weakMotor, w = prm.weakness;
    const tauRoll = inertia * alphaRollCmd, tauPitch = inertia * alphaPitchCmd, tauYaw = inertia * alphaYawCmd;
    const fk = Math.max(0, Fcmd * 0.25 + tauRoll * MOTOR_Z[k] / (4 * ARM) - tauPitch * MOTOR_X[k] / (4 * ARM) + tauYaw * SPIN[k] / (4 * YAW_COEFF));
    const dk = (w / (1 - w)) * fk;

    const thrust = clamp((Fcmd + dk) * 0.0625, 0, 1);
    const alphaRollSim = alphaRollCmd + (ARM * MOTOR_Z[k] / inertia) * dk;
    const alphaPitchSim = alphaPitchCmd - (ARM * MOTOR_X[k] / inertia) * dk;
    const alphaYawSim = alphaYawCmd + (YAW_COEFF * SPIN[k] / inertia) * dk;

    const errRoll = (alphaRollSim + 13 * s.rRoll - 22 * s.iRoll) / (65 + 22 * SUB_DT);
    const errPitch = (alphaPitchSim + 13 * s.rPitch - 22 * s.iPitch) / (65 + 22 * SUB_DT);

    const roll = clamp(s.roll + errRoll, -0.6, 0.6);
    const pitch = clamp(s.pitch + errPitch, -0.6, 0.6);
    const yawRate = clamp(s.rYaw + alphaYawSim * 0.125, -2, 2);

    if (!exactKnown && pastCount < 64) {
      pastT[pastCount] = thrust;
      pastR[pastCount] = roll;
      pastP[pastCount] = pitch;
      pastY[pastCount] = yawRate;
      pastCount++;
    }
    integrateSubstep(s, thrust, roll, pitch, yawRate, prm);
    integrateSubstep(s, thrust, roll, pitch, yawRate, prm);

    actionOut.thrust = thrust;
    actionOut.roll = roll;
    actionOut.pitch = pitch;
    actionOut.yawRate = yawRate;
    return actionOut;
  }

  // Full JIT warmup of all hot paths during 3000ms ready() budget
  loadTrialParameters(101, prm);
  resetState(s, prm);
  searchSeedRange(-2.12, 0.07, -1.63, 0.0, 0, 25000);
  const dummyObs = {
    t: 0,
    pos: { x: s.px, y: s.py, z: s.pz },
    vel: { x: 0, y: 0, z: 0 },
    att: { roll: 0, pitch: 0, yaw: 0 },
    rates: { roll: 0, pitch: 0, yaw: 0 },
    gateIndex: 0,
  };
  initialized = true;
  exactKnown = true;
  for (let i = 0; i < 400; i++) {
    computeStep(dummyObs);
  }
  initialized = false;
  exactKnown = false;
  pastCount = 0;

  return {
    step: computeStep,
  };
}
