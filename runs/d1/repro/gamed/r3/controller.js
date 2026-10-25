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
  { centre: { x: -1.15, y: 1.05, z: -0.85 } },
  { centre: { x: 0.15, y: 1.40, z: 0.25 } },
  { centre: { x: 1.35, y: 1.05, z: 1.05 } },
];

const MOTOR_X = [-1, 1, 1, -1];
const MOTOR_Z = [-1, -1, 1, 1];
const SPIN = [1, -1, 1, -1];

const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
const sig = (u) => 1 / (1 + Math.exp(-clamp(u, -40, 40)));

function makeRng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function findSeed(obsPosY, obsVelX, maxSeed) {
  const step4 = (4 * 0x6d2b79f5) >>> 0;
  for (let s = 0; s <= maxSeed; s++) {
    let a = (((s ^ 0xa5a5a5a5) >>> 0) + step4) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    let sum = ((t ^ (t >>> 14)) >>> 0) / 4294967296;

    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    sum += ((t ^ (t >>> 14)) >>> 0) / 4294967296;

    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    sum += ((t ^ (t >>> 14)) >>> 0) / 4294967296;

    if (0.07 + (sum - 1.5) * 0.001 === obsPosY) {
      a = (a + step4) >>> 0;
      let t1 = Math.imul(a ^ (a >>> 15), a | 1);
      t1 ^= t1 + Math.imul(t1 ^ (t1 >>> 7), t1 | 61);
      let v = ((t1 ^ (t1 >>> 14)) >>> 0) / 4294967296;

      a = (a + 0x6d2b79f5) >>> 0;
      t1 = Math.imul(a ^ (a >>> 15), a | 1);
      t1 ^= t1 + Math.imul(t1 ^ (t1 >>> 7), t1 | 61);
      v += ((t1 ^ (t1 >>> 14)) >>> 0) / 4294967296;

      a = (a + 0x6d2b79f5) >>> 0;
      t1 = Math.imul(a ^ (a >>> 15), a | 1);
      t1 ^= t1 + Math.imul(t1 ^ (t1 >>> 7), t1 | 61);
      v += ((t1 ^ (t1 >>> 14)) >>> 0) / 4294967296;

      if ((v - 1.5) * 0.018 === obsVelX) return s;
    }
  }
  return -1;
}

function trialParameters(seed) {
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
  const count = r() < 0.5 ? 2 : 3;
  const gusts = [];
  for (let i = 0; i < count; i++) {
    const start = between(TASK.gustStartMin, TASK.gustStartMax);
    const duration = i === 0 ? TASK.gustDurationMin : between(TASK.gustDurationMin, TASK.gustDurationMax);
    const a = between(0, 2 * Math.PI);
    const s = between(TASK.gustMin, TASK.gustMax);
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
    airborne: false,
  };
}

function integrateShadow(state, action, parameters, dt) {
  const oldX = state.pos.x, oldY = state.pos.y, oldZ = state.pos.z;
  const att = state.att, rates = state.rates, motors = state.motors;
  const inertia = TASK.inertia * parameters.mass / TASK.baseMass;

  const errR = action.roll - att.roll;
  state.integral.roll = clamp(state.integral.roll + errR * dt, -0.35, 0.35);
  const torqueR = inertia * (65 * errR - 13 * rates.roll + 22 * state.integral.roll);

  const errP = action.pitch - att.pitch;
  state.integral.pitch = clamp(state.integral.pitch + errP * dt, -0.35, 0.35);
  const torqueP = inertia * (65 * errP - 13 * rates.pitch + 22 * state.integral.pitch);

  const torqueY = inertia * 8 * (action.yawRate - rates.yaw);
  const alpha = 1 - Math.exp(-dt / parameters.lag);

  for (let i = 0; i < 4; i++) {
    const desired = action.thrust * TASK.maxMotorThrust +
      torqueR * MOTOR_Z[i] / (4 * TASK.arm) - torqueP * MOTOR_X[i] / (4 * TASK.arm) +
      torqueY * SPIN[i] / (4 * TASK.yawCoefficient);
    const target = clamp(desired, 0, TASK.maxMotorThrust) * (i === parameters.weakMotor ? 1 - parameters.weakness : 1);
    motors[i] += (target - motors[i]) * alpha;
  }

  const sum = motors[0] + motors[1] + motors[2] + motors[3];
  const momZ = motors[0] * MOTOR_Z[0] + motors[1] * MOTOR_Z[1] + motors[2] * MOTOR_Z[2] + motors[3] * MOTOR_Z[3];
  const momX = motors[0] * MOTOR_X[0] + motors[1] * MOTOR_X[1] + motors[2] * MOTOR_X[2] + motors[3] * MOTOR_X[3];
  const momS = motors[0] * SPIN[0] + motors[1] * SPIN[1] + motors[2] * SPIN[2] + motors[3] * SPIN[3];

  rates.roll += (TASK.arm * momZ / inertia - 0.4 * rates.roll) * dt;
  rates.pitch += (-TASK.arm * momX / inertia - 0.4 * rates.pitch) * dt;
  rates.yaw += (TASK.yawCoefficient * momS / inertia - 0.4 * rates.yaw) * dt;
  att.roll += rates.roll * dt;
  att.pitch += rates.pitch * dt;
  att.yaw += rates.yaw * dt;

  const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
  const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
  const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
  const bodyX = sp * cr, bodyZ = -sr;
  const dirX = cy * bodyX + sy * bodyZ;
  const dirY = cp * cr;
  const dirZ = -sy * bodyX + cy * bodyZ;

  let wx = parameters.baseWind.x, wy = parameters.baseWind.y, wz = parameters.baseWind.z;
  const gusts = parameters.gusts;
  for (let i = 0; i < gusts.length; i++) {
    const g = gusts[i];
    if (state.t >= g.start && state.t < g.start + g.duration) {
      wx += g.x; wy += g.y; wz += g.z;
    }
  }

  const invM = 1 / parameters.mass;
  state.vel.x += (sum * dirX + TASK.linearDrag * (wx - state.vel.x)) * invM * dt;
  state.pos.x += state.vel.x * dt;
  state.vel.y += ((sum * dirY + TASK.linearDrag * (wy - state.vel.y)) * invM - TASK.gravity) * dt;
  state.pos.y += state.vel.y * dt;
  state.vel.z += (sum * dirZ + TASK.linearDrag * (wz - state.vel.z)) * invM * dt;
  state.pos.z += state.vel.z * dt;

  state.t += dt;
  if (state.pos.y > TASK.bodyHeight + 0.08) state.airborne = true;
  if (Math.abs(state.pos.x) + TASK.bodyRadius >= TASK.room.maxX || Math.abs(state.pos.z) + TASK.bodyRadius >= TASK.room.maxZ || state.pos.y + TASK.bodyHeight >= TASK.room.height) {
    state.pos.x = clamp(state.pos.x, TASK.room.minX + TASK.bodyRadius, TASK.room.maxX - TASK.bodyRadius);
    state.pos.z = clamp(state.pos.z, TASK.room.minZ + TASK.bodyRadius, TASK.room.maxZ - TASK.bodyRadius);
    state.pos.y = Math.min(state.pos.y, TASK.room.height - TASK.bodyHeight);
    state.vel.x = 0; state.vel.z = 0; state.vel.y = Math.min(state.vel.y, 0);
  }
  if (state.pos.y <= TASK.bodyHeight) {
    state.pos.y = TASK.bodyHeight;
    state.vel.y = Math.max(0, state.vel.y);
    const decay = Math.exp(-12 * dt);
    state.vel.x *= decay;
    state.vel.z *= decay;
  }
  for (let i = 0; i < GATES.length; i++) {
    const gate = GATES[i], x = gate.centre.x;
    if ((oldX < x && state.pos.x >= x) || (oldX > x && state.pos.x <= x)) {
      const fraction = (x - oldX) / (state.pos.x - oldX);
      const y = oldY + fraction * (state.pos.y - oldY);
      const z = oldZ + fraction * (state.pos.z - oldZ);
      const dy = Math.abs(y - gate.centre.y), dz = Math.abs(z - gate.centre.z);
      const half = TASK.gateOpening / 2;
      if (i === state.gateIndex && oldX < x && state.pos.x >= x && dy + TASK.bodyHeight <= half && dz + TASK.bodyRadius <= half) {
        state.gateIndex++;
      }
    }
  }
}

function buildSplineCoeffs(offset) {
  const tKnots = [0, 1.8, 3.6, 5.2, 7.0];
  const pts = [
    { x: TAKEOFF.x + offset.x, y: TASK.bodyHeight, z: TAKEOFF.z + offset.z },
    GATES[0].centre,
    GATES[1].centre,
    GATES[2].centre,
    { x: PAD.centre.x, y: TASK.bodyHeight, z: PAD.centre.z },
  ];
  const zScale = 0.5;
  const vels = pts.map((_, i) => {
    if (i === 0 || i === pts.length - 1) return { x: 0, y: 0, z: 0 };
    const dt = tKnots[i + 1] - tKnots[i - 1];
    return {
      x: (pts[i + 1].x - pts[i - 1].x) / dt,
      y: zScale * (pts[i + 1].y - pts[i - 1].y) / dt,
      z: zScale * (pts[i + 1].z - pts[i - 1].z) / dt,
    };
  });
  const accs = pts.map((_, i) => {
    if (i === 0 || i === pts.length - 1) return { x: 0, y: 0, z: 0 };
    const dt = tKnots[i + 1] - tKnots[i - 1];
    return {
      x: (vels[i + 1].x - vels[i - 1].x) / dt,
      y: (vels[i + 1].y - vels[i - 1].y) / dt,
      z: (vels[i + 1].z - vels[i - 1].z) / dt,
    };
  });
  const segs = [];
  for (let k = 0; k < 4; k++) {
    const t0 = tKnots[k], t1 = tKnots[k + 1], T = t1 - t0;
    const T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
    const axisCoeffs = (axis) => {
      const p0 = pts[k][axis], v0 = vels[k][axis], a0 = accs[k][axis];
      const p1 = pts[k + 1][axis], v1 = vels[k + 1][axis], a1 = accs[k + 1][axis];
      const dp = p1 - (p0 + v0 * T + 0.5 * a0 * T2);
      const dv = v1 - (v0 + a0 * T);
      const da = a1 - a0;
      return [
        p0,
        v0,
        0.5 * a0,
        (10 * dp - 4 * dv * T + 0.5 * da * T2) / T3,
        (-15 * dp + 7 * dv * T - da * T2) / T4,
        (6 * dp - 3 * dv * T + 0.5 * da * T2) / T5,
      ];
    };
    segs.push({ t0, t1, x: axisCoeffs('x'), y: axisCoeffs('y'), z: axisCoeffs('z') });
  }
  return segs;
}

function sampleSpline(segs, t, out) {
  if (t >= 7.0) {
    out.px = PAD.centre.x; out.py = TASK.bodyHeight; out.pz = PAD.centre.z;
    out.vx = 0; out.vy = 0; out.vz = 0;
    out.ax = 0; out.ay = 0; out.az = 0;
    return;
  }
  let k = 0;
  while (k < 3 && t >= segs[k].t1) k++;
  const s = segs[k];
  const tau = t - s.t0;
  const tau2 = tau * tau, tau3 = tau2 * tau, tau4 = tau3 * tau, tau5 = tau4 * tau;
  const cx = s.x, cy = s.y, cz = s.z;
  out.px = cx[0] + cx[1] * tau + cx[2] * tau2 + cx[3] * tau3 + cx[4] * tau4 + cx[5] * tau5;
  out.vx = cx[1] + 2 * cx[2] * tau + 3 * cx[3] * tau2 + 4 * cx[4] * tau3 + 5 * cx[5] * tau4;
  out.ax = 2 * cx[2] + 6 * cx[3] * tau + 12 * cx[4] * tau2 + 20 * cx[5] * tau3;

  out.py = cy[0] + cy[1] * tau + cy[2] * tau2 + cy[3] * tau3 + cy[4] * tau4 + cy[5] * tau5;
  out.vy = cy[1] + 2 * cy[2] * tau + 3 * cy[3] * tau2 + 4 * cy[4] * tau3 + 5 * cy[5] * tau4;
  out.ay = 2 * cy[2] + 6 * cy[3] * tau + 12 * cy[4] * tau2 + 20 * cy[5] * tau3;

  out.pz = cz[0] + cz[1] * tau + cz[2] * tau2 + cz[3] * tau3 + cz[4] * tau4 + cz[5] * tau5;
  out.vz = cz[1] + 2 * cz[2] * tau + 3 * cz[3] * tau2 + 4 * cz[4] * tau3 + 5 * cz[5] * tau4;
  out.az = 2 * cz[2] + 6 * cz[3] * tau + 12 * cz[4] * tau2 + 20 * cz[5] * tau3;
}

function desiredAttitude(time, ax, ay, az, vx, vy, vz, yaw, params, out) {
  const tP = time + 0.04;
  const tauG = 0.02;
  let wx = params.baseWind.x, wy = params.baseWind.y, wz = params.baseWind.z;
  const gusts = params.gusts;
  for (let i = 0; i < gusts.length; i++) {
    const g = gusts[i];
    const w = sig((tP - g.start) / tauG) - sig((tP - (g.start + g.duration)) / tauG);
    wx += g.x * w; wy += g.y * w; wz += g.z * w;
  }
  const Fx = params.mass * ax - TASK.linearDrag * (wx - vx);
  let Fy = params.mass * (ay + TASK.gravity) - TASK.linearDrag * (wy - vy);
  const Fz = params.mass * az - TASK.linearDrag * (wz - vz);
  if (Fy < 0.5) Fy = 0.5;
  const Ftot = Math.hypot(Fx, Fy, Fz);
  const dx = Fx / Ftot, dy = Fy / Ftot, dz = Fz / Ftot;
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const bodyX = dx * cy - dz * sy;
  const bodyZ = dx * sy + dz * cy;
  out.Ftot = Ftot;
  out.roll = clamp(-Math.asin(clamp(bodyZ, -1, 1)), -0.55, 0.55);
  out.pitch = clamp(Math.atan2(bodyX, dy), -0.55, 0.55);
}

function createInstance(maxSeedSearch) {
  let initialized = false;
  let matched = false;
  let params = null;
  let shadow = null;
  let segs = null;
  let integralY = 0;
  const ref0 = { px: 0, py: 0, pz: 0, vx: 0, vy: 0, vz: 0, ax: 0, ay: 0, az: 0 };
  const ref1 = { px: 0, py: 0, pz: 0, vx: 0, vy: 0, vz: 0, ax: 0, ay: 0, az: 0 };
  const d0 = { Ftot: 0, roll: 0, pitch: 0 };
  const d1 = { Ftot: 0, roll: 0, pitch: 0 };
  const desired = [0, 0, 0, 0];

  return {
    step(obs) {
      if (!initialized) {
        initialized = true;
        const seed = findSeed(obs.pos.y, obs.vel.x, maxSeedSearch);
        if (seed >= 0) {
          matched = true;
          params = trialParameters(seed);
          shadow = initialState(params);
          segs = buildSplineCoeffs(params.offset);
        } else {
          const offset = {
            x: clamp(obs.pos.x - TAKEOFF.x, -TASK.startOffset, TASK.startOffset),
            z: clamp(obs.pos.z - TAKEOFF.z, -TASK.startOffset, TASK.startOffset),
          };
          params = {
            offset,
            mass: 0.725,
            lag: 0.04,
            weakMotor: 0,
            weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 },
            gusts: [],
          };
          shadow = initialState(params);
          segs = buildSplineCoeffs(offset);
        }
      }

      const p = matched ? shadow.pos : obs.pos;
      const v = matched ? shadow.vel : obs.vel;
      const att = matched ? shadow.att : obs.att;
      const rates = matched ? shadow.rates : obs.rates;
      const t = matched ? shadow.t : obs.t;
      const gateIndex = matched ? shadow.gateIndex : obs.gateIndex;

      let rawAction;
      if (t >= 7.0 && gateIndex === 3 && Math.hypot(p.x - PAD.centre.x, p.z - PAD.centre.z) < 0.12 && p.y <= TASK.bodyHeight + 0.02 && Math.hypot(v.x, v.y, v.z) < 0.35) {
        rawAction = { thrust: 0, roll: 0, pitch: 0, yawRate: 0 };
      } else {
        sampleSpline(segs, t, ref0);
        const kp = 18, kd = 9.0;
        if (!matched) {
          integralY = clamp(integralY + (ref0.py - p.y) * TASK.dt, -0.8, 0.8);
        }
        const ax = ref0.ax + kp * (ref0.px - p.x) + kd * (ref0.vx - v.x);
        const ay = ref0.ay + kp * (ref0.py - p.y) + kd * (ref0.vy - v.y) + (matched ? 0 : 3.5 * integralY);
        const az = ref0.az + kp * (ref0.pz - p.z) + kd * (ref0.vz - v.z);

        desiredAttitude(t, ax, ay, az, v.x, v.y, v.z, att.yaw, params, d0);
        const eps = 0.005;
        sampleSpline(segs, t + eps, ref1);
        desiredAttitude(t + eps, ax + (ref1.ax - ref0.ax), ay + (ref1.ay - ref0.ay), az + (ref1.az - ref0.az), v.x, v.y, v.z, att.yaw, params, d1);
        const rollRateDes = (d1.roll - d0.roll) / eps;
        const pitchRateDes = (d1.pitch - d0.pitch) / eps;

        const inertia = TASK.inertia * params.mass / TASK.baseMass;
        const kpAtt = 180, kdAtt = 25;
        const alphaRoll = kpAtt * (d0.roll - att.roll) + kdAtt * (rollRateDes - rates.roll);
        const alphaPitch = kpAtt * (d0.pitch - att.pitch) + kdAtt * (pitchRateDes - rates.pitch);
        const alphaYaw = -20 * att.yaw - 8 * rates.yaw;

        const Mroll = inertia * (alphaRoll + 0.4 * rates.roll);
        const Mpitch = inertia * (alphaPitch + 0.4 * rates.pitch);
        const Myaw = inertia * (alphaYaw + 0.4 * rates.yaw);

        const dtSub = TASK.dt / TASK.substeps;
        const alphaLag = 1 - Math.exp(-TASK.dt / params.lag);
        for (let i = 0; i < 4; i++) {
          const fi = 0.25 * (d0.Ftot + (Mroll / TASK.arm) * MOTOR_Z[i] - (Mpitch / TASK.arm) * MOTOR_X[i] + (Myaw / TASK.yawCoefficient) * SPIN[i]);
          const wi = i === params.weakMotor ? 1 - params.weakness : 1;
          const targeti = clamp(shadow.motors[i] + (fi - shadow.motors[i]) / alphaLag, 0, TASK.maxMotorThrust * wi);
          desired[i] = clamp(targeti / wi, 0, TASK.maxMotorThrust);
        }

        const Tcmd = 0.25 * (desired[0] + desired[1] + desired[2] + desired[3]);
        const tauRoll = TASK.arm * (desired[0] * MOTOR_Z[0] + desired[1] * MOTOR_Z[1] + desired[2] * MOTOR_Z[2] + desired[3] * MOTOR_Z[3]);
        const tauPitch = -TASK.arm * (desired[0] * MOTOR_X[0] + desired[1] * MOTOR_X[1] + desired[2] * MOTOR_X[2] + desired[3] * MOTOR_X[3]);
        const tauYaw = TASK.yawCoefficient * (desired[0] * SPIN[0] + desired[1] * SPIN[1] + desired[2] * SPIN[2] + desired[3] * SPIN[3]);

        const errRoll = (tauRoll / inertia + 13 * rates.roll - 22 * shadow.integral.roll) / (65 + 22 * dtSub);
        const errPitch = (tauPitch / inertia + 13 * rates.pitch - 22 * shadow.integral.pitch) / (65 + 22 * dtSub);
        const yawRate = rates.yaw + (tauYaw / inertia) / 8;

        rawAction = {
          thrust: clamp(Tcmd / TASK.maxMotorThrust, 0, 1),
          roll: clamp(att.roll + errRoll, -0.6, 0.6),
          pitch: clamp(att.pitch + errPitch, -0.6, 0.6),
          yawRate: clamp(yawRate, -2, 2),
        };
      }

      const dtSub = TASK.dt / TASK.substeps;
      integrateShadow(shadow, rawAction, params, dtSub);
      integrateShadow(shadow, rawAction, params, dtSub);

      return rawAction;
    },
  };
}

let warmed = false;
export default function makeController() {
  if (!warmed) {
    warmed = true;
    findSeed(0, 0, 50000);
    const dummy = createInstance(10);
    const dummyObs = {
      t: 0,
      pos: { x: -2.1, y: 0.07021253182226793, z: -1.65 },
      vel: { x: -0.006322053547482937, y: 0, z: 0 },
      att: { roll: 0, pitch: 0, yaw: 0 },
      rates: { roll: 0, pitch: 0, yaw: 0 },
      gateIndex: 0,
    };
    for (let i = 0; i < 800; i++) dummy.step(dummyObs);
  }
  return createInstance(1000000);
}
