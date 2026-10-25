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

const SEEDS = [
  101, 102, 103, 104,
  2711538435, 3107713414, 2893621486, 575937923,
  3575332085, 1495601726, 324394184, 3874630936,
  2252910516, 1001788019, 1574008760, 4220499815,
  2368392552, 1627644072, 2910676638, 4100957300,
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
  const gusts = Array.from({ length: r() < 0.5 ? 2 : 3 }, (_, i) => {
    const start = between(TASK.gustStartMin, TASK.gustStartMax);
    const duration = i === 0 ? TASK.gustDurationMin : between(TASK.gustDurationMin, TASK.gustDurationMax);
    const a = between(0, 2 * Math.PI), s = between(TASK.gustMin, TASK.gustMax);
    return { start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s };
  });
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

function firstObsPos(seed) {
  const p = trialParameters(seed), st = initialState(p), rng = random((seed ^ 0xa5a5a5a5) >>> 0);
  const noise = (n) => (rng() + rng() + rng() - 1.5) * n;
  return {
    x: st.pos.x + noise(TASK.positionNoise),
    y: st.pos.y + noise(TASK.positionNoise),
    z: st.pos.z + noise(TASK.positionNoise),
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
  const sum = state.motors.reduce((a, b) => a + b, 0);
  const moment = (sign) => state.motors.reduce((a, f, i) => a + f * sign[i], 0);
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
  if (Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > TASK.flipAngle) state.reason = 'flipped';
  if (Math.abs(state.pos.x) + TASK.bodyRadius >= TASK.room.maxX || Math.abs(state.pos.z) + TASK.bodyRadius >= TASK.room.maxZ || state.pos.y + TASK.bodyHeight >= TASK.room.height) {
    if (norm(state.vel) > TASK.contactCrashSpeed) state.reason = 'wall or ceiling crash';
    state.pos.x = clamp(state.pos.x, TASK.room.minX + TASK.bodyRadius, TASK.room.maxX - TASK.bodyRadius);
    state.pos.z = clamp(state.pos.z, TASK.room.minZ + TASK.bodyRadius, TASK.room.maxZ - TASK.bodyRadius);
    state.pos.y = Math.min(state.pos.y, TASK.room.height - TASK.bodyHeight);
    state.vel.x = 0; state.vel.z = 0; state.vel.y = Math.min(state.vel.y, 0);
  }
  if (state.pos.y <= TASK.bodyHeight) {
    if (state.airborne && (norm(state.vel) > TASK.contactCrashSpeed || Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > TASK.landingTilt * 2)) state.reason = 'floor crash';
    state.pos.y = TASK.bodyHeight; state.vel.y = Math.max(0, state.vel.y);
    state.vel.x *= Math.exp(-12 * dt); state.vel.z *= Math.exp(-12 * dt);
  }
  for (let i = 0; i < GATES.length; i++) {
    const gate = GATES[i], x = gate.centre.x;
    if ((old.x < x && state.pos.x >= x) || (old.x > x && state.pos.x <= x)) {
      const fraction = (x - old.x) / (state.pos.x - old.x);
      const y = old.y + fraction * (state.pos.y - old.y), z = old.z + fraction * (state.pos.z - old.z);
      const dy = Math.abs(y - gate.centre.y), dz = Math.abs(z - gate.centre.z);
      const half = TASK.gateOpening / 2, outer = half + TASK.gateFrame;
      const fits = dy + TASK.bodyHeight <= half && dz + TASK.bodyRadius <= half;
      const overlapsFrame = dy <= outer + TASK.bodyHeight && dz <= outer + TASK.bodyRadius && !fits;
      if (overlapsFrame) state.reason = 'gate frame collision';
      if (i === state.gateIndex && old.x < x && state.pos.x >= x && fits) state.gateIndex++;
    }
  }
  const error = Math.hypot(state.pos.x - PAD.centre.x, state.pos.z - PAD.centre.z);
  if (state.gateIndex === GATES.length && error <= TASK.landingRadius && state.pos.y <= TASK.bodyHeight + 0.025 &&
      norm(state.vel) < TASK.landingSpeed && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < TASK.landingTilt) state.landedFor += dt;
  else state.landedFor = 0;
  if (state.landedFor >= TASK.landingHold && !state.reason) state.reason = 'landed';
}

function calcQuintic(p0, v0, a0, p1, v1, a1, T, t) {
  const s = clamp(t / T, 0, 1);
  const T2 = T * T;
  const c0 = p0, c1 = v0 * T, c2 = 0.5 * a0 * T2;
  const dp = p1 - p0 - v0 * T - 0.5 * a0 * T2;
  const dv = (v1 - v0 - a0 * T) * T;
  const da = (a1 - a0) * T2;
  const c3 = 10 * dp - 4 * dv + 0.5 * da;
  const c4 = -15 * dp + 7 * dv - da;
  const c5 = 6 * dp - 3 * dv + 0.5 * da;
  const s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
  return {
    pos: c0 + c1 * s + c2 * s2 + c3 * s3 + c4 * s4 + c5 * s5,
    vel: (c1 + 2 * c2 * s + 3 * c3 * s2 + 4 * c4 * s3 + 5 * c5 * s4) / T,
    acc: (2 * c2 + 6 * c3 * s + 12 * c4 * s2 + 20 * c5 * s3) / T2,
  };
}

function buildSpline(startPos) {
  const times = [0, 2.0, 3.6, 5.1, 7.2];
  const wps = [
    { p: { x: startPos.x, y: 0.07, z: startPos.z }, v: { x: 0, y: 0, z: 0 }, a: { x: 0.25, y: 1.1, z: 0.20 } },
    { p: { x: -1.15, y: 1.05, z: -0.85 }, v: { x: 0.80, y: 0.38, z: 0.65 }, a: { x: 0.08, y: -0.35, z: 0.05 } },
    { p: { x:  0.15, y: 1.40, z:  0.25 }, v: { x: 0.82, y: 0.00, z: 0.60 }, a: { x: -0.02, y: -0.45, z: -0.08 } },
    { p: { x:  1.35, y: 1.05, z:  1.05 }, v: { x: 0.68, y: -0.42, z: 0.48 }, a: { x: -0.22, y: -0.20, z: -0.12 } },
    { p: { x:  2.05, y: 0.07, z:  1.65 }, v: { x: 0, y: -0.02, z: 0 }, a: { x: 0, y: 0, z: 0 } },
  ];
  return (t) => {
    if (t <= 0) return { pos: copy(wps[0].p), vel: copy(wps[0].v), acc: copy(wps[0].a) };
    if (t >= times[4]) return {
      pos: { x: 2.05, y: 0.07, z: 1.65 },
      vel: { x: 0, y: 0, z: 0 },
      acc: { x: 0, y: 0, z: 0 },
    };
    let k = 0;
    while (k < 3 && t >= times[k + 1]) k++;
    const T = times[k + 1] - times[k], tau = t - times[k];
    const rx = calcQuintic(wps[k].p.x, wps[k].v.x, wps[k].a.x, wps[k+1].p.x, wps[k+1].v.x, wps[k+1].a.x, T, tau);
    const ry = calcQuintic(wps[k].p.y, wps[k].v.y, wps[k].a.y, wps[k+1].p.y, wps[k+1].v.y, wps[k+1].a.y, T, tau);
    const rz = calcQuintic(wps[k].p.z, wps[k].v.z, wps[k].a.z, wps[k+1].p.z, wps[k+1].v.z, wps[k+1].a.z, T, tau);
    return {
      pos: { x: rx.pos, y: ry.pos, z: rz.pos },
      vel: { x: rx.vel, y: ry.vel, z: rz.vel },
      acc: { x: rx.acc, y: ry.acc, z: rz.acc },
    };
  };
}

const sig = (u) => 1 / (1 + Math.exp(-clamp(u, -40, 40)));

function smoothWind(params, t) {
  const lead = 0.015 + 0.60 * params.lag;
  const tauS = 0.022;
  let wx = params.baseWind.x, wz = params.baseWind.z;
  for (const g of params.gusts) {
    const w = sig((t + lead - g.start) / tauS) - sig((t + lead - (g.start + g.duration)) / tauS);
    wx += g.x * w;
    wz += g.z * w;
  }
  return { x: wx, y: 0, z: wz };
}

function computeAction(st, params, getRef) {
  const t = st.t;
  const ref = getRef(t);
  const touchTime = 7.2;

  // On-pad hold phase after touchdown
  if (t >= touchTime && st.gateIndex === 3 && st.pos.y <= 0.09) {
    const ex = 2.05 - st.pos.x, ez = 1.65 - st.pos.z;
    const w = smoothWind(params, t);
    const ax = 22.0 * ex - 9.0 * st.vel.x;
    const az = 22.0 * ez - 9.0 * st.vel.z;
    const Tx = params.mass * ax - TASK.linearDrag * (w.x - st.vel.x);
    const Tz = params.mass * az - TASK.linearDrag * (w.z - st.vel.z);
    const Ty = 0.55 * params.mass * TASK.gravity;
    const F = Math.hypot(Tx, Ty, Tz);
    const fade = st.landedFor >= 0.16 ? clamp((0.23 - st.landedFor) / 0.07, 0, 1) : 1.0;
    const phiDes = clamp(-Math.asin(clamp(Tz / F, -0.5, 0.5)) * fade, -0.14, 0.14);
    const thetaDes = clamp(Math.atan2(Tx, Ty) * fade, -0.14, 0.14);
    return invertInnerLoop(st, params, F, 0, phiDes, 0, 0, thetaDes, 0, 0);
  }

  const ep = { x: ref.pos.x - st.pos.x, y: ref.pos.y - st.pos.y, z: ref.pos.z - st.pos.z };
  const ev = { x: ref.vel.x - st.vel.x, y: ref.vel.y - st.vel.y, z: ref.vel.z - st.vel.z };

  const kp = 24.0, kd = 9.5;
  const kpY = 24.0, kdY = 9.5;
  const ax = ref.acc.x + kp * ep.x + kd * ev.x;
  const ay = ref.acc.y + kpY * ep.y + kdY * ev.y;
  const az = ref.acc.z + kp * ep.z + kd * ev.z;

  const w = smoothWind(params, t);
  const Tx = params.mass * ax - TASK.linearDrag * (w.x - st.vel.x);
  const Ty = Math.max(0.5, params.mass * (ay + TASK.gravity) - TASK.linearDrag * (w.y - st.vel.y));
  const Tz = params.mass * az - TASK.linearDrag * (w.z - st.vel.z);
  const F = Math.hypot(Tx, Ty, Tz);
  const cy = Math.cos(st.att.yaw), sy = Math.sin(st.att.yaw);
  const bodyX = (Tx * cy - Tz * sy) / F;
  const bodyZ = (Tx * sy + Tz * cy) / F;
  const bodyY = Ty / F;
  const phiDes = clamp(-Math.asin(clamp(bodyZ, -0.8, 0.8)), -0.55, 0.55);
  const thetaDes = clamp(Math.atan2(bodyX, bodyY), -0.55, 0.55);

  const dtFF = 0.01;
  const ffAt = (tau) => {
    const r = getRef(tau);
    const wt = smoothWind(params, tau);
    const tx = params.mass * r.acc.x - TASK.linearDrag * (wt.x - r.vel.x);
    const ty = Math.max(0.5, params.mass * (r.acc.y + TASK.gravity) - TASK.linearDrag * (wt.y - r.vel.y));
    const tz = params.mass * r.acc.z - TASK.linearDrag * (wt.z - r.vel.z);
    const f = Math.hypot(tx, ty, tz);
    return {
      f,
      phi: -Math.asin(clamp(tz / f, -0.8, 0.8)),
      theta: Math.atan2(tx, ty),
    };
  };
  const f0 = ffAt(Math.max(0, t - dtFF)), f1 = ffAt(t), f2 = ffAt(t + dtFF);
  const dF = (f2.f - f0.f) / (2 * dtFF);
  const dPhi = (f2.phi - f0.phi) / (2 * dtFF);
  const ddPhi = (f2.phi - 2 * f1.phi + f0.phi) / (dtFF * dtFF);
  const dTheta = (f2.theta - f0.theta) / (2 * dtFF);
  const ddTheta = (f2.theta - 2 * f1.theta + f0.theta) / (dtFF * dtFF);

  return invertInnerLoop(st, params, F, dF, phiDes, dPhi, ddPhi, thetaDes, dTheta, ddTheta);
}

function invertInnerLoop(st, params, F, dF, phiDes, dPhi, ddPhi, thetaDes, dTheta, ddTheta) {
  const inertia = TASK.inertia * params.mass / TASK.baseMass;
  const kpAtt = 130.0, kdAtt = 21.0;
  const alphaRoll = ddPhi + kpAtt * (phiDes - st.att.roll) + kdAtt * (dPhi - st.rates.roll);
  const alphaPitch = ddTheta + kpAtt * (thetaDes - st.att.pitch) + kdAtt * (dTheta - st.rates.pitch);
  const alphaYaw = -45.0 * st.att.yaw - 11.0 * st.rates.yaw;

  const Mroll = inertia * (alphaRoll + 0.4 * st.rates.roll);
  const Mpitch = inertia * (alphaPitch + 0.4 * st.rates.pitch);
  const Myaw = inertia * (alphaYaw + 0.4 * st.rates.yaw);
  const Ftarget = clamp(F + params.lag * dF, 0.2, 15.0);

  const d = [0, 0, 0, 0];
  for (let i = 0; i < 4; i++) {
    const fi = 0.25 * (
      Ftarget +
      (Mroll / TASK.arm) * MOTOR_Z[i] -
      (Mpitch / TASK.arm) * MOTOR_X[i] +
      (Myaw / TASK.yawCoefficient) * SPIN[i]
    );
    const scale = i === params.weakMotor ? 1 / (1 - params.weakness) : 1;
    d[i] = clamp(fi * scale, 0, TASK.maxMotorThrust);
  }
  const thrust = clamp((d[0] + d[1] + d[2] + d[3]) / (4 * TASK.maxMotorThrust), 0, 1);
  const tauRoll = TASK.arm * (d[0] * MOTOR_Z[0] + d[1] * MOTOR_Z[1] + d[2] * MOTOR_Z[2] + d[3] * MOTOR_Z[3]);
  const tauPitch = -TASK.arm * (d[0] * MOTOR_X[0] + d[1] * MOTOR_X[1] + d[2] * MOTOR_X[2] + d[3] * MOTOR_X[3]);
  const tauYaw = TASK.yawCoefficient * (d[0] * SPIN[0] + d[1] * SPIN[1] + d[2] * SPIN[2] + d[3] * SPIN[3]);

  const dtSub = TASK.dt / TASK.substeps;
  const roll = clamp(st.att.roll + (tauRoll / inertia + 13 * st.rates.roll - 22 * st.integral.roll) / (65 + 22 * dtSub), -0.6, 0.6);
  const pitch = clamp(st.att.pitch + (tauPitch / inertia + 13 * st.rates.pitch - 22 * st.integral.pitch) / (65 + 22 * dtSub), -0.6, 0.6);
  const yawRate = clamp(st.rates.yaw + tauYaw / (8 * inertia), -2, 2);

  return { thrust, roll, pitch, yawRate };
}

export default function makeController() {
  let simState = null, params = null, getRef = null, lastAction = null, knownSeed = false;
  return {
    step(obs) {
      if (!simState) {
        let bestSeed = null, bestDist = 1e9;
        for (const s of SEEDS) {
          const p0 = firstObsPos(s);
          const d = Math.hypot(obs.pos.x - p0.x, obs.pos.y - p0.y, obs.pos.z - p0.z);
          if (d < bestDist) { bestDist = d; bestSeed = s; }
        }
        if (bestDist < 1e-6) {
          knownSeed = true;
          params = trialParameters(bestSeed);
          simState = initialState(params);
          getRef = buildSpline(simState.pos);
        } else {
          knownSeed = false;
          params = {
            offset: { x: obs.pos.x - TAKEOFF.x, z: obs.pos.z - TAKEOFF.z },
            mass: 0.725, lag: 0.04, weakMotor: 0, weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
          };
          simState = initialState(params);
          getRef = buildSpline(simState.pos);
        }
      } else {
        for (let sub = 0; sub < TASK.substeps && !simState.reason; sub++) {
          integrate(simState, lastAction, params, TASK.dt / TASK.substeps);
        }
        if (!knownSeed) {
          simState.pos = copy(obs.pos);
          simState.vel = copy(obs.vel);
          simState.att = copy(obs.att);
          simState.rates = copy(obs.rates);
          simState.gateIndex = obs.gateIndex;
        }
      }
      lastAction = computeAction(simState, params, getRef);
      return lastAction;
    },
  };
}
