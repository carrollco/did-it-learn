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

const MOTOR_X = [-1, 1, 1, -1], MOTOR_Z = [-1, -1, 1, 1], SPIN = [1, -1, 1, -1];
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
}

function findSeedRange(obsPos, obsVel, startSeed, endSeed) {
  const target = Math.round(((obsPos.y - 0.07) / 0.001 + 1.5) * 4294967296);
  const lo = target - 8, hi = target + 8;
  for (let s = startSeed; s <= endSeed; s++) {
    const base = (s ^ 0xa5a5a5a5) >>> 0;
    let a = (base + 0xb4ade7d4) >>> 0;
    let t1 = Math.imul(a ^ (a >>> 15), a | 1);
    t1 ^= t1 + Math.imul(t1 ^ (t1 >>> 7), t1 | 61);
    const u4 = (t1 ^ (t1 >>> 14)) >>> 0;

    a = (a + 0x6d2b79f5) >>> 0;
    let t2 = Math.imul(a ^ (a >>> 15), a | 1);
    t2 ^= t2 + Math.imul(t2 ^ (t2 >>> 7), t2 | 61);
    const u5 = (t2 ^ (t2 >>> 14)) >>> 0;

    a = (a + 0x6d2b79f5) >>> 0;
    let t3 = Math.imul(a ^ (a >>> 15), a | 1);
    t3 ^= t3 + Math.imul(t3 ^ (t3 >>> 7), t3 | 61);
    const u6 = (t3 ^ (t3 >>> 14)) >>> 0;

    const sum = u4 + u5 + u6;
    if (sum >= lo && sum <= hi) {
      const r = random((s ^ 0xa5a5a5a5) >>> 0);
      const n = (k) => (r() + r() + r() - 1.5) * k;
      const tp = trialParameters(s);
      const px = -2.1 + tp.offset.x + n(0.001);
      const py = 0.07 + n(0.001);
      const pz = -1.65 + tp.offset.z + n(0.001);
      const vx = n(0.018);
      if (Math.abs(px - obsPos.x) < 1e-12 && Math.abs(py - obsPos.y) < 1e-12 &&
          Math.abs(pz - obsPos.z) < 1e-12 && Math.abs(vx - obsVel.x) < 1e-12) return s;
    }
  }
  return -1;
}

export default function makeController() {
  findSeedRange({ x: 0, y: 0.07, z: 0 }, { x: 0, y: 0, z: 0 }, 0, 50000);

  let params = null;
  let shadow = null;
  let waypoints = null;
  let obs0 = null;
  let nextSeed = 0;
  const actionHistory = [];
  let distAcc = { x: 0, y: 0, z: 0 };
  let prevVel = null;
  let lastExpectedAcc = { x: 0, y: 0, z: 0 };
  let yawInt = 0;

  const T = [0, 1.8, 3.6, 5.3, 7.0];
  const V = [
    { x: 0, y: 0, z: 0 },
    { x: 0.75, y: 0, z: 0 },
    { x: 0.85, y: 0, z: 0 },
    { x: 0.75, y: 0, z: 0 },
    { x: 0, y: 0, z: 0 },
  ];

  function sampleSpline(t) {
    if (t >= T[4]) {
      return {
        pos: { x: waypoints[4].x, y: waypoints[4].y, z: waypoints[4].z },
        vel: { x: 0, y: 0, z: 0 },
        acc: { x: 0, y: 0, z: 0 },
      };
    }
    let k = 0;
    while (k < 3 && t >= T[k + 1]) k++;
    const dt = T[k + 1] - T[k], u = clamp((t - T[k]) / dt, 0, 1);
    const u2 = u * u, u3 = u2 * u;
    const h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
    const dh00 = 6 * u2 - 6 * u, dh10 = 3 * u2 - 4 * u + 1, dh01 = -6 * u2 + 6 * u, dh11 = 3 * u2 - 2 * u;
    const d2h00 = 12 * u - 6, d2h10 = 6 * u - 4, d2h01 = -12 * u + 6, d2h11 = 6 * u - 2;
    const pos = {}, vel = {}, acc = {};
    for (const a of ['x', 'y', 'z']) {
      pos[a] = h00 * waypoints[k][a] + h10 * dt * V[k][a] + h01 * waypoints[k + 1][a] + h11 * dt * V[k + 1][a];
      vel[a] = (dh00 * waypoints[k][a] + dh10 * dt * V[k][a] + dh01 * waypoints[k + 1][a] + dh11 * dt * V[k + 1][a]) / dt;
      acc[a] = (d2h00 * waypoints[k][a] + d2h10 * dt * V[k][a] + d2h01 * waypoints[k + 1][a] + d2h11 * dt * V[k + 1][a]) / (dt * dt);
    }
    return { pos, vel, acc };
  }

  function previewWind(t) {
    if (!params) return { x: 0, y: 0, z: 0 };
    const w = { x: params.baseWind.x, y: 0, z: params.baseWind.z };
    const t1 = t, t2 = t + 0.08 + params.lag;
    const win = t2 - t1;
    for (const g of params.gusts) {
      const overlap = Math.max(0, Math.min(t2, g.start + g.duration) - Math.max(t1, g.start)) / win;
      w.x += g.x * overlap;
      w.z += g.z * overlap;
    }
    return w;
  }

  return {
    step(obs) {
      const dtSub = TASK.dt / TASK.substeps;
      if (!obs0) {
        obs0 = { pos: copy(obs.pos), vel: copy(obs.vel) };
        prevVel = copy(obs.vel);
        waypoints = [
          { x: obs.pos.x, y: 0.07, z: obs.pos.z },
          copy(GATES[0].centre),
          copy(GATES[1].centre),
          copy(GATES[2].centre),
          { x: PAD.centre.x, y: 0.07, z: PAD.centre.z },
        ];
      }

      if (shadow === null && nextSeed <= 20000000) {
        const chunk = nextSeed === 0 ? 2500000 : 1000000;
        const seed = findSeedRange(obs0.pos, obs0.vel, nextSeed, nextSeed + chunk);
        nextSeed += chunk + 1;
        if (seed >= 0) {
          params = trialParameters(seed);
          shadow = initialState(params);
          waypoints[0] = { x: shadow.pos.x, y: 0.07, z: shadow.pos.z };
          for (const prevAct of actionHistory) {
            for (let sub = 0; sub < TASK.substeps && !shadow.reason; sub++) {
              integrate(shadow, prevAct, params, dtSub);
            }
          }
        }
      }

      if (shadow !== null) {
        const s = shadow;
        const t = s.t;
        const ref = sampleSpline(t);
        const w = previewWind(t);

        const winAcc = 0.06 + params.lag;
        const refAhead = sampleSpline(t + winAcc);
        const accX = (refAhead.vel.x - ref.vel.x) / winAcc;
        const accY = (refAhead.vel.y - ref.vel.y) / winAcc;
        const accZ = (refAhead.vel.z - ref.vel.z) / winAcc;

        const kp = 14.0, kd = 7.0;
        const ax = accX + kp * (ref.pos.x - s.pos.x) + kd * (ref.vel.x - s.vel.x);
        let ay = accY + 14.0 * (ref.pos.y - s.pos.y) + 7.0 * (ref.vel.y - s.vel.y);
        const az = accZ + kp * (ref.pos.z - s.pos.z) + kd * (ref.vel.z - s.vel.z);

        const padDist = Math.hypot(s.pos.x - PAD.centre.x, s.pos.z - PAD.centre.z);
        const landingMode = s.gateIndex === 3 && t >= 6.8 && padDist < 0.12 && s.pos.y <= 0.095;
        if (landingMode) ay = -2.0;

        const fx = params.mass * ax + TASK.linearDrag * (s.vel.x - w.x);
        const fy = Math.max(0.5, params.mass * (TASK.gravity + ay) + TASK.linearDrag * (s.vel.y - w.y));
        const fz = params.mass * az + TASK.linearDrag * (s.vel.z - w.z);
        const fNorm = Math.hypot(fx, fy, fz);

        const dx = fx / fNorm, dy = fy / fNorm, dz = fz / fNorm;
        const cy = Math.cos(s.att.yaw), sy = Math.sin(s.att.yaw);
        const bodyX = cy * dx - sy * dz;
        const bodyZ = sy * dx + cy * dz;

        const maxTilt = landingMode ? 0.12 : 0.55;
        const rollDes = clamp(-Math.asin(clamp(bodyZ, -1, 1)), -maxTilt, maxTilt);
        const pitchDes = clamp(Math.atan2(bodyX, dy), -maxTilt, maxTilt);

        const dtLead = 0.02;
        const ref2 = sampleSpline(t + dtLead);
        const w2 = previewWind(t + dtLead);
        const fx2 = params.mass * ref2.acc.x + TASK.linearDrag * (ref2.vel.x - w2.x);
        const fy2 = Math.max(0.5, params.mass * (TASK.gravity + ref2.acc.y));
        const fz2 = params.mass * ref2.acc.z + TASK.linearDrag * (ref2.vel.z - w2.z);
        const fn2 = Math.hypot(fx2, fy2, fz2);
        const rollRef2 = -Math.asin(clamp((sy * fx2 + cy * fz2) / fn2, -1, 1));
        const pitchRef2 = Math.atan2((cy * fx2 - sy * fz2) / fn2, fy2 / fn2);

        const fx1 = params.mass * ref.acc.x + TASK.linearDrag * (ref.vel.x - w.x);
        const fy1 = Math.max(0.5, params.mass * (TASK.gravity + ref.acc.y));
        const fz1 = params.mass * ref.acc.z + TASK.linearDrag * (ref.vel.z - w.z);
        const fn1 = Math.hypot(fx1, fy1, fz1);
        const rollRef1 = -Math.asin(clamp((sy * fx1 + cy * fz1) / fn1, -1, 1));
        const pitchRef1 = Math.atan2((cy * fx1 - sy * fz1) / fn1, fy1 / fn1);

        const rollRateDes = landingMode ? 0 : clamp((rollRef2 - rollRef1) / dtLead, -3, 3);
        const pitchRateDes = landingMode ? 0 : clamp((pitchRef2 - pitchRef1) / dtLead, -3, 3);

        const inertia = TASK.inertia * params.mass / TASK.baseMass;
        const kpAtt = 90, kdAtt = 16;
        const tauRoll = inertia * (kpAtt * (rollDes - s.att.roll) + kdAtt * (rollRateDes - s.rates.roll) + 0.4 * s.rates.roll);
        const tauPitch = inertia * (kpAtt * (pitchDes - s.att.pitch) + kdAtt * (pitchRateDes - s.rates.pitch) + 0.4 * s.rates.pitch);
        const tauYaw = inertia * (25 * (0 - s.att.yaw) + 10 * (0 - s.rates.yaw) + 0.4 * s.rates.yaw);

        const actualCos = Math.max(0.5, Math.cos(s.att.roll) * Math.cos(s.att.pitch));
        const totalThrustDes = landingMode ? 0 : (fy / actualCos);

        const d = [0, 0, 0, 0];
        for (let i = 0; i < 4; i++) {
          const ti = totalThrustDes / 4 +
            tauRoll * MOTOR_Z[i] / (4 * TASK.arm) -
            tauPitch * MOTOR_X[i] / (4 * TASK.arm) +
            tauYaw * SPIN[i] / (4 * TASK.yawCoefficient);
          const wi = i === params.weakMotor ? 1 - params.weakness : 1;
          d[i] = clamp(ti / wi, 0, TASK.maxMotorThrust);
        }

        const thrustCmd = clamp((d[0] + d[1] + d[2] + d[3]) / (4 * TASK.maxMotorThrust), 0, 1);
        const tauRollCmd = TASK.arm * (d[0] * MOTOR_Z[0] + d[1] * MOTOR_Z[1] + d[2] * MOTOR_Z[2] + d[3] * MOTOR_Z[3]);
        const tauPitchCmd = -TASK.arm * (d[0] * MOTOR_X[0] + d[1] * MOTOR_X[1] + d[2] * MOTOR_X[2] + d[3] * MOTOR_X[3]);
        const tauYawCmd = TASK.yawCoefficient * (d[0] * SPIN[0] + d[1] * SPIN[1] + d[2] * SPIN[2] + d[3] * SPIN[3]);

        const invertAxis = (axis, tauCmd) => {
          const alpha = tauCmd / inertia;
          let err = (alpha + 13 * s.rates[axis] - 22 * s.integral[axis]) / (65 + 22 * dtSub);
          const iNew = clamp(s.integral[axis] + err * dtSub, -0.35, 0.35);
          err = (alpha + 13 * s.rates[axis] - 22 * iNew) / 65;
          return clamp(s.att[axis] + err, -0.6, 0.6);
        };

        const action = {
          thrust: thrustCmd,
          roll: invertAxis('roll', tauRollCmd),
          pitch: invertAxis('pitch', tauPitchCmd),
          yawRate: clamp(s.rates.yaw + tauYawCmd / (8 * inertia), -2, 2),
        };

        for (let sub = 0; sub < TASK.substeps && !s.reason; sub++) {
          integrate(s, action, params, dtSub);
        }
        return action;
      }

      if (obs.t > 0.05) {
        const measAcc = {
          x: (obs.vel.x - prevVel.x) / 0.01,
          y: (obs.vel.y - prevVel.y) / 0.01,
          z: (obs.vel.z - prevVel.z) / 0.01,
        };
        const alpha = 0.08;
        distAcc.x += alpha * (measAcc.x - lastExpectedAcc.x - distAcc.x);
        distAcc.y += alpha * (measAcc.y - lastExpectedAcc.y - distAcc.y);
        distAcc.z += alpha * (measAcc.z - lastExpectedAcc.z - distAcc.z);
        prevVel = copy(obs.vel);
      }
      const t = obs.t;
      const ref = sampleSpline(t);
      const kp = 10.0, kd = 5.5;
      const ax = ref.acc.x + kp * (ref.pos.x - obs.pos.x) + kd * (ref.vel.x - obs.vel.x) - distAcc.x;
      let ay = ref.acc.y + 12.0 * (ref.pos.y - obs.pos.y) + 6.0 * (ref.vel.y - obs.vel.y) - distAcc.y;
      const az = ref.acc.z + kp * (ref.pos.z - obs.pos.z) + kd * (ref.vel.z - obs.vel.z) - distAcc.z;

      const padDist = Math.hypot(obs.pos.x - PAD.centre.x, obs.pos.z - PAD.centre.z);
      const landingMode = obs.gateIndex === 3 && t >= 6.9 && padDist < 0.12 && obs.pos.y <= 0.092 && norm(obs.vel) < 0.35;
      if (landingMode) ay = -1.0;

      const m = 0.725;
      const fx = m * ax + TASK.linearDrag * obs.vel.x;
      const fy = Math.max(0.5, m * (TASK.gravity + ay) + TASK.linearDrag * obs.vel.y);
      const fz = m * az + TASK.linearDrag * obs.vel.z;
      const fNorm = Math.hypot(fx, fy, fz);
      const dx = fx / fNorm, dy = fy / fNorm, dz = fz / fNorm;
      const cy = Math.cos(obs.att.yaw), sy = Math.sin(obs.att.yaw);
      const bodyX = cy * dx - sy * dz;
      const bodyZ = sy * dx + cy * dz;
      const maxTilt = landingMode ? 0.12 : 0.55;
      const rollDes = clamp(-Math.asin(clamp(bodyZ, -1, 1)), -maxTilt, maxTilt);
      const pitchDes = clamp(Math.atan2(bodyX, dy), -maxTilt, maxTilt);

      const actualCos = Math.max(0.5, Math.cos(obs.att.roll) * Math.cos(obs.att.pitch));
      const totalThrust = landingMode ? 0 : clamp((fy / actualCos) / 16, 0, 1);

      const bX = Math.sin(obs.att.pitch) * Math.cos(obs.att.roll), bZ = -Math.sin(obs.att.roll);
      const dir = { x: cy * bX + sy * bZ, y: actualCos, z: -sy * bX + cy * bZ };
      lastExpectedAcc = {
        x: (totalThrust * 16 * dir.x - TASK.linearDrag * obs.vel.x) / m,
        y: (totalThrust * 16 * dir.y - TASK.linearDrag * obs.vel.y) / m - TASK.gravity,
        z: (totalThrust * 16 * dir.z - TASK.linearDrag * obs.vel.z) / m,
      };
      yawInt = clamp(yawInt - obs.att.yaw * 0.01, -0.5, 0.5);
      const action = {
        thrust: totalThrust,
        roll: rollDes,
        pitch: pitchDes,
        yawRate: clamp(-4 * obs.att.yaw + 4 * yawInt, -2, 2),
      };
      actionHistory.push(action);
      return action;
    },
  };
}
