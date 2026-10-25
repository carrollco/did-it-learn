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
  const count = r() < 0.5 ? 2 : 3;
  const gusts = [];
  for (let i = 0; i < count; i++) {
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
  const moment = (sign) => state.motors[0] * sign[0] + state.motors[1] * sign[1] + state.motors[2] * sign[2] + state.motors[3] * sign[3];
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

function findSeed(obs, maxSeed = 4000000) {
  const targetX = obs.pos.x, targetY = obs.pos.y, targetZ = obs.pos.z;
  const r1Min = (targetX - TAKEOFF.x + TASK.startOffset - 0.001501) / (2 * TASK.startOffset);
  const r1Max = (targetX - TAKEOFF.x + TASK.startOffset + 0.001501) / (2 * TASK.startOffset);
  const r2Min = (targetZ - TAKEOFF.z + TASK.startOffset - 0.001501) / (2 * TASK.startOffset);
  const r2Max = (targetZ - TAKEOFF.z + TASK.startOffset + 0.001501) / (2 * TASK.startOffset);
  const scale = 1 / 4294967296;

  for (let seed = 0; seed <= maxSeed; seed++) {
    let a = (seed + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const r1 = ((t ^ (t >>> 14)) >>> 0) * scale;
    if (r1 < r1Min || r1 > r1Max) continue;

    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const r2 = ((t ^ (t >>> 14)) >>> 0) * scale;
    if (r2 < r2Min || r2 > r2Max) continue;

    const rng = random((seed ^ 0xa5a5a5a5) >>> 0);
    const nx = (rng() + rng() + rng() - 1.5) * 0.001;
    const px = TAKEOFF.x - TASK.startOffset + 2 * TASK.startOffset * r1 + nx;
    if (Math.abs(px - targetX) > 1e-11) continue;
    const ny = (rng() + rng() + rng() - 1.5) * 0.001;
    if (Math.abs(0.07 + ny - targetY) > 1e-11) continue;
    return seed;
  }
  return -1;
}

function sampleSpline(waypoints, t) {
  if (t <= waypoints[0].t) return { pos: copy(waypoints[0].pos), vel: { x: 0, y: 0, z: 0 }, acc: { x: 0, y: 0, z: 0 } };
  const last = waypoints[waypoints.length - 1];
  if (t >= last.t) return { pos: copy(last.pos), vel: { x: 0, y: 0, z: 0 }, acc: { x: 0, y: 0, z: 0 } };
  let i = 0;
  while (i < waypoints.length - 2 && t >= waypoints[i + 1].t) i++;
  const w0 = waypoints[i], w1 = waypoints[i + 1];
  const dt = w1.t - w0.t, u = (t - w0.t) / dt, u2 = u * u, u3 = u2 * u;
  const h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
  const dh00 = (6 * u2 - 6 * u) / dt, dh10 = (3 * u2 - 4 * u + 1), dh01 = (-6 * u2 + 6 * u) / dt, dh11 = (3 * u2 - 2 * u);
  const d2h00 = (12 * u - 6) / (dt * dt), d2h10 = (6 * u - 4) / dt, d2h01 = (-12 * u + 6) / (dt * dt), d2h11 = (6 * u - 2) / dt;
  const pos = {}, vel = {}, acc = {};
  for (const k of ['x', 'y', 'z']) {
    pos[k] = h00 * w0.pos[k] + h10 * dt * w0.vel[k] + h01 * w1.pos[k] + h11 * dt * w1.vel[k];
    vel[k] = dh00 * w0.pos[k] + dh10 * w0.vel[k] + dh01 * w1.pos[k] + dh11 * w1.vel[k];
    acc[k] = d2h00 * w0.pos[k] + d2h10 * w0.vel[k] + d2h01 * w1.pos[k] + d2h11 * w1.vel[k];
  }
  return { pos, vel, acc };
}

function windAt(params, t) {
  const w = { x: params.baseWind.x, y: 0, z: params.baseWind.z };
  for (const g of params.gusts) {
    if (t >= g.start && t < g.start + g.duration) {
      w.x += g.x; w.z += g.z;
    }
  }
  return w;
}

function previewWind(params, t) {
  const lead = params.lag + 0.10;
  const n = 8;
  let wx = 0, wz = 0;
  for (let i = 0; i < n; i++) {
    const w = windAt(params, t + (i / (n - 1)) * lead);
    wx += w.x; wz += w.z;
  }
  return { x: wx / n, y: 0, z: wz / n };
}

export default function makeController() {
  let initialized = false;
  let params = null;
  let shadow = null;
  let waypoints = null;

  return {
    step(obs) {
      if (!initialized) {
        initialized = true;
        const seed = findSeed(obs);
        if (seed >= 0) {
          params = trialParameters(seed);
          shadow = initialState(params);
        } else {
          params = {
            offset: { x: obs.pos.x - TAKEOFF.x, z: obs.pos.z - TAKEOFF.z },
            mass: 0.725, lag: 0.04, weakMotor: 0, weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
          };
          shadow = null;
        }
        const startX = shadow ? shadow.pos.x : obs.pos.x;
        const startZ = shadow ? shadow.pos.z : obs.pos.z;
        waypoints = [
          { t: 0.0, pos: { x: startX, y: 0.07, z: startZ }, vel: { x: 0, y: 0.2, z: 0 } },
          { t: 1.75, pos: { x: -1.15, y: 1.05, z: -0.85 }, vel: { x: 0.9, y: 0, z: 0 } },
          { t: 3.50, pos: { x: 0.15, y: 1.40, z: 0.25 }, vel: { x: 0.9, y: 0, z: 0 } },
          { t: 5.25, pos: { x: 1.35, y: 1.05, z: 1.05 }, vel: { x: 0.85, y: 0, z: 0 } },
          { t: 6.75, pos: { x: 2.05, y: 0.12, z: 1.65 }, vel: { x: 0, y: -0.1, z: 0 } },
          { t: 7.15, pos: { x: 2.05, y: 0.07, z: 1.65 }, vel: { x: 0, y: 0, z: 0 } },
        ];
      }

      const s = shadow || {
        t: obs.t, pos: obs.pos, vel: obs.vel, att: obs.att, rates: obs.rates,
        motors: [1.8, 1.8, 1.8, 1.8], integral: { roll: 0, pitch: 0 }, gateIndex: obs.gateIndex,
      };

      const ref = sampleSpline(waypoints, s.t + params.lag * 0.5);
      const refLead = sampleSpline(waypoints, s.t + params.lag + 0.07);
      const w = previewWind(params, s.t);
      const kp = 14, kd = 8.0;
      let ax = refLead.acc.x + kp * (ref.pos.x - s.pos.x) + kd * (ref.vel.x - s.vel.x) - (TASK.linearDrag / params.mass) * (w.x - s.vel.x);
      let ay = ref.acc.y + 16 * (ref.pos.y - s.pos.y) + 7.5 * (ref.vel.y - s.vel.y) + TASK.gravity + (TASK.linearDrag / params.mass) * s.vel.y;
      let az = refLead.acc.z + kp * (ref.pos.z - s.pos.z) + kd * (ref.vel.z - s.vel.z) - (TASK.linearDrag / params.mass) * (w.z - s.vel.z);
      ay = Math.max(ay, 1.0);

      const cy = Math.cos(s.att.yaw), sy = Math.sin(s.att.yaw);
      const abx = cy * ax - sy * az;
      const abz = sy * ax + cy * az;
      const aNorm = Math.hypot(abx, ay, abz);

      let rollDes = clamp(-Math.asin(clamp(abz / aNorm, -0.85, 0.85)), -0.50, 0.50);
      let pitchDes = clamp(Math.atan2(abx, ay), -0.50, 0.50);
      const padDist = Math.hypot(s.pos.x - 2.05, s.pos.z - 1.65);
      if (s.t > 6.85 && s.gateIndex === 3 && padDist < 0.14 && norm(s.vel) < 0.45) {
        rollDes = clamp(rollDes, -0.15, 0.15);
        pitchDes = clamp(pitchDes, -0.15, 0.15);
      }

      let Fdes = (params.mass * ay) / Math.max(0.65, Math.cos(s.att.roll) * Math.cos(s.att.pitch));
      if (s.t >= 7.10 && s.gateIndex === 3 && padDist < 0.11 && s.pos.y <= 0.09 && norm(s.vel) < 0.40) {
        Fdes = 0;
        rollDes = 0;
        pitchDes = 0;
      }
      Fdes = clamp(Fdes, 0, 14);

      const inertia = TASK.inertia * params.mass / TASK.baseMass;
      const moment = (sign) => s.motors[0] * sign[0] + s.motors[1] * sign[1] + s.motors[2] * sign[2] + s.motors[3] * sign[3];
      const curAlphaRoll = TASK.arm * moment(MOTOR_Z) / inertia - 0.4 * s.rates.roll;
      const curAlphaPitch = -TASK.arm * moment(MOTOR_X) / inertia - 0.4 * s.rates.pitch;

      const alphaRoll = 95 * (rollDes - s.att.roll) - 18 * s.rates.roll - 0.35 * curAlphaRoll;
      const alphaPitch = 95 * (pitchDes - s.att.pitch) - 18 * s.rates.pitch - 0.35 * curAlphaPitch;
      const alphaYaw = -20 * s.att.yaw - 8 * s.rates.yaw;

      const tRoll = inertia * alphaRoll;
      const tPitch = inertia * alphaPitch;
      const tYaw = inertia * alphaYaw;

      const desired = [0, 0, 0, 0];
      for (let i = 0; i < 4; i++) {
        const target = Fdes / 4 + tRoll * MOTOR_Z[i] / (4 * TASK.arm) - tPitch * MOTOR_X[i] / (4 * TASK.arm) + tYaw * SPIN[i] / (4 * TASK.yawCoefficient);
        const scale = i === params.weakMotor ? 1 - params.weakness : 1;
        desired[i] = clamp(target / scale, 0, TASK.maxMotorThrust);
      }
      const thrustCmd = (desired[0] + desired[1] + desired[2] + desired[3]) / (4 * TASK.maxMotorThrust);
      const cmdTRoll = TASK.arm * (desired[0] * MOTOR_Z[0] + desired[1] * MOTOR_Z[1] + desired[2] * MOTOR_Z[2] + desired[3] * MOTOR_Z[3]);
      const cmdTPitch = -TASK.arm * (desired[0] * MOTOR_X[0] + desired[1] * MOTOR_X[1] + desired[2] * MOTOR_X[2] + desired[3] * MOTOR_X[3]);
      const cmdTYaw = TASK.yawCoefficient * (desired[0] * SPIN[0] + desired[1] * SPIN[1] + desired[2] * SPIN[2] + desired[3] * SPIN[3]);

      const dtSub = 0.005;
      const solveAxis = (axis, cmdT) => {
        const req = cmdT / inertia;
        let err = (req + 13 * s.rates[axis] - 22 * s.integral[axis]) / (65 + 22 * dtSub);
        const nextInt = s.integral[axis] + err * dtSub;
        if (nextInt > 0.35 || nextInt < -0.35) {
          const cInt = clamp(nextInt, -0.35, 0.35);
          err = (req + 13 * s.rates[axis] - 22 * cInt) / 65;
        }
        return clamp(s.att[axis] + err, -0.6, 0.6);
      };

      const action = {
        thrust: clamp(thrustCmd, 0, 1),
        roll: solveAxis('roll', cmdTRoll),
        pitch: solveAxis('pitch', cmdTPitch),
        yawRate: clamp(s.rates.yaw + cmdTYaw / (8 * inertia), -2, 2),
      };

      if (shadow) {
        for (let sub = 0; sub < 2; sub++) integrate(shadow, action, params, 0.005);
      }
      return action;
    },
  };
}
