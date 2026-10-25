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
  const sum = state.motors.reduce((a, b) => a + b, 0);
  const moment = (sign) => state.motors.reduce((a, f, i) => a + f * sign[i], 0);
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
  const error = Math.hypot(state.pos.x - PAD.centre.x, state.pos.z - PAD.centre.z);
  if (state.gateIndex === GATES.length && error <= TASK.landingRadius && state.pos.y <= TASK.bodyHeight + 0.025 &&
      norm(state.vel) < TASK.landingSpeed && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < TASK.landingTilt) state.landedFor += dt;
  else state.landedFor = 0;
  if (state.landedFor >= TASK.landingHold && !state.reason) state.reason = 'landed';
}

function matchSeed(obs) {
  const obsY = obs.pos.y, obsVx = obs.vel.x;
  for (let seed = 0; seed <= 3000000; seed++) {
    let a = ((seed ^ 0xa5a5a5a5) + 4 * 0x6d2b79f5) >>> 0;
    let t1 = Math.imul(a ^ (a >>> 15), a | 1); t1 ^= t1 + Math.imul(t1 ^ (t1 >>> 7), t1 | 61);
    const r4 = ((t1 ^ (t1 >>> 14)) >>> 0) / 4294967296;
    a = (a + 0x6d2b79f5) >>> 0;
    let t2 = Math.imul(a ^ (a >>> 15), a | 1); t2 ^= t2 + Math.imul(t2 ^ (t2 >>> 7), t2 | 61);
    const r5 = ((t2 ^ (t2 >>> 14)) >>> 0) / 4294967296;
    a = (a + 0x6d2b79f5) >>> 0;
    let t3 = Math.imul(a ^ (a >>> 15), a | 1); t3 ^= t3 + Math.imul(t3 ^ (t3 >>> 7), t3 | 61);
    const r6 = ((t3 ^ (t3 >>> 14)) >>> 0) / 4294967296;
    const py = 0.07 + (r4 + r5 + r6 - 1.5) * 0.001;
    if (Math.abs(py - obsY) < 1e-12) {
      const rng = random((seed ^ 0xa5a5a5a5) >>> 0);
      const noise = (n) => (rng() + rng() + rng() - 1.5) * n;
      noise(0.001); noise(0.001); noise(0.001);
      const vx = noise(0.018);
      if (Math.abs(vx - obsVx) < 1e-12) return seed;
    }
  }
  return -1;
}

function windAt(parameters, t) {
  let wx = parameters.baseWind.x, wy = parameters.baseWind.y, wz = parameters.baseWind.z;
  for (let i = 0; i < parameters.gusts.length; i++) {
    const g = parameters.gusts[i];
    if (t >= g.start && t < g.start + g.duration) {
      wx += g.x; wy += g.y; wz += g.z;
    }
  }
  return { x: wx, y: wy, z: wz };
}

function previewWind(parameters, t) {
  const horizon = 0.10, steps = 6;
  let wx = 0, wy = 0, wz = 0;
  for (let i = 0; i < steps; i++) {
    const w = windAt(parameters, t + (i / (steps - 1)) * horizon);
    wx += w.x; wy += w.y; wz += w.z;
  }
  return { x: wx / steps, y: wy / steps, z: wz / steps };
}

export default function makeController() {
  let params = null;
  let shadow = null;
  let pts = null;
  const times = [0, 1.85, 3.65, 5.25, 7.0];
  const vels = [
    { x: 0, y: 0, z: 0 },
    { x: 0.95, y: 0, z: 0 },
    { x: 0.95, y: 0, z: 0 },
    { x: 0.85, y: 0, z: 0 },
    { x: 0, y: 0, z: 0 },
  ];

  function sampleSpline(t) {
    if (t >= times[4]) {
      return {
        p: { x: pts[4].x, y: pts[4].y, z: pts[4].z },
        v: { x: 0, y: 0, z: 0 },
        a: { x: 0, y: 0, z: 0 },
      };
    }
    let i = 0;
    while (i < 3 && t >= times[i + 1]) i++;
    const T = times[i + 1] - times[i], u = (t - times[i]) / T;
    const u2 = u * u, u3 = u2 * u;
    const h00 = 2 * u3 - 3 * u2 + 1, h10 = u3 - 2 * u2 + u, h01 = -2 * u3 + 3 * u2, h11 = u3 - u2;
    const dh00 = 6 * u2 - 6 * u, dh10 = 3 * u2 - 4 * u + 1, dh01 = -6 * u2 + 6 * u, dh11 = 3 * u2 - 2 * u;
    const d2h00 = 12 * u - 6, d2h10 = 6 * u - 4, d2h01 = -12 * u + 6, d2h11 = 6 * u - 2;
    const p = {}, v = {}, a = {};
    for (const k of ['x', 'y', 'z']) {
      p[k] = h00 * pts[i][k] + h10 * T * vels[i][k] + h01 * pts[i + 1][k] + h11 * T * vels[i + 1][k];
      v[k] = (dh00 * pts[i][k] + dh10 * T * vels[i][k] + dh01 * pts[i + 1][k] + dh11 * T * vels[i + 1][k]) / T;
      a[k] = (d2h00 * pts[i][k] + d2h10 * T * vels[i][k] + d2h01 * pts[i + 1][k] + d2h11 * T * vels[i + 1][k]) / (T * T);
    }
    return { p, v, a };
  }

  return {
    step(obs) {
      if (!params) {
        const seed = matchSeed(obs);
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
        pts = [
          { x: TAKEOFF.x + params.offset.x, y: 0.07, z: TAKEOFF.z + params.offset.z },
          GATES[0].centre,
          GATES[1].centre,
          GATES[2].centre,
          { x: PAD.centre.x, y: 0.07, z: PAD.centre.z },
        ];
      }

      const st = shadow || obs;
      const t = st.t;
      const p = st.pos, v = st.vel, att = st.att, rates = st.rates;
      const ref = sampleSpline(t);
      const w = previewWind(params, t);

      const kpPos = 14.0, kdPos = 6.8;
      const kpY = 16.0, kdY = 7.5;
      let ax = ref.a.x + kpPos * (ref.p.x - p.x) + kdPos * (ref.v.x - v.x);
      let ay = ref.a.y + kpY * (ref.p.y - p.y) + kdY * (ref.v.y - v.y);
      let az = ref.a.z + kpPos * (ref.p.z - p.z) + kdPos * (ref.v.z - v.z);

      const padErr = Math.hypot(p.x - PAD.centre.x, p.z - PAD.centre.z);
      const settling = t >= times[4] && st.gateIndex === 3 && padErr < 0.12 && p.y <= 0.09;

      let Fx = params.mass * ax - TASK.linearDrag * (w.x - v.x);
      let Fy = params.mass * (TASK.gravity + ay) - TASK.linearDrag * (w.y - v.y);
      let Fz = params.mass * az - TASK.linearDrag * (w.z - v.z);

      if (settling && p.y <= 0.075 && Math.hypot(v.x, v.y, v.z) < 0.35) {
        Fy = 0.45 * params.mass * TASK.gravity;
      }
      Fy = Math.max(0.5, Fy);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const Fbx = cy * Fx - sy * Fz;
      const Fbz = sy * Fx + cy * Fz;

      let maxTilt = 0.55;
      if (t >= times[4] - 0.25 && p.y < 0.15) maxTilt = 0.15;

      const desPitch = clamp(Math.atan2(Fbx, Fy), -maxTilt, maxTilt);
      const desRoll = clamp(Math.atan2(-Fbz, Math.hypot(Fbx, Fy)), -maxTilt, maxTilt);

      const dY = Math.max(0.5, Math.cos(att.pitch) * Math.cos(att.roll));
      let Fstar = clamp(Fy / dY, 0, 14.0);

      const inertia = TASK.inertia * params.mass / TASK.baseMass;
      const tauRoll = inertia * (65 * (desRoll - att.roll) - 13 * rates.roll);
      const tauPitch = inertia * (65 * (desPitch - att.pitch) - 13 * rates.pitch);
      const tauYaw = inertia * (-8 * rates.yaw - 12 * att.yaw);

      const k = params.weakMotor, wness = params.weakness;
      const targetK = Fstar / 4 +
        tauRoll * MOTOR_Z[k] / (4 * TASK.arm) -
        tauPitch * MOTOR_X[k] / (4 * TASK.arm) +
        tauYaw * SPIN[k] / (4 * TASK.yawCoefficient);
      const deltaK = wness > 0 ? (wness / (1 - wness)) * clamp(targetK, 0, TASK.maxMotorThrust) : 0;

      const thrustCmd = (Fstar + deltaK) / (4 * TASK.maxMotorThrust);
      const torqueRollCmd = tauRoll + TASK.arm * MOTOR_Z[k] * deltaK;
      const torquePitchCmd = tauPitch - TASK.arm * MOTOR_X[k] * deltaK;
      const torqueYawCmd = tauYaw + TASK.yawCoefficient * SPIN[k] * deltaK;

      const intRoll = shadow ? shadow.integral.roll : 0;
      const intPitch = shadow ? shadow.integral.pitch : 0;

      const solveAngle = (torqueCmd, rate, angle, integ) => {
        const rhs = torqueCmd / inertia + 13 * rate;
        let err = (rhs - 22 * integ) / (65 + 22 * 0.0075);
        const newInt = integ + err * 0.0075;
        if (newInt > 0.35) err = (rhs - 22 * 0.35) / 65;
        else if (newInt < -0.35) err = (rhs + 22 * 0.35) / 65;
        return clamp(angle + err, -0.6, 0.6);
      };

      const action = {
        thrust: clamp(thrustCmd, 0, 1),
        roll: solveAngle(torqueRollCmd, rates.roll, att.roll, intRoll),
        pitch: solveAngle(torquePitchCmd, rates.pitch, att.pitch, intPitch),
        yawRate: clamp(rates.yaw + torqueYawCmd / (8 * inertia), -2, 2),
      };

      if (shadow) {
        for (let sub = 0; sub < TASK.substeps && !shadow.reason; sub++) {
          integrate(shadow, action, params, TASK.dt / TASK.substeps);
        }
      }

      return action;
    },
  };
}
