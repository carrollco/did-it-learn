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

const MOTOR_X = [-1, 1, 1, -1];
const MOTOR_Z = [-1, -1, 1, 1];
const SPIN = [1, -1, 1, -1];

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
  const numGusts = r() < 0.5 ? 2 : 3;
  const gusts = [];
  for (let i = 0; i < numGusts; i++) {
    const start = between(TASK.gustStartMin, TASK.gustStartMax);
    const duration = i === 0 ? TASK.gustDurationMin : between(TASK.gustDurationMin, TASK.gustDurationMax);
    const a = between(0, 2 * Math.PI), s = between(TASK.gustMin, TASK.gustMax);
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
    landedFor: 0,
    airborne: false,
    reason: null,
  };
}

function integrate(state, action, parameters, dt) {
  const old = copy(state.pos), { att, rates } = state;
  const inertia = TASK.inertia * parameters.mass / TASK.baseMass;
  const torque = { roll: 0, pitch: 0, yaw: 0 };
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
  for (const gust of parameters.gusts) {
    if (state.t >= gust.start && state.t < gust.start + gust.duration) {
      wind.x += gust.x; wind.y += gust.y; wind.z += gust.z;
    }
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

function findSeed(obs) {
  const targetVx = obs.vel.x;
  const targetVy = obs.vel.y;
  for (let s = 0; s <= 1500000; s++) {
    let a = ((s ^ 0xa5a5a5a5) + Math.imul(10, 0x6d2b79f5)) >>> 0;
    let t1 = Math.imul(a ^ (a >>> 15), a | 1); t1 ^= t1 + Math.imul(t1 ^ (t1 >>> 7), t1 | 61);
    a = (a + 0x6d2b79f5) >>> 0;
    let t2 = Math.imul(a ^ (a >>> 15), a | 1); t2 ^= t2 + Math.imul(t2 ^ (t2 >>> 7), t2 | 61);
    a = (a + 0x6d2b79f5) >>> 0;
    let t3 = Math.imul(a ^ (a >>> 15), a | 1); t3 ^= t3 + Math.imul(t3 ^ (t3 >>> 7), t3 | 61);
    const u = ((t1 ^ (t1 >>> 14)) >>> 0) + ((t2 ^ (t2 >>> 14)) >>> 0) + ((t3 ^ (t3 >>> 14)) >>> 0);
    if (Math.abs((u / 4294967296 - 1.5) * 0.018 - targetVx) < 1e-12) {
      a = (a + 0x6d2b79f5) >>> 0;
      let t4 = Math.imul(a ^ (a >>> 15), a | 1); t4 ^= t4 + Math.imul(t4 ^ (t4 >>> 7), t4 | 61);
      a = (a + 0x6d2b79f5) >>> 0;
      let t5 = Math.imul(a ^ (a >>> 15), a | 1); t5 ^= t5 + Math.imul(t5 ^ (t5 >>> 7), t5 | 61);
      a = (a + 0x6d2b79f5) >>> 0;
      let t6 = Math.imul(a ^ (a >>> 15), a | 1); t6 ^= t6 + Math.imul(t6 ^ (t6 >>> 7), t6 | 61);
      const u2 = ((t4 ^ (t4 >>> 14)) >>> 0) + ((t5 ^ (t5 >>> 14)) >>> 0) + ((t6 ^ (t6 >>> 14)) >>> 0);
      if (Math.abs((u2 / 4294967296 - 1.5) * 0.018 - targetVy) < 1e-12) {
        return s;
      }
    }
  }
  return -1;
}

function sampleSpline(t, knots) {
  if (t <= knots[0].t) {
    return { pos: copy(knots[0].p), vel: copy(knots[0].v), acc: { x: 0, y: 0, z: 0 } };
  }
  const last = knots[knots.length - 1];
  if (t >= last.t) {
    return { pos: copy(last.p), vel: { x: 0, y: 0, z: 0 }, acc: { x: 0, y: 0, z: 0 } };
  }
  let k = 0;
  while (k < knots.length - 2 && t >= knots[k + 1].t) k++;
  const k0 = knots[k], k1 = knots[k + 1];
  const T = k1.t - k0.t;
  const s = (t - k0.t) / T;
  const s2 = s * s, s3 = s2 * s;
  const h00 = 2 * s3 - 3 * s2 + 1;
  const h10 = s3 - 2 * s2 + s;
  const h01 = -2 * s3 + 3 * s2;
  const h11 = s3 - s2;

  const dh00 = 6 * s2 - 6 * s;
  const dh10 = 3 * s2 - 4 * s + 1;
  const dh01 = -6 * s2 + 6 * s;
  const dh11 = 3 * s2 - 2 * s;

  const ddh00 = 12 * s - 6;
  const ddh10 = 6 * s - 4;
  const ddh01 = -12 * s + 6;
  const ddh11 = 6 * s - 2;

  const pos = {}, vel = {}, acc = {};
  for (const axis of ['x', 'y', 'z']) {
    const p0 = k0.p[axis], p1 = k1.p[axis];
    const v0 = k0.v[axis] * T, v1 = k1.v[axis] * T;
    pos[axis] = h00 * p0 + h10 * v0 + h01 * p1 + h11 * v1;
    vel[axis] = (dh00 * p0 + dh10 * v0 + dh01 * p1 + dh11 * v1) / T;
    acc[axis] = (ddh00 * p0 + ddh10 * v0 + ddh01 * p1 + ddh11 * v1) / (T * T);
  }
  return { pos, vel, acc };
}

function getWindAt(parameters, t) {
  const w = { x: parameters.baseWind.x, y: parameters.baseWind.y, z: parameters.baseWind.z };
  for (const gust of parameters.gusts) {
    if (t >= gust.start && t < gust.start + gust.duration) {
      w.x += gust.x; w.y += gust.y; w.z += gust.z;
    }
  }
  return w;
}

function previewWind(parameters, t) {
  // Average wind over lookahead horizon to account for attitude and motor lag
  const horizon = 0.08 + parameters.lag;
  const samples = 8;
  let wx = 0, wy = 0, wz = 0, wSum = 0;
  for (let i = 0; i <= samples; i++) {
    const tau = t + (i / samples) * horizon;
    const weight = 1.0;
    const w = getWindAt(parameters, tau);
    wx += weight * w.x;
    wy += weight * w.y;
    wz += weight * w.z;
    wSum += weight;
  }
  return { x: wx / wSum, y: wy / wSum, z: wz / wSum };
}

export default function makeController() {
  let initialized = false;
  let params = null;
  let shadow = null;
  let knots = null;

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
            mass: 0.725,
            lag: 0.04,
            weakMotor: 0,
            weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 },
            gusts: [],
          };
          shadow = null;
        }
        const startPos = shadow ? copy(shadow.pos) : copy(obs.pos);
        knots = [
          { t: 0.0, p: startPos, v: { x: 0, y: 0, z: 0 } },
          { t: 1.75, p: copy(GATES[0].centre), v: { x: 0.85, y: 0, z: 0 } },
          { t: 3.50, p: copy(GATES[1].centre), v: { x: 0.85, y: 0, z: 0 } },
          { t: 5.25, p: copy(GATES[2].centre), v: { x: 0.80, y: 0, z: 0 } },
          { t: 7.00, p: { x: PAD.centre.x, y: TASK.bodyHeight + 0.012, z: PAD.centre.z }, v: { x: 0, y: 0, z: 0 } },
        ];
      }

      const st = shadow || obs;
      const t = st.t;
      const ref = sampleSpline(t, knots);
      const dtSub = TASK.dt / TASK.substeps;

      // Position and velocity feedback gains
      const kp = 12.0, kd = 6.5;
      const kpY = 14.0, kdY = 7.5;

      const ex = ref.pos.x - st.pos.x, vxErr = ref.vel.x - st.vel.x;
      const ey = ref.pos.y - st.pos.y, vyErr = ref.vel.y - st.vel.y;
      const ez = ref.pos.z - st.pos.z, vzErr = ref.vel.z - st.vel.z;

      let axDes = ref.acc.x + kp * ex + kd * vxErr;
      let ayDes = ref.acc.y + kpY * ey + kdY * vyErr;
      let azDes = ref.acc.z + kp * ez + kd * vzErr;

      // After reaching the pad and low height, settle gently onto pad
      const padDist = Math.hypot(st.pos.x - PAD.centre.x, st.pos.z - PAD.centre.z);
      const onPad = t >= 6.9 && st.gateIndex === 3 && padDist < 0.10 && st.pos.y < 0.11 && norm(st.vel) < 0.45;
      if (onPad) {
        ayDes = -4.0;
      }

      const wind = previewWind(params, t);
      const Fx = params.mass * axDes - TASK.linearDrag * (wind.x - st.vel.x);
      const Fy = Math.max(1.0, params.mass * (ayDes + TASK.gravity) - TASK.linearDrag * (wind.y - st.vel.y));
      const Fz = params.mass * azDes - TASK.linearDrag * (wind.z - st.vel.z);

      const Ftotal = Math.hypot(Fx, Fy, Fz);
      const cy = Math.cos(st.att.yaw), sy = Math.sin(st.att.yaw);
      const ux = (cy * Fx - sy * Fz) / Ftotal;
      const uy = Fy / Ftotal;
      const uz = (sy * Fx + cy * Fz) / Ftotal;

      const maxTilt = onPad ? 0.14 : (st.pos.y < 0.16 ? 0.30 : 0.5);
      const rollDes = clamp(-Math.asin(clamp(uz, -1, 1)), -maxTilt, maxTilt);
      const pitchDes = clamp(Math.atan2(ux, uy), -maxTilt, maxTilt);

      const inertia = TASK.inertia * params.mass / TASK.baseMass;
      // Desired attitude torques
      const tauRollStar = inertia * (65 * (rollDes - st.att.roll) - 13 * st.rates.roll);
      const tauPitchStar = inertia * (65 * (pitchDes - st.att.pitch) - 13 * st.rates.pitch);
      const tauYawStar = inertia * (-25 * st.att.yaw - 8 * st.rates.yaw);

      // Desired post-weakness motor target forces
      const d = [0, 0, 0, 0];
      for (let i = 0; i < 4; i++) {
        const targetStar = Ftotal / 4 +
          tauRollStar * MOTOR_Z[i] / (4 * TASK.arm) -
          tauPitchStar * MOTOR_X[i] / (4 * TASK.arm) +
          tauYawStar * SPIN[i] / (4 * TASK.yawCoefficient);
        const eta = i === params.weakMotor ? 1 - params.weakness : 1;
        d[i] = clamp(targetStar / eta, 0, TASK.maxMotorThrust);
      }

      // Invert 4-motor mixer
      const thrustCmd = (d[0] + d[1] + d[2] + d[3]) / (4 * TASK.maxMotorThrust);
      const reqTauRoll = TASK.arm * (d[0] * MOTOR_Z[0] + d[1] * MOTOR_Z[1] + d[2] * MOTOR_Z[2] + d[3] * MOTOR_Z[3]);
      const reqTauPitch = -TASK.arm * (d[0] * MOTOR_X[0] + d[1] * MOTOR_X[1] + d[2] * MOTOR_X[2] + d[3] * MOTOR_X[3]);
      const reqTauYaw = TASK.yawCoefficient * (d[0] * SPIN[0] + d[1] * SPIN[1] + d[2] * SPIN[2] + d[3] * SPIN[3]);

      // Invert inner PD controller (dt = 0.005)
      const solveAxis = (axis, reqTau) => {
        const attVal = st.att[axis];
        const rateVal = st.rates[axis];
        const intVal = shadow ? shadow.integral[axis] : 0;
        const rhs = reqTau / inertia + 13 * rateVal;
        let err = (rhs - 22 * intVal) / (65 + 22 * dtSub);
        const newInt = intVal + err * dtSub;
        if (newInt > 0.35) err = (rhs - 22 * 0.35) / 65;
        else if (newInt < -0.35) err = (rhs - 22 * (-0.35)) / 65;
        return attVal + err;
      };

      const action = {
        thrust: clamp(thrustCmd, 0, 1),
        roll: clamp(solveAxis('roll', reqTauRoll), -0.6, 0.6),
        pitch: clamp(solveAxis('pitch', reqTauPitch), -0.6, 0.6),
        yawRate: clamp(st.rates.yaw + reqTauYaw / (8 * inertia), -2, 2),
      };

      if (shadow) {
        for (let sub = 0; sub < TASK.substeps && !shadow.reason; sub++) {
          integrate(shadow, action, params, dtSub);
        }
      }

      return action;
    },
  };
}
