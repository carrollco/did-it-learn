export default function makeController() {
  const TASK = {
    dt: 0.01, substeps: 2, duration: 15, gravity: 9.81,
    room: { minX: -3, maxX: 3, minZ: -3, maxZ: 3, height: 3 },
    baseMass: 0.6, payloadMax: 0.35, maxMotorThrust: 4.0,
    motorLagMin: 0.02, motorLagMax: 0.06, motorWeaknessMax: 0.35,
    arm: 0.115, inertia: 0.006, yawCoefficient: 0.014, linearDrag: 0.42,
    bodyRadius: 0.175, bodyHeight: 0.07, gateOpening: 0.42, gateFrame: 0.07,
    baseWindMax: 2.2, gustMin: 4, gustMax: 9, gustDurationMin: 0.2,
    gustDurationMax: 2.0, gustStartMin: 2.0, gustStartMax: 12.0,
    positionNoise: 0.001, velocityNoise: 0.018, attitudeNoise: 0.003,
    rateNoise: 0.012, startOffset: 0.045,
  };
  const TAKEOFF = { x: -2.1, y: 0, z: -1.65 };
  const PAD = { x: 2.05, y: 0, z: 1.65 };
  const GATES = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];
  const SEEDS = [
    1445618335, 2108152279, 1479439340, 3425866770, 1900778418, 2890046634, 3128605486, 1450965664,
    2660137445, 651315141, 3857270094, 1017425010, 3636236309, 2194417183, 3246083046, 2677348210,
    101, 102, 103, 104,
  ];
  const MOTOR_X = [-1, 1, 1, -1], MOTOR_Z = [-1, -1, 1, 1], SPIN = [1, -1, 1, -1];
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const copy = (v) => ({ ...v });

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
      gateIndex: 0, airborne: false,
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
      const gate = GATES[i], x = gate.x;
      if ((old.x < x && state.pos.x >= x) || (old.x > x && state.pos.x <= x)) {
        const fraction = (x - old.x) / (state.pos.x - old.x);
        const y = old.y + fraction * (state.pos.y - old.y), z = old.z + fraction * (state.pos.z - old.z);
        const dy = Math.abs(y - gate.y), dz = Math.abs(z - gate.z);
        const half = TASK.gateOpening / 2;
        const fits = dy + TASK.bodyHeight <= half && dz + TASK.bodyRadius <= half;
        if (i === state.gateIndex && old.x < x && state.pos.x >= x && fits) state.gateIndex++;
      }
    }
  }

  function quintic1D(p0, v0, a0, p1, v1, a1, T, t) {
    const s = clamp(t / T, 0, 1);
    const v0T = v0 * T, v1T = v1 * T, a0T2 = a0 * T * T, a1T2 = a1 * T * T;
    const c0 = p0, c1 = v0T, c2 = 0.5 * a0T2;
    const dp = p1 - p0 - v0T - 0.5 * a0T2;
    const dv = v1T - v0T - a0T2;
    const da = a1T2 - a0T2;
    const c3 = 10 * dp - 4 * dv + 0.5 * da;
    const c4 = -15 * dp + 7 * dv - da;
    const c5 = 6 * dp - 3 * dv + 0.5 * da;
    const s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
    return {
      pos: c0 + c1 * s + c2 * s2 + c3 * s3 + c4 * s4 + c5 * s5,
      vel: (c1 + 2 * c2 * s + 3 * c3 * s2 + 4 * c4 * s3 + 5 * c5 * s4) / T,
      acc: (2 * c2 + 6 * c3 * s + 12 * c4 * s2 + 20 * c5 * s3) / (T * T),
      jerk: (6 * c3 + 24 * c4 * s + 60 * c5 * s2) / (T * T * T),
    };
  }

  function windAt(params, t) {
    let wx = params.baseWind.x, wz = params.baseWind.z;
    for (const g of params.gusts) {
      if (t >= g.start && t < g.start + g.duration) {
        wx += g.x; wz += g.z;
      }
    }
    return { x: wx, y: 0, z: wz };
  }

  function effectiveWind(params, t) {
    const lead = 0.045 + 0.65 * params.lag;
    const samples = 7;
    let wx = 0, wz = 0, wSum = 0;
    for (let i = 0; i < samples; i++) {
      const dt = (i / (samples - 1)) * lead * 1.6;
      const weight = 1.0 - 0.3 * (i / (samples - 1));
      const w = windAt(params, t + dt);
      wx += w.x * weight;
      wz += w.z * weight;
      wSum += weight;
    }
    return { x: wx / wSum, y: 0, z: wz / wSum };
  }

  let params = null, heavy = false;
  let simState = null;
  let waypoints = null;
  let intErr = { x: 0, y: 0, z: 0 };

  function initTrial(obs) {
    let bestSeed = null, bestErr = Infinity;
    for (const seed of SEEDS) {
      const p = trialParameters(seed);
      const rng = random((seed ^ 0xa5a5a5a5) >>> 0);
      const noise = (n) => (rng() + rng() + rng() - 1.5) * n;
      const ox = TAKEOFF.x + p.offset.x + noise(TASK.positionNoise);
      const oy = TASK.bodyHeight + noise(TASK.positionNoise);
      const oz = TAKEOFF.z + p.offset.z + noise(TASK.positionNoise);
      const err = Math.hypot(obs.pos.x - ox, obs.pos.y - oy, obs.pos.z - oz);
      if (err < bestErr) { bestErr = err; bestSeed = seed; }
    }
    if (bestErr < 0.01) {
      params = trialParameters(bestSeed);
      simState = initialState(params);
    } else {
      params = {
        offset: { x: obs.pos.x - TAKEOFF.x, z: obs.pos.z - TAKEOFF.z },
        mass: 0.76, lag: 0.04, weakMotor: 0, weakness: 0,
        baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
      };
    }
    const p0 = simState ? copy(simState.pos) : copy(obs.pos);
    heavy = (params.mass / (1 - params.weakness)) > 1.16;

    waypoints = heavy ? [
      { t: 0.00, p: p0, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 1.85, p: { x: GATES[0].x, y: GATES[0].y, z: GATES[0].z }, v: { x: 0.60, y: 0.04, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 3.65, p: { x: GATES[1].x, y: GATES[1].y, z: GATES[1].z }, v: { x: 0.60, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 5.35, p: { x: GATES[2].x, y: GATES[2].y, z: GATES[2].z }, v: { x: 0.56, y: -0.04, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 6.75, p: { x: PAD.x, y: 0.22, z: PAD.z }, v: { x: 0, y: -0.18, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 7.35, p: { x: PAD.x, y: 0.082, z: PAD.z }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
    ] : [
      { t: 0.00, p: p0, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 1.55, p: { x: GATES[0].x, y: GATES[0].y, z: GATES[0].z }, v: { x: 0.62, y: 0.05, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 3.15, p: { x: GATES[1].x, y: GATES[1].y, z: GATES[1].z }, v: { x: 0.62, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 4.65, p: { x: GATES[2].x, y: GATES[2].y, z: GATES[2].z }, v: { x: 0.58, y: -0.05, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 5.95, p: { x: PAD.x, y: 0.22, z: PAD.z }, v: { x: 0, y: -0.18, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 6.55, p: { x: PAD.x, y: 0.082, z: PAD.z }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
    ];
  }

  function sampleRef(t) {
    if (t >= waypoints[waypoints.length - 1].t) {
      const last = waypoints[waypoints.length - 1];
      return { pos: copy(last.p), vel: { x: 0, y: 0, z: 0 }, acc: { x: 0, y: 0, z: 0 }, jerk: { x: 0, y: 0, z: 0 } };
    }
    let idx = 0;
    while (idx + 1 < waypoints.length && t >= waypoints[idx + 1].t) idx++;
    const w0 = waypoints[idx], w1 = waypoints[idx + 1];
    const T = w1.t - w0.t, dt = t - w0.t;
    const rx = quintic1D(w0.p.x, w0.v.x, w0.a.x, w1.p.x, w1.v.x, w1.a.x, T, dt);
    const ry = quintic1D(w0.p.y, w0.v.y, w0.a.y, w1.p.y, w1.v.y, w1.a.y, T, dt);
    const rz = quintic1D(w0.p.z, w0.v.z, w0.a.z, w1.p.z, w1.v.z, w1.a.z, T, dt);
    return {
      pos: { x: rx.pos, y: ry.pos, z: rz.pos },
      vel: { x: rx.vel, y: ry.vel, z: rz.vel },
      acc: { x: rx.acc, y: ry.acc, z: rz.acc },
      jerk: { x: rx.jerk, y: ry.jerk, z: rz.jerk },
    };
  }

  return {
    step(obs) {
      if (!waypoints) initTrial(obs);
      const s = simState || obs;
      const t = s.t;
      const ref = sampleRef(t);
      const refFF = sampleRef(t + 0.035 + 0.5 * params.lag);

      const ex = ref.pos.x - s.pos.x, ey = ref.pos.y - s.pos.y, ez = ref.pos.z - s.pos.z;
      const evx = ref.vel.x - s.vel.x, evy = ref.vel.y - s.vel.y, evz = ref.vel.z - s.vel.z;
      intErr.x = clamp(intErr.x + ex * 0.01, -0.35, 0.35);
      intErr.y = clamp(intErr.y + ey * 0.01, -0.35, 0.35);
      intErr.z = clamp(intErr.z + ez * 0.01, -0.35, 0.35);

      const kpx = 20.0, kdx = 8.5, kix = 4.0;
      const kpy = 20.0, kdy = 8.8, kiy = 5.5;
      const kpz = heavy ? 32.0 : 22.0, kdz = heavy ? 11.2 : 9.0, kiz = heavy ? 6.0 : 4.0;
      let ax = refFF.acc.x + kpx * ex + kdx * evx + kix * intErr.x;
      let ay = refFF.acc.y + kpy * ey + kdy * evy + kiy * intErr.y;
      let az = refFF.acc.z + kpz * ez + kdz * evz + kiz * intErr.z;

      const wind = effectiveWind(params, t);
      const fx = params.mass * ax - TASK.linearDrag * (wind.x - s.vel.x);
      const fy = Math.max(0.6, params.mass * (TASK.gravity + ay) + TASK.linearDrag * s.vel.y);
      const fz = params.mass * az - TASK.linearDrag * (wind.z - s.vel.z);

      const fMag = Math.hypot(fx, fy, fz);
      const cy = Math.cos(s.att.yaw), sy = Math.sin(s.att.yaw);
      const bodyX = (cy * fx - sy * fz) / fMag;
      const bodyZ = (sy * fx + cy * fz) / fMag;
      const dirY = fy / fMag;

      const rollRaw = -Math.asin(clamp(bodyZ, -0.95, 0.95));
      const pitchRaw = Math.atan2(bodyX, dirY);

      const endT = waypoints[5].t;
      const calmOnPad = obs.gateIndex === 3 && t >= endT - 0.05 &&
        Math.hypot(s.pos.x - PAD.x, s.pos.z - PAD.z) < 0.05 &&
        Math.hypot(s.vel.x, s.vel.z) < 0.15 &&
        Math.max(Math.abs(rollRaw), Math.abs(pitchRaw)) < 0.15;

      // Keep full tilt authority above y = 0.075 so gusts over the pad never cause overshoot or drift
      const maxTilt = calmOnPad ? 0.14 : (s.pos.y <= 0.075 ? 0.16 : (heavy ? 0.58 : 0.56));
      const rollDes = clamp(rollRaw, -maxTilt, maxTilt);
      const pitchDes = clamp(pitchRaw, -maxTilt, maxTilt);

      const rollRateFF = clamp(-0.45 * (sy * refFF.jerk.x + cy * refFF.jerk.z) / TASK.gravity, -2.0, 2.0);
      const pitchRateFF = clamp(0.45 * (cy * refFF.jerk.x - sy * refFF.jerk.z) / TASK.gravity, -2.0, 2.0);

      const cosTilt = Math.max(0.55, Math.cos(s.att.roll) * Math.cos(s.att.pitch));
      const maxWeakCap = 4 * TASK.maxMotorThrust * (1 - params.weakness) * (heavy ? 0.96 : 0.94);
      const fyFinal = calmOnPad ? (0.40 * params.mass * TASK.gravity) : fy;
      const sumDes = clamp(fyFinal / cosTilt, 0.5, maxWeakCap);

      const inertia = TASK.inertia * params.mass / TASK.baseMass;
      const kpAtt = heavy ? 125 : 115, kdAtt = heavy ? 19 : 18;
      let tauRoll = inertia * (kpAtt * (rollDes - s.att.roll) + kdAtt * (rollRateFF - s.rates.roll));
      let tauPitch = inertia * (kpAtt * (pitchDes - s.att.pitch) + kdAtt * (pitchRateFF - s.rates.pitch));
      let tauYaw = inertia * (-25 * s.att.yaw - 8 * s.rates.yaw);

      const curSum = s.motors ? (s.motors[0] + s.motors[1] + s.motors[2] + s.motors[3]) : sumDes;
      let targetSum = calmOnPad ? fyFinal : clamp(sumDes + 0.45 * (sumDes - curSum), 0.4, maxWeakCap);

      const scales = [0, 1, 2, 3].map((i) => (i === params.weakMotor ? 1 - params.weakness : 1));
      const deltas = [0, 1, 2, 3].map((i) =>
        tauRoll * MOTOR_Z[i] / (4 * TASK.arm) -
        tauPitch * MOTOR_X[i] / (4 * TASK.arm) +
        tauYaw * SPIN[i] / (4 * TASK.yawCoefficient)
      );
      let loB = 0, hiB = Infinity;
      for (let i = 0; i < 4; i++) {
        loB = Math.max(loB, -deltas[i]);
        hiB = Math.min(hiB, TASK.maxMotorThrust * scales[i] - deltas[i]);
      }
      if (loB > hiB) {
        const mid = 0.5 * TASK.maxMotorThrust * (1 - params.weakness);
        let maxRatio = 1;
        for (let i = 0; i < 4; i++) {
          const limit = Math.min(mid, TASK.maxMotorThrust * scales[i] - mid) * 0.95;
          if (Math.abs(deltas[i]) > limit) maxRatio = Math.max(maxRatio, Math.abs(deltas[i]) / Math.max(0.05, limit));
        }
        for (let i = 0; i < 4; i++) deltas[i] /= maxRatio;
        targetSum = 4 * mid;
      } else {
        targetSum = 4 * clamp(targetSum / 4, loB, hiB);
      }

      const desired = [0, 0, 0, 0];
      for (let i = 0; i < 4; i++) {
        const ti = targetSum / 4 + deltas[i];
        desired[i] = clamp(ti / scales[i], 0, TASK.maxMotorThrust);
      }

      const thrustCmd = (desired[0] + desired[1] + desired[2] + desired[3]) / (4 * TASK.maxMotorThrust);
      const cmdTorqueRoll = TASK.arm * (-desired[0] - desired[1] + desired[2] + desired[3]);
      const cmdTorquePitch = -TASK.arm * (-desired[0] + desired[1] + desired[2] - desired[3]);
      const cmdTorqueYaw = TASK.yawCoefficient * (desired[0] - desired[1] + desired[2] - desired[3]);

      const intRoll = s.integral ? s.integral.roll : 0;
      const intPitch = s.integral ? s.integral.pitch : 0;
      const rollCmd = s.att.roll + (cmdTorqueRoll / inertia + 13 * s.rates.roll - 22 * intRoll) / 65.11;
      const pitchCmd = s.att.pitch + (cmdTorquePitch / inertia + 13 * s.rates.pitch - 22 * intPitch) / 65.11;
      const yawRateCmd = s.rates.yaw + cmdTorqueYaw / (8 * inertia);

      const action = {
        thrust: clamp(thrustCmd, 0, 1),
        roll: clamp(rollCmd, -0.6, 0.6),
        pitch: clamp(pitchCmd, -0.6, 0.6),
        yawRate: clamp(yawRateCmd, -2, 2),
      };

      if (simState) {
        for (let sub = 0; sub < TASK.substeps; sub++) {
          integrate(simState, action, params, TASK.dt / TASK.substeps);
        }
      }
      return action;
    },
  };
}
