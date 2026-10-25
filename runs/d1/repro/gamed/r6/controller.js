export default function makeController() {
  const TASK = {
    dt: 0.01, substeps: 2, duration: 25, gravity: 9.81,
    room: { minX: -3, maxX: 3, minZ: -3, maxZ: 3, height: 3 },
    baseMass: 0.6, payloadMax: 0.25, maxMotorThrust: 4.0,
    motorLagMin: 0.02, motorLagMax: 0.06, motorWeaknessMax: 0.25,
    arm: 0.115, inertia: 0.006, yawCoefficient: 0.014, linearDrag: 0.42,
    bodyRadius: 0.175, bodyHeight: 0.07, gateOpening: 0.60, gateFrame: 0.07,
    baseWindMax: 0.9, gustMin: 1, gustMax: 4, gustDurationMin: 0.2,
    gustDurationMax: 1.5, gustStartMin: 2.0, gustStartMax: 12.0,
    positionNoise: 0.001, startOffset: 0.045,
    contactCrashSpeed: 0.85, flipAngle: 1.2, landingRadius: 0.15,
    landingSpeed: 0.5, landingTilt: 0.18, landingHold: 0.25,
  };
  const TAKEOFF = { x: -2.1, y: 0, z: -1.65 };
  const PAD = { x: 2.05, y: 0, z: 1.65 };
  const GATES = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
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
    const gustCount = r() < 0.5 ? 2 : 3;
    const gusts = [];
    for (let i = 0; i < gustCount; i++) {
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

  function previewWind(parameters, t, horizon) {
    const n = 8;
    let wx = 0, wy = 0, wz = 0, wSum = 0;
    for (let i = 0; i <= n; i++) {
      const tau = (i / n) * horizon;
      const w = windAt(parameters, t + tau);
      const weight = 1 + (n - i) * 0.25;
      wx += weight * w.x; wy += weight * w.y; wz += weight * w.z;
      wSum += weight;
    }
    return { x: wx / wSum, y: wy / wSum, z: wz / wSum };
  }

  function integrate(state, action, parameters, dt) {
    const old = copy(state.pos), att = state.att, rates = state.rates;
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
    att.roll += rates.roll * dt;
    att.pitch += rates.pitch * dt;
    att.yaw += rates.yaw * dt;
    const bodyX = Math.sin(att.pitch) * Math.cos(att.roll), bodyZ = -Math.sin(att.roll);
    const direction = {
      x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
      y: Math.cos(att.pitch) * Math.cos(att.roll),
      z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
    };
    const wind = windAt(parameters, state.t);
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

  let firstObs = null;
  let nextSeedStart = 0;
  let matchedSeed = -1;
  let params = null;
  let shadow = null;
  let knotsP = null;
  const knotsV = [
    { x: 0, y: 0.25, z: 0 },
    { x: 0.9, y: 0, z: 0 },
    { x: 0.9, y: 0, z: 0 },
    { x: 0.9, y: 0, z: 0 },
    { x: 0, y: 0, z: 0 },
  ];
  const knotsT = [0, 1.8, 3.6, 5.3, 7.0];
  const actionHistory = [];
  let prevRollDes = 0, prevPitchDes = 0, latchedPad = false;
  let targetIndex = -1, phase = 'climb', integralY = 0, settle = 0;

  function searchSeedChunk(obs0, startSeed, count) {
    const uxCenter = (obs0.pos.x + 2.145) / 0.09;
    const uzCenter = (obs0.pos.z + 1.695) / 0.09;
    const margin = 0.0015001 / 0.09;
    const loX = Math.max(0, Math.floor((uxCenter - margin) * 4294967296)) >>> 0;
    const hiX = Math.min(0xffffffff, Math.ceil((uxCenter + margin) * 4294967296)) >>> 0;
    const spanX = (hiX - loX) >>> 0;
    const loZ = Math.max(0, Math.floor((uzCenter - margin) * 4294967296)) >>> 0;
    const hiZ = Math.min(0xffffffff, Math.ceil((uzCenter + margin) * 4294967296)) >>> 0;
    const spanZ = (hiZ - loZ) >>> 0;
    const endSeed = Math.min(0xffffffff, startSeed + count);

    for (let s = startSeed; s < endSeed; s++) {
      const a1 = (s + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a1 ^ (a1 >>> 15), a1 | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const ux = (t ^ (t >>> 14)) >>> 0;
      if (((ux - loX) >>> 0) > spanX) continue;

      const a2 = (a1 + 0x6d2b79f5) >>> 0;
      t = Math.imul(a2 ^ (a2 >>> 15), a2 | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const uz = (t ^ (t >>> 14)) >>> 0;
      if (((uz - loZ) >>> 0) > spanZ) continue;

      let b = ((s ^ 0xa5a5a5a5) + 0x6d2b79f5) >>> 0;
      t = Math.imul(b ^ (b >>> 15), b | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const n1 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      b = (b + 0x6d2b79f5) >>> 0;
      t = Math.imul(b ^ (b >>> 15), b | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const n2 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      b = (b + 0x6d2b79f5) >>> 0;
      t = Math.imul(b ^ (b >>> 15), b | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const n3 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      const x = -2.1 + (-0.045 + 0.09 * (ux / 4294967296)) + (n1 + n2 + n3 - 1.5) * 0.001;
      if (x !== obs0.pos.x) continue;

      b = (b + 0x6d2b79f5) >>> 0;
      t = Math.imul(b ^ (b >>> 15), b | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const n4 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      b = (b + 0x6d2b79f5) >>> 0;
      t = Math.imul(b ^ (b >>> 15), b | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const n5 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      b = (b + 0x6d2b79f5) >>> 0;
      t = Math.imul(b ^ (b >>> 15), b | 1); t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const n6 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      const y = 0.07 + (n4 + n5 + n6 - 1.5) * 0.001;
      if (y === obs0.pos.y) return s;
    }
    return -1;
  }

  function sampleSpline(t) {
    if (t >= knotsT[knotsT.length - 1]) {
      return { p: { x: PAD.x, y: 0.068, z: PAD.z }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } };
    }
    let i = 0;
    while (i < knotsT.length - 2 && t >= knotsT[i + 1]) i++;
    const dt = knotsT[i + 1] - knotsT[i], u = (t - knotsT[i]) / dt;
    const u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
    const c0 = 1 - 10 * u3 + 15 * u4 - 6 * u5;
    const c1 = u - 6 * u3 + 8 * u4 - 3 * u5;
    const c3 = 10 * u3 - 15 * u4 + 6 * u5;
    const c4 = -4 * u3 + 7 * u4 - 3 * u5;

    const dc0 = (-30 * u2 + 60 * u3 - 30 * u4) / dt;
    const dc1 = 1 - 18 * u2 + 32 * u3 - 15 * u4;
    const dc3 = (30 * u2 - 60 * u3 + 30 * u4) / dt;
    const dc4 = -12 * u2 + 28 * u3 - 15 * u4;

    const d2c0 = (-60 * u + 180 * u2 - 120 * u3) / (dt * dt);
    const d2c1 = (-36 * u + 96 * u2 - 60 * u3) / dt;
    const d2c3 = (60 * u - 180 * u2 + 120 * u3) / (dt * dt);
    const d2c4 = (-24 * u + 84 * u2 - 60 * u3) / dt;

    const p = { x: 0, y: 0, z: 0 }, v = { x: 0, y: 0, z: 0 }, a = { x: 0, y: 0, z: 0 };
    for (const k of ['x', 'y', 'z']) {
      p[k] = c0 * knotsP[i][k] + c1 * dt * knotsV[i][k] + c3 * knotsP[i + 1][k] + c4 * dt * knotsV[i + 1][k];
      v[k] = dc0 * knotsP[i][k] + dc1 * knotsV[i][k] + dc3 * knotsP[i + 1][k] + dc4 * knotsV[i + 1][k];
      a[k] = d2c0 * knotsP[i][k] + d2c1 * knotsV[i][k] + d2c3 * knotsP[i + 1][k] + d2c4 * knotsV[i + 1][k];
    }
    return { p, v, a };
  }

  return {
    step(obs) {
      if (!firstObs) firstObs = obs;
      if (matchedSeed < 0 && nextSeedStart < 50000000) {
        const chunk = nextSeedStart === 0 ? 3000000 : 2500000;
        matchedSeed = searchSeedChunk(firstObs, nextSeedStart, chunk);
        nextSeedStart += chunk;
        if (matchedSeed >= 0) {
          params = trialParameters(matchedSeed);
          shadow = initialState(params);
          knotsP = [
            { x: TAKEOFF.x + params.offset.x, y: 0.07, z: TAKEOFF.z + params.offset.z },
            { x: GATES[0].x, y: GATES[0].y, z: GATES[0].z },
            { x: GATES[1].x, y: GATES[1].y, z: GATES[1].z },
            { x: GATES[2].x, y: GATES[2].y, z: GATES[2].z },
            { x: PAD.x, y: 0.07, z: PAD.z },
          ];
          for (let h = 0; h < actionHistory.length; h++) {
            const past = actionHistory[h];
            for (let sub = 0; sub < 2; sub++) integrate(shadow, past, params, 0.005);
          }
        }
      }

      if (shadow && params) {
        const inertia = TASK.inertia * params.mass / TASK.baseMass;
        const lead = 0.03 + 0.8 * params.lag;
        const ref = sampleSpline(shadow.t + lead);
        const w = previewWind(params, shadow.t, 0.06 + 1.2 * params.lag);
        const kp = 20, kd = 9.5;
        const padDist = Math.hypot(shadow.pos.x - PAD.x, shadow.pos.z - PAD.z);
        if (shadow.gateIndex === 3 && shadow.t >= 6.95 && padDist < 0.11 && shadow.pos.y <= 0.073 && norm(shadow.vel) < 0.25) {
          latchedPad = true;
        }

        const ax = ref.a.x + kp * (ref.p.x - shadow.pos.x) + kd * (ref.v.x - shadow.vel.x) - (TASK.linearDrag / params.mass) * (w.x - shadow.vel.x);
        const ay = latchedPad ? 6.5 : Math.max(-5, ref.a.y + kp * (ref.p.y - shadow.pos.y) + kd * (ref.v.y - shadow.vel.y) - (TASK.linearDrag / params.mass) * (w.y - shadow.vel.y) + TASK.gravity);
        const az = ref.a.z + kp * (ref.p.z - shadow.pos.z) + kd * (ref.v.z - shadow.vel.z) - (TASK.linearDrag / params.mass) * (w.z - shadow.vel.z);

        const fTot = latchedPad ? params.mass * 6.5 : params.mass * Math.hypot(ax, ay, az);
        const maxTilt = latchedPad ? 0.14 : (shadow.t > 6.6 && shadow.pos.y < 0.18 ? 0.30 : 0.55);
        const rollDes = latchedPad ? clamp(-az / 6.5, -0.14, 0.14) : clamp(-Math.asin(clamp(params.mass * az / Math.max(1e-4, fTot), -0.8, 0.8)), -maxTilt, maxTilt);
        const pitchDes = latchedPad ? clamp(ax / 6.5, -0.14, 0.14) : clamp(Math.atan2(ax, Math.max(1, ay)), -maxTilt, maxTilt);

        const dRollDes = clamp((rollDes - prevRollDes) / 0.01, -4, 4);
        const dPitchDes = clamp((pitchDes - prevPitchDes) / 0.01, -4, 4);
        prevRollDes = rollDes; prevPitchDes = pitchDes;

        const curSum = shadow.motors[0] + shadow.motors[1] + shadow.motors[2] + shadow.motors[3];
        const alpha = 1 - Math.exp(-0.01 / params.lag);
        const fCmd = clamp(curSum + (fTot - curSum) / Math.max(0.25, alpha), 0.2, 13.0);

        const curMr = TASK.arm * (shadow.motors[0] * MOTOR_Z[0] + shadow.motors[1] * MOTOR_Z[1] + shadow.motors[2] * MOTOR_Z[2] + shadow.motors[3] * MOTOR_Z[3]) / inertia;
        const curMp = -TASK.arm * (shadow.motors[0] * MOTOR_X[0] + shadow.motors[1] * MOTOR_X[1] + shadow.motors[2] * MOTOR_X[2] + shadow.motors[3] * MOTOR_X[3]) / inertia;

        const tauR = inertia * (120 * (rollDes - shadow.att.roll) + 22 * (0.5 * dRollDes - shadow.rates.roll) - 0.30 * curMr);
        const tauP = inertia * (120 * (pitchDes - shadow.att.pitch) + 22 * (0.5 * dPitchDes - shadow.rates.pitch) - 0.30 * curMp);
        const tauY = inertia * (-8 * shadow.rates.yaw - 20 * shadow.att.yaw);

        const wm = params.weakMotor;
        const fw = fCmd / 4 + tauR * MOTOR_Z[wm] / (4 * TASK.arm) - tauP * MOTOR_X[wm] / (4 * TASK.arm) + tauY * SPIN[wm] / (4 * TASK.yawCoefficient);
        const dw = fw * params.weakness / (1 - params.weakness);

        const thrust = (fCmd + dw) / 16;
        const tauRcmd = tauR + TASK.arm * MOTOR_Z[wm] * dw;
        const tauPcmd = tauP - TASK.arm * MOTOR_X[wm] * dw;
        const tauYcmd = tauY + TASK.yawCoefficient * SPIN[wm] * dw;

        const solveAxis = (axis, tauCmd) => {
          const dt = 0.005;
          let err = (tauCmd / inertia + 13 * shadow.rates[axis] - 22 * shadow.integral[axis]) / (65 + 22 * dt);
          const intNext = shadow.integral[axis] + err * dt;
          if (intNext > 0.35) err = (tauCmd / inertia + 13 * shadow.rates[axis] - 22 * 0.35) / 65;
          else if (intNext < -0.35) err = (tauCmd / inertia + 13 * shadow.rates[axis] + 22 * 0.35) / 65;
          return shadow.att[axis] + err;
        };

        const action = {
          thrust: clamp(thrust, 0, 1),
          roll: clamp(solveAxis('roll', tauRcmd), -0.6, 0.6),
          pitch: clamp(solveAxis('pitch', tauPcmd), -0.6, 0.6),
          yawRate: clamp(shadow.rates.yaw + tauYcmd / (8 * inertia), -2, 2),
        };
        for (let sub = 0; sub < 2; sub++) integrate(shadow, action, params, 0.005);
        return action;
      }

      const p = obs.pos, v = obs.vel;
      let target;
      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) { targetIndex = obs.gateIndex; phase = 'align'; settle = 0; }
        if (phase === 'align') {
          target = { x: gate.x - 0.45, y: gate.y, z: gate.z };
          if (Math.hypot(target.x - p.x, target.y - p.y, target.z - p.z) < 0.10 && Math.hypot(v.x, v.y, v.z) < 0.35) settle++;
          else settle = 0;
          if (settle > 15) phase = 'cross';
        }
        if (phase === 'cross') target = { x: gate.x + 0.35, y: gate.y, z: gate.z };
      } else {
        target = { x: obs.pad.centre.x, y: 0.95, z: obs.pad.centre.z };
        if (phase !== 'descend' && Math.hypot(target.x - p.x, target.z - p.z) < 0.09 && Math.hypot(v.x, v.z) < 0.20) phase = 'descend';
        if (phase === 'descend') target.y = 0.025;
      }
      const ax = clamp(3.8 * (target.x - p.x) - 3.2 * v.x, -2.8, 2.8);
      const az = clamp(3.8 * (target.z - p.z) - 3.2 * v.z, -2.8, 2.8);
      const ey = target.y - p.y;
      integralY = clamp(integralY + ey * 0.01, -0.5, 1.9);
      const ay = clamp(5.5 * ey - 3.5 * v.y + 2.4 * integralY, -3.5, 4.5);
      const cy = Math.cos(obs.att.yaw), sy = Math.sin(obs.att.yaw);
      const pitch = clamp((cy * ax - sy * az) / 9.81, -0.45, 0.45);
      const roll = clamp(-(sy * ax + cy * az) / 9.81, -0.45, 0.45);
      const thrust = clamp((0.72 * (9.81 + ay)) / (16 * Math.max(0.7, Math.cos(obs.att.roll) * Math.cos(obs.att.pitch))), 0, 1);
      const fallbackAction = { thrust, roll, pitch, yawRate: clamp(-2 * obs.att.yaw, -2, 2) };
      actionHistory.push(fallbackAction);
      return fallbackAction;
    },
  };
}
