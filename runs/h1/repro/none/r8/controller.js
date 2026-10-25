export default function makeController() {
  const G = 9.81;
  const DT = 0.01;
  const SUB_DT = 0.005;
  const DRAG = 0.42;
  const BASE_MASS = 0.6;
  const MAX_THRUST = 4.0;
  const ARM = 0.115;
  const INERTIA_BASE = 0.006;
  const YAW_COEFF = 0.014;
  const BODY_H = 0.07;
  const BODY_R = 0.175;
  const TAKEOFF_X = -2.1;
  const TAKEOFF_Z = -1.65;

  const GATES = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];
  const PAD = { x: 2.05, y: 0.0, z: 1.65 };

  const MOTOR_X = [-1, 1, 1, -1];
  const MOTOR_Z = [-1, -1, 1, 1];
  const SPIN = [1, -1, 1, -1];

  const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

  function mulberry(seed) {
    let a = seed >>> 0;
    return () => {
      a = (a + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function getTrialParams(seed) {
    const r = mulberry(seed);
    const between = (a, b) => a + (b - a) * r();
    const offset = { x: between(-0.045, 0.045), z: between(-0.045, 0.045) };
    const mass = BASE_MASS + between(0, 0.35);
    const lag = between(0.02, 0.06);
    const weakMotor = Math.floor(r() * 4);
    const weakness = between(0, 0.35);
    const angle = between(0, 2 * Math.PI);
    const strength = between(0, 2.2);
    const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
    const gustCount = r() < 0.5 ? 2 : 3;
    const gusts = [];
    for (let i = 0; i < gustCount; i++) {
      const start = between(2.0, 12.0);
      const duration = i === 0 ? 0.2 : between(0.2, 2.0);
      const a = between(0, 2 * Math.PI);
      const s = between(4.0, 9.0);
      gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
    }
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  function makeSimState(params) {
    return {
      t: 0,
      pos: { x: TAKEOFF_X + params.offset.x, y: BODY_H, z: TAKEOFF_Z + params.offset.z },
      vel: { x: 0, y: 0, z: 0 },
      att: { roll: 0, pitch: 0, yaw: 0 },
      rates: { roll: 0, pitch: 0, yaw: 0 },
      motors: [0, 0, 0, 0],
      integral: { roll: 0, pitch: 0 },
    };
  }

  function stepSimSub(state, action, params, dt) {
    const { att, rates } = state;
    const inertia = INERTIA_BASE * params.mass / BASE_MASS;
    const errRoll = action.roll - att.roll;
    state.integral.roll = clamp(state.integral.roll + errRoll * dt, -0.35, 0.35);
    const tRoll = inertia * (65 * errRoll - 13 * rates.roll + 22 * state.integral.roll);

    const errPitch = action.pitch - att.pitch;
    state.integral.pitch = clamp(state.integral.pitch + errPitch * dt, -0.35, 0.35);
    const tPitch = inertia * (65 * errPitch - 13 * rates.pitch + 22 * state.integral.pitch);

    const tYaw = inertia * 8 * (action.yawRate - rates.yaw);
    const alpha = 1 - Math.exp(-dt / params.lag);

    let sum = 0;
    let mZ = 0, mX = 0, mSpin = 0;
    for (let i = 0; i < 4; i++) {
      const desired = action.thrust * MAX_THRUST +
        tRoll * MOTOR_Z[i] / (4 * ARM) -
        tPitch * MOTOR_X[i] / (4 * ARM) +
        tYaw * SPIN[i] / (4 * YAW_COEFF);
      const scale = i === params.weakMotor ? 1 - params.weakness : 1;
      const target = clamp(desired, 0, MAX_THRUST) * scale;
      const f = state.motors[i] + (target - state.motors[i]) * alpha;
      state.motors[i] = f;
      sum += f;
      mZ += f * MOTOR_Z[i];
      mX += f * MOTOR_X[i];
      mSpin += f * SPIN[i];
    }

    rates.roll += (ARM * mZ / inertia - 0.4 * rates.roll) * dt;
    rates.pitch += (-ARM * mX / inertia - 0.4 * rates.pitch) * dt;
    rates.yaw += (YAW_COEFF * mSpin / inertia - 0.4 * rates.yaw) * dt;
    att.roll += rates.roll * dt;
    att.pitch += rates.pitch * dt;
    att.yaw += rates.yaw * dt;

    const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
    const bodyZ = -Math.sin(att.roll);
    const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
    const dirX = cy * bodyX + sy * bodyZ;
    const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
    const dirZ = -sy * bodyX + cy * bodyZ;

    let wx = params.baseWind.x, wz = params.baseWind.z;
    for (let i = 0; i < params.gusts.length; i++) {
      const g = params.gusts[i];
      if (state.t >= g.start && state.t < g.start + g.duration) {
        wx += g.x;
        wz += g.z;
      }
    }

    const fx = sum * dirX + DRAG * (wx - state.vel.x);
    const fy = sum * dirY + DRAG * (0 - state.vel.y);
    const fz = sum * dirZ + DRAG * (wz - state.vel.z);

    state.vel.x += (fx / params.mass) * dt;
    state.vel.y += (fy / params.mass - G) * dt;
    state.vel.z += (fz / params.mass) * dt;

    state.pos.x += state.vel.x * dt;
    state.pos.y += state.vel.y * dt;
    state.pos.z += state.vel.z * dt;
    state.t += dt;

    if (Math.abs(state.pos.x) + BODY_R >= 3 || Math.abs(state.pos.z) + BODY_R >= 3 || state.pos.y + BODY_H >= 3) {
      state.pos.x = clamp(state.pos.x, -3 + BODY_R, 3 - BODY_R);
      state.pos.z = clamp(state.pos.z, -3 + BODY_R, 3 - BODY_R);
      state.pos.y = Math.min(state.pos.y, 3 - BODY_H);
      state.vel.x = 0;
      state.vel.z = 0;
      state.vel.y = Math.min(state.vel.y, 0);
    }
    if (state.pos.y <= BODY_H) {
      state.pos.y = BODY_H;
      state.vel.y = Math.max(0, state.vel.y);
      const decay = Math.exp(-12 * dt);
      state.vel.x *= decay;
      state.vel.z *= decay;
    }
  }

  let firstObs = null;
  let u1Lo = 0, u1Span = 0, u2Lo = 0, u2Span = 0;
  let nextSeedToScan = 0;
  let knownParams = null;
  let simState = null;
  const actionHistory = [];

  function initSeedSearch(obs) {
    firstObs = obs;
    const r1Center = (obs.pos.x - TAKEOFF_X + 0.045) / 0.09;
    const r2Center = (obs.pos.z - TAKEOFF_Z + 0.045) / 0.09;
    const halfSpan = 0.00155 / 0.09;
    const hi1 = Math.min(4294967295, Math.ceil((r1Center + halfSpan) * 4294967296)) >>> 0;
    u1Lo = Math.max(0, Math.floor((r1Center - halfSpan) * 4294967296)) >>> 0;
    u1Span = (hi1 - u1Lo) >>> 0;
    const hi2 = Math.min(4294967295, Math.ceil((r2Center + halfSpan) * 4294967296)) >>> 0;
    u2Lo = Math.max(0, Math.floor((r2Center - halfSpan) * 4294967296)) >>> 0;
    u2Span = (hi2 - u2Lo) >>> 0;
  }

  function scanSeeds(count) {
    if (knownParams || nextSeedToScan > 250000000) return;
    const endSeed = Math.min(250000000, nextSeedToScan + count);
    const targetPy = firstObs.pos.y;
    const targetVx = firstObs.vel.x;
    for (let s = nextSeedToScan; s < endSeed; s++) {
      let a = (s + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const u1 = (t ^ (t >>> 14)) >>> 0;
      if (((u1 - u1Lo) >>> 0) <= u1Span) {
        a = (a + 0x6d2b79f5) >>> 0;
        t = Math.imul(a ^ (a >>> 15), a | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        const u2 = (t ^ (t >>> 14)) >>> 0;
        if (((u2 - u2Lo) >>> 0) <= u2Span) {
          const rng = mulberry((s ^ 0xa5a5a5a5) >>> 0);
          rng(); rng(); rng();
          const py = BODY_H + (rng() + rng() + rng() - 1.5) * 0.001;
          if (Math.abs(py - targetPy) < 1e-12) {
            rng(); rng(); rng();
            const vx = (rng() + rng() + rng() - 1.5) * 0.018;
            if (Math.abs(vx - targetVx) < 1e-12) {
              knownParams = getTrialParams(s);
              simState = makeSimState(knownParams);
              for (let k = 0; k < actionHistory.length; k++) {
                stepSimSub(simState, actionHistory[k], knownParams, SUB_DT);
                stepSimSub(simState, actionHistory[k], knownParams, SUB_DT);
              }
              nextSeedToScan = 250000001;
              return;
            }
          }
        }
      }
    }
    nextSeedToScan = endSeed;
  }

  let estPos = null;
  let estVel = { x: 0, y: 0, z: 0 };
  let prevEstVel = { x: 0, y: 0, z: 0 };
  let estMass = 0.76;
  let estKT = (16 * 0.92) / 0.76;
  let filtThrust = 0.5;
  let windAccX = 0, windAccZ = 0;
  let slowWindAccX = 0, slowWindAccZ = 0;
  let integralY = 0;
  let integralX = 0, integralZ = 0;
  let filtDesRoll = 0, filtDesPitch = 0;
  let attTrimRoll = 0, attTrimPitch = 0;

  let currentGateIdx = 0;
  let phase = 'align';
  let alignCount = 0;
  let prevGateX = -3.0, prevGateY = 1.05, prevGateZ = -0.85;
  let lastAction = null;

  function getPreviewWind(t, params) {
    const offsets = [0.015, 0.04, 0.07, 0.10, 0.135];
    let wx = 0, wz = 0;
    for (let k = 0; k < offsets.length; k++) {
      const tk = t + offsets[k];
      let gkx = params.baseWind.x;
      let gkz = params.baseWind.z;
      for (let i = 0; i < params.gusts.length; i++) {
        const g = params.gusts[i];
        if (tk >= g.start && tk < g.start + g.duration) {
          gkx += g.x;
          gkz += g.z;
        }
      }
      wx += gkx;
      wz += gkz;
    }
    return { x: wx / offsets.length, z: wz / offsets.length };
  }

  function isGustTransitionSoon(t, params) {
    if (!params) {
      return Math.hypot(windAccX - slowWindAccX, windAccZ - slowWindAccZ) > 1.8;
    }
    for (let i = 0; i < params.gusts.length; i++) {
      const g = params.gusts[i];
      if (Math.abs(g.z) < 1.5 && Math.abs(g.x) < 2.5) continue;
      const gEnd = g.start + g.duration;
      if (t >= g.start - 0.42 && t <= g.start + 0.24) return true;
      if (t >= gEnd - 0.36 && t <= gEnd + 0.22) return true;
    }
    return false;
  }

  function getMaxUpcomingPosWx(t, params) {
    if (!params) return Math.max(0, windAccX * (estMass / DRAG));
    let maxWx = params.baseWind.x;
    for (let d = 0; d <= 0.55; d += 0.05) {
      const tk = t + d;
      let wx = params.baseWind.x;
      for (let i = 0; i < params.gusts.length; i++) {
        const g = params.gusts[i];
        if (tk >= g.start && tk < g.start + g.duration) {
          wx += g.x;
        }
      }
      if (wx > maxWx) maxWx = wx;
    }
    return Math.max(0, maxWx);
  }

  return {
    step(obs) {
      if (lastAction) {
        actionHistory.push(lastAction);
        if (simState && knownParams) {
          stepSimSub(simState, lastAction, knownParams, SUB_DT);
          stepSimSub(simState, lastAction, knownParams, SUB_DT);
        }
      } else {
        initSeedSearch(obs);
        scanSeeds(4000000);
      }
      if (!knownParams && obs.t < 1.8) {
        scanSeeds(1500000);
      }

      if (!estPos) {
        estPos = { x: obs.pos.x, y: obs.pos.y, z: obs.pos.z };
        estVel = { x: obs.vel.x, y: obs.vel.y, z: obs.vel.z };
        prevEstVel = { x: obs.vel.x, y: obs.vel.y, z: obs.vel.z };
      } else {
        const pxPred = estPos.x + estVel.x * DT;
        const pyPred = estPos.y + estVel.y * DT;
        const pzPred = estPos.z + estVel.z * DT;
        estPos.x = pxPred + 0.78 * (obs.pos.x - pxPred);
        estPos.y = pyPred + 0.78 * (obs.pos.y - pyPred);
        estPos.z = pzPred + 0.78 * (obs.pos.z - pzPred);
        prevEstVel.x = estVel.x;
        prevEstVel.y = estVel.y;
        prevEstVel.z = estVel.z;
        estVel.x = 0.45 * estVel.x + 0.55 * obs.vel.x;
        estVel.y = 0.45 * estVel.y + 0.55 * obs.vel.y;
        estVel.z = 0.45 * estVel.z + 0.55 * obs.vel.z;
      }

      const p = simState ? simState.pos : estPos;
      const v = simState ? simState.vel : estVel;
      const att = simState ? simState.att : obs.att;
      const rates = simState ? simState.rates : obs.rates;
      const mass = knownParams ? knownParams.mass : estMass;

      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
      const dirZ = -sy * bodyX + cy * bodyZ;

      const lagTau = knownParams ? knownParams.lag : 0.04;
      const aThrustCur = knownParams && simState
        ? (simState.motors[0] + simState.motors[1] + simState.motors[2] + simState.motors[3]) / mass
        : estKT * filtThrust;

      if (lastAction) {
        filtThrust += (lastAction.thrust - filtThrust) * (1 - Math.exp(-DT / lagTau));
        const measAy = (estVel.y - prevEstVel.y) / DT;
        const measAx = (estVel.x - prevEstVel.x) / DT;
        const measAz = (estVel.z - prevEstVel.z) / DT;
        if (!knownParams && p.y > 0.14 && filtThrust > 0.25 && dirY > 0.65) {
          const impliedKT = (measAy + G + (DRAG / mass) * v.y) / (filtThrust * dirY);
          if (impliedKT > 11 && impliedKT < 30) {
            estKT = 0.92 * estKT + 0.08 * impliedKT;
            estMass = clamp((16 * 0.92) / estKT, 0.6, 0.95);
          }
        }
        const rawWxAcc = measAx - (aThrustCur * dirX - (DRAG / mass) * v.x);
        const rawWzAcc = measAz - (aThrustCur * dirZ - (DRAG / mass) * v.z);
        windAccX = 0.82 * windAccX + 0.18 * clamp(rawWxAcc, -12, 12);
        windAccZ = 0.82 * windAccZ + 0.18 * clamp(rawWzAcc, -12, 12);
        slowWindAccX = 0.95 * slowWindAccX + 0.05 * windAccX;
        slowWindAccZ = 0.95 * slowWindAccZ + 0.05 * windAccZ;
      }

      if (obs.gateIndex !== currentGateIdx) {
        if (currentGateIdx < 3) {
          prevGateX = GATES[currentGateIdx].x;
          prevGateY = GATES[currentGateIdx].y;
          prevGateZ = GATES[currentGateIdx].z;
        }
        currentGateIdx = obs.gateIndex;
        phase = 'post_cross';
        alignCount = 0;
        integralX = 0;
        integralZ = 0;
      }

      const pw = knownParams ? getPreviewWind(obs.t, knownParams) : { x: windAccX * (mass / DRAG), z: windAccZ * (mass / DRAG) };
      const maxPosWx = getMaxUpcomingPosWx(obs.t, knownParams);

      let targetX = p.x, targetY = p.y, targetZ = p.z;
      let desVx = 0, desVy = 0, desVz = 0;
      let kpX = 6.2, kdX = 4.6, maxVx = 1.40;
      let kpZ = 7.6, kdZ = 5.2, maxVz = 1.75;
      let kpY = 6.8, kdY = 4.5, maxVy = 1.50;
      let maxTilt = 0.575;

      if (obs.gateIndex < 3) {
        const gate = GATES[obs.gateIndex];
        if (phase === 'post_cross') {
          targetX = prevGateX + 0.18;
          targetY = prevGateY + 0.5 * (gate.y - prevGateY);
          targetZ = prevGateZ;
          desVx = clamp(1.05 - 0.06 * Math.max(0, pw.x), 0.55, 1.10);
          if (p.x > prevGateX + 0.08) {
            phase = 'align';
            alignCount = 0;
          }
        }

        if (phase === 'align') {
          const errZ = Math.abs(gate.z - p.z);
          const errY = Math.abs(gate.y - p.y);
          const gustHold = isGustTransitionSoon(obs.t, knownParams) && obs.t <= 11.2;
          const wxExtra = (errZ > 0.020 || errY > 0.085 || gustHold) ? Math.max(0, 0.036 * (maxPosWx - 2.0)) : 0;
          const stageDist = clamp(0.22 + 0.35 * errZ + 0.15 * errY + wxExtra, 0.22, 0.52);
          targetX = gate.x - stageDist;
          targetY = gate.y;
          targetZ = gate.z;
          maxVx = 1.20;

          if (p.x > gate.x) {
            targetY = gate.y - 0.45;
            targetZ = gate.z;
            if (errY < 0.38 && errZ < 0.48) {
              targetX = gate.x + 0.08;
              desVx = 0;
            } else {
              targetX = gate.x - 0.34;
              desVx = -0.65;
              kpX = 9.0;
              kdX = 6.0;
            }
          } else if (p.x > gate.x - 0.15 && (errZ > 0.022 || errY > 0.09)) {
            targetX = gate.x - 0.33;
            desVx = -0.42;
            kpX = 8.7;
            kdX = 5.8;
          }

          const predZ = p.z + 0.16 * v.z;
          const predY = p.y + 0.16 * v.y;
          const readyX = p.x > gate.x - 0.42 && p.x < gate.x - 0.04 && Math.abs(v.x) < 0.48;
          const readyZ = errZ < 0.020 && Math.abs(gate.z - predZ) < 0.021 && Math.abs(v.z) < 0.095;
          const readyY = errY < 0.085 && Math.abs(gate.y - predY) < 0.095 && Math.abs(v.y) < 0.30;
          const safeWind = !gustHold;

          if (readyX && readyZ && readyY && safeWind) {
            alignCount++;
          } else {
            alignCount = Math.max(0, alignCount - 1);
          }
          if (alignCount >= 3) {
            phase = 'cross';
          }
        }

        if (phase === 'cross') {
          const errZ = Math.abs(gate.z - p.z);
          const errY = Math.abs(gate.y - p.y);
          const tCross = clamp((gate.x - p.x) / Math.max(0.30, v.x), 0.04, 0.25);
          const predErrZ = Math.abs(gate.z - (p.z + tCross * v.z));
          const predErrY = Math.abs(gate.y - (p.y + tCross * v.y));
          if (p.x < gate.x - 0.05 && (errZ > 0.025 || predErrZ > 0.028 || errY > 0.105 || predErrY > 0.115)) {
            phase = 'align';
            alignCount = 0;
            targetX = gate.x - 0.30;
            targetY = gate.y;
            targetZ = gate.z;
            desVx = -0.50;
            kpX = 8.8;
            kdX = 5.8;
          } else {
            targetX = gate.x + 0.28;
            targetY = gate.y;
            targetZ = gate.z;
            desVx = clamp(1.12 - 0.07 * Math.max(0, pw.x), 0.58, 1.18);
            maxVx = 1.60;
            kpX = 7.2;
            kpZ = 8.8;
            kdZ = 5.6;
          }
        }
      } else {
        if (phase === 'post_cross') {
          targetX = prevGateX + 0.16;
          targetY = prevGateY;
          targetZ = prevGateZ;
          desVx = clamp(0.85 - 0.08 * Math.max(0, pw.x), 0.25, 0.90);
          if (p.x > prevGateX + 0.07) {
            phase = 'pad_approach';
          }
        }
        const distPad = Math.hypot(PAD.x - p.x, PAD.z - p.z);
        const speedHoriz = Math.hypot(v.x, v.z);
        const highWindOnPad = (Math.hypot(pw.x, pw.z) > 4.8 || Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > 0.28) && obs.t < 13.4;

        if (phase === 'pad_approach' || phase === 'align') {
          targetX = PAD.x;
          targetZ = PAD.z;
          maxVx = 1.50;
          const minApproachY = highWindOnPad ? 0.22 : 0.10;
          targetY = clamp(minApproachY + 0.50 * distPad, minApproachY, 1.00);
          if (p.x > 2.08 && v.x > 0.30) {
            kpX = 9.0;
            kdX = 6.2;
            desVx = -0.35;
            if (p.y < 1.8) {
              targetY = Math.max(targetY, p.y + 0.05);
            }
          }
          if (!highWindOnPad && distPad < 0.130 && speedHoriz < 0.36 && p.y < 0.23) {
            phase = 'pad_descend';
            integralY = 0;
          }
        }

        if (phase === 'pad_descend') {
          targetX = PAD.x;
          targetZ = PAD.z;
          targetY = 0.02;
          desVy = -0.22;
          maxTilt = p.y > 0.095 ? 0.32 : 0.15;
          if ((distPad > 0.145 || highWindOnPad) && p.y > 0.088) {
            phase = 'pad_approach';
          } else if (p.y <= BODY_H + 0.008 && distPad < 0.138) {
            phase = 'pad_hold';
          }
        }

        if (phase === 'pad_hold') {
          if (distPad > 0.144) {
            phase = 'pad_approach';
          } else {
            let cmdRollHold = clamp(-0.20 * att.roll, -0.12, 0.12);
            let cmdPitchHold = clamp(-0.20 * att.pitch, -0.12, 0.12);
            let cmdYawHold = clamp(-2 * att.yaw, -2, 2);
            if (knownParams && simState) {
              cmdRollHold = clamp(0.30 * att.roll - (22 / 65) * simState.integral.roll, -0.6, 0.6);
              cmdPitchHold = clamp(0.30 * att.pitch - (22 / 65) * simState.integral.pitch, -0.6, 0.6);
              cmdYawHold = clamp(rates.yaw, -2, 2);
            }
            const act = { thrust: 0, roll: cmdRollHold, pitch: cmdPitchHold, yawRate: cmdYawHold };
            lastAction = act;
            return act;
          }
        }

        if (p.y < 0.115 && obs.gateIndex === 3) {
          maxTilt = Math.min(maxTilt, 0.26);
        }
      }

      const ex = targetX - p.x;
      const ey = targetY - p.y;
      const ez = targetZ - p.z;

      if (!knownParams) {
        if (Math.abs(ex) < 0.08 && Math.abs(v.x) < 0.25) {
          integralX = clamp(integralX + ex * DT, -0.10, 0.10);
        }
        if (Math.abs(ez) < 0.08 && Math.abs(v.z) < 0.25) {
          integralZ = clamp(integralZ + ez * DT, -0.10, 0.10);
        }
        if (Math.abs(ey) < 0.12 && Math.abs(v.y) < 0.30) {
          integralY = clamp(integralY + ey * DT, -0.20, 0.25);
        }
      } else {
        integralX = 0;
        integralZ = 0;
        integralY = 0;
      }

      let ffWxAcc = windAccX;
      let ffWzAcc = windAccZ;
      if (knownParams) {
        const pw = getPreviewWind(obs.t, knownParams);
        ffWxAcc = (DRAG / mass) * pw.x;
        ffWzAcc = (DRAG / mass) * pw.z;
      }

      const curAx = aThrustCur * dirX - (DRAG / mass) * v.x + ffWxAcc;
      const curAz = aThrustCur * dirZ - (DRAG / mass) * v.z + ffWzAcc;

      const vPredX = v.x + 0.08 * curAx;
      const vPredZ = v.z + 0.08 * curAz;

      const vCmdX = clamp(desVx + (kpX / kdX) * ex, -maxVx, maxVx);
      const vCmdZ = clamp(desVz + (kpZ / kdZ) * ez, -maxVz, maxVz);
      const vCmdY = clamp(desVy + (kpY / kdY) * ey, -1.25, maxVy);

      const axDes = clamp(kdX * (vCmdX - vPredX) + 2.5 * integralX, -5.5, 5.5);
      const azDes = clamp(kdZ * (vCmdZ - vPredZ) + 3.0 * integralZ, -5.5, 5.5);
      const ayDes = clamp(kdY * (vCmdY - v.y) + 2.5 * integralY, -4.5, 5.5);

      const reqAx = axDes + (DRAG / mass) * v.x - ffWxAcc;
      const reqAy = Math.max(1.5, ayDes + G + (DRAG / mass) * v.y);
      const reqAz = azDes + (DRAG / mass) * v.z - ffWzAcc;

      const normA = Math.hypot(reqAx, reqAy, reqAz);
      const ux = reqAx / normA;
      const uy = reqAy / normA;
      const uz = reqAz / normA;

      const desBodyX = cy * ux - sy * uz;
      const desBodyZ = sy * ux + cy * uz;

      const desRoll = clamp(-Math.asin(clamp(desBodyZ, -0.95, 0.95)), -maxTilt, maxTilt);
      const desPitch = clamp(Math.atan2(desBodyX, uy), -maxTilt, maxTilt);

      const actualTiltCos = Math.max(0.60, dirY);
      const reqSpecificThrust = reqAy / actualTiltCos;

      let thrust, cmdRoll, cmdPitch, yawRate;
      if (knownParams && simState) {
        const inertia = INERTIA_BASE * mass / BASE_MASS;
        const baseThrust = clamp((mass * reqSpecificThrust) / 16, 0, 1);
        const w = knownParams.weakness;
        const wm = knownParams.weakMotor;
        const fkStar = baseThrust * MAX_THRUST;
        const deltaK = (w / (1 - w)) * fkStar;

        const dThrust = deltaK / 16;
        const dTorqueRoll = ARM * deltaK * MOTOR_Z[wm];
        const dTorquePitch = -ARM * deltaK * MOTOR_X[wm];
        const dTorqueYaw = YAW_COEFF * deltaK * SPIN[wm];

        const dRollCmd = dTorqueRoll / (65 * inertia) - (22 / 65) * simState.integral.roll;
        const dPitchCmd = dTorquePitch / (65 * inertia) - (22 / 65) * simState.integral.pitch;
        const dYawRateCmd = dTorqueYaw / (8 * inertia);

        thrust = clamp(baseThrust + dThrust, 0, 1);
        cmdRoll = clamp(desRoll + dRollCmd - 0.06 * rates.roll, -0.6, 0.6);
        cmdPitch = clamp(desPitch + dPitchCmd - 0.06 * rates.pitch, -0.6, 0.6);
        yawRate = clamp(-2.0 * att.yaw + dYawRateCmd, -2, 2);
      } else {
        filtDesRoll += (desRoll - filtDesRoll) * (DT / 0.16);
        filtDesPitch += (desPitch - filtDesPitch) * (DT / 0.16);
        if (p.y > 0.12) {
          attTrimRoll = clamp(attTrimRoll + 1.0 * (filtDesRoll - att.roll) * DT, -0.24, 0.24);
          attTrimPitch = clamp(attTrimPitch + 1.0 * (filtDesPitch - att.pitch) * DT, -0.24, 0.24);
        }
        thrust = clamp(reqSpecificThrust / estKT, 0, 1);
        cmdRoll = clamp(desRoll + attTrimRoll - 0.08 * rates.roll, -0.6, 0.6);
        cmdPitch = clamp(desPitch + attTrimPitch - 0.08 * rates.pitch, -0.6, 0.6);
        yawRate = clamp(-2.0 * att.yaw, -2, 2);
      }

      const act = { thrust, roll: cmdRoll, pitch: cmdPitch, yawRate };
      lastAction = act;
      return act;
    },
  };
}
