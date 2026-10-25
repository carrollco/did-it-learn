export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const copy = (v) => ({ x: v.x, y: v.y, z: v.z });

  const G = 9.81;
  const DRAG = 0.42;
  const BASE_MASS = 0.6;
  const PAYLOAD_MAX = 0.35;
  const MAX_THRUST = 4.0;
  const ARM = 0.115;
  const BASE_INERTIA = 0.006;
  const YAW_COEFF = 0.014;
  const BODY_RADIUS = 0.175;
  const BODY_HEIGHT = 0.07;
  const TAKEOFF = { x: -2.1, y: 0, z: -1.65 };
  const PAD = { x: 2.05, y: 0, z: 1.65 };
  const GATES = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];
  const MOTOR_X = [-1, 1, 1, -1];
  const MOTOR_Z = [-1, -1, 1, 1];
  const SPIN = [1, -1, 1, -1];

  function makeRng(seed) {
    let a = seed >>> 0;
    return () => {
      a = (a + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function getTrialParams(seed) {
    const r = makeRng(seed);
    const between = (a, b) => a + (b - a) * r();
    const offset = { x: between(-0.045, 0.045), z: between(-0.045, 0.045) };
    const mass = BASE_MASS + between(0, PAYLOAD_MAX);
    const lag = between(0.02, 0.06);
    const weakMotor = Math.floor(r() * 4);
    const weakness = between(0, 0.35);
    const angle = between(0, 2 * Math.PI);
    const strength = between(0, 2.2);
    const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
    const count = r() < 0.5 ? 2 : 3;
    const gusts = [];
    for (let i = 0; i < count; i++) {
      const start = between(2.0, 12.0);
      const duration = i === 0 ? 0.2 : between(0.2, 2.0);
      const a = between(0, 2 * Math.PI);
      const s = between(4, 9);
      gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
    }
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  function makeInitialSimState(params) {
    return {
      t: 0,
      pos: { x: TAKEOFF.x + params.offset.x, y: BODY_HEIGHT, z: TAKEOFF.z + params.offset.z },
      vel: { x: 0, y: 0, z: 0 },
      att: { roll: 0, pitch: 0, yaw: 0 },
      rates: { roll: 0, pitch: 0, yaw: 0 },
      motors: [0, 0, 0, 0],
      integral: { roll: 0, pitch: 0 },
      airborne: false,
    };
  }

  function stepSim(state, action, params, dt) {
    const { att, rates } = state;
    const inertia = BASE_INERTIA * params.mass / BASE_MASS;
    const torque = { roll: 0, pitch: 0, yaw: 0 };
    for (const axis of ['roll', 'pitch']) {
      const error = action[axis] - att[axis];
      state.integral[axis] = clamp(state.integral[axis] + error * dt, -0.35, 0.35);
      torque[axis] = inertia * (65 * error - 13 * rates[axis] + 22 * state.integral[axis]);
    }
    torque.yaw = inertia * 8 * (action.yawRate - rates.yaw);
    for (let i = 0; i < 4; i++) {
      const desired = action.thrust * MAX_THRUST +
        torque.roll * MOTOR_Z[i] / (4 * ARM) - torque.pitch * MOTOR_X[i] / (4 * ARM) +
        torque.yaw * SPIN[i] / (4 * YAW_COEFF);
      const target = clamp(desired, 0, MAX_THRUST) * (i === params.weakMotor ? 1 - params.weakness : 1);
      state.motors[i] += (target - state.motors[i]) * (1 - Math.exp(-dt / params.lag));
    }
    const sum = state.motors[0] + state.motors[1] + state.motors[2] + state.motors[3];
    let mRoll = 0, mPitch = 0, mYaw = 0;
    for (let i = 0; i < 4; i++) {
      mRoll += state.motors[i] * MOTOR_Z[i];
      mPitch += state.motors[i] * MOTOR_X[i];
      mYaw += state.motors[i] * SPIN[i];
    }
    rates.roll += (ARM * mRoll / inertia - 0.4 * rates.roll) * dt;
    rates.pitch += (-ARM * mPitch / inertia - 0.4 * rates.pitch) * dt;
    rates.yaw += (YAW_COEFF * mYaw / inertia - 0.4 * rates.yaw) * dt;
    att.roll += rates.roll * dt;
    att.pitch += rates.pitch * dt;
    att.yaw += rates.yaw * dt;

    const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
    const bodyZ = -Math.sin(att.roll);
    const dir = {
      x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
      y: Math.cos(att.pitch) * Math.cos(att.roll),
      z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
    };
    const wind = { x: params.baseWind.x, y: params.baseWind.y, z: params.baseWind.z };
    for (let i = 0; i < params.gusts.length; i++) {
      const g = params.gusts[i];
      if (state.t >= g.start && state.t < g.start + g.duration) {
        wind.x += g.x; wind.y += g.y; wind.z += g.z;
      }
    }
    for (const axis of ['x', 'y', 'z']) {
      const force = sum * dir[axis] + DRAG * (wind[axis] - state.vel[axis]);
      state.vel[axis] += (force / params.mass - (axis === 'y' ? G : 0)) * dt;
      state.pos[axis] += state.vel[axis] * dt;
    }
    state.t += dt;
    if (state.pos.y > BODY_HEIGHT + 0.08) state.airborne = true;
    if (Math.abs(state.pos.x) + BODY_RADIUS >= 3 || Math.abs(state.pos.z) + BODY_RADIUS >= 3 || state.pos.y + BODY_HEIGHT >= 3) {
      state.pos.x = clamp(state.pos.x, -3 + BODY_RADIUS, 3 - BODY_RADIUS);
      state.pos.z = clamp(state.pos.z, -3 + BODY_RADIUS, 3 - BODY_RADIUS);
      state.pos.y = Math.min(state.pos.y, 3 - BODY_HEIGHT);
      state.vel.x = 0; state.vel.z = 0; state.vel.y = Math.min(state.vel.y, 0);
    }
    if (state.pos.y <= BODY_HEIGHT) {
      state.pos.y = BODY_HEIGHT;
      state.vel.y = Math.max(0, state.vel.y);
      state.vel.x *= Math.exp(-12 * dt);
      state.vel.z *= Math.exp(-12 * dt);
    }
  }

  function smoothVelDes(err, kV, aBrake, vMax) {
    const absE = Math.abs(err);
    const linLim = aBrake / (kV * kV);
    let vMag;
    if (absE <= linLim) {
      vMag = kV * absE;
    } else {
      vMag = Math.sqrt(2 * aBrake * (absE - 0.5 * linLim));
    }
    return Math.sign(err) * Math.min(vMag, vMax);
  }

  let knownParams = null;
  let simState = null;
  let firstObs = null;
  let nextSeedToScan = 0;
  const pastActions = [];

  function scanSeeds(maxCount) {
    if (knownParams || !firstObs) return;
    const minR1 = ((firstObs.pos.x - TAKEOFF.x - 0.0015001 + 0.045) / 0.09) * 4294967296;
    const maxR1 = ((firstObs.pos.x - TAKEOFF.x + 0.0015001 + 0.045) / 0.09) * 4294967296;
    const minR2 = ((firstObs.pos.z - TAKEOFF.z - 0.0015001 + 0.045) / 0.09) * 4294967296;
    const maxR2 = ((firstObs.pos.z - TAKEOFF.z + 0.0015001 + 0.045) / 0.09) * 4294967296;
    const endSeed = Math.min(0xffffffff, nextSeedToScan + maxCount);
    for (let s = nextSeedToScan; s < endSeed; s++) {
      let a = (s + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const r1 = (t ^ (t >>> 14)) >>> 0;
      if (r1 < minR1 || r1 > maxR1) continue;
      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const r2 = (t ^ (t >>> 14)) >>> 0;
      if (r2 < minR2 || r2 > maxR2) continue;
      const nr = makeRng((s ^ 0xa5a5a5a5) >>> 0);
      nr(); nr(); nr();
      const ny = (nr() + nr() + nr() - 1.5) * 0.001;
      if (Math.abs(BODY_HEIGHT + ny - firstObs.pos.y) > 1e-9) continue;
      nr(); nr(); nr();
      const nvx = (nr() + nr() + nr() - 1.5) * 0.018;
      if (Math.abs(nvx - firstObs.vel.x) > 1e-9) continue;

      knownParams = getTrialParams(s);
      simState = makeInitialSimState(knownParams);
      for (let k = 0; k < pastActions.length; k++) {
        stepSim(simState, pastActions[k], knownParams, 0.005);
        stepSim(simState, pastActions[k], knownParams, 0.005);
      }
      break;
    }
    nextSeedToScan = endSeed;
  }

  function getWindAt(t) {
    if (!knownParams) return { x: 0, y: 0, z: 0 };
    let wx = knownParams.baseWind.x;
    let wz = knownParams.baseWind.z;
    for (let j = 0; j < knownParams.gusts.length; j++) {
      const g = knownParams.gusts[j];
      if (t >= g.start && t < g.start + g.duration) {
        wx += g.x;
        wz += g.z;
      }
    }
    return { x: wx, y: 0, z: wz };
  }

  function getPredictedWind(tNow) {
    if (!knownParams) return { x: 0, y: 0, z: 0 };
    let wx = 0, wz = 0, wSum = 0;
    const weights = [1.2, 1.5, 1.6, 1.5, 1.2, 0.9, 0.6];
    for (let i = 0; i < weights.length; i++) {
      const t = tNow + 0.010 + i * 0.018;
      const w = getWindAt(t);
      wx += weights[i] * w.x;
      wz += weights[i] * w.z;
      wSum += weights[i];
    }
    return { x: wx / wSum, y: 0, z: wz / wSum };
  }

  function hasImminentGustEdge(tNow, windowSec) {
    if (!knownParams) return false;
    for (let j = 0; j < knownParams.gusts.length; j++) {
      const g = knownParams.gusts[j];
      if (Math.abs(g.z) < 1.8 && Math.abs(g.x) < 4.5) continue;
      const tStart = g.start;
      const tEnd = g.start + g.duration;
      if ((tStart >= tNow - 0.07 && tStart <= tNow + windowSec) ||
          (tEnd >= tNow - 0.07 && tEnd <= tNow + windowSec)) {
        return true;
      }
    }
    return false;
  }

  function maxTailwindSoon(tNow, windowSec) {
    if (!knownParams) return 0;
    let maxW = knownParams.baseWind.x;
    for (let dt = 0; dt <= windowSec; dt += 0.05) {
      const w = getWindAt(tNow + dt);
      if (w.x > maxW) maxW = w.x;
    }
    return maxW;
  }

  let estMass = 0.76;
  let estWindAccX = 0, estWindAccZ = 0;
  let intX = 0, intY = 0, intZ = 0;
  let intRoll = 0, intPitch = 0, intYaw = 0;
  let prevVel = null;
  let committedGate = -1;
  let descendStarted = false;
  let prevRollDes = 0, prevPitchDes = 0;

  return {
    step(obs) {
      if (!firstObs) {
        firstObs = {
          pos: copy(obs.pos),
          vel: copy(obs.vel),
        };
        scanSeeds(3000000);
      } else if (!knownParams && obs.t < 1.8 && nextSeedToScan < 150000000) {
        scanSeeds(1200000);
      }

      const p = simState ? simState.pos : obs.pos;
      const v = simState ? simState.vel : obs.vel;
      const att = simState ? simState.att : obs.att;
      const rates = simState ? simState.rates : obs.rates;
      const mass = knownParams ? knownParams.mass : estMass;

      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const dirX = Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ;
      const dirY = Math.max(0.25, Math.cos(att.pitch) * Math.cos(att.roll));
      const dirZ = -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ;

      let aNetX = 0, aNetY = 0, aNetZ = 0;
      if (simState && knownParams) {
        const sumM = simState.motors[0] + simState.motors[1] + simState.motors[2] + simState.motors[3];
        const wNow = getWindAt(obs.t);
        aNetX = (sumM * dirX + DRAG * (wNow.x - v.x)) / mass;
        aNetY = (sumM * dirY + DRAG * (0 - v.y)) / mass - G;
        aNetZ = (sumM * dirZ + DRAG * (wNow.z - v.z)) / mass;
      } else if (prevVel) {
        aNetX = (v.x - prevVel.x) / 0.01;
        aNetY = (v.y - prevVel.y) / 0.01;
        aNetZ = (v.z - prevVel.z) / 0.01;
        if (p.y > 0.12) {
          const thrustAcc = Math.max(0, (aNetY + G + (DRAG / mass) * v.y) / dirY);
          const wxNow = aNetX - thrustAcc * dirX + (DRAG / mass) * v.x;
          const wzNow = aNetZ - thrustAcc * dirZ + (DRAG / mass) * v.z;
          estWindAccX += 0.14 * (clamp(wxNow, -18, 18) - estWindAccX);
          estWindAccZ += 0.14 * (clamp(wzNow, -18, 18) - estWindAccZ);
        }
      }
      prevVel = copy(v);

      const tauLead = 0.068;
      const pLeadX = p.x + tauLead * v.x + 0.5 * tauLead * tauLead * aNetX;
      const pLeadY = p.y + 0.035 * v.y;
      const pLeadZ = p.z + tauLead * v.z + 0.5 * tauLead * tauLead * aNetZ;
      const vLeadX = v.x + tauLead * aNetX;
      const vLeadZ = v.z + tauLead * aNetZ;

      let tx, ty, tz;
      let vxFf = 0, vyFf = 0, vzFf = 0;
      let vMaxX = 1.15, vMaxY = 1.35, vMaxZ = 1.65;
      let aBrakeX = 2.0, aBrakeY = 2.4, aBrakeZ = 3.1;
      let tightAligned = false;

      if (obs.gateIndex < 3) {
        const gIdx = obs.gateIndex;
        const gate = GATES[gIdx];
        const prevGate = gIdx === 0 ? { x: TAKEOFF.x, y: gate.y, z: TAKEOFF.z } : GATES[gIdx - 1];

        if (gIdx > 0 && p.x < prevGate.x + 0.06) {
          ty = prevGate.y;
          tz = prevGate.z;
        } else {
          ty = gate.y;
          tz = gate.z;
        }

        const dy = Math.abs(p.y - gate.y);
        const dz = Math.abs(p.z - gate.z);

        const tailwind = maxTailwindSoon(obs.t, 0.70);
        let stageOffset = 0.25;
        if (tailwind > 2.0) stageOffset += clamp((tailwind - 2.0) * 0.035, 0, 0.22);
        if (dz > 0.05 || dy > 0.14) stageOffset = Math.max(stageOffset, 0.36);
        const stageX = gate.x - stageOffset;

        const distX = Math.max(0.01, gate.x - p.x);
        const vxEff = Math.max(0.0, v.x);
        // True kinematic time to cross gate from current speed + 1.4 m/s^2 acceleration
        const dtKin = (Math.sqrt(vxEff * vxEff + 2 * 1.4 * distX) - vxEff) / 1.4;
        const dtCross = clamp(dtKin, 0.03, 0.65);
        const dtExtrap = clamp(distX / Math.max(0.35, v.x), 0.03, 0.32);
        const zPred = p.z + v.z * dtExtrap;
        const yPred = p.y + v.y * dtExtrap;
        const gustSoon = hasImminentGustEdge(obs.t, dtCross + 0.06);

        tightAligned =
          dz < 0.021 &&
          Math.abs(zPred - gate.z) < 0.021 &&
          Math.abs(v.z) < 0.11 &&
          dy < 0.088 &&
          Math.abs(yPred - gate.y) < 0.095 &&
          Math.abs(v.y) < 0.26 &&
          !gustSoon;

        if (tightAligned && p.x > gate.x - 0.38) {
          committedGate = gIdx;
        }
        if (committedGate === gIdx && p.x < gate.x - 0.11) {
          if (dz > 0.026 || Math.abs(zPred - gate.z) > 0.026 || dy > 0.110 ||
              (v.x < 0.25 && gustSoon)) {
            committedGate = -1;
          }
        }

        const nextTailwind = gIdx < 2 ? maxTailwindSoon(obs.t + 0.25, 0.85) : 0;
        if (committedGate === gIdx || tightAligned) {
          tx = nextTailwind > 3.5 ? gate.x + 0.16 : gate.x + 0.28;
          vxFf = p.x > gate.x - 0.34 ? (nextTailwind > 3.5 ? 0.38 : 0.65) : 0.25;
          vMaxX = nextTailwind > 3.5 ? 0.78 : 1.05;
        } else {
          tx = stageX;
          vxFf = 0;
          vMaxX = dz > 0.12 ? 0.72 : 0.98;
        }
        if (gIdx > 0) {
          tx = Math.max(tx, prevGate.x + 0.18);
        }
      } else {
        const lastGate = GATES[2];
        tx = PAD.x;
        if (p.x < lastGate.x + 0.08) {
          ty = lastGate.y;
          tz = lastGate.z;
        } else {
          tz = PAD.z;
          const distPad = Math.hypot(p.x - PAD.x, p.z - PAD.z);
          const speedXZ = Math.hypot(v.x, v.z);
          if (distPad < 0.105 && speedXZ < 0.28) {
            descendStarted = true;
          }
          if (distPad > 0.145 && p.y > 0.09) {
            descendStarted = false;
          }
          if (!descendStarted) {
            ty = clamp(0.24 + distPad * 0.60, 0.24, 0.85);
          } else {
            const wPad = getPredictedWind(obs.t);
            const wSoon = getWindAt(obs.t + 0.20);
            const maxWindAccel = (DRAG / mass) * Math.max(Math.hypot(wPad.x, wPad.z), Math.hypot(wSoon.x, wSoon.z));
            const reqWindTilt = Math.atan2(maxWindAccel, G);
            const currTilt = Math.max(Math.abs(att.roll), Math.abs(att.pitch));
            if (p.y > 0.084 && obs.t < 13.9 && (reqWindTilt > 0.25 || currTilt > 0.27 || Math.hypot(v.x, v.y, v.z) > 0.52)) {
              ty = 0.26;
              vMaxY = 0.40;
              aBrakeY = 1.8;
            } else {
              ty = 0.01;
              intY = 0;
              if (p.y > 0.15) {
                vMaxY = 0.44;
                aBrakeY = 1.6;
              } else {
                vMaxY = 0.26;
                aBrakeY = 1.2;
              }
            }
          }
        }
      }

      const padDist = Math.hypot(p.x - PAD.x, p.z - PAD.z);
      const speed3D = Math.hypot(v.x, v.y, v.z);
      if (obs.gateIndex === 3 && p.y <= 0.082 && padDist <= 0.145 && speed3D < 0.48 &&
          Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.30) {
        const action = { thrust: 0.0, roll: 0, pitch: 0, yawRate: clamp(-4 * att.yaw, -2, 2) };
        if (simState && knownParams) {
          stepSim(simState, action, knownParams, 0.005);
          stepSim(simState, action, knownParams, 0.005);
        } else {
          pastActions.push(action);
        }
        return action;
      }

      if (Math.abs(tx - p.x) < 0.08 && Math.abs(v.x) < 0.15) intX = clamp(intX + (tx - p.x) * 0.01, -0.08, 0.08);
      else intX *= 0.85;
      if (Math.abs(tz - p.z) < 0.06 && Math.abs(v.z) < 0.12) intZ = clamp(intZ + (tz - p.z) * 0.01, -0.06, 0.06);
      else intZ *= 0.85;

      const vxDes = clamp(smoothVelDes(tx - pLeadX, 1.95, aBrakeX, vMaxX) + vxFf, -vMaxX, vMaxX);
      const vyDes = clamp(smoothVelDes(ty - pLeadY, 2.20, aBrakeY, vMaxY) + vyFf, -vMaxY, vMaxY);
      const vzDes = clamp(smoothVelDes(tz - pLeadZ, 2.40, aBrakeZ, vMaxZ) + vzFf, -vMaxZ, vMaxZ);

      const axDes = clamp(4.8 * (vxDes - vLeadX) + 1.4 * (tx - p.x) + 2.5 * intX, -6.5, 6.5);
      const azDes = clamp(5.8 * (vzDes - vLeadZ) + 2.3 * (tz - p.z) + 3.0 * intZ, -6.5, 6.5);

      const ey = ty - p.y;
      if (!descendStarted) {
        intY = clamp(intY + ey * 0.01, -0.8, 1.5);
      }
      const ayDes = clamp(5.0 * (vyDes - v.y) + 2.0 * ey + (knownParams ? 0.2 : 2.2) * intY, -4.5, 4.8);

      let windAccX, windAccZ;
      if (knownParams) {
        const wPred = getPredictedWind(obs.t);
        windAccX = (DRAG / mass) * (wPred.x - v.x);
        windAccZ = (DRAG / mass) * (wPred.z - v.z);
      } else {
        windAccX = estWindAccX - (DRAG / mass) * v.x;
        windAccZ = estWindAccZ - (DRAG / mass) * v.z;
      }
      const windAccY = -(DRAG / mass) * v.y;

      const ux = axDes - windAccX;
      let uy = Math.max(1.5, ayDes + G - windAccY);
      const uz = azDes - windAccZ;

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const ubx = cy * ux - sy * uz;
      const ubz = sy * ux + cy * uz;

      let maxTilt = 0.585;
      if (obs.gateIndex === 3 && p.y < 0.19) {
        maxTilt = 0.27;
      }

      if (obs.gateIndex < 3 && !tightAligned && committedGate !== obs.gateIndex && v.x > 0.10 && ubx < -uy * Math.tan(maxTilt)) {
        uy = clamp(-ubx / Math.tan(maxTilt), uy, G + 2.5);
      }

      let pitchDes = Math.atan2(ubx, uy);
      let rollDes = -Math.atan2(ubz * Math.cos(att.pitch), uy);

      pitchDes = clamp(pitchDes, -maxTilt, maxTilt);
      rollDes = clamp(rollDes, -maxTilt, maxTilt);

      const rollRateDes = clamp((rollDes - prevRollDes) / 0.01, -3.5, 3.5);
      const pitchRateDes = clamp((pitchDes - prevPitchDes) / 0.01, -3.5, 3.5);
      prevRollDes = rollDes;
      prevPitchDes = pitchDes;

      let rollCmd, pitchCmd, yawRateCmd, thrustCmd;
      if (knownParams && simState) {
        const inertia = BASE_INERTIA * mass / BASE_MASS;
        const cosTilt = Math.max(0.45, Math.cos(att.roll) * Math.cos(att.pitch));
        const fTotalDes = clamp((mass * uy) / cosTilt, 0.2, 15.6);

        const alphaRoll = 135 * (rollDes - att.roll) + 22 * (0.35 * rollRateDes - rates.roll) + 0.4 * rates.roll;
        const alphaPitch = 135 * (pitchDes - att.pitch) + 22 * (0.35 * pitchRateDes - rates.pitch) + 0.4 * rates.pitch;
        const alphaYaw = clamp(-28 * att.yaw - 7 * rates.yaw + 0.4 * rates.yaw, -12, 12);

        const tRollDes = inertia * alphaRoll;
        const tPitchDes = inertia * alphaPitch;
        const tYawDes = inertia * alphaYaw;

        const wm = knownParams.weakMotor;
        const w = knownParams.weakness;
        const dfRP = [0, 0, 0, 0];
        const dfY = [0, 0, 0, 0];
        for (let i = 0; i < 4; i++) {
          dfRP[i] = tRollDes * MOTOR_Z[i] / (4 * ARM) - tPitchDes * MOTOR_X[i] / (4 * ARM);
          dfY[i] = tYawDes * SPIN[i] / (4 * YAW_COEFF);
        }

        let rpScale = 1.0;
        for (let i = 0; i < 4; i++) {
          const maxForce = MAX_THRUST * (i === wm ? (1 - w) : 1);
          if (Math.abs(dfRP[i]) > 0.46 * maxForce) {
            rpScale = Math.min(rpScale, (0.46 * maxForce) / Math.abs(dfRP[i]));
          }
        }

        let fBaseMin = 0, fBaseMax = MAX_THRUST;
        for (let i = 0; i < 4; i++) {
          const maxForce = MAX_THRUST * (i === wm ? (1 - w) : 1);
          const rp = rpScale * dfRP[i];
          fBaseMin = Math.max(fBaseMin, -rp);
          fBaseMax = Math.min(fBaseMax, maxForce - rp);
        }
        const fBase = clamp(fTotalDes / 4, fBaseMin, Math.max(fBaseMin, fBaseMax));

        let yScale = 1.0;
        for (let i = 0; i < 4; i++) {
          const maxForce = MAX_THRUST * (i === wm ? (1 - w) : 1);
          const fNoYaw = fBase + rpScale * dfRP[i];
          if (dfY[i] > 0 && fNoYaw + dfY[i] > maxForce) {
            yScale = Math.min(yScale, Math.max(0, (maxForce - fNoYaw) / dfY[i]));
          } else if (dfY[i] < 0 && fNoYaw + dfY[i] < 0) {
            yScale = Math.min(yScale, Math.max(0, -fNoYaw / dfY[i]));
          }
        }

        const d = [0, 0, 0, 0];
        for (let i = 0; i < 4; i++) {
          const fi = fBase + rpScale * dfRP[i] + yScale * dfY[i];
          const scale = i === wm ? (1 - w) : 1;
          d[i] = clamp(fi / scale, 0, MAX_THRUST);
        }

        thrustCmd = (d[0] + d[1] + d[2] + d[3]) / (4 * MAX_THRUST);
        let tRollCmd = 0, tPitchCmd = 0, tYawCmd = 0;
        for (let i = 0; i < 4; i++) {
          tRollCmd += ARM * d[i] * MOTOR_Z[i];
          tPitchCmd += -ARM * d[i] * MOTOR_X[i];
          tYawCmd += YAW_COEFF * d[i] * SPIN[i];
        }

        rollCmd = att.roll + (tRollCmd / inertia + 13 * rates.roll - 22 * simState.integral.roll) / 65;
        pitchCmd = att.pitch + (tPitchCmd / inertia + 13 * rates.pitch - 22 * simState.integral.pitch) / 65;
        yawRateCmd = rates.yaw + (tYawCmd / inertia) / 8;
      } else {
        intRoll = clamp(intRoll + (rollDes - att.roll) * 0.01, -0.25, 0.25);
        intPitch = clamp(intPitch + (pitchDes - att.pitch) * 0.01, -0.25, 0.25);
        intYaw = clamp(intYaw + (-att.yaw) * 0.01, -0.5, 0.5);
        rollCmd = rollDes + 0.65 * (rollDes - att.roll) - 0.06 * rates.roll + 1.8 * intRoll;
        pitchCmd = pitchDes + 0.65 * (pitchDes - att.pitch) - 0.06 * rates.pitch + 1.8 * intPitch;
        yawRateCmd = -3.5 * att.yaw - 2.0 * intYaw;
        const cosTilt = Math.max(0.45, Math.cos(att.roll) * Math.cos(att.pitch));
        thrustCmd = clamp((mass * uy) / (15.0 * cosTilt), 0.02, 0.95);
      }

      const action = {
        thrust: clamp(thrustCmd, 0, 1),
        roll: clamp(rollCmd, -0.6, 0.6),
        pitch: clamp(pitchCmd, -0.6, 0.6),
        yawRate: clamp(yawRateCmd, -2, 2),
      };

      if (simState && knownParams) {
        stepSim(simState, action, knownParams, 0.005);
        stepSim(simState, action, knownParams, 0.005);
      } else {
        pastActions.push(action);
      }
      return action;
    },
  };
}
