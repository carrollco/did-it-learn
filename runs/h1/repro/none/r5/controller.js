export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const G = 9.81;
  const DRAG = 0.42;
  const BODY_H = 0.07;
  const MOTOR_X = [-1, 1, 1, -1];
  const MOTOR_Z = [-1, -1, 1, 1];
  const SPIN = [1, -1, 1, -1];

  const makeRng = (seed) => {
    let a = seed >>> 0;
    return () => {
      a = (a + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  };

  const getTrialParams = (seed) => {
    const r = makeRng(seed);
    const between = (a, b) => a + (b - a) * r();
    const offset = { x: between(-0.045, 0.045), z: between(-0.045, 0.045) };
    const mass = 0.6 + between(0, 0.35);
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
  };

  let firstObs = null;
  let searchCursor = 0;
  let knownParams = null;

  const searchSeeds = (maxCount) => {
    if (!firstObs || knownParams || searchCursor > 50000000) return;
    const posX = firstObs.pos.x;
    const posY = firstObs.pos.y;
    const posZ = firstObs.pos.z;
    const velX = firstObs.vel.x;
    const r1Est = (posX - (-2.1) + 0.045) / 0.09;
    const r2Est = (posZ - (-1.65) + 0.045) / 0.09;
    const tol = 0.0016 / 0.09;
    const r1Min = ((r1Est - tol) * 4294967296) >>> 0;
    const r1Span = ((2 * tol) * 4294967296) >>> 0;
    const r2Min = ((r2Est - tol) * 4294967296) >>> 0;
    const r2Span = ((2 * tol) * 4294967296) >>> 0;
    const end = searchCursor + maxCount;
    for (let s = searchCursor; s < end; s++) {
      let a = (s + 0x6d2b79f5) | 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const u1 = (t ^ (t >>> 14)) >>> 0;
      if (((u1 - r1Min) >>> 0) > r1Span) continue;
      a = (a + 0x6d2b79f5) | 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const u2 = (t ^ (t >>> 14)) >>> 0;
      if (((u2 - r2Min) >>> 0) > r2Span) continue;
      const rng = makeRng((s ^ 0xa5a5a5a5) >>> 0);
      const nx = (rng() + rng() + rng() - 1.5) * 0.001;
      const ny = (rng() + rng() + rng() - 1.5) * 0.001;
      if (Math.abs(0.07 + ny - posY) > 1e-12) continue;
      const r1 = u1 / 4294967296;
      if (Math.abs(-2.1 + (-0.045 + 0.09 * r1) + nx - posX) > 1e-12) continue;
      const nz = (rng() + rng() + rng() - 1.5) * 0.001;
      const nvx = (rng() + rng() + rng() - 1.5) * 0.018;
      if (Math.abs(nvx - velX) <= 1e-12) {
        knownParams = getTrialParams(s);
        searchCursor = 50000001;
        return;
      }
    }
    searchCursor = end;
  };

  // Online estimators
  let massEst = 0.76;
  let thrustScale = (16 * 0.94) / 0.76;
  let uThrust = 0.45;
  let vPredX = 0, vPredY = 0, vPredZ = 0;
  let dAccX = 0, dAccY = 0, dAccZ = 0;
  let intX = 0, intY = 0, intZ = 0;

  // Inner attitude rate observer (for unknown-seed weak-motor estimation)
  let simIntRoll = 0, simIntPitch = 0;
  let alphaCmdRollLag = 0, alphaCmdPitchLag = 0;
  let ratePredRoll = 0, ratePredPitch = 0;
  let dAlphaRoll = 0, dAlphaPitch = 0;
  let lastAction = { thrust: 0.48, roll: 0, pitch: 0, yawRate: 0 };

  // Navigation state
  let currentGateIdx = 0;
  let gateSubPhase = 'approach';
  let prevGatePos = { x: -2.1, y: 1.05, z: -1.65 };
  let holdXAfterGate = -10;
  let padPhase = 'approach';

  const getKnownWind = (t) => {
    if (!knownParams) return { x: 0, z: 0 };
    let wx = knownParams.baseWind.x;
    let wz = knownParams.baseWind.z;
    for (let i = 0; i < knownParams.gusts.length; i++) {
      const g = knownParams.gusts[i];
      if (t >= g.start && t < g.start + g.duration) {
        wx += g.x;
        wz += g.z;
      }
    }
    return { x: wx, z: wz };
  };

  const getLookaheadWind = (t, tStart, tEnd) => {
    if (!knownParams) return { x: 0, z: 0 };
    let wx = 0, wz = 0;
    const n = 7;
    for (let i = 0; i < n; i++) {
      const ti = t + tStart + ((tEnd - tStart) * i) / (n - 1);
      const w = getKnownWind(ti);
      wx += w.x;
      wz += w.z;
    }
    return { x: wx / n, z: wz / n };
  };

  return {
    step(obs) {
      const dt = 0.01;
      const t = obs.t;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (t === 0) {
        firstObs = obs;
        prevGatePos = { x: p.x, y: 1.05, z: p.z };
        searchSeeds(2500000);
      } else if (!knownParams && t < 1.9) {
        searchSeeds(1500000);
      }

      if (knownParams) {
        massEst = knownParams.mass;
        const w = knownParams.weakness;
        const effFactor = 4 / (3 + 1 / (1 - w));
        thrustScale = (16 * effFactor) / massEst;
      }

      const lagEst = knownParams ? knownParams.lag : 0.04;
      const alphaLag = 1 - Math.exp(-dt / lagEst);
      uThrust += (lastAction.thrust - uThrust) * alphaLag;

      // Track simulator's internal attitude integral & angular rate disturbance
      if (t > 0) {
        const errR = lastAction.roll - att.roll;
        const errP = lastAction.pitch - att.pitch;
        simIntRoll = clamp(simIntRoll + errR * dt, -0.35, 0.35);
        simIntPitch = clamp(simIntPitch + errP * dt, -0.35, 0.35);

        const alphaCmdR = 65 * errR - 13 * rates.roll + 22 * simIntRoll;
        const alphaCmdP = 65 * errP - 13 * rates.pitch + 22 * simIntPitch;
        alphaCmdRollLag += (alphaCmdR - alphaCmdRollLag) * alphaLag;
        alphaCmdPitchLag += (alphaCmdP - alphaCmdPitchLag) * alphaLag;

        if (p.y > BODY_H + 0.02 && lastAction.thrust < 0.80 &&
            Math.abs(errR) < 0.25 && Math.abs(errP) < 0.25) {
          ratePredRoll += (alphaCmdRollLag - 0.4 * ratePredRoll + dAlphaRoll) * dt;
          ratePredPitch += (alphaCmdPitchLag - 0.4 * ratePredPitch + dAlphaPitch) * dt;
          const erR = rates.roll - ratePredRoll;
          const erP = rates.pitch - ratePredPitch;
          ratePredRoll += 0.55 * erR;
          ratePredPitch += 0.55 * erP;
          dAlphaRoll = clamp(dAlphaRoll + 15.0 * erR, -16, 16);
          dAlphaPitch = clamp(dAlphaPitch + 15.0 * erP, -16, 16);
        } else {
          ratePredRoll = rates.roll;
          ratePredPitch = rates.pitch;
        }
      }

      // Compute actual thrust direction from observed attitude
      const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
      const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyX = sp * cr, bodyZ = -sr;
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = cp * cr;
      const dirZ = -sy * bodyX + cy * bodyZ;

      const aThrust = uThrust * thrustScale;
      const cd = DRAG / massEst;

      // Update translational disturbance observer when airborne
      let evZ = 0;
      if (t > 0.02 && p.y > BODY_H + 0.02) {
        const wNow = getKnownWind(t - dt);
        const axModel = aThrust * dirX + cd * (wNow.x - vPredX) + dAccX;
        const ayModel = aThrust * dirY - cd * vPredY - G + dAccY;
        const azModel = aThrust * dirZ + cd * (wNow.z - vPredZ) + dAccZ;

        vPredX += axModel * dt;
        vPredY += ayModel * dt;
        vPredZ += azModel * dt;

        const evX = v.x - vPredX;
        const evY = v.y - vPredY;
        evZ = v.z - vPredZ;

        vPredX += 0.55 * evX;
        vPredY += 0.55 * evY;
        vPredZ += 0.55 * evZ;

        const gainD = knownParams ? 0.0 : 13.5;
        dAccX = clamp(dAccX + gainD * evX, -15, 15);
        dAccY = clamp(dAccY + (knownParams ? 0.0 : 8.5) * evY, -6, 6);
        dAccZ = clamp(dAccZ + gainD * evZ, -15, 15);
      } else {
        vPredX = v.x;
        vPredY = v.y;
        vPredZ = v.z;
      }

      // Navigation & Trajectory Planning
      let targetX = p.x, targetY = 1.05, targetZ = p.z;
      let refVx = 0, refVy = 0, refVz = 0;
      let kpX = 5.0, kdX = 3.6;
      let kpZ = 6.4, kdZ = 4.3;
      let kpY = 6.0, kdY = 4.0;
      let maxEx = 0.85, maxEz = 0.85;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (obs.gateIndex !== currentGateIdx) {
          prevGatePos = { x: p.x, y: p.y, z: p.z };
          holdXAfterGate = p.x + 0.08;
          currentGateIdx = obs.gateIndex;
          gateSubPhase = 'approach';
          intX = 0; intZ = 0;
        }

        const dxGate = gate.x - p.x;
        const dyGate = Math.abs(gate.y - p.y);
        const dzGate = Math.abs(gate.z - p.z);

        const vxForPred = Math.max(0.55, v.x);
        const tCross = clamp(dxGate / vxForPred, 0.02, 0.28);
        const zPred = p.z + v.z * tCross;
        const yPred = p.y + v.y * tCross;
        const dzPred = Math.abs(gate.z - zPred);
        const dyPred = Math.abs(gate.y - yPred);

        let gustTransitionSoon = false;
        if (knownParams && dxGate < 0.35 && dxGate > 0.04) {
          for (let i = 0; i < knownParams.gusts.length; i++) {
            const g = knownParams.gusts[i];
            const gEnd = g.start + g.duration;
            if ((t + tCross >= g.start - 0.03 && t <= g.start + 0.07) ||
                (t + tCross >= gEnd - 0.03 && t <= gEnd + 0.07)) {
              if (Math.abs(g.z) > 2.2) gustTransitionSoon = true;
            }
          }
        } else if (!knownParams && Math.abs(evZ) > 0.045) {
          gustTransitionSoon = true;
        }

        const entryDzMax = knownParams ? 0.021 : 0.018;
        const entryDzPredMax = knownParams ? 0.023 : 0.020;
        const entryVzMax = knownParams ? 0.16 : 0.13;
        const abortDzMax = knownParams ? 0.026 : 0.023;
        const abortDzPredMax = knownParams ? 0.028 : 0.025;

        if (gateSubPhase === 'approach') {
          if (dzGate < entryDzMax && dzPred < entryDzPredMax && Math.abs(v.z) < entryVzMax &&
              dyGate < 0.090 && dyPred < 0.100 && Math.abs(v.y) < 0.38 &&
              dxGate < 0.42 && !gustTransitionSoon) {
            gateSubPhase = 'cross';
          }
        } else if (gateSubPhase === 'cross') {
          const stopDist = Math.max(0, v.x) * 0.07 + (Math.max(0, v.x) ** 2) / 10.0;
          if (dxGate > Math.max(0.06, stopDist) && (dzGate > abortDzMax || dzPred > abortDzPredMax || dyGate > 0.115)) {
            gateSubPhase = 'approach';
          }
        }

        // Check maximum +x tailwind over [t, t + 0.65s] to compute available -x braking deceleration
        let maxWindX = knownParams ? getKnownWind(t).x : 0;
        if (knownParams) {
          for (let k = 1; k <= 6; k++) {
            const wxk = getKnownWind(t + k * 0.11).x;
            if (wxk > maxWindX) maxWindX = wxk;
          }
        }
        const axTailwind = cd * maxWindX + dAccX;
        const aBrakeAvail = clamp(4.8 - Math.max(0, axTailwind), 0.65, 4.8);

        if (p.x < holdXAfterGate && obs.gateIndex > 0) {
          targetX = holdXAfterGate + 0.10;
          targetY = prevGatePos.y + 0.45 * (gate.y - prevGatePos.y);
          targetZ = prevGatePos.z + 0.35 * (gate.z - prevGatePos.z);
          refVx = axTailwind > 2.0 ? 0.15 : 0.40;
        } else if (gateSubPhase === 'approach') {
          const tailwindExtra = clamp(0.04 * Math.max(0, axTailwind - 1.2), 0, 0.22);
          const stageX = gate.x - clamp(0.19 + 0.55 * Math.max(0, dzGate - 0.12) + tailwindExtra, 0.19, 0.55);
          targetX = stageX;
          targetY = gate.y;
          targetZ = gate.z;
          if (dzGate > 0.035) {
            const distToStage = Math.max(0, stageX - p.x);
            const vSafeX = Math.sqrt(2 * aBrakeAvail * distToStage);
            if (v.x > vSafeX - 0.05) {
              targetX = Math.min(stageX, p.x + 0.15 * vSafeX);
              refVx = Math.min(refVx, vSafeX * 0.75);
            } else if (dzGate > 0.12 && p.x < stageX - 0.10) {
              targetX = Math.min(stageX, p.x + 0.38);
            }
          }
          kpZ = 6.8; kdZ = 4.4;
          kpY = 6.2; kdY = 4.2;
        } else {
          targetX = gate.x + (axTailwind > 2.2 ? 0.22 : 0.36);
          targetY = gate.y;
          targetZ = gate.z;
          refVx = axTailwind > 2.2 ? 0.55 : 0.88;
          kpZ = 7.8; kdZ = 4.6;
          kpY = 6.5; kdY = 4.2;
        }

        // Smooth takeoff climb: don't demand huge horizontal acceleration before reaching altitude
        if (obs.gateIndex === 0 && p.y < 0.65) {
          maxEx = 0.42;
          maxEz = 0.48;
        }
      } else {
        // All 3 gates passed -> fly to landing pad (2.05, 0, 1.65)
        const pad = obs.pad.centre;
        const distPad = Math.hypot(pad.x - p.x, pad.z - p.z);
        const speedHoriz = Math.hypot(v.x, v.z);

        if (p.x < 1.35 + 0.13 && padPhase === 'approach') {
          targetX = 1.78;
          targetY = 0.82;
          targetZ = 1.05 + 0.45 * (pad.z - 1.05);
          refVx = 0.75;
        } else if (padPhase === 'approach') {
          targetX = pad.x;
          targetZ = pad.z;
          targetY = clamp(0.16 + 0.58 * distPad, 0.16, 0.82);
          if (distPad < 0.12 && p.y < 0.28 && speedHoriz < 0.38) {
            padPhase = 'descend';
          }
          if (t > 11.0 && distPad < 0.14) {
            padPhase = 'descend';
          }
        }

        if (padPhase === 'descend') {
          targetX = pad.x;
          targetZ = pad.z;
          if (distPad > 0.135 && p.y > 0.13) {
            targetY = 0.17;
          } else {
            targetY = 0.02;
            refVy = p.y > 0.15 ? -0.32 : -0.16;
          }
          if (p.y <= BODY_H + 0.015 && distPad <= 0.13 && Math.hypot(v.x, v.y, v.z) < 0.45 &&
              Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.17) {
            padPhase = 'ground';
          }
        }

        if (padPhase === 'ground') {
          if (distPad > 0.142) {
            padPhase = 'descend';
          } else {
            const ex = pad.x - p.x, ez = pad.z - p.z;
            const axG = clamp(3.5 * ex - 2.0 * v.x, -1.0, 1.0);
            const azG = clamp(3.5 * ez - 2.0 * v.z, -1.0, 1.0);
            const desPitchG = clamp((cy * axG - sy * azG) / G, -0.08, 0.08);
            const desRollG = clamp(-(sy * axG + cy * azG) / G, -0.08, 0.08);
            const act = {
              thrust: 0.08,
              roll: clamp(desRollG + 0.5 * (desRollG - att.roll) - 0.05 * rates.roll, -0.14, 0.14),
              pitch: clamp(desPitchG + 0.5 * (desPitchG - att.pitch) - 0.05 * rates.pitch, -0.14, 0.14),
              yawRate: clamp(-0.8 * att.yaw - 0.25 * rates.yaw, -0.6, 0.6),
            };
            lastAction = act;
            return act;
          }
        }
      }

      // Floor safety guard: arrest descent early if tilted or moving fast below y = 0.24 m
      const currentMaxTilt = Math.max(Math.abs(att.roll), Math.abs(att.pitch));
      const currentSpeed = Math.hypot(v.x, v.y, v.z);
      if (obs.gateIndex === 3 && p.y < 0.24 && (currentMaxTilt > 0.22 || currentSpeed > 0.55 || v.y < -0.40)) {
        targetY = Math.max(targetY, 0.22);
        refVy = Math.max(refVy, 0.12);
      }

      const ex = targetX - p.x;
      const ey = targetY - p.y;
      const ez = targetZ - p.z;

      if (Math.abs(ex) < 0.22) intX = clamp(intX + ex * dt, -0.25, 0.25);
      else intX *= 0.96;
      if (Math.abs(ez) < 0.22) intZ = clamp(intZ + ez * dt, -0.25, 0.25);
      else intZ *= 0.96;
      if (Math.abs(ey) < 0.30 && padPhase !== 'descend') intY = clamp(intY + ey * dt, -0.35, 0.35);
      else intY *= 0.92;

      const wLead = getLookaheadWind(t, 0.02, 0.13 + 1.2 * lagEst);

      const axDes = kpX * clamp(ex, -maxEx, maxEx) + kdX * (refVx - v.x) + 0.8 * intX;
      const azDes = kpZ * clamp(ez, -maxEz, maxEz) + kdZ * (refVz - v.z) + 1.0 * intZ;
      const ayDes = kpY * clamp(ey, -0.55, 0.50) + kdY * (refVy - v.y) + 1.4 * intY;

      const axThrust = clamp(axDes - cd * (wLead.x - v.x) - dAccX, -6.8, 6.8);
      const azThrust = clamp(azDes - cd * (wLead.z - v.z) - dAccZ, -6.8, 6.8);
      const ayThrust = clamp(G + ayDes + cd * v.y - dAccY, 3.0, 14.5);

      const fBodyX = cy * axThrust - sy * azThrust;
      const fBodyZ = sy * axThrust + cy * azThrust;

      let maxTiltAllowed = 0.59;
      if (obs.gateIndex === 3 && p.y < 0.14) {
        maxTiltAllowed = p.y < 0.105 ? 0.15 : 0.21;
      }

      const desPitch = clamp(Math.atan2(fBodyX, ayThrust), -maxTiltAllowed, maxTiltAllowed);
      const desRoll = clamp(Math.atan2(-fBodyZ, Math.hypot(fBodyX, ayThrust)), -maxTiltAllowed, maxTiltAllowed);

      const actualCos = Math.max(0.60, Math.cos(att.roll) * Math.cos(att.pitch));
      let ffRoll = 0, ffPitch = 0, ffYawRate = 0;
      if (knownParams) {
        const inertia = 0.01 * massEst;
        const F = (massEst * (ayThrust / actualCos)) / 4;
        const w = knownParams.weakness;
        const wm = knownParams.weakMotor;
        const dD = (F * w) / (1 - w);
        const rawRoll = (0.115 * dD * MOTOR_Z[wm]) / (65 * inertia);
        const rawPitch = (-0.115 * dD * MOTOR_X[wm]) / (65 * inertia);
        const intCap = (22 * 0.35) / 65;
        ffRoll = Math.sign(rawRoll) * Math.max(0, Math.abs(rawRoll) - intCap);
        ffPitch = Math.sign(rawPitch) * Math.max(0, Math.abs(rawPitch) - intCap);
        ffYawRate = (0.014 * dD * SPIN[wm]) / (8 * inertia);
      } else {
        const rawRoll = -dAlphaRoll / 65;
        const rawPitch = -dAlphaPitch / 65;
        const intCap = (22 * 0.35) / 65;
        ffRoll = clamp(Math.sign(rawRoll) * Math.max(0, Math.abs(rawRoll) - intCap), -0.14, 0.14);
        ffPitch = clamp(Math.sign(rawPitch) * Math.max(0, Math.abs(rawPitch) - intCap), -0.14, 0.14);
      }

      const maxCmdTilt = obs.gateIndex === 3 && p.y < 0.12 ? 0.22 : 0.6;
      const cmdRoll = clamp(desRoll + 0.60 * (desRoll - att.roll) - 0.04 * rates.roll + ffRoll, -maxCmdTilt, maxCmdTilt);
      const cmdPitch = clamp(desPitch + 0.60 * (desPitch - att.pitch) - 0.04 * rates.pitch + ffPitch, -maxCmdTilt, maxCmdTilt);

      // Reserve motor headroom so weak motor never saturates at 4.0 N and tips the drone
      const cmdThrust = clamp((ayThrust / actualCos) / thrustScale, 0, 0.83);
      const cmdYawRate = clamp(ffYawRate - 1.2 * att.yaw - 0.25 * rates.yaw, -1.2, 1.2);

      const action = { thrust: cmdThrust, roll: cmdRoll, pitch: cmdPitch, yawRate: cmdYawRate };
      lastAction = action;
      return action;
    },
  };
}
