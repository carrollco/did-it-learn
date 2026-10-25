const GATE_LIST = [
  { x: -1.15, y: 1.05, z: -0.85 },
  { x: 0.15, y: 1.40, z: 0.25 },
  { x: 1.35, y: 1.05, z: 1.05 },
];

export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const hypot2 = (x, z) => Math.hypot(x, z);
  const hypot3 = (x, y, z) => Math.hypot(x, y, z);

  let prevObs = null;
  let lastAction = { thrust: 0.5, roll: 0, pitch: 0, yawRate: 0 };
  let simInt = { roll: 0, pitch: 0 };
  let uEff = { roll: 0, pitch: 0, yaw: 0 };
  let thrustEff = 0.5;

  // Torque bias observer (per unit thrust + fast residual)
  let kBias = { roll: 0, pitch: 0, yaw: 0 };
  let fastBias = { roll: 0, pitch: 0 };

  // Thrust scale observer: effective (maxThrustSum / mass), nominal ~20.0
  let thrustScale = 20.0;
  let vyHat = 0;

  // Wind acceleration observers (m/s^2)
  let vHat = { x: 0, z: 0 };
  let wAcc = { x: 0, z: 0 };
  let wFast = { x: 0, z: 0 };
  let baseWindAcc = { x: 0, z: 0 };

  // Navigation state
  let targetGate = -1;
  let phase = 'align';
  let centeredCount = 0;
  let padPinned = false;

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos;
      const v = obs.vel;
      const att = obs.att;
      const rates = obs.rates;

      if (prevObs !== null) {
        const alphaLag = 0.23; // ~0.038s motor lag
        for (const axis of ['roll', 'pitch']) {
          const e1 = lastAction[axis] - prevObs.att[axis];
          simInt[axis] = clamp(simInt[axis] + e1 * 0.005, -0.35, 0.35);
          const u1 = 65 * e1 - 13 * prevObs.rates[axis] + 22 * simInt[axis];

          const attMid = 0.5 * (prevObs.att[axis] + att[axis]);
          const rateMid = 0.5 * (prevObs.rates[axis] + rates[axis]);
          const e2 = lastAction[axis] - attMid;
          simInt[axis] = clamp(simInt[axis] + e2 * 0.005, -0.35, 0.35);
          const u2 = 65 * e2 - 13 * rateMid + 22 * simInt[axis];

          const uCmd = 0.5 * (u1 + u2);
          uEff[axis] += (uCmd - uEff[axis]) * alphaLag;
        }
        const uYawCmd = 8 * (lastAction.yawRate - 0.5 * (prevObs.rates.yaw + rates.yaw));
        uEff.yaw += (uYawCmd - uEff.yaw) * alphaLag;
        thrustEff += (lastAction.thrust - thrustEff) * alphaLag;

        const airborne = p.y > 0.085 && prevObs.pos.y > 0.085 && p.y < 2.91;

        if (airborne && thrustEff > 0.12 && thrustEff < 0.82) {
          for (const axis of ['roll', 'pitch']) {
            if (Math.abs(lastAction[axis]) < 0.56) {
              const rateMid = 0.5 * (prevObs.rates[axis] + rates[axis]);
              const alphaMeas = (rates[axis] - prevObs.rates[axis]) / dt + 0.4 * rateMid;
              const res = alphaMeas - uEff[axis];
              const predBias = kBias[axis] * thrustEff;
              const err = clamp(res - predBias, -20, 20);
              const gain = 0.20 * thrustEff / (thrustEff * thrustEff + 0.04);
              kBias[axis] = clamp(kBias[axis] + gain * err, -25, 25);
              fastBias[axis] = clamp(fastBias[axis] + 0.07 * (res - kBias[axis] * thrustEff - fastBias[axis]), -5, 5);
            }
          }
          const rateYawMid = 0.5 * (prevObs.rates.yaw + rates.yaw);
          const alphaYawMeas = (rates.yaw - prevObs.rates.yaw) / dt + 0.4 * rateYawMid;
          const resYaw = alphaYawMeas - uEff.yaw;
          const errYaw = clamp(resYaw - kBias.yaw * thrustEff, -8, 8);
          kBias.yaw = clamp(kBias.yaw + 0.15 * thrustEff * errYaw / (thrustEff * thrustEff + 0.04), -6, 6);
        }

        const attR = 0.5 * (prevObs.att.roll + att.roll);
        const attP = 0.5 * (prevObs.att.pitch + att.pitch);
        const attY = 0.5 * (prevObs.att.yaw + att.yaw);
        const bodyX = Math.sin(attP) * Math.cos(attR);
        const bodyZ = -Math.sin(attR);
        const dirX = Math.cos(attY) * bodyX + Math.sin(attY) * bodyZ;
        const dirY = Math.cos(attP) * Math.cos(attR);
        const dirZ = -Math.sin(attY) * bodyX + Math.cos(attY) * bodyZ;

        if (airborne) {
          const vyMid = 0.5 * (prevObs.vel.y + v.y);
          const uy = thrustEff * dirY - 0.0275 * vyMid;
          const vyPred = vyHat + (thrustScale * uy - 9.81) * dt;
          const errVy = v.y - vyPred;
          vyHat = vyPred + 0.42 * errVy;
          if (uy > 0.15 && thrustEff < 0.82) {
            thrustScale = clamp(thrustScale + 5.0 * errVy * uy, 14.8, 26.8);
          }
        } else {
          vyHat = v.y;
        }

        const clearWalls = Math.abs(p.x) < 2.81 && Math.abs(p.z) < 2.81;
        if (airborne && clearWalls) {
          const cDrag = 0.0275 * thrustScale;
          for (const axis of ['x', 'z']) {
            const dirA = axis === 'x' ? dirX : dirZ;
            const vMid = 0.5 * (prevObs.vel[axis] + v[axis]);
            const aKnown = thrustScale * thrustEff * dirA - cDrag * vMid;
            const vPred = vHat[axis] + (aKnown + wAcc[axis]) * dt;
            const errV = v[axis] - vPred;
            vHat[axis] = vPred + 0.46 * errV;
            wAcc[axis] = clamp(wAcc[axis] + 6.8 * errV, -8.5, 8.5);

            const aMeas = (v[axis] - prevObs.vel[axis]) / dt;
            const wInst = clamp(aMeas - aKnown, -10.5, 10.5);
            wFast[axis] += 0.36 * (wInst - wFast[axis]);
          }
          if (obs.t > 0.2 && obs.t < 1.98) {
            baseWindAcc.x += 0.08 * (wAcc.x - baseWindAcc.x);
            baseWindAcc.z += 0.08 * (wAcc.z - baseWindAcc.z);
          } else if (obs.t >= 1.98) {
            const gDelta = hypot2(wFast.x - baseWindAcc.x, wFast.z - baseWindAcc.z);
            if (gDelta < 1.0) {
              baseWindAcc.x += 0.015 * (wAcc.x - baseWindAcc.x);
              baseWindAcc.z += 0.015 * (wAcc.z - baseWindAcc.z);
            }
          }
        } else {
          vHat.x = v.x;
          vHat.z = v.z;
        }
      } else {
        vyHat = v.y;
        vHat.x = v.x;
        vHat.z = v.z;
      }

      const gustVecX = obs.t >= 1.98 ? wFast.x - baseWindAcc.x : 0;
      const gustVecZ = obs.t >= 1.98 ? wFast.z - baseWindAcc.z : 0;
      const gustMag = hypot2(gustVecX, gustVecZ);
      const gustTransient = hypot2(wFast.x - wAcc.x, wFast.z - wAcc.z);
      const gustActive = gustMag > 1.35 || gustTransient > 1.1;

      const cDrag = 0.0275 * thrustScale;
      let axDes = 0, ayDes = 0, azDes = 0;

      if (obs.gateIndex < 3) {
        padPinned = false;
        const gate = obs.nextGate.centre;
        if (targetGate !== obs.gateIndex) {
          targetGate = obs.gateIndex;
          phase = 'align';
          centeredCount = 0;
        }

        const dxGate = gate.x - p.x;
        const dyGate = gate.y - p.y;
        const dzGate = gate.z - p.z;

        const zPred = p.z + v.z * 0.15;
        const yPred = p.y + v.y * 0.15;
        const ezNow = Math.abs(dzGate);
        const eyNow = Math.abs(dyGate);
        const ezPred = Math.abs(gate.z - zPred);
        const eyPred = Math.abs(gate.y - yPred);

        const timeUrgent = obs.t > 10.2;
        const zTol = timeUrgent ? 0.021 : 0.014;
        const yTol = timeUrgent ? 0.075 : 0.052;
        const vzTol = timeUrgent ? 0.13 : 0.080;
        const vyTol = timeUrgent ? 0.22 : 0.15;

        const xStop = p.x + v.x * 0.22 + (v.x > 0 ? (v.x * v.x) / 4.8 : 0);

        if (phase === 'align') {
          const latNear = ezNow < 0.030 && eyNow < 0.075;
          const baseStandOff = latNear ? (gustActive ? 0.33 : 0.27) : (gustActive ? 0.40 : 0.33);
          const xStandOff = baseStandOff + clamp(0.45 * Math.max(0, ezNow - 0.04), 0, 0.25);
          const tx = gate.x - xStandOff;
          let tz = gate.z;
          let ty = gate.y;

          if (obs.gateIndex > 0) {
            const prevG = GATE_LIST[obs.gateIndex - 1];
            if (p.x < prevG.x + 0.12 && v.x < 0.25) {
              tz = prevG.z;
              ty = prevG.y;
            }
          }

          const vzDes = clamp(1.6 * (tz - p.z), -1.15, 1.15);
          azDes = 4.8 * (vzDes - v.z);

          const vyDes = clamp(1.6 * (ty - p.y), -1.15, 1.15);
          ayDes = 4.6 * (vyDes - v.y);

          const closeX = p.x > gate.x - 0.40 && p.x < gate.x - 0.07;
          const isCentered = closeX && ezNow < zTol && ezPred < zTol + 0.004 &&
            eyNow < yTol && eyPred < yTol + 0.01 &&
            Math.abs(v.z) < vzTol && Math.abs(v.y) < vyTol &&
            (!gustActive || timeUrgent || gustTransient < 0.72);

          if (isCentered) centeredCount++;
          else centeredCount = Math.max(0, centeredCount - 1);

          const neededCount = timeUrgent ? 3 : 5;
          if (centeredCount >= neededCount) {
            phase = 'lunge';
          } else {
            const tailwind = Math.max(0, wFast.x);
            const vxCap = clamp((gustActive ? 0.56 : 0.84) - 0.08 * tailwind, 0.38, 0.84);
            const vxDes = clamp(1.25 * (tx - p.x), -0.95, vxCap);
            axDes = 4.4 * (vxDes - v.x);

            if (obs.gateIndex > 0) {
              const prevG = GATE_LIST[obs.gateIndex - 1];
              if (p.x < prevG.x + 0.20) {
                axDes = Math.max(axDes, 6.5 * (0.65 - v.x) + 1.5);
              }
            }

            // Gate plane barrier: never drift across gate.x while uncentered
            if (xStop > gate.x - 0.12) {
              axDes = Math.min(axDes, -7.5 * (xStop - (gate.x - 0.12)) - 5.5 * Math.max(0, v.x));
            }
          }
        }

        if (phase === 'lunge') {
          const dtCross = clamp(dxGate / Math.max(0.45, v.x + 0.45), 0.02, 0.26);
          const zAtCross = Math.abs(dzGate - v.z * dtCross);
          const yAtCross = Math.abs(dyGate - v.y * dtCross);
          const canStop = dxGate > 0.14 && v.x < 0.85 && (p.x + v.x * 0.16 + (v.x * v.x) / 9.0 < gate.x - 0.03);

          if (canStop && (zAtCross > 0.026 || yAtCross > 0.105)) {
            phase = 'align';
            centeredCount = 0;
            axDes = -7.5 * Math.max(0, v.x) + 4.5 * ((gate.x - 0.30) - p.x);
            azDes = 10.0 * dzGate - 5.8 * v.z;
            ayDes = 8.0 * dyGate - 4.8 * v.y;
          } else {
            const tx = gate.x + 0.34;
            const vxLungeCap = obs.gateIndex === 2 ? 1.10 : 1.40;
            axDes = clamp(6.2 * (tx - p.x) - 3.4 * v.x + 2.1, -6.0, 5.5 * (vxLungeCap - v.x) + 2.1);
            azDes = 12.5 * dzGate - 6.4 * v.z;
            ayDes = 8.5 * dyGate - 4.8 * v.y;
          }
        }
      } else {
        // Gate 3 passed -> fly to pad and land
        const pad = obs.pad.centre;
        const dxPad = pad.x - p.x;
        const dzPad = pad.z - p.z;
        const distH = hypot2(dxPad, dzPad);
        const speedH = hypot2(v.x, v.z);
        const speed3 = hypot3(v.x, v.y, v.z);

        if (padPinned) {
          if (distH > 0.145 || p.y > 0.094) {
            padPinned = false;
          }
        } else {
          if (distH < 0.136 && p.y <= 0.092 && speed3 < 0.46 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.175) {
            padPinned = true;
          }
        }

        if (padPinned) {
          const axPin = clamp(4.0 * dxPad - 3.0 * v.x - wAcc.x, -1.1, 1.1);
          const azPin = clamp(4.0 * dzPad - 3.0 * v.z - wAcc.z, -1.1, 1.1);
          const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
          const pitchDes = clamp((cy * axPin - sy * azPin) / 9.81, -0.10, 0.10);
          const rollDes = clamp(-(sy * axPin + cy * azPin) / 9.81, -0.10, 0.10);
          const thrustCmd = 0.01;

          const alphaRoll = 75 * (rollDes - att.roll) - 15 * rates.roll;
          const alphaPitch = 75 * (pitchDes - att.pitch) - 15 * rates.pitch;
          const alphaYaw = -24 * att.yaw - 8 * rates.yaw;

          const rollCmd = clamp(att.roll + (alphaRoll + 13.4 * rates.roll - 22 * simInt.roll - kBias.roll * thrustCmd - fastBias.roll) / 65, -0.14, 0.14);
          const pitchCmd = clamp(att.pitch + (alphaPitch + 13.4 * rates.pitch - 22 * simInt.pitch - kBias.pitch * thrustCmd - fastBias.pitch) / 65, -0.14, 0.14);
          const yawRateCmd = clamp(rates.yaw + (alphaYaw + 0.4 * rates.yaw - kBias.yaw * thrustCmd) / 8, -2, 2);

          lastAction = { thrust: thrustCmd, roll: rollCmd, pitch: pitchCmd, yawRate: yawRateCmd };
          prevObs = obs;
          return lastAction;
        }

        let ty = clamp(0.03 + 0.56 * Math.max(0, distH - 0.11), 0.03, 0.88);
        if (distH < 0.17 && speedH < 0.45) ty = 0.02;
        if (obs.t > 12.2 && distH < 0.24) ty = 0.02;

        const vxDes = clamp(1.36 * dxPad, -1.05, 1.05);
        const vzDes = clamp(1.36 * dzPad, -1.05, 1.05);
        axDes = 4.8 * (vxDes - v.x);
        azDes = 4.8 * (vzDes - v.z);

        if (p.x < 1.35 + 0.18) {
          axDes = Math.max(axDes, 6.0 * (0.55 - v.x) + 1.2);
        }

        const ey = ty - p.y;
        const vyTarget = clamp(3.3 * ey, p.y < 0.22 ? -0.45 : -0.88, 1.0);
        ayDes = 6.5 * (vyTarget - v.y);
      }

      // Brake gusts & room boundary protection
      if (gustActive && phase !== 'lunge') {
        axDes -= 0.7 * v.x;
        azDes -= 0.7 * v.z;
      }
      if (p.x < -2.35 && v.x < 0) axDes = Math.max(axDes, -6.5 * v.x + 5.5 * (-2.35 - p.x));
      if (p.x > 2.32 && v.x > 0) axDes = Math.min(axDes, -6.5 * v.x + 5.5 * (2.32 - p.x));
      if (p.z < -2.35 && v.z < 0) azDes = Math.max(azDes, -6.5 * v.z + 5.5 * (-2.35 - p.z));
      if (p.z > 2.32 && v.z > 0) azDes = Math.min(azDes, -6.5 * v.z + 5.5 * (2.32 - p.z));
      if (p.y > 2.65 && v.y > 0) ayDes = Math.min(ayDes, -6.0 * v.y + 5.0 * (2.65 - p.y));

      // Cancel linear drag and observed wind acceleration
      const wWeight = phase === 'lunge' ? 0.58 : 0.48;
      const wCtrlX = wAcc.x + wWeight * (wFast.x - wAcc.x);
      const wCtrlZ = wAcc.z + wWeight * (wFast.z - wAcc.z);
      const axThrust = clamp(axDes + cDrag * v.x - wCtrlX, -8.5, 8.5);
      const azThrust = clamp(azDes + cDrag * v.z - wCtrlZ, -8.5, 8.5);

      let maxTilt = 0.57;
      if (p.y < 0.22) maxTilt = 0.28;
      if (obs.gateIndex === 3 && p.y < 0.18 && hypot2(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z) < 0.16) maxTilt = 0.15;

      let ayThrust = clamp(9.81 + ayDes + cDrag * v.y, 1.5, 16.0);
      const aHoriz = hypot2(axThrust, azThrust);
      if (p.y > 0.24 && p.y < 2.45 && aHoriz > 5.0) {
        ayThrust = Math.max(ayThrust, clamp(aHoriz / Math.tan(maxTilt), 9.81, 13.8));
      }

      const dirY = Math.max(0.50, Math.cos(att.roll) * Math.cos(att.pitch));
      const thrustCmd = clamp(ayThrust / (thrustScale * dirY), 0.05, 0.85);

      // Desired attitude from desired thrust vector (axThrust, ayThrust, azThrust)
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bx = cy * axThrust - sy * azThrust;
      const bz = sy * axThrust + cy * azThrust;
      const pitchDes = clamp(Math.atan2(bx, ayThrust), -maxTilt, maxTilt);
      const rollDes = clamp(-Math.atan2(bz, Math.hypot(bx, ayThrust)), -maxTilt, maxTilt);

      // Invert inner attitude loop with torque bias
      const alphaRoll = 95 * (rollDes - att.roll) - 18 * rates.roll;
      const alphaPitch = 95 * (pitchDes - att.pitch) - 18 * rates.pitch;
      const alphaYaw = -28 * att.yaw - 9 * rates.yaw;

      const rollCmd = clamp(att.roll + (alphaRoll + 13.4 * rates.roll - 22 * simInt.roll - kBias.roll * thrustCmd - fastBias.roll) / 65, -0.6, 0.6);
      const pitchCmd = clamp(att.pitch + (alphaPitch + 13.4 * rates.pitch - 22 * simInt.pitch - kBias.pitch * thrustCmd - fastBias.pitch) / 65, -0.6, 0.6);
      const yawRateCmd = clamp(rates.yaw + (alphaYaw + 0.4 * rates.yaw - kBias.yaw * thrustCmd) / 8, -2, 2);

      lastAction = { thrust: thrustCmd, roll: rollCmd, pitch: pitchCmd, yawRate: yawRateCmd };
      prevObs = obs;
      return lastAction;
    },
  };
}
