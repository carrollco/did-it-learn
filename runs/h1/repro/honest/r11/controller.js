export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const MOTOR_TAU = 0.038;
  const DT = 0.01;

  let prevObs = null;
  let prevAction = { thrust: 0.45, roll: 0, pitch: 0, yawRate: 0 };

  const simInt = { roll: 0, pitch: 0 };

  let lagThrust = 0.45;
  const lagAlpha = { roll: 0, pitch: 0, yaw: 0 };

  let thrustScale = 20.0;
  let airborneSteps = 0;

  const rateEst = { roll: 0, pitch: 0, yaw: 0 };
  const beta = { roll: 0, pitch: 0, yaw: 0 };
  const bRes = { roll: 0, pitch: 0, yaw: 0 };

  const velEst = { x: 0, y: 0, z: 0 };
  const accEst = { x: 0, y: 0, z: 0 };
  const windFast = { x: 0, z: 0 };
  const windSlow = { x: 0, z: 0 };

  let prevRollDes = 0;
  let prevPitchDes = 0;

  let targetGate = -1;
  let lastGate = null;
  let curGate = null;
  let phase = 'align';
  let settle = 0;
  let intZ = 0;
  let intY = 0;
  let padPinned = false;

  function smoothProfileVel(err, maxV, decel, kp) {
    const aErr = Math.abs(err);
    const vLin = kp * aErr;
    const vSqrt = Math.sqrt(2 * decel * aErr);
    return Math.sign(err) * Math.min(maxV, vLin, vSqrt);
  }

  return {
    step(obs) {
      const p = obs.pos;
      const v = obs.vel;
      const att = obs.att;
      const rates = obs.rates;

      if (prevObs === null) {
        velEst.x = v.x; velEst.y = v.y; velEst.z = v.z;
        rateEst.roll = rates.roll; rateEst.pitch = rates.pitch; rateEst.yaw = rates.yaw;
      } else {
        for (let sub = 0; sub < 2; sub++) {
          const frac = (sub + 0.5) * 0.5;
          const rEst = prevObs.att.roll + frac * (att.roll - prevObs.att.roll);
          const pEst = prevObs.att.pitch + frac * (att.pitch - prevObs.att.pitch);
          simInt.roll = clamp(simInt.roll + (prevAction.roll - rEst) * 0.005, -0.35, 0.35);
          simInt.pitch = clamp(simInt.pitch + (prevAction.pitch - pEst) * 0.005, -0.35, 0.35);
        }

        const cmdAlpha = {
          roll: 65 * (prevAction.roll - prevObs.att.roll) - 13 * prevObs.rates.roll + 22 * simInt.roll,
          pitch: 65 * (prevAction.pitch - prevObs.att.pitch) - 13 * prevObs.rates.pitch + 22 * simInt.pitch,
          yaw: 8 * (prevAction.yawRate - prevObs.rates.yaw),
        };

        const alphaDecay = 1 - Math.exp(-DT / MOTOR_TAU);
        lagThrust += (prevAction.thrust - lagThrust) * alphaDecay;
        lagAlpha.roll += (cmdAlpha.roll - lagAlpha.roll) * alphaDecay;
        lagAlpha.pitch += (cmdAlpha.pitch - lagAlpha.pitch) * alphaDecay;
        lagAlpha.yaw += (cmdAlpha.yaw - lagAlpha.yaw) * alphaDecay;

        const estMass = clamp(15.3 / thrustScale, 0.6, 0.95);
        const maxBeta = 16.5 / estMass;

        for (const axis of ['roll', 'pitch', 'yaw']) {
          const predRate = rateEst[axis] + (lagAlpha[axis] - 0.4 * rateEst[axis] + beta[axis] * lagThrust + bRes[axis]) * DT;
          const errRate = rates[axis] - predRate;
          rateEst[axis] = predRate + 0.45 * errRate;
          if (lagThrust > 0.18 && prevAction.thrust < 0.80 && p.y > 0.085 && Math.abs(cmdAlpha[axis]) < 32) {
            const gB = airborneSteps < 35 ? 5.2 : 2.0;
            const lim = axis === 'yaw' ? 4.5 : maxBeta;
            beta[axis] = clamp(beta[axis] + gB * errRate / Math.max(0.22, lagThrust), -lim, lim);
            bRes[axis] = clamp(bRes[axis] + 0.5 * errRate, -2.0, 2.0);
          }
        }

        const midRoll = 0.5 * (prevObs.att.roll + att.roll);
        const midPitch = 0.5 * (prevObs.att.pitch + att.pitch);
        const midYaw = 0.5 * (prevObs.att.yaw + att.yaw);

        const bodyX = Math.sin(midPitch) * Math.cos(midRoll);
        const bodyZ = -Math.sin(midRoll);
        const dirX = Math.cos(midYaw) * bodyX + Math.sin(midYaw) * bodyZ;
        const dirY = Math.cos(midPitch) * Math.cos(midRoll);
        const dirZ = -Math.sin(midYaw) * bodyX + Math.cos(midYaw) * bodyZ;

        const cd = 0.42 / estMass;

        if (p.y > 0.09 && p.y < 2.88 && prevObs.pos.y > 0.09) {
          airborneSteps++;
          const ayModel = thrustScale * lagThrust * dirY - cd * velEst.y - 9.81;
          const predVy = velEst.y + ayModel * DT;
          const errVy = v.y - predVy;
          velEst.y = predVy + 0.42 * errVy;
          accEst.y = ayModel;

          const uEff = lagThrust * dirY;
          if (uEff > 0.22 && prevAction.thrust > 0.08 && prevAction.thrust < 0.90) {
            const gS = airborneSteps < 30 ? 7.5 : 2.8;
            thrustScale = clamp(thrustScale + gS * errVy / uEff, 15.0, 27.0);
          }

          const specThrust = thrustScale * lagThrust;
          const axModel = specThrust * dirX - cd * velEst.x + windFast.x;
          const azModel = specThrust * dirZ - cd * velEst.z + windFast.z;
          const predVx = velEst.x + axModel * DT;
          const predVz = velEst.z + azModel * DT;
          const errVx = v.x - predVx;
          const errVz = v.z - predVz;
          velEst.x = predVx + 0.44 * errVx;
          velEst.z = predVz + 0.44 * errVz;
          accEst.x = axModel;
          accEst.z = azModel;

          windFast.x = clamp(windFast.x + 5.5 * errVx, -12, 12);
          windFast.z = clamp(windFast.z + 6.0 * errVz, -12, 12);
          windSlow.x += 0.04 * (windFast.x - windSlow.x);
          windSlow.z += 0.04 * (windFast.z - windSlow.z);
        } else {
          velEst.x = v.x; velEst.y = v.y; velEst.z = v.z;
          accEst.x = 0; accEst.y = 0; accEst.z = 0;
        }
      }

      const estMass = clamp(15.3 / thrustScale, 0.6, 0.95);
      const cd = 0.42 / estMass;

      const gustDeltaZ = windFast.z - windSlow.z;
      const gustDeltaX = windFast.x - windSlow.x;

      let axDes = 0;
      let ayDes = 0;
      let azDes = 0;
      let maxTilt = obs.t < 0.22 ? 0.38 : 0.52;
      let lowThrustPin = false;

      if (obs.gateIndex < 3) {
        padPinned = false;
        const gate = obs.nextGate.centre;
        if (targetGate !== obs.gateIndex) {
          if (curGate !== null) lastGate = curGate;
          curGate = { x: gate.x, y: gate.y, z: gate.z };
          targetGate = obs.gateIndex;
          phase = 'align';
          settle = 0;
          intZ = 0;
          intY = 0;
        }

        const dxGate = gate.x - p.x;
        const dyGate = gate.y - p.y;
        const dzGate = gate.z - p.z;

        const vx = velEst.x;
        const vy = velEst.y;
        const vz = velEst.z;

        const tCross = clamp(dxGate / Math.max(0.35, vx + 0.15), 0.02, 0.24);
        const zPred = p.z + vz * tCross + 0.25 * accEst.z * tCross * tCross;
        const yPred = p.y + vy * tCross + 0.25 * accEst.y * tCross * tCross;

        const vxPos = Math.max(0, vx);
        const stopDistX = vxPos * 0.075 + (vxPos * vxPos) / (2 * 2.6);

        if (dxGate <= 0.0) {
          phase = 'align';
          settle = 0;
        } else if (phase === 'align') {
          const zCentered = Math.abs(dzGate) < 0.012 && Math.abs(vz) < 0.075 && Math.abs(zPred - gate.z) < 0.014;
          const yCentered = Math.abs(dyGate) < 0.065 && Math.abs(vy) < 0.24 && Math.abs(yPred - gate.y) < 0.085;
          const xReady = dxGate > 0.04 && dxGate < 0.32;
          const gustCalm = Math.abs(gustDeltaZ) < 1.35;

          if (zCentered && yCentered && xReady && gustCalm) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }

          if (settle >= 3) {
            phase = 'lunge';
          }
        } else if (phase === 'lunge') {
          const zUnsafe = Math.abs(zPred - gate.z) > 0.019 || Math.abs(dzGate) > 0.020 || Math.abs(gustDeltaZ) > 2.2;
          const yUnsafe = Math.abs(yPred - gate.y) > 0.095 || Math.abs(dyGate) > 0.095;
          const canBrake = dxGate > stopDistX + 0.015 && dxGate > 0.05;

          if ((zUnsafe || yUnsafe) && canBrake) {
            phase = 'align';
            settle = 0;
          }
        }

        if (Math.abs(dzGate) < 0.12 && Math.abs(dxGate) < 0.45) {
          intZ = clamp(intZ + dzGate * DT, -0.25, 0.25);
        } else {
          intZ *= 0.94;
        }
        if (Math.abs(dyGate) < 0.20) {
          intY = clamp(intY + dyGate * DT, -0.35, 0.35);
        }

        if (Math.abs(dzGate) > 0.16) {
          const vzDes = smoothProfileVel(dzGate, 1.40, 2.2, 4.5);
          azDes = 6.5 * (vzDes - vz) - windFast.z - 0.38 * gustDeltaZ;
        } else {
          azDes = 15.5 * dzGate - 7.0 * vz + 3.8 * intZ - windFast.z - 0.38 * gustDeltaZ;
        }
        azDes = clamp(azDes, -5.4, 5.4);

        const vyDes = smoothProfileVel(dyGate, 1.35, 2.2, 4.4);
        ayDes = clamp(6.0 * (vyDes - vy) + 2.2 * intY, -3.5, 3.6);

        if (dxGate <= 0.0) {
          if (Math.abs(dzGate) < 0.015 && Math.abs(dyGate) < 0.07) {
            const targetX = gate.x - 0.20;
            const vxDes = smoothProfileVel(targetX - p.x, 0.65, 1.8, 3.5);
            axDes = 5.8 * (vxDes - vx) - windFast.x;
          } else {
            const targetX = gate.x + 0.12;
            const vxDes = smoothProfileVel(targetX - p.x, 0.60, 1.8, 3.5);
            axDes = 5.8 * (vxDes - vx) - windFast.x;
          }
          axDes = clamp(axDes, -4.8, 4.5);
        } else if (phase === 'align') {
          const zUnaligned = Math.abs(dzGate) > 0.025 || Math.abs(vz) > 0.15;
          const standOff = zUnaligned ? (0.24 + clamp(0.42 * Math.abs(dzGate), 0, 0.22)) : 0.17;
          const targetX = gate.x - standOff;
          const ex = targetX - p.x;
          const maxVx = Math.abs(dzGate) > 0.12 ? 1.00 : 1.35;
          const vxDes = smoothProfileVel(ex, maxVx, 1.9, 4.2);
          axDes = 5.8 * (vxDes - vx) - windFast.x - 0.35 * gustDeltaX;

          if (dxGate < stopDistX + 0.08 && (Math.abs(dzGate) > 0.017 || Math.abs(zPred - gate.z) > 0.019 || Math.abs(dyGate) > 0.08)) {
            axDes = Math.min(axDes, -5.2 - windFast.x);
          }
          axDes = clamp(axDes, -5.4, 4.4);
        } else {
          const targetX = gate.x + 0.32;
          const ex = targetX - p.x;
          const vxDes = clamp(5.0 * ex + 0.60, 0.80, 1.55);
          axDes = clamp(6.5 * (vxDes - vx) - windFast.x - 0.35 * gustDeltaX, -4.5, 4.8);
        }

        if (lastGate !== null && p.x > lastGate.x - 0.02 && p.x < lastGate.x + 0.15) {
          const minVx = 0.58;
          if (vx < minVx) {
            axDes = Math.max(axDes, 6.2 * (minVx - vx) - windFast.x);
          }
        }
      } else {
        const pad = obs.pad.centre;
        const ex = pad.x - p.x;
        const ez = pad.z - p.z;
        const distXZ = Math.hypot(ex, ez);
        const speedXZ = Math.hypot(velEst.x, velEst.z);
        const speed3D = Math.hypot(velEst.x, velEst.y, velEst.z);

        if (p.y <= 0.086 && distXZ <= 0.138 && speed3D < 0.55) {
          padPinned = true;
        }
        if (padPinned && (distXZ > 0.146 || p.y > 0.105)) {
          padPinned = false;
        }

        if (padPinned) {
          lowThrustPin = true;
          maxTilt = 0.12;
          axDes = clamp(7.0 * ex - 3.5 * velEst.x - 0.55 * windFast.x, -1.8, 1.8);
          azDes = clamp(7.0 * ez - 3.5 * velEst.z - 0.55 * windFast.z, -1.8, 1.8);
          ayDes = -6.0;
        } else {
          if (distXZ < 0.16) {
            intZ = clamp(intZ + ez * DT, -0.25, 0.25);
          } else {
            intZ *= 0.95;
          }

          const vxDes = smoothProfileVel(ex, 1.35, 2.0, 4.3);
          const vzDes = smoothProfileVel(ez, 1.35, 2.0, 4.3);
          const axPD = clamp(6.2 * (vxDes - velEst.x), -4.5, 4.5);
          const azPD = clamp(6.2 * (vzDes - velEst.z) + 2.2 * intZ, -4.5, 4.5);
          axDes = clamp(axPD - windFast.x - 0.35 * gustDeltaX, -7.2, 7.2);
          azDes = clamp(azPD - windFast.z - 0.35 * gustDeltaZ, -7.2, 7.2);

          if (p.x < 1.48 && velEst.x < 0.52) {
            axDes = Math.max(axDes, 5.5 * (0.52 - velEst.x) - windFast.x);
          }

          let targetY;
          if (distXZ > 0.13 || (p.y < 0.16 && speedXZ > 0.38)) {
            targetY = clamp(0.11 + 0.55 * distXZ, 0.11, 0.85);
          } else {
            targetY = 0.01;
          }

          const ey = targetY - p.y;
          const maxDesc = p.y > 0.25 ? -0.95 : (p.y > 0.14 ? -0.42 : -0.24);
          let vyDes = clamp(4.0 * ey, maxDesc, 1.4);
          maxTilt = p.y < 0.18 ? 0.31 : 0.55;
          ayDes = clamp(6.4 * (vyDes - velEst.y), -3.8, 3.8);
        }
      }

      if (p.x < -2.42 && velEst.x < -0.25) axDes = Math.max(axDes, 7.0 * (-0.1 - velEst.x) - windFast.x);
      if (p.x > 2.42 && velEst.x > 0.25) axDes = Math.min(axDes, 7.0 * (0.1 - velEst.x) - windFast.x);
      if (p.z < -2.42 && velEst.z < -0.25) azDes = Math.max(azDes, 7.0 * (-0.1 - velEst.z) - windFast.z);
      if (p.z > 2.42 && velEst.z > 0.25) azDes = Math.min(azDes, 7.0 * (0.1 - velEst.z) - windFast.z);
      if (p.y > 2.65 && velEst.y > 0.2) ayDes = Math.min(ayDes, 6.5 * (0.1 - velEst.y));

      const aTx = axDes + cd * velEst.x;
      const aTz = azDes + cd * velEst.z;
      const aTy = Math.max(2.0, 9.81 + cd * velEst.y + ayDes);

      const cy = Math.cos(att.yaw);
      const sy = Math.sin(att.yaw);
      const fx = cy * aTx - sy * aTz;
      const fz = sy * aTx + cy * aTz;
      const aTotal = Math.hypot(fx, aTy, fz);

      let pitchDes = clamp(Math.atan2(fx, aTy), -maxTilt, maxTilt);
      let rollDes = clamp(-Math.asin(clamp(fz / aTotal, -0.6, 0.6)), -maxTilt, maxTilt);

      let thrust;
      if (lowThrustPin) {
        thrust = 0.12;
      } else {
        const cosTilt = Math.max(0.5, Math.cos(att.roll) * Math.cos(att.pitch));
        thrust = clamp(aTy / (thrustScale * cosTilt), 0.05, 0.92);
      }

      const rollRateDes = clamp((rollDes - prevRollDes) / DT, -3.5, 3.5);
      const pitchRateDes = clamp((pitchDes - prevPitchDes) / DT, -3.5, 3.5);
      prevRollDes = rollDes;
      prevPitchDes = pitchDes;

      const kpAtt = 85;
      const kdAtt = 17;
      const alphaRollDes = kpAtt * (rollDes - att.roll) - kdAtt * rateEst.roll + 10 * rollRateDes;
      const alphaPitchDes = kpAtt * (pitchDes - att.pitch) - kdAtt * rateEst.pitch + 10 * pitchRateDes;

      const biasRoll = beta.roll * thrust + bRes.roll * (lowThrustPin ? 0.25 : 1.0);
      const biasPitch = beta.pitch * thrust + bRes.pitch * (lowThrustPin ? 0.25 : 1.0);
      const biasYaw = beta.yaw * thrust + bRes.yaw * (lowThrustPin ? 0.25 : 1.0);

      const uAlphaRoll = alphaRollDes + 0.4 * rateEst.roll - biasRoll;
      const uAlphaPitch = alphaPitchDes + 0.4 * rateEst.pitch - biasPitch;

      const rollCmd = clamp(att.roll + (uAlphaRoll + 13 * rateEst.roll - 22 * simInt.roll) / 65.165, -0.6, 0.6);
      const pitchCmd = clamp(att.pitch + (uAlphaPitch + 13 * rateEst.pitch - 22 * simInt.pitch) / 65.165, -0.6, 0.6);

      const alphaYawDes = -14 * att.yaw - 5.5 * rateEst.yaw;
      const yawRateCmd = clamp(rates.yaw + (alphaYawDes + 0.4 * rateEst.yaw - biasYaw) / 8, -2, 2);

      const action = {
        thrust,
        roll: rollCmd,
        pitch: pitchCmd,
        yawRate: yawRateCmd,
      };

      prevObs = obs;
      prevAction = action;
      return action;
    },
  };
}
