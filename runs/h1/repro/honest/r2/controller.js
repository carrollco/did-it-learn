export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01;
  const g = 9.81;

  let stepCount = 0;
  let prevPos = null;
  let prevVel = null;
  let prevAtt = null;
  let prevRates = null;
  let prevRollDes = 0;
  let prevPitchDes = 0;
  let rollRateDesFilt = 0;
  let pitchRateDesFilt = 0;

  let lastAction = { thrust: 0.5, roll: 0, pitch: 0, yawRate: 0 };
  let simIntegral = { roll: 0, pitch: 0 };
  let laggedCmdAlpha = { roll: 0, pitch: 0, yaw: 0 };
  let laggedThrust = 0.5;

  // Observers: torque bias, thrust scale (KT), and wind acceleration
  let torqueBias = { roll: 0, pitch: 0, yaw: 0 };
  let kt = 20.0;
  let windFast = { x: 0, z: 0 };
  let windSlow = { x: 0, z: 0 };
  let velFilt = { x: 0, y: 0, z: 0 };
  let ratesFilt = { roll: 0, pitch: 0, yaw: 0 };

  // Navigation state
  let targetGate = -1;
  let phase = 'align';
  let centeredCount = 0;
  let zTrim = 0;
  let yTrim = 0;
  let padPinned = false;

  return {
    step(obs) {
      const p = obs.pos;
      const v = obs.vel;
      const att = obs.att;
      const rates = obs.rates;

      if (stepCount === 0) {
        velFilt = { x: v.x, y: v.y, z: v.z };
        ratesFilt = { roll: rates.roll, pitch: rates.pitch, yaw: rates.yaw };
      } else {
        velFilt.x += 0.68 * (v.x - velFilt.x);
        velFilt.y += 0.68 * (v.y - velFilt.y);
        velFilt.z += 0.68 * (v.z - velFilt.z);
        ratesFilt.roll += 0.65 * (rates.roll - ratesFilt.roll);
        ratesFilt.pitch += 0.65 * (rates.pitch - ratesFilt.pitch);
        ratesFilt.yaw += 0.65 * (rates.yaw - ratesFilt.yaw);

        const alphaMotor = 1 - Math.exp(-dt / 0.038);
        const attMidRoll = 0.5 * (prevAtt.roll + att.roll);
        const attMidPitch = 0.5 * (prevAtt.pitch + att.pitch);
        const rateMidRoll = 0.5 * (prevRates.roll + rates.roll);
        const rateMidPitch = 0.5 * (prevRates.pitch + rates.pitch);
        const rateMidYaw = 0.5 * (prevRates.yaw + rates.yaw);

        simIntegral.roll = clamp(simIntegral.roll + (lastAction.roll - attMidRoll) * dt, -0.35, 0.35);
        simIntegral.pitch = clamp(simIntegral.pitch + (lastAction.pitch - attMidPitch) * dt, -0.35, 0.35);

        const cmdAlphaRoll = 65 * (lastAction.roll - attMidRoll) - 13 * rateMidRoll + 22 * simIntegral.roll;
        const cmdAlphaPitch = 65 * (lastAction.pitch - attMidPitch) - 13 * rateMidPitch + 22 * simIntegral.pitch;
        const cmdAlphaYaw = 8 * (lastAction.yawRate - rateMidYaw);

        laggedCmdAlpha.roll += alphaMotor * (cmdAlphaRoll - laggedCmdAlpha.roll);
        laggedCmdAlpha.pitch += alphaMotor * (cmdAlphaPitch - laggedCmdAlpha.pitch);
        laggedCmdAlpha.yaw += alphaMotor * (cmdAlphaYaw - laggedCmdAlpha.yaw);
        laggedThrust += alphaMotor * (lastAction.thrust - laggedThrust);

        const measAlphaRoll = (rates.roll - prevRates.roll) / dt;
        const measAlphaPitch = (rates.pitch - prevRates.pitch) / dt;
        const measAlphaYaw = (rates.yaw - prevRates.yaw) / dt;

        if (lastAction.thrust > 0.15) {
          const resRoll = measAlphaRoll - (laggedCmdAlpha.roll - 0.4 * rateMidRoll);
          const resPitch = measAlphaPitch - (laggedCmdAlpha.pitch - 0.4 * rateMidPitch);
          const resYaw = measAlphaYaw - (laggedCmdAlpha.yaw - 0.4 * rateMidYaw);

          const biasGain = stepCount < 25 ? 0.18 : 0.10;
          torqueBias.roll = clamp(torqueBias.roll + biasGain * (resRoll - torqueBias.roll), -25, 25);
          torqueBias.pitch = clamp(torqueBias.pitch + biasGain * (resPitch - torqueBias.pitch), -25, 25);
          torqueBias.yaw = clamp(torqueBias.yaw + biasGain * (resYaw - torqueBias.yaw), -10, 10);
        }

        // Direction of thrust at midpoint attitude
        const yawMid = 0.5 * (prevAtt.yaw + att.yaw);
        const cr = Math.cos(attMidRoll), sr = Math.sin(attMidRoll);
        const cp = Math.cos(attMidPitch), sp = Math.sin(attMidPitch);
        const cy = Math.cos(yawMid), sy = Math.sin(yawMid);
        const bodyX = sp * cr, bodyZ = -sr;
        const dirX = cy * bodyX + sy * bodyZ;
        const dirY = cp * cr;
        const dirZ = -sy * bodyX + cy * bodyZ;

        const airborneNow = p.y > 0.085 && prevPos.y > 0.085;
        const massEst = clamp(15.3 / kt, 0.6, 0.95);
        const dragCoeff = 0.42 / massEst;

        if (airborneNow) {
          const measAy = (v.y - prevVel.y) / dt;
          const vMidY = 0.5 * (prevVel.y + v.y);
          const uY = laggedThrust * dirY;
          if (uY > 0.22) {
            const predAy = kt * uY - dragCoeff * vMidY - g;
            const ktGain = stepCount < 60 ? 0.12 : 0.05;
            kt = clamp(kt + ktGain * ((measAy - predAy) / uY), 14.5, 27.5);
          }

          const measAx = (v.x - prevVel.x) / dt;
          const measAz = (v.z - prevVel.z) / dt;
          const vMidX = 0.5 * (prevVel.x + v.x);
          const vMidZ = 0.5 * (prevVel.z + v.z);

          const wAx = clamp(measAx - kt * laggedThrust * dirX + dragCoeff * vMidX, -14, 14);
          const wAz = clamp(measAz - kt * laggedThrust * dirZ + dragCoeff * vMidZ, -14, 14);

          windFast.x += 0.24 * (wAx - windFast.x);
          windFast.z += 0.24 * (wAz - windFast.z);
          windSlow.x += 0.04 * (wAx - windSlow.x);
          windSlow.z += 0.04 * (wAz - windSlow.z);
        }
      }

      prevPos = { x: p.x, y: p.y, z: p.z };
      prevVel = { x: v.x, y: v.y, z: v.z };
      prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };
      prevRates = { roll: rates.roll, pitch: rates.pitch, yaw: rates.yaw };
      stepCount++;

      const massEst = clamp(15.3 / kt, 0.6, 0.95);
      const dragCoeff = 0.42 / massEst;

      // Gust detection & wind compensation
      const gustDeltaX = windFast.x - windSlow.x;
      const gustDeltaZ = windFast.z - windSlow.z;
      const gustMag = Math.hypot(gustDeltaX, gustDeltaZ);
      const windCompX = windFast.x + 0.50 * gustDeltaX;
      const windCompZ = windFast.z + 0.50 * gustDeltaZ;

      let axDes = 0, ayDes = 0, azDes = 0;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetGate !== obs.gateIndex) {
          targetGate = obs.gateIndex;
          phase = 'align';
          centeredCount = 0;
          zTrim = 0;
          yTrim = 0;
        }

        const dxGate = gate.x - p.x;
        const dyGate = gate.y - p.y;
        const dzGate = gate.z - p.z;

        const zPred = p.z + 0.08 * velFilt.z;
        const yPred = p.y + 0.08 * velFilt.y;
        const zErrPred = Math.abs(gate.z - zPred);
        const yErrPred = Math.abs(gate.y - yPred);

        const isCentered =
          dxGate > 0.03 &&
          dxGate < 0.40 &&
          Math.abs(dzGate) < 0.015 &&
          zErrPred < 0.016 &&
          Math.abs(velFilt.z) < 0.11 &&
          Math.abs(v.z) < 0.12 &&
          Math.abs(dyGate) < 0.065 &&
          yErrPred < 0.075 &&
          Math.abs(velFilt.y) < 0.24 &&
          gustMag < 1.35;

        if (phase === 'align') {
          if (isCentered) centeredCount++;
          else centeredCount = Math.max(0, centeredCount - 1);
          if (centeredCount >= 4) {
            phase = 'lunge';
          }
        } else if (phase === 'lunge') {
          if (dxGate > 0.11 && velFilt.x < 1.10 && (Math.abs(dzGate) > 0.022 || zErrPred > 0.023 || Math.abs(dyGate) > 0.095)) {
            phase = 'align';
            centeredCount = 0;
          }
        }

        // Integral trim only when very close to centerline, decay when blown away
        if (Math.abs(dxGate) < 0.45 && Math.abs(dzGate) < 0.06) {
          zTrim = clamp(zTrim + dzGate * dt * 2.0, -0.55, 0.55);
        } else {
          zTrim *= 0.98;
        }
        if (Math.abs(dxGate) < 0.45 && Math.abs(dyGate) < 0.12) {
          yTrim = clamp(yTrim + dyGate * dt * 1.6, -0.6, 0.6);
        }

        let targetX;
        let vxRef = 0;
        const unalignedZY = Math.max(Math.abs(dzGate), zErrPred) > 0.022 || Math.abs(velFilt.z) > 0.15 || Math.abs(dyGate) > 0.09;
        if (phase === 'align') {
          if (obs.gateIndex === 2) {
            const alignDist = unalignedZY ? 0.38 : 0.25;
            targetX = gate.x - alignDist;
            const ex = clamp(targetX - p.x, -0.65, 0.58);
            axDes = 7.5 * ex - 5.7 * velFilt.x;
            if (gustMag > 1.1) {
              axDes -= 2.0 * velFilt.x;
            }
            if (dxGate < 0.26 && unalignedZY) {
              axDes = Math.min(axDes, -8.5 * (0.26 - dxGate) - 6.8 * Math.max(0, velFilt.x));
            }
          } else {
            const alignDist = unalignedZY ? 0.36 : 0.25;
            targetX = gate.x - alignDist;
            const ex = clamp(targetX - p.x, -0.65, 0.62);
            axDes = 7.5 * ex - 5.5 * velFilt.x;
            if (gustMag > 1.1) {
              axDes -= 2.0 * velFilt.x;
            }
            if (dxGate < 0.24 && unalignedZY) {
              axDes = Math.min(axDes, -8.0 * (0.24 - dxGate) - 6.5 * Math.max(0, velFilt.x));
            }
          }
        } else {
          targetX = gate.x + 0.35;
          vxRef = 1.38;
          axDes = 9.2 * (targetX - p.x) + 4.6 * (vxRef - velFilt.x);
        }

        const nearGate = Math.abs(dxGate) < 0.55;
        const kpZ = (phase === 'lunge') ? 17.5 : (nearGate ? 14.5 : 8.8);
        const kdZ = (phase === 'lunge') ? 8.2 : (nearGate ? 7.8 : 5.5);
        azDes = kpZ * dzGate - kdZ * velFilt.z + zTrim;

        const kpY = nearGate ? 12.0 : 8.2;
        const kdY = nearGate ? 6.0 : 4.8;
        ayDes = kpY * dyGate - kdY * velFilt.y + yTrim;
        if (obs.gateIndex === 2) {
          ayDes = clamp(ayDes, -2.6, 4.5);
        }
      } else {
        // After all 3 gates: fly to landing pad and pin pad with low thrust
        const pad = obs.pad.centre;
        const dxPad = pad.x - p.x;
        const dzPad = pad.z - p.z;
        const distH = Math.hypot(dxPad, dzPad);
        const speedH = Math.hypot(velFilt.x, velFilt.z);

        if (phase !== 'padApproach' && phase !== 'padDescend' && phase !== 'padPin') {
          phase = 'padApproach';
          zTrim = 0;
        }

        if (distH < 0.138 && p.y <= 0.093 && speedH < 0.44 && Math.abs(v.y) < 0.45) {
          padPinned = true;
        }
        if (distH > 0.147 || p.y > 0.095) {
          padPinned = false;
        }

        if (padPinned) {
          const axHold = clamp(5.0 * dxPad - 3.5 * velFilt.x - 0.5 * windCompX, -0.8, 0.8);
          const azHold = clamp(5.0 * dzPad - 3.5 * velFilt.z - 0.5 * windCompZ, -0.8, 0.8);
          const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
          const pitchDes = clamp((cy * axHold - sy * azHold) / g, -0.07, 0.07);
          const rollDes = clamp(-(sy * axHold + cy * azHold) / g, -0.07, 0.07);

          const pinThrust = 0.08;
          const biasScale = pinThrust / 0.52;
          const alphaRoll = 80 * (rollDes - att.roll) - 16 * ratesFilt.roll;
          const alphaPitch = 80 * (pitchDes - att.pitch) - 16 * ratesFilt.pitch;
          const cmdRoll = clamp(att.roll + (alphaRoll + 13 * rates.roll - 22 * simIntegral.roll - torqueBias.roll * biasScale) / 65, -0.12, 0.12);
          const cmdPitch = clamp(att.pitch + (alphaPitch + 13 * rates.pitch - 22 * simIntegral.pitch - torqueBias.pitch * biasScale) / 65, -0.12, 0.12);
          const cmdYawRate = clamp(-2.5 * att.yaw - (torqueBias.yaw * biasScale) / 8, -2, 2);

          lastAction = { thrust: pinThrust, roll: cmdRoll, pitch: cmdPitch, yawRate: cmdYawRate };
          return lastAction;
        }

        let targetY;
        if (distH > 0.24) {
          targetY = Math.max(0.18, Math.min(0.68, 0.10 + 0.58 * distH));
        } else if (distH > 0.132 || speedH > 0.36) {
          targetY = 0.10;
        } else {
          targetY = -0.02;
        }

        axDes = 8.5 * dxPad - 5.6 * velFilt.x;
        azDes = 8.5 * dzPad - 5.6 * velFilt.z;
        ayDes = 10.5 * (targetY - p.y) - 6.2 * velFilt.y;

        if (p.y < 0.25 && velFilt.y < -0.32) {
          ayDes = Math.max(ayDes, -8.0 * (velFilt.y + 0.24));
        }
      }

      // Wall proximity guard: damp speed and boost authority near room boundaries
      const nearWallZ = Math.abs(p.z) > 2.30 && p.z * v.z > 0;
      const nearWallX = Math.abs(p.x) > 2.40 && p.x * v.x > 0;
      if (nearWallZ || nearWallX) {
        if (p.y < 2.2) ayDes = Math.max(ayDes, 2.2);
      }

      axDes = clamp(axDes, -7.5, 7.5);
      azDes = clamp(azDes, -7.5, 7.5);
      ayDes = clamp(ayDes, -4.2, 6.5);

      // Total needed thrust acceleration vector
      const Ax = axDes + dragCoeff * velFilt.x - windCompX;
      const Az = azDes + dragCoeff * velFilt.z - windCompZ;
      const Ay = Math.max(3.2, g + ayDes + dragCoeff * velFilt.y);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const Bx = cy * Ax - sy * Az;
      const Bz = sy * Ax + cy * Az;

      const maxTilt = (obs.gateIndex === 3 && p.y < 0.20) ? 0.20 : ((nearWallZ || nearWallX) ? 0.59 : 0.56);
      const pitchDes = clamp(Math.atan2(Bx, Ay), -maxTilt, maxTilt);
      const rollDes = clamp(-Math.atan2(Bz, Math.hypot(Bx, Ay)), -maxTilt, maxTilt);

      const rawRollRateDes = stepCount > 1 ? clamp((rollDes - prevRollDes) / dt, -3.0, 3.0) : 0;
      const rawPitchRateDes = stepCount > 1 ? clamp((pitchDes - prevPitchDes) / dt, -3.0, 3.0) : 0;
      rollRateDesFilt += 0.45 * (rawRollRateDes - rollRateDesFilt);
      pitchRateDesFilt += 0.45 * (rawPitchRateDes - pitchRateDesFilt);
      prevRollDes = rollDes;
      prevPitchDes = pitchDes;

      // Invert inner attitude loop with rate feedforward
      const kpAtt = 92;
      const kdAtt = 17.5;
      const alphaRollDes = kpAtt * (rollDes - att.roll) + 7.5 * rollRateDesFilt - kdAtt * ratesFilt.roll;
      const alphaPitchDes = kpAtt * (pitchDes - att.pitch) + 7.5 * pitchRateDesFilt - kdAtt * ratesFilt.pitch;

      const cmdRoll = clamp(
        att.roll + (alphaRollDes + 13 * rates.roll - 22 * simIntegral.roll - torqueBias.roll) / 65,
        -0.6,
        0.6
      );
      const cmdPitch = clamp(
        att.pitch + (alphaPitchDes + 13 * rates.pitch - 22 * simIntegral.pitch - torqueBias.pitch) / 65,
        -0.6,
        0.6
      );
      const cmdYawRate = clamp(-3.0 * att.yaw - 0.3 * ratesFilt.yaw - torqueBias.yaw / 8, -2, 2);

      const tiltCos = Math.max(0.5, Math.cos(att.roll) * Math.cos(att.pitch));
      const cmdThrust = clamp(Ay / (kt * tiltCos), 0, 1);

      lastAction = { thrust: cmdThrust, roll: cmdRoll, pitch: cmdPitch, yawRate: cmdYawRate };
      return lastAction;
    },
  };
}
