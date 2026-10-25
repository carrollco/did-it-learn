export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  // Inner loop & motor state estimates
  let simIntRoll = 0, simIntPitch = 0;
  let alphaLagRoll = 0, alphaLagPitch = 0, alphaLagYaw = 0;
  let thrustLag = 0.45;
  let prevAction = null;
  let prevObs = null;

  // Observers: thrust scale, torque bias (per unit thrust), wind acceleration
  let thrustScale = 20.5;
  let biasScaleRoll = 0, biasScalePitch = 0, biasScaleYaw = 0;
  let windFastX = 0, windFastZ = 0;
  let windCleanX = 0, windCleanZ = 0;
  let windMedX = 0, windMedZ = 0;
  let desRollFilt = 0, desPitchFilt = 0;

  // Smoothed velocity
  let velFiltX = 0, velFiltY = 0, velFiltZ = 0;

  // Navigation state
  let targetGate = -1;
  let phase = 'align';
  let centeredCount = 0;

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (!prevObs) {
        velFiltX = v.x; velFiltY = v.y; velFiltZ = v.z;
      } else {
        velFiltX += 0.55 * (v.x - velFiltX);
        velFiltY += 0.55 * (v.y - velFiltY);
        velFiltZ += 0.55 * (v.z - velFiltZ);
      }

      // 1. Update observers from previous step
      if (prevObs && prevAction) {
        const tauM = 0.04;
        const alphaM = 1 - Math.exp(-dt / tauM);

        // Reconstruct simulator inner loop over 2 substeps
        const h = 0.005;
        const eR1 = prevAction.roll - prevObs.att.roll;
        simIntRoll = clamp(simIntRoll + eR1 * h, -0.35, 0.35);
        const cmdR1 = 65 * eR1 - 13 * prevObs.rates.roll + 22 * simIntRoll;

        const attRMid = prevObs.att.roll + prevObs.rates.roll * h;
        const rateRMid = 0.5 * (prevObs.rates.roll + rates.roll);
        const eR2 = prevAction.roll - attRMid;
        simIntRoll = clamp(simIntRoll + eR2 * h, -0.35, 0.35);
        const cmdR2 = 65 * eR2 - 13 * rateRMid + 22 * simIntRoll;
        const cmdRoll = 0.5 * (cmdR1 + cmdR2);

        const eP1 = prevAction.pitch - prevObs.att.pitch;
        simIntPitch = clamp(simIntPitch + eP1 * h, -0.35, 0.35);
        const cmdP1 = 65 * eP1 - 13 * prevObs.rates.pitch + 22 * simIntPitch;

        const attPMid = prevObs.att.pitch + prevObs.rates.pitch * h;
        const ratePMid = 0.5 * (prevObs.rates.pitch + rates.pitch);
        const eP2 = prevAction.pitch - attPMid;
        simIntPitch = clamp(simIntPitch + eP2 * h, -0.35, 0.35);
        const cmdP2 = 65 * eP2 - 13 * rateRMid + 22 * simIntPitch;
        const cmdPitch = 0.5 * (cmdP1 + cmdP2);

        const cmdYaw = 8 * (prevAction.yawRate - 0.5 * (prevObs.rates.yaw + rates.yaw));

        alphaLagRoll += (cmdRoll - alphaLagRoll) * alphaM;
        alphaLagPitch += (cmdPitch - alphaLagPitch) * alphaM;
        alphaLagYaw += (cmdYaw - alphaLagYaw) * alphaM;
        thrustLag += (prevAction.thrust - thrustLag) * alphaM;

        // Measured angular accelerations
        const measAlphaRoll = (rates.roll - prevObs.rates.roll) / dt + 0.4 * 0.5 * (rates.roll + prevObs.rates.roll);
        const measAlphaPitch = (rates.pitch - prevObs.rates.pitch) / dt + 0.4 * 0.5 * (rates.pitch + prevObs.rates.pitch);
        const measAlphaYaw = (rates.yaw - prevObs.rates.yaw) / dt + 0.4 * 0.5 * (rates.yaw + prevObs.rates.yaw);

        if (thrustLag > 0.18 && p.y > 0.08) {
          const gainB = obs.t < 0.6 ? 0.16 : 0.03;
          if (Math.abs(cmdRoll - alphaLagRoll) < 12 && Math.abs(prevAction.roll) < 0.55) {
            const rawBiasScaleR = clamp((measAlphaRoll - alphaLagRoll) / thrustLag, -30, 30);
            biasScaleRoll += gainB * (rawBiasScaleR - biasScaleRoll);
          }
          if (Math.abs(cmdPitch - alphaLagPitch) < 12 && Math.abs(prevAction.pitch) < 0.55) {
            const rawBiasScaleP = clamp((measAlphaPitch - alphaLagPitch) / thrustLag, -30, 30);
            biasScalePitch += gainB * (rawBiasScaleP - biasScalePitch);
          }
          const rawBiasScaleY = clamp((measAlphaYaw - alphaLagYaw) / thrustLag, -15, 15);
          biasScaleYaw += gainB * (rawBiasScaleY - biasScaleYaw);
        }

        // Mid-step attitude & thrust direction
        const midRoll = 0.5 * (prevObs.att.roll + att.roll);
        const midPitch = 0.5 * (prevObs.att.pitch + att.pitch);
        const midYaw = 0.5 * (prevObs.att.yaw + att.yaw);
        const bodyX = Math.sin(midPitch) * Math.cos(midRoll);
        const bodyZ = -Math.sin(midRoll);
        const dirX = Math.cos(midYaw) * bodyX + Math.sin(midYaw) * bodyZ;
        const dirY = Math.cos(midPitch) * Math.cos(midRoll);
        const dirZ = -Math.sin(midYaw) * bodyX + Math.cos(midYaw) * bodyZ;

        const estMass = clamp(15.0 / thrustScale, 0.6, 0.95);
        const cd = 0.42 / estMass;

        // Airborne vertical observer for thrustScale
        if (p.y > 0.09 && p.y < 2.85 && prevObs.pos.y > 0.09) {
          const accY = (v.y - prevObs.vel.y) / dt;
          const uEffY = thrustLag * dirY;
          if (uEffY > 0.22 && Math.abs(prevAction.thrust - thrustLag) < 0.12) {
            const measST = clamp((accY + 9.81 + cd * 0.5 * (v.y + prevObs.vel.y)) / uEffY, 14.5, 27.5);
            const gainT = obs.t < 0.6 ? 0.16 : 0.035;
            thrustScale += gainT * (measST - thrustScale);
          }
        }

        // Airborne horizontal wind observer
        if (p.y > 0.08 && prevObs.pos.y > 0.08 && Math.abs(p.x) < 2.81 && Math.abs(p.z) < 2.81) {
          const accX = (v.x - prevObs.vel.x) / dt;
          const accZ = (v.z - prevObs.vel.z) / dt;
          const thrustAccX = thrustScale * thrustLag * dirX;
          const thrustAccZ = thrustScale * thrustLag * dirZ;
          const rawWindAccX = clamp(accX - thrustAccX + cd * 0.5 * (v.x + prevObs.vel.x), -9.0, 9.0);
          const rawWindAccZ = clamp(accZ - thrustAccZ + cd * 0.5 * (v.z + prevObs.vel.z), -9.0, 9.0);

          windFastX += 0.30 * (rawWindAccX - windFastX);
          windFastZ += 0.30 * (rawWindAccZ - windFastZ);
          windCleanX += 0.50 * (windFastX - windCleanX);
          windCleanZ += 0.50 * (windFastZ - windCleanZ);
          windMedX += 0.06 * (rawWindAccX - windMedX);
          windMedZ += 0.06 * (rawWindAccZ - windMedZ);
        }
      }

      const estMass = clamp(15.0 / thrustScale, 0.6, 0.95);
      const cd = 0.42 / estMass;

      // Gust jump (onset or end of a gust)
      const gustDeltaX = windCleanX - windMedX;
      const gustDeltaZ = windCleanZ - windMedZ;

      // 2. Navigation state machine
      let tx = p.x, ty = p.y, tz = p.z;
      let vxDes = 0, vyDes = 0, vzDes = 0;
      let kpX = 4.5, kdX = 3.6;
      let kpZ = 6.5, kdZ = 4.2;
      let kpY = 6.5, kdY = 4.0;
      let pinPad = false;
      let maxAxPD = 6.0;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetGate !== obs.gateIndex) {
          targetGate = obs.gateIndex;
          phase = 'align';
          centeredCount = 0;
        }

        const dxGate = gate.x - p.x;
        const dyGate = gate.y - p.y;
        const dzGate = gate.z - p.z;
        const tCross = clamp(dxGate / Math.max(0.45, v.x), 0.04, 0.25);
        const predDy = Math.abs(dyGate - velFiltY * tCross * 0.65);
        const predDz = Math.abs(dzGate - velFiltZ * tCross * 0.65);

        if (phase === 'align') {
          const alignDist = 0.26;
          const yzErr = Math.hypot(dyGate, dzGate * 2.0);
          const holdBack = yzErr > 0.35 ? 0.38 : alignDist;
          tx = gate.x - holdBack;
          ty = gate.y;
          tz = gate.z;

          kpZ = 9.5; kdZ = 5.0;
          kpY = 7.5; kdY = 4.2;
          kpX = 5.0; kdX = 3.9;

          // Emergency brake if close to gate plane and moving forward while off-center or gusting
          if (dxGate < 0.24 && v.x > 0.06 && (Math.abs(dzGate) > 0.018 || predDz > 0.020 || Math.abs(dyGate) > 0.08 || Math.abs(gustDeltaZ) > 0.85)) {
            vxDes = -0.40;
            kpX = 8.5; kdX = 5.8;
            kpZ = 13.0; kdZ = 6.0;
          }

          const zCentered = Math.abs(dzGate) < 0.015 && Math.abs(velFiltZ) < 0.085 && predDz < 0.016;
          const yCentered = Math.abs(dyGate) < 0.055 && Math.abs(velFiltY) < 0.20;
          const xReady = dxGate > 0.04 && dxGate < 0.36 && v.x > -0.22;
          const gustSafe = Math.abs(gustDeltaZ) < 0.75;

          if (zCentered && yCentered && xReady && gustSafe) {
            centeredCount++;
          } else {
            centeredCount = Math.max(0, centeredCount - 1);
          }

          if (centeredCount >= 4) {
            phase = 'lunge';
          }
        }

        if (phase === 'lunge') {
          const gustShockZ = dxGate > 0.09 && Math.abs(gustDeltaZ) > 0.80;
          const offCenterZ = (dxGate > 0.035 && (Math.abs(dzGate) > 0.021 || predDz > 0.023 || Math.abs(velFiltZ) > 0.18 || gustShockZ));
          const offCenterY = (dxGate > 0.035 && (Math.abs(dyGate) > 0.090 || predDy > 0.100));
          if (offCenterZ || offCenterY) {
            phase = 'align';
            centeredCount = 0;
            tx = gate.x - 0.27;
            ty = gate.y;
            tz = gate.z;
            vxDes = -0.40;
            kpX = 8.5; kdX = 5.8;
            kpZ = 13.0; kdZ = 6.0;
          } else {
            tx = gate.x + 0.34;
            ty = gate.y;
            tz = gate.z;
            vxDes = 0.75;
            kpX = 5.0; kdX = 3.6;
            kpZ = 13.5; kdZ = 6.2;
            kpY = 8.0; kdY = 4.4;
            maxAxPD = 2.9;
          }
        }
      } else {
        const pad = obs.pad.centre;
        if (phase !== 'pad_approach' && phase !== 'pad_descend' && phase !== 'pad_pin') {
          phase = 'pad_approach';
        }
        const distPadXZ = Math.hypot(pad.x - p.x, pad.z - p.z);
        const speedXZ = Math.hypot(v.x, v.z);

        if (phase === 'pad_approach') {
          tx = pad.x;
          tz = pad.z;
          ty = clamp(0.18 + 0.82 * distPadXZ, 0.22, 0.92);
          kpX = 5.2; kdX = 4.0;
          kpZ = 5.2; kdZ = 4.0;
          kpY = 6.5; kdY = 4.2;
          if (distPadXZ < 0.105 && speedXZ < 0.28 && p.y < 0.35) {
            phase = 'pad_descend';
          }
        }

        if (phase === 'pad_descend') {
          tx = pad.x;
          tz = pad.z;
          ty = 0.05;
          vyDes = -0.20;
          kpX = 6.5; kdX = 4.5;
          kpZ = 6.5; kdZ = 4.5;
          kpY = 6.2; kdY = 4.5;
          if (distPadXZ > 0.125) {
            phase = 'pad_approach';
          } else if (p.y <= 0.094 && distPadXZ <= 0.12 && Math.abs(v.y) < 0.36) {
            phase = 'pad_pin';
          }
        }

        if (phase === 'pad_pin') {
          tx = pad.x;
          tz = pad.z;
          ty = 0.0;
          pinPad = true;
          kpX = 12.0; kdX = 5.0;
          kpZ = 12.0; kdZ = 5.0;
          if (distPadXZ > 0.18) {
            phase = 'pad_approach';
            pinPad = false;
          }
        }
      }

      // 3. Position / Velocity Outer Loop with Wind & Gust Compensation
      const ex = clamp(tx - p.x, -0.85, 0.85);
      const ez = clamp(tz - p.z, -0.85, 0.85);
      const ey = clamp(ty - p.y, -0.85, 0.85);

      const gustBrakeX = 0.38 * gustDeltaX;
      const gustBrakeZ = 0.38 * gustDeltaZ;

      const axPD = Math.min(maxAxPD, kpX * ex + kdX * (vxDes - velFiltX));
      const azPD = kpZ * ez + kdZ * (vzDes - velFiltZ);
      const ayPD = kpY * ey + kdY * (vyDes - velFiltY);

      const minAx = p.x > 2.15 ? -7.2 : -6.5;
      const Ax = clamp(axPD + cd * velFiltX - windCleanX - gustBrakeX, minAx, 6.0);
      const Az = clamp(azPD + cd * velFiltZ - windCleanZ - gustBrakeZ, -7.2, 7.2);
      const Ay = pinPad ? 6.0 : clamp(9.81 + ayPD + cd * velFiltY, 2.5, 16.5);

      // 4. Nonlinear mapping to desired roll, pitch, and thrust
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const AbodyX = cy * Ax - sy * Az;
      const AbodyZ = sy * Ax + cy * Az;

      let maxPitch = p.x > 2.15 ? 0.56 : 0.48;
      let maxRoll = 0.56;
      if (obs.gateIndex === 3 && p.y < 0.16) {
        maxPitch = pinPad ? 0.155 : 0.22;
        maxRoll = pinPad ? 0.155 : 0.22;
      }
      const desPitch = clamp(Math.atan2(AbodyX, Ay), -maxPitch, maxPitch);
      const desRoll = clamp(-Math.atan2(AbodyZ, Math.hypot(AbodyX, Ay)), -maxRoll, maxRoll);

      const dirY = Math.max(0.52, Math.cos(att.roll) * Math.cos(att.pitch));
      const thrust = clamp(Ay / (thrustScale * dirY), 0.05, 0.95);

      const prevRollF = desRollFilt, prevPitchF = desPitchFilt;
      desRollFilt += 0.45 * (desRoll - desRollFilt);
      desPitchFilt += 0.45 * (desPitch - desPitchFilt);
      const desRollRate = clamp((desRollFilt - prevRollF) / dt, -3.0, 3.0);
      const desPitchRate = clamp((desPitchFilt - prevPitchF) / dt, -3.0, 3.0);

      // 5. Invert inner attitude loop with torque bias compensation
      const kpAtt = 72;
      const kdAtt = 14.5;
      const estBiasRoll = biasScaleRoll * thrust;
      const estBiasPitch = biasScalePitch * thrust;
      const estBiasYaw = biasScaleYaw * thrust;

      const alphaDesRoll = kpAtt * (desRoll - att.roll) + kdAtt * (0.4 * desRollRate - rates.roll) - estBiasRoll;
      const alphaDesPitch = kpAtt * (desPitch - att.pitch) + kdAtt * (0.4 * desPitchRate - rates.pitch) - estBiasPitch;

      const cmdRoll = clamp(att.roll + (alphaDesRoll + 13 * rates.roll - 22 * simIntRoll) / 65.11, -0.6, 0.6);
      const cmdPitch = clamp(att.pitch + (alphaDesPitch + 13 * rates.pitch - 22 * simIntPitch) / 65.11, -0.6, 0.6);
      const cmdYawRate = clamp(-3.2 * att.yaw - 0.25 * rates.yaw - estBiasYaw / 8, -2, 2);

      const action = {
        thrust,
        roll: cmdRoll,
        pitch: cmdPitch,
        yawRate: cmdYawRate,
      };

      prevObs = obs;
      prevAction = action;
      return action;
    },
  };
}
