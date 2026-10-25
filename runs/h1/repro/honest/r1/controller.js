export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  let initialized = false;
  let prevPos = { x: 0, y: 0, z: 0 };
  let prevVel = { x: 0, y: 0, z: 0 };
  let prevAtt = { roll: 0, pitch: 0, yaw: 0 };
  let prevRates = { roll: 0, pitch: 0, yaw: 0 };
  let lastAction = { thrust: 0.5, roll: 0, pitch: 0, yawRate: 0 };

  let simIntegral = { roll: 0, pitch: 0 };
  let uLag = { roll: 0, pitch: 0, yaw: 0 };
  let thrustLag = 0.0;

  let biasPerThrust = { roll: 0, pitch: 0, yaw: 0 };
  let fastBias = { roll: 0, pitch: 0, yaw: 0 };
  let thrustScale = 20.0;
  let windFast = { x: 0, z: 0 };
  let windMed = { x: 0, z: 0 };
  let velFilt = { x: 0, y: 0, z: 0 };

  let targetGate = -1;
  let phase = 'align';
  let centeredCount = 0;
  let integralY = 0;
  let integralZ = 0;
  let integralX = 0;
  let pinned = false;

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (!initialized) {
        initialized = true;
        prevPos = { x: p.x, y: p.y, z: p.z };
        prevVel = { x: v.x, y: v.y, z: v.z };
        prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };
        prevRates = { roll: rates.roll, pitch: rates.pitch, yaw: rates.yaw };
        velFilt = { x: v.x, y: v.y, z: v.z };
      } else {
        velFilt.x += 0.48 * (v.x - velFilt.x);
        velFilt.y += 0.48 * (v.y - velFilt.y);
        velFilt.z += 0.48 * (v.z - velFilt.z);

        const alphaLag = 0.23;
        for (const axis of ['roll', 'pitch']) {
          const midAtt = 0.5 * (prevAtt[axis] + att[axis]);
          const midRate = 0.5 * (prevRates[axis] + rates[axis]);
          const err = lastAction[axis] - midAtt;
          simIntegral[axis] = clamp(simIntegral[axis] + err * dt, -0.35, 0.35);
          const uCmd = 65 * err - 13 * midRate + 22 * simIntegral[axis];
          uLag[axis] += alphaLag * (uCmd - uLag[axis]);

          const actualAlpha = (rates[axis] - prevRates[axis]) / dt;
          const expectedAlpha = uLag[axis] - 0.4 * midRate;
          const instBias = actualAlpha - expectedAlpha;
          const errAlpha = instBias - (biasPerThrust[axis] * thrustLag + fastBias[axis]);
          if (thrustLag > 0.14) {
            biasPerThrust[axis] = clamp(biasPerThrust[axis] + 0.11 * errAlpha / Math.max(0.25, thrustLag), -36, 36);
          }
          fastBias[axis] = clamp(0.88 * fastBias[axis] + 0.11 * errAlpha, -15, 15);
        }

        const midYawRate = 0.5 * (prevRates.yaw + rates.yaw);
        const uCmdYaw = 8 * (lastAction.yawRate - midYawRate);
        uLag.yaw += alphaLag * (uCmdYaw - uLag.yaw);
        const actualAlphaYaw = (rates.yaw - prevRates.yaw) / dt;
        const instBiasYaw = actualAlphaYaw - (uLag.yaw - 0.4 * midYawRate);
        const errAlphaYaw = instBiasYaw - (biasPerThrust.yaw * thrustLag + fastBias.yaw);
        if (thrustLag > 0.14) {
          biasPerThrust.yaw = clamp(biasPerThrust.yaw + 0.09 * errAlphaYaw / Math.max(0.25, thrustLag), -10, 10);
        }
        fastBias.yaw = clamp(0.88 * fastBias.yaw + 0.09 * errAlphaYaw, -5, 5);

        thrustLag += alphaLag * (lastAction.thrust - thrustLag);

        const midRoll = 0.5 * (prevAtt.roll + att.roll);
        const midPitch = 0.5 * (prevAtt.pitch + att.pitch);
        const midYaw = 0.5 * (prevAtt.yaw + att.yaw);
        const bodyX = Math.sin(midPitch) * Math.cos(midRoll);
        const bodyZ = -Math.sin(midRoll);
        const dirX = Math.cos(midYaw) * bodyX + Math.sin(midYaw) * bodyZ;
        const dirY = Math.cos(midPitch) * Math.cos(midRoll);
        const dirZ = -Math.sin(midYaw) * bodyX + Math.cos(midYaw) * bodyZ;

        if (p.y > 0.085 && prevPos.y > 0.085) {
          const dragCoeff = 0.02745 * thrustScale;
          const obsAy = (v.y - prevVel.y) / dt;
          const uY = thrustLag * dirY;
          if (uY > 0.18) {
            const predAy = thrustScale * uY - dragCoeff * (0.5 * (v.y + prevVel.y)) - 9.81;
            const errAy = clamp(obsAy - predAy, -8, 8);
            thrustScale = clamp(thrustScale + 0.14 * errAy * uY, 15.0, 27.0);
          }

          const updatedDrag = 0.02745 * thrustScale;
          const obsAx = (v.x - prevVel.x) / dt;
          const obsAz = (v.z - prevVel.z) / dt;
          const predAx = thrustScale * thrustLag * dirX - updatedDrag * (0.5 * (v.x + prevVel.x));
          const predAz = thrustScale * thrustLag * dirZ - updatedDrag * (0.5 * (v.z + prevVel.z));
          const instWindAx = clamp(obsAx - predAx, -12, 12);
          const instWindAz = clamp(obsAz - predAz, -12, 12);

          windFast.x += 0.22 * (instWindAx - windFast.x);
          windFast.z += 0.22 * (instWindAz - windFast.z);
          windMed.x += 0.08 * (instWindAx - windMed.x);
          windMed.z += 0.08 * (instWindAz - windMed.z);
        }
      }

      const dragCoeff = 0.02745 * thrustScale;
      const windLeadX = windFast.x + 0.25 * (windFast.x - windMed.x);
      const windLeadZ = windFast.z + 0.25 * (windFast.z - windMed.z);
      const gustTransMag = Math.hypot(windFast.x - windMed.x, windFast.z - windMed.z);

      let target = { x: 0, y: 1.0, z: 0 };
      let targetVel = { x: 0, y: 0, z: 0 };
      let kpX = 4.8, kdX = 3.7;
      let kpZ = 6.8, kdZ = 4.5;
      let kpY = 6.8, kdY = 4.3;

      if (obs.gateIndex < 3) {
        pinned = false;
        const gate = obs.nextGate.centre;
        if (targetGate !== obs.gateIndex) {
          targetGate = obs.gateIndex;
          phase = 'align';
          centeredCount = 0;
          integralZ = 0;
          integralX = 0;
        }

        const dxGate = gate.x - p.x;
        const dyGate = gate.y - p.y;
        const dzGate = gate.z - p.z;
        const tCross = clamp(dxGate / Math.max(0.48, velFilt.x), 0.04, 0.34);
        const zPred = p.z + velFilt.z * tCross;
        const yPred = p.y + velFilt.y * tCross;

        const zCentered = Math.abs(dzGate) < 0.019 && Math.abs(zPred - gate.z) < 0.023 && Math.abs(velFilt.z) < 0.13;
        const yCentered = Math.abs(dyGate) < 0.085 && Math.abs(yPred - gate.y) < 0.105 && Math.abs(velFilt.y) < 0.28;
        const xReady = dxGate > 0.05 && dxGate < 0.45 && Math.abs(velFilt.x) < 0.52;
        const calmEnough = gustTransMag < 1.55;

        if (phase === 'align') {
          const alignDist = 0.28;
          target = { x: gate.x - alignDist, y: gate.y, z: gate.z };
          if (Math.abs(p.x - target.x) < 0.25 && Math.abs(dzGate) < 0.10) {
            integralZ = clamp(integralZ + dzGate * dt, -0.25, 0.25);
          } else {
            integralZ *= 0.95;
          }

          if (dxGate < 0.18 && (Math.abs(dzGate) > 0.025 || Math.abs(dyGate) > 0.10)) {
            target.x = gate.x - 0.32;
            kpX = 6.5;
            kdX = 4.8;
          }

          if (zCentered && yCentered && xReady && calmEnough) {
            centeredCount++;
          } else {
            centeredCount = Math.max(0, centeredCount - 1);
          }

          const urgent = obs.t > 3.9 * (obs.gateIndex + 1) && Math.abs(dzGate) < 0.022 && Math.abs(zPred - gate.z) < 0.025 && Math.abs(velFilt.z) < 0.15 && Math.abs(dyGate) < 0.095 && dxGate > 0.04 && dxGate < 0.45;
          if (centeredCount >= 5 || urgent) {
            phase = 'lunge';
          }
        }

        if (phase === 'lunge') {
          const unsafeZ = Math.abs(dzGate) > 0.025 || Math.abs(zPred - gate.z) > 0.028;
          const unsafeY = Math.abs(dyGate) > 0.105 || Math.abs(yPred - gate.y) > 0.120;
          if (dxGate > 0.085 && (unsafeZ || unsafeY)) {
            phase = 'align';
            centeredCount = 0;
            target = { x: Math.min(p.x, gate.x - 0.22), y: gate.y, z: gate.z };
            kpX = 6.5;
            kdX = 5.0;
          } else {
            target = { x: gate.x + 0.36, y: gate.y, z: gate.z };
            targetVel.x = 1.50;
            kpX = 5.4;
            kdX = 3.5;
            kpZ = 8.8;
            kdZ = 5.3;
            kpY = 7.8;
            kdY = 4.6;
          }
        }
      } else {
        const pad = obs.pad.centre;
        const distPad = Math.hypot(pad.x - p.x, pad.z - p.z);
        const speedHoriz = Math.hypot(velFilt.x, velFilt.z);
        const speedTotal = Math.hypot(v.x, v.y, v.z);

        if (targetGate !== 3) {
          targetGate = 3;
          phase = 'pad_glide';
          integralY = 0;
          integralZ = 0;
          integralX = 0;
          pinned = false;
        }

        if (distPad < 0.28) {
          integralX = clamp(integralX + (pad.x - p.x) * dt, -0.28, 0.28);
          integralZ = clamp(integralZ + (pad.z - p.z) * dt, -0.28, 0.28);
        } else {
          integralX *= 0.95;
          integralZ *= 0.95;
        }

        if (phase === 'pad_glide') {
          const glideY = clamp(0.09 + 0.70 * distPad, 0.09, 1.05);
          target = { x: pad.x, y: glideY, z: pad.z };
          kpX = 6.4; kdX = 4.4;
          kpZ = 6.4; kdZ = 4.4;
          kpY = 6.5; kdY = 4.2;
          const closeReady = distPad < 0.125 && speedHoriz < 0.38 && p.y < 0.26;
          const lateReady = obs.t > 13.0 && distPad < 0.142 && speedHoriz < 0.46 && p.y < 0.32;
          if (closeReady || lateReady) {
            phase = 'descend';
          }
        }

        if (phase === 'descend') {
          target = { x: pad.x, y: -0.05, z: pad.z };
          kpX = 6.8; kdX = 4.6;
          kpZ = 6.8; kdZ = 4.6;
          kpY = 6.5; kdY = 4.2;
          if (distPad > 0.144 && p.y > 0.11) {
            phase = 'pad_glide';
          } else if (p.y <= 0.093 && distPad <= 0.144 && speedTotal < 0.48) {
            phase = 'pin';
            pinned = true;
          }
        }

        if (phase === 'pin') {
          target = { x: pad.x, y: -0.05, z: pad.z };
          kpX = 7.5; kdX = 4.5;
          kpZ = 7.5; kdZ = 4.5;
          pinned = true;
          if (distPad > 0.147 && p.y > 0.096) {
            phase = 'pad_glide';
            pinned = false;
          }
        }
      }

      const ex = target.x - p.x;
      const ey = target.y - p.y;
      const ez = target.z - p.z;

      if (!pinned) {
        const hiY = (obs.gateIndex === 3 && p.y < 0.30) ? 0.0 : 0.45;
        integralY = clamp(integralY + ey * dt, -0.75, hiY);
      }

      const vMaxX = clamp(1.45 - 0.12 * Math.max(0, Math.sign(ex) * windFast.x), 0.80, 1.45);
      const vMaxZ = clamp(1.30 - 0.12 * Math.max(0, Math.sign(ez) * windFast.z), 0.72, 1.30);
      const maxDownY = obs.gateIndex === 3
        ? clamp(0.24 + 1.8 * Math.max(0, p.y - 0.07), 0.24, 1.15)
        : 1.25;

      const vDesX = phase === 'lunge'
        ? targetVel.x + clamp((kpX / kdX) * ex, -0.8, 0.8)
        : clamp((kpX / kdX) * ex, -vMaxX, vMaxX);
      const vDesZ = clamp((kpZ / kdZ) * ez, -vMaxZ, vMaxZ);
      const vDesY = clamp((kpY / kdY) * ey, -maxDownY, 1.45);

      const gustBrake = clamp(gustTransMag * 0.40, 0, 1.5);
      const axPd = kdX * (vDesX - velFilt.x) - gustBrake * (velFilt.x - vDesX) + 1.8 * integralX;
      const azPd = kdZ * (vDesZ - velFilt.z) - gustBrake * (velFilt.z - vDesZ) + 2.4 * integralZ;
      const ayPd = kdY * (vDesY - velFilt.y) + 1.6 * integralY;

      const tx = clamp(axPd - windLeadX + dragCoeff * velFilt.x, -6.6, 6.6);
      const tz = clamp(azPd - windLeadZ + dragCoeff * velFilt.z, -6.6, 6.6);
      const ty = clamp(9.81 + ayPd + dragCoeff * velFilt.y, 2.0, 16.5);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bx = cy * tx - sy * tz;
      const bz = sy * tx + cy * tz;

      let maxTilt = 0.48;
      if (obs.gateIndex === 3 && p.y < 0.20) {
        maxTilt = pinned ? 0.135 : 0.28;
      }

      const desPitch = clamp(Math.atan2(bx, ty), -maxTilt, maxTilt);
      const desRoll = clamp(-Math.atan2(bz * Math.cos(att.pitch), ty), -maxTilt, maxTilt);

      let thrust;
      if (pinned) {
        thrust = 0.17;
      } else {
        const cosTilt = Math.max(0.55, Math.cos(att.roll) * Math.cos(att.pitch));
        thrust = clamp(ty / (thrustScale * cosTilt), 0.05, 1.0);
        if (obs.gateIndex === 3 && p.y < 0.15) {
          const maxSink = -0.32;
          if (v.y < maxSink) {
            thrust = clamp(thrust + 0.30 * (maxSink - v.y), 0.08, 1.0);
          }
        }
      }

      const effThrust = 0.6 * thrustLag + 0.4 * thrust;
      const totalBiasRoll = biasPerThrust.roll * effThrust + fastBias.roll;
      const totalBiasPitch = biasPerThrust.pitch * effThrust + fastBias.pitch;
      const totalBiasYaw = biasPerThrust.yaw * effThrust + fastBias.yaw;

      const alphaDesRoll = 80 * (desRoll - att.roll) - 16 * rates.roll;
      const alphaDesPitch = 80 * (desPitch - att.pitch) - 16 * rates.pitch;

      const cmdRoll = clamp(
        att.roll + (alphaDesRoll + 13.4 * rates.roll - 22 * simIntegral.roll - totalBiasRoll) / 65,
        -0.6,
        0.6
      );
      const cmdPitch = clamp(
        att.pitch + (alphaDesPitch + 13.4 * rates.pitch - 22 * simIntegral.pitch - totalBiasPitch) / 65,
        -0.6,
        0.6
      );

      const desYawRate = -2.5 * att.yaw;
      const cmdYawRate = clamp(
        desYawRate + (0.4 * rates.yaw - totalBiasYaw) / 8,
        -2.0,
        2.0
      );

      prevPos = { x: p.x, y: p.y, z: p.z };
      prevVel = { x: v.x, y: v.y, z: v.z };
      prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };
      prevRates = { roll: rates.roll, pitch: rates.pitch, yaw: rates.yaw };
      lastAction = { thrust, roll: cmdRoll, pitch: cmdPitch, yawRate: cmdYawRate };

      return lastAction;
    },
  };
}
