export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  let intRoll = 0, intPitch = 0;
  let biasRoll = 0, biasPitch = 0;
  let prevRates = null, prevVel = null, prevAtt = null;
  let kT = 18.8;
  let windAx = 0, windAz = 0;
  let slowWindAx = 0, slowWindAz = 0;
  let prevCmdRoll = 0, prevCmdPitch = 0, prevThrust = 0.5;
  let effThrust = 0.5;
  let effRollTorque = 0, effPitchTorque = 0;
  let targetIndex = -1, phase = 'align', settle = 0, alignSteps = 0;

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      const lagAlpha = 1 - Math.exp(-dt / 0.038);
      effThrust += (prevThrust - effThrust) * lagAlpha;

      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll), bodyZ = -Math.sin(att.roll);
      const dirX = Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ;
      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
      const dirZ = -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ;

      if (prevRates && prevVel && obs.t > 0.03 && p.y > 0.11) {
        const desTorqueRoll = 65 * (prevCmdRoll - prevAtt.roll) - 13 * prevRates.roll + 22 * intRoll;
        const desTorquePitch = 65 * (prevCmdPitch - prevAtt.pitch) - 13 * prevRates.pitch + 22 * intPitch;
        effRollTorque += (desTorqueRoll - effRollTorque) * lagAlpha;
        effPitchTorque += (desTorquePitch - effPitchTorque) * lagAlpha;

        const actualAlphaRoll = (rates.roll - prevRates.roll) / dt + 0.4 * rates.roll;
        const actualAlphaPitch = (rates.pitch - prevRates.pitch) / dt + 0.4 * rates.pitch;

        if (Math.abs(rates.roll) < 3.0 && Math.abs(prevCmdRoll) < 0.58) {
          const errRoll = (actualAlphaRoll - effRollTorque) / 65;
          biasRoll = clamp(biasRoll + 0.06 * (errRoll - biasRoll), -0.25, 0.25);
        }
        if (Math.abs(rates.pitch) < 3.0 && Math.abs(prevCmdPitch) < 0.58) {
          const errPitch = (actualAlphaPitch - effPitchTorque) / 65;
          biasPitch = clamp(biasPitch + 0.06 * (errPitch - biasPitch), -0.25, 0.25);
        }

        const accX = (v.x - prevVel.x) / dt;
        const accY = (v.y - prevVel.y) / dt;
        const accZ = (v.z - prevVel.z) / dt;

        if (effThrust * dirY > 0.28 && prevThrust < 0.95 && p.y > 0.14 && p.y < 2.65) {
          const dragCoeff = 0.42 / (15.2 / kT);
          const estKT = (accY + 9.81 + dragCoeff * v.y) / (effThrust * dirY);
          if (estKT > 11 && estKT < 28) {
            kT += 0.06 * (estKT - kT);
          }
        }

        const dragCoeff = clamp(0.42 / (15.2 / kT), 0.44, 0.70);
        const rawWAx = clamp(accX - kT * effThrust * dirX + dragCoeff * v.x, -9.5, 9.5);
        const rawWAz = clamp(accZ - kT * effThrust * dirZ + dragCoeff * v.z, -9.5, 9.5);

        windAx += 0.30 * (rawWAx - windAx);
        windAz += 0.30 * (rawWAz - windAz);
        slowWindAx += 0.04 * (rawWAx - slowWindAx);
        slowWindAz += 0.04 * (rawWAz - slowWindAz);
      }

      prevRates = { roll: rates.roll, pitch: rates.pitch, yaw: rates.yaw };
      prevVel = { x: v.x, y: v.y, z: v.z };
      prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };

      const gustMag = Math.hypot(windAx - slowWindAx, windAz - slowWindAz);
      const isGust = gustMag > 0.85;

      let targetX, targetY, targetZ;
      let targetVx = 0, targetVy = 0, targetVz = 0;
      let kpX = 5.5, kdX = 4.2, kpZ = 6.5, kdZ = 4.5, kpY = 8.5, kdY = 5.0;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          targetIndex = obs.gateIndex;
          phase = 'align';
          settle = 0;
          alignSteps = 0;
        }
        alignSteps++;
        const dx = p.x - gate.x;
        const dy = p.y - gate.y;
        const dz = p.z - gate.z;

        const tCross = v.x > 0.25 && dx < 0 ? clamp(-dx / v.x, 0.04, 0.25) : 0.14;
        const predDy = dy + v.y * tCross;
        const predDz = dz + v.z * tCross;
        const curAccZ = kT * effThrust * dirZ + windAz - 0.55 * v.z;

        const relax = alignSteps > 220 ? 1.15 : 1.0;
        const zCentered = Math.abs(dz) < 0.021 * relax && Math.abs(predDz) < 0.023 * relax && Math.abs(v.z) < 0.15 * relax && Math.abs(curAccZ) < 1.3 * relax;
        const yCentered = Math.abs(dy) < 0.065 * relax && Math.abs(predDy) < 0.070 * relax && Math.abs(v.y) < 0.30 * relax;

        if (phase === 'align') {
          if (dx > -0.45 && dx < -0.04 && zCentered && yCentered && (!isGust || alignSteps > 250)) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= 6) {
            phase = 'lunge';
          }
        } else if (phase === 'lunge') {
          const unsafeZ = Math.abs(dz) > 0.028 || Math.abs(predDz) > 0.029 || Math.abs(v.z) > 0.23;
          const unsafeY = Math.abs(dy) > 0.105 || Math.abs(predDy) > 0.110;
          if (dx < -0.04 && (unsafeZ || unsafeY || (dx < -0.10 && isGust && Math.abs(predDz) > 0.019))) {
            phase = 'align';
            settle = 0;
          }
        }

        targetY = gate.y;
        targetZ = gate.z;

        if (phase === 'align') {
          if (dx >= -0.08) {
            targetX = gate.x - 0.38;
            if (Math.abs(dz) > 0.03) {
              targetZ = gate.z + (dz > 0 ? 0.42 : -0.42);
            }
          } else {
            const distYZ = Math.hypot(dy * 0.5, dz * 1.5);
            const standOff = distYZ > 0.12 ? 0.38 : 0.26;
            targetX = gate.x - standOff;
            const predX = dx + v.x * 0.18;
            if ((dx > -0.25 || predX > -0.10) && (!zCentered || !yCentered || isGust) && v.x > 0.05) {
              targetX = gate.x - 0.40;
              targetVx = -0.35;
              kpX = 8.5;
              kdX = 6.5;
            }
          }
          kpZ = 8.5;
          kdZ = 5.2;
        } else {
          targetX = gate.x + 0.35;
          targetVx = 1.35;
          kpX = 6.5;
          kdX = 4.2;
          kpZ = 9.5;
          kdZ = 5.5;
        }
      } else {
        const pad = obs.pad.centre;
        if (targetIndex !== 3) {
          targetIndex = 3;
          phase = 'approach';
        }
        const horizErr = Math.hypot(p.x - pad.x, p.z - pad.z);
        const horizSpeed = Math.hypot(v.x, v.z);
        targetX = pad.x;
        targetZ = pad.z;
        kpX = 6.2; kdX = 4.6;
        kpZ = 6.2; kdZ = 4.6;

        if (p.y <= 0.093 && horizErr < 0.142 && Math.hypot(v.x, v.y, v.z) < 0.46 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.17) {
          prevThrust = 0.01;
          prevCmdRoll = 0;
          prevCmdPitch = 0;
          intRoll = clamp(intRoll + (0 - att.roll) * dt, -0.35, 0.35);
          intPitch = clamp(intPitch + (0 - att.pitch) * dt, -0.35, 0.35);
          return { thrust: 0.01, roll: 0, pitch: 0, yawRate: clamp(-3 * att.yaw, -2, 2) };
        }

        if (horizErr < 0.125 && horizSpeed < 0.38 && p.y < 0.28) {
          targetY = 0.02;
          targetVy = -0.12;
          kpY = 9.5; kdY = 5.2;
        } else if (horizErr < 0.25) {
          targetY = 0.12;
        } else {
          targetY = clamp(0.16 + 0.45 * horizErr, 0.18, 0.80);
        }
      }

      const dragCoeff = clamp(0.42 / (15.2 / kT), 0.44, 0.70);
      let ax = kpX * (targetX - p.x) + kdX * (targetVx - v.x) + dragCoeff * v.x - windAx;
      let az = kpZ * (targetZ - p.z) + kdZ * (targetVz - v.z) + dragCoeff * v.z - windAz;

      if (p.x + v.x * 0.25 > 2.45 && v.x > 0) ax -= 8.0 * v.x;
      if (p.x + v.x * 0.25 < -2.45 && v.x < 0) ax -= 8.0 * v.x;
      if (p.z + v.z * 0.25 > 2.45 && v.z > 0) az -= 8.0 * v.z;
      if (p.z + v.z * 0.25 < -2.45 && v.z < 0) az -= 8.0 * v.z;

      const maxA = 5.8;
      ax = clamp(ax, -maxA, maxA);
      az = clamp(az, -maxA, maxA);

      let ay = clamp(kpY * (targetY - p.y) + kdY * (targetVy - v.y) + dragCoeff * v.y, -4.5, 5.5);
      if (p.y < 0.22 && v.y < -0.35) {
        ay = Math.max(ay, -6.5 * (v.y + 0.20));
      }

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyAx = cy * ax - sy * az;
      const bodyAz = sy * ax + cy * az;
      const totalAy = Math.max(2.5, 9.81 + ay);

      let maxTilt = 0.50;
      if (obs.gateIndex === 3 && p.y < 0.18) {
        maxTilt = Math.hypot(p.x - obs.pad.centre.x, p.z - obs.pad.centre.z) < 0.14 ? 0.15 : 0.28;
      }
      const desPitch = clamp(Math.atan2(bodyAx, totalAy), -maxTilt, maxTilt);
      const desRoll = clamp(Math.atan2(-bodyAz, Math.hypot(bodyAx, totalAy)), -maxTilt, maxTilt);

      const desAlphaRoll = 160 * (desRoll - att.roll) - 20 * rates.roll;
      const desAlphaPitch = 160 * (desPitch - att.pitch) - 20 * rates.pitch;

      const cmdRoll = clamp(att.roll + (desAlphaRoll + 13 * rates.roll - 22 * intRoll) / 65 - biasRoll, -0.6, 0.6);
      const cmdPitch = clamp(att.pitch + (desAlphaPitch + 13 * rates.pitch - 22 * intPitch) / 65 - biasPitch, -0.6, 0.6);

      intRoll = clamp(intRoll + (cmdRoll - att.roll) * dt, -0.35, 0.35);
      intPitch = clamp(intPitch + (cmdPitch - att.pitch) * dt, -0.35, 0.35);

      const tiltCos = Math.max(0.5, Math.cos(att.roll) * Math.cos(att.pitch));
      const thrust = clamp(totalAy / (kT * tiltCos), 0.01, 1.0);

      prevCmdRoll = cmdRoll;
      prevCmdPitch = cmdPitch;
      prevThrust = thrust;

      return { thrust, roll: cmdRoll, pitch: cmdPitch, yawRate: clamp(-3 * att.yaw, -2, 2) };
    }
  };
}
