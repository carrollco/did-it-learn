export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  // Inner loop state tracking
  let intRoll = 0, intPitch = 0;
  let biasRoll = 0, biasPitch = 0, biasYaw = 0;

  // Outer loop observers
  let thrustScale = 0.052; // mass / (4 * maxMotorThrust * eff)
  let slowWindX = 0, slowWindZ = 0;
  let fastWindX = 0, fastWindZ = 0;

  let prevVel = null;
  let prevAtt = null;
  let prevRates = null;
  let prevAction = { thrust: 0.45, roll: 0, pitch: 0, yawRate: 0 };

  let targetIndex = -1;
  let phase = 'align';
  let settle = 0;

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      // Update simulator's internal integral estimate using previous action
      if (prevAtt !== null) {
        intRoll = clamp(intRoll + (prevAction.roll - att.roll) * dt, -0.35, 0.35);
        intPitch = clamp(intPitch + (prevAction.pitch - att.pitch) * dt, -0.35, 0.35);
      }

      const airborne = p.y > 0.12 && p.y < 2.85 && Math.abs(p.x) < 2.75 && Math.abs(p.z) < 2.75;

      // 1. Update observers from previous step
      if (prevVel !== null && airborne) {
        const alphaRoll = clamp((rates.roll - prevRates.roll) / dt, -50, 50);
        const alphaPitch = clamp((rates.pitch - prevRates.pitch) / dt, -50, 50);
        const alphaYaw = clamp((rates.yaw - prevRates.yaw) / dt, -25, 25);

        const mRoll = 0.5 * (att.roll + prevAtt.roll);
        const mPitch = 0.5 * (att.pitch + prevAtt.pitch);
        const mRateRoll = 0.5 * (rates.roll + prevRates.roll);
        const mRatePitch = 0.5 * (rates.pitch + prevRates.pitch);
        const mRateYaw = 0.5 * (rates.yaw + prevRates.yaw);

        const cmdAlphaRoll = 65 * (prevAction.roll - mRoll) - 13.4 * mRateRoll + 22 * intRoll;
        const cmdAlphaPitch = 65 * (prevAction.pitch - mPitch) - 13.4 * mRatePitch + 22 * intPitch;
        const cmdAlphaYaw = 8 * (prevAction.yawRate - mRateYaw) - 0.4 * mRateYaw;

        if (Math.abs(prevAction.roll) < 0.52 && Math.abs(prevAction.pitch) < 0.52 && prevAction.thrust > 0.22 && prevAction.thrust < 0.82) {
          biasRoll = clamp(biasRoll + 0.045 * (alphaRoll - cmdAlphaRoll - biasRoll), -18, 18);
          biasPitch = clamp(biasPitch + 0.045 * (alphaPitch - cmdAlphaPitch - biasPitch), -18, 18);
        }
        if (Math.abs(prevAction.yawRate) < 1.5 && prevAction.thrust > 0.22 && prevAction.thrust < 0.82) {
          biasYaw = clamp(biasYaw + 0.045 * (alphaYaw - cmdAlphaYaw - biasYaw), -12, 12);
        }

        const axMeas = (v.x - prevVel.x) / dt;
        const ayMeas = (v.y - prevVel.y) / dt;
        const azMeas = (v.z - prevVel.z) / dt;

        const mYaw = 0.5 * (att.yaw + prevAtt.yaw);
        const bodyX = Math.sin(mPitch) * Math.cos(mRoll);
        const bodyZ = -Math.sin(mRoll);
        const dirX = Math.cos(mYaw) * bodyX + Math.sin(mYaw) * bodyZ;
        const dirY = Math.cos(mPitch) * Math.cos(mRoll);
        const dirZ = -Math.sin(mYaw) * bodyX + Math.cos(mYaw) * bodyZ;

        const kDrag = 0.42 / (thrustScale * 15.0);
        if (prevAction.thrust > 0.18 && prevAction.thrust < 0.88 && dirY > 0.6) {
          const expectedAy = (prevAction.thrust / thrustScale) * dirY - 9.81 - kDrag * v.y;
          const errAy = clamp(ayMeas - expectedAy, -8, 8);
          thrustScale = clamp(thrustScale - 0.0022 * errAy, 0.038, 0.080);
        }

        const accThrustX = (prevAction.thrust / thrustScale) * dirX;
        const accThrustZ = (prevAction.thrust / thrustScale) * dirZ;
        const instWindAccX = clamp(axMeas - accThrustX + kDrag * v.x, -9.0, 9.0);
        const instWindAccZ = clamp(azMeas - accThrustZ + kDrag * v.z, -9.0, 9.0);

        slowWindX += 0.06 * (instWindAccX - slowWindX);
        slowWindZ += 0.06 * (instWindAccZ - slowWindZ);
        fastWindX += 0.45 * (instWindAccX - fastWindX);
        fastWindZ += 0.45 * (instWindAccZ - fastWindZ);
      }

      prevVel = { x: v.x, y: v.y, z: v.z };
      prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };
      prevRates = { roll: rates.roll, pitch: rates.pitch, yaw: rates.yaw };

      // 2. Navigation: align before gate, lunge when centered, brake gusts, pin pad low thrust
      let target = { x: p.x, y: 1.05, z: p.z };
      let targetVel = { x: 0, y: 0, z: 0 };

      const gustX = fastWindX - slowWindX;
      const gustZ = fastWindZ - slowWindZ;
      const gustMag = Math.hypot(gustX, gustZ);

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          targetIndex = obs.gateIndex;
          phase = 'align';
          settle = 0;
        }

        const dy = p.y - gate.y;
        const dz = p.z - gate.z;
        const dx = gate.x - p.x;

        const vxEff = Math.max(0.65, v.x);
        const tCross = clamp(dx / vxEff, 0.015, 0.32);
        const predDy = dy + v.y * tCross;
        const predDz = dz + v.z * tCross + 0.1 * fastWindZ * tCross * tCross;

        if (phase === 'align') {
          // If not yet aligned in Y/Z, stay further back in X so we never cross the gate plane unaligned!
          const unaligned = Math.abs(dz) > 0.04 || Math.abs(dy) > 0.10;
          const alignX = gate.x - (unaligned ? 0.48 : 0.32);
          target = { x: alignX, y: gate.y, z: gate.z };

          // Prevent flying backwards into a previous gate
          if (obs.gateIndex > 0) {
            const prevGateX = obs.gateIndex === 1 ? -1.15 : 0.15;
            if (p.x < prevGateX + 0.30 && v.x < 0) {
              target.x = Math.max(target.x, prevGateX + 0.35);
            }
          }

          const zCentered = Math.abs(dz) < 0.020 && Math.abs(v.z) < 0.12 && Math.abs(predDz) < 0.023;
          const yCentered = Math.abs(dy) < 0.070 && Math.abs(v.y) < 0.28 && Math.abs(predDy) < 0.085;
          const xReady = p.x < gate.x - 0.06 && p.x > gate.x - 0.55;

          if (zCentered && yCentered && xReady && gustMag < 1.6) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }

          if (settle >= 3) {
            phase = 'lunge';
          }
        }

        if (phase === 'lunge') {
          const abortZ = Math.abs(dz) > 0.026 || Math.abs(predDz) > 0.029;
          const abortY = Math.abs(dy) > 0.102 || Math.abs(predDy) > 0.112;
          if (dx > 0.045 && (abortZ || abortY)) {
            phase = 'align';
            settle = 0;
            target = { x: gate.x - 0.36, y: gate.y, z: gate.z };
            targetVel = { x: -0.4, y: 0, z: 0 };
          } else {
            target = { x: gate.x + 0.35, y: gate.y, z: gate.z };
            targetVel = { x: 1.65, y: 0, z: 0 };
          }
        }
      } else {
        const pad = obs.pad.centre;
        const dPad = Math.hypot(pad.x - p.x, pad.z - p.z);
        const vHoriz = Math.hypot(v.x, v.z);
        if (phase !== 'descend' && phase !== 'pin') {
          phase = 'pad_approach';
        }
        if (phase === 'pad_approach') {
          const safeX = p.x < 1.55 && v.x < 0 ? Math.max(pad.x, 1.65) : pad.x;
          // Stay at safe altitude (y >= 0.42) until horizontally near the pad and not fighting a strong gust
          const hTarget = (dPad > 0.22 || gustMag > 1.8) ? 0.48 : 0.20;
          target = { x: safeX, y: hTarget, z: pad.z };
          if (dPad < 0.115 && vHoriz < 0.36 && p.y < 0.32 && gustMag < 1.8) {
            phase = 'descend';
          }
        }
        if (phase === 'descend') {
          target = { x: pad.x, y: 0.04, z: pad.z };
          // If blown away from pad or hit by strong gust while descending, climb back to safe approach altitude!
          if (dPad > 0.14 || (p.y > 0.12 && gustMag > 2.2)) {
            phase = 'pad_approach';
            target.y = 0.35;
          } else if (p.y <= 0.093 && dPad <= 0.138 && vHoriz < 0.42 && Math.abs(v.y) < 0.60 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.175) {
            phase = 'pin';
          }
        }
        if (phase === 'pin') {
          if (dPad > 0.145) {
            phase = 'pad_approach';
            target = { x: pad.x, y: 0.25, z: pad.z };
          } else {
            const desAlphaRoll = -80 * att.roll - 16 * rates.roll;
            const desAlphaPitch = -80 * att.pitch - 16 * rates.pitch;
            const cmdRoll = clamp(att.roll + (desAlphaRoll + 13.4 * rates.roll - 22 * intRoll - biasRoll) / 65, -0.6, 0.6);
            const cmdPitch = clamp(att.pitch + (desAlphaPitch + 13.4 * rates.pitch - 22 * intPitch - biasPitch) / 65, -0.6, 0.6);
            prevAction = { thrust: 0.0, roll: cmdRoll, pitch: cmdPitch, yawRate: 0 };
            return prevAction;
          }
        }
      }

      // 3. Outer loop PD + Wind & Gust compensation ("brake gusts")
      const kDrag = 0.42 / (thrustScale * 15.0);
      const windCompX = fastWindX + 0.52 * gustX;
      const windCompZ = fastWindZ + 0.62 * gustZ;

      const isGateZ = obs.gateIndex < 3;
      const kpX = phase === 'lunge' ? 8.2 : 6.4;
      const kdX = phase === 'lunge' ? 4.6 : 5.4;
      const kpZ = isGateZ ? 13.5 : 7.2;
      const kdZ = isGateZ ? 7.2 : 5.4;

      let ex = clamp(target.x - p.x, -0.70, 0.70);
      if (phase === 'align' && obs.gateIndex < 3) {
        const distToGateX = obs.nextGate.centre.x - p.x;
        const unaligned = Math.abs(p.z - obs.nextGate.centre.z) > 0.032 || Math.abs(p.y - obs.nextGate.centre.y) > 0.09;
        // Max approach speed toward gate plane when unaligned: v.x <= 1.2 * (distToGateX - 0.28)
        if (unaligned && distToGateX < 0.75) {
          const maxVx = Math.max(-0.5, 1.5 * (distToGateX - 0.34));
          if (v.x > maxVx) {
            targetVel.x = maxVx;
            ex = Math.min(ex, 0);
          }
        }
      }

      const ez = clamp(target.z - p.z, -0.75, 0.75);
      const ey = clamp(target.y - p.y, -0.75, 0.75);

      let ax = kpX * ex + kdX * (targetVel.x - v.x) - windCompX + kDrag * v.x;
      let az = kpZ * ez + kdZ * (targetVel.z - v.z) - windCompZ + kDrag * v.z;
      let ay = 10.2 * ey - 6.0 * v.y + kDrag * v.y;

      // Room boundary safety braking
      if (p.x < -2.45 && v.x < -0.3) ax = Math.max(ax, 4.5);
      if (p.x > 2.45 && v.x > 0.3) ax = Math.min(ax, -4.5);
      if (p.z < -2.45 && v.z < -0.3) az = Math.max(az, 4.5);
      if (p.z > 2.45 && v.z > 0.3) az = Math.min(az, -4.5);

      ax = clamp(ax, -6.5, 6.5);
      az = clamp(az, -6.5, 6.5);
      ay = clamp(ay, -4.5, 6.5);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const axBody = cy * ax - sy * az;
      const azBody = sy * ax + cy * az;
      const totalAccY = Math.max(2.5, 9.81 + ay);

      let desPitch = clamp(Math.atan2(axBody, totalAccY), -0.48, 0.48);
      let desRoll = clamp(-Math.atan2(azBody, totalAccY), -0.48, 0.48);

      // Floor crash protection: whenever close to floor (p.y < 0.22), strictly cap desired tilt so we can never hit floor tilted > 0.36 rad!
      if (p.y < 0.22 && obs.gateIndex === 3) {
        const maxTilt = p.y < 0.14 ? 0.14 : 0.22;
        desPitch = clamp(desPitch, -maxTilt, maxTilt);
        desRoll = clamp(desRoll, -maxTilt, maxTilt);
      }

      // 4. Invert inner attitude loop with torque bias
      const wn = 13.5, zeta = 0.85;
      const desAlphaRoll = wn * wn * (desRoll - att.roll) - 2 * wn * zeta * rates.roll;
      const desAlphaPitch = wn * wn * (desPitch - att.pitch) - 2 * wn * zeta * rates.pitch;

      const cmdRoll = clamp(att.roll + (desAlphaRoll + 13.4 * rates.roll - 22 * intRoll - biasRoll) / 65, -0.6, 0.6);
      const cmdPitch = clamp(att.pitch + (desAlphaPitch + 13.4 * rates.pitch - 22 * intPitch - biasPitch) / 65, -0.6, 0.6);

      const desYawRate = -3.2 * att.yaw;
      const desAlphaYaw = 9.0 * (desYawRate - rates.yaw);
      const cmdYawRate = clamp(rates.yaw + (desAlphaYaw + 0.4 * rates.yaw - biasYaw) / 8, -1.6, 1.6);

      const cosTilt = Math.max(0.55, Math.cos(att.roll) * Math.cos(att.pitch));
      const cmdThrust = clamp((totalAccY * thrustScale) / cosTilt, 0.06, 0.94);

      prevAction = { thrust: cmdThrust, roll: cmdRoll, pitch: cmdPitch, yawRate: cmdYawRate };
      return prevAction;
    },
  };
}
