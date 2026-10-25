export default function makeController() {
  const dt = 0.01;
  const g = 9.81;
  const m0 = 0.725;
  const drag0 = 0.42;

  let initialized = false;
  let vel = { x: 0, y: 0, z: 0 };
  let prevAtt = { roll: 0, pitch: 0, yaw: 0 };
  let thrustEst = 0.45;

  let dSlow = { x: 0, y: 0, z: 0 };
  let dFast = { x: 0, y: 0, z: 0 };

  let intYaw = 0;
  let intPos = { x: 0, y: 0, z: 0 };

  let ref = { x: 0, y: 0, z: 0 };

  let targetIndex = 0;
  let activeGate = null;
  let phase = 'align';
  let settle = 0;
  let padY = 0.55;

  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  return {
    step(obs) {
      const p = obs.pos;
      const att = obs.att;

      if (!initialized) {
        initialized = true;
        vel = { x: obs.vel.x, y: obs.vel.y, z: obs.vel.z };
        prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };
        ref = { x: p.x, y: p.y, z: p.z };
        targetIndex = obs.gateIndex;
        if (obs.nextGate) {
          activeGate = {
            x: obs.nextGate.centre.x,
            y: obs.nextGate.centre.y,
            z: obs.nextGate.centre.z,
          };
        }
      }

      // 1. Filter velocity
      const prevVel = { x: vel.x, y: vel.y, z: vel.z };
      const alphaV = 0.42;
      vel.x += alphaV * (obs.vel.x - vel.x);
      vel.y += alphaV * (obs.vel.y - vel.y);
      vel.z += alphaV * (obs.vel.z - vel.z);

      // 2. Fast/slow acceleration disturbance observer
      const aMeasX = (vel.x - prevVel.x) / dt;
      const aMeasY = (vel.y - prevVel.y) / dt;
      const aMeasZ = (vel.z - prevVel.z) / dt;

      const avgRoll = 0.5 * (prevAtt.roll + att.roll);
      const avgPitch = 0.5 * (prevAtt.pitch + att.pitch);
      const avgYaw = 0.5 * (prevAtt.yaw + att.yaw);
      prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };

      const bodyX = Math.sin(avgPitch) * Math.cos(avgRoll);
      const bodyZ = -Math.sin(avgRoll);
      const dirX = Math.cos(avgYaw) * bodyX + Math.sin(avgYaw) * bodyZ;
      const dirY = Math.cos(avgPitch) * Math.cos(avgRoll);
      const dirZ = -Math.sin(avgYaw) * bodyX + Math.cos(avgYaw) * bodyZ;

      const thrustAcc = (16 * thrustEst) / m0;
      const aModelX = thrustAcc * dirX - (drag0 / m0) * vel.x;
      const aModelY = thrustAcc * dirY - g - (drag0 / m0) * vel.y;
      const aModelZ = thrustAcc * dirZ - (drag0 / m0) * vel.z;

      if (p.y > 0.09 && obs.t > 0.03) {
        const dRawX = clamp(aMeasX - aModelX, -6.5, 6.5);
        const dRawY = clamp(aMeasY - aModelY, -5.0, 5.0);
        const dRawZ = clamp(aMeasZ - aModelZ, -6.5, 6.5);

        const alphaSlow = 0.04;
        const alphaFast = 0.28;
        dSlow.x += alphaSlow * (dRawX - dSlow.x);
        dSlow.y += alphaSlow * (dRawY - dSlow.y);
        dSlow.z += alphaSlow * (dRawZ - dSlow.z);

        dFast.x += alphaFast * (dRawX - dFast.x);
        dFast.y += alphaFast * (dRawY - dFast.y);
        dFast.z += alphaFast * (dRawZ - dFast.z);
      }

      const gustX = dFast.x - dSlow.x;
      const gustY = dFast.y - dSlow.y;
      const gustZ = dFast.z - dSlow.z;

      // 5 & 6. Gate navigation with gate exit lock and predictive cross abort
      let target;
      if (targetIndex < 3) {
        if (obs.gateIndex > targetIndex) {
          // Gate just crossed: lock exit trajectory until clear of gate frame
          phase = 'exit';
          ref.x = Math.max(ref.x, activeGate.x + 0.16);
          ref.y = activeGate.y;
          ref.z = activeGate.z;
          if (p.x >= activeGate.x + 0.24 && vel.x >= -0.05) {
            targetIndex = obs.gateIndex;
            settle = 0;
            intPos.x = 0;
            intPos.z = 0;
            if (targetIndex < 3 && obs.nextGate) {
              activeGate = {
                x: obs.nextGate.centre.x,
                y: obs.nextGate.centre.y,
                z: obs.nextGate.centre.z,
              };
              phase = 'align';
            } else {
              phase = 'pad_approach';
              padY = clamp(p.y, 0.35, 0.65);
            }
          }
        } else if (obs.nextGate) {
          activeGate = {
            x: obs.nextGate.centre.x,
            y: obs.nextGate.centre.y,
            z: obs.nextGate.centre.z,
          };
        }
      }

      if (targetIndex < 3 && activeGate) {
        const gx = activeGate.x;
        const gy = activeGate.y;
        const gz = activeGate.z;

        if (phase === 'align') {
          const alignX = gx - 0.45;
          target = { x: alignX, y: gy, z: gz };
          const ex = Math.abs(p.x - alignX);
          const ey = Math.abs(p.y - gy);
          const ez = Math.abs(p.z - gz);
          const spd = Math.hypot(vel.x, vel.y, vel.z);
          const gustMag = Math.hypot(gustX, gustZ);

          if (
            ex < 0.11 &&
            ey < 0.065 &&
            ez < 0.048 &&
            spd < 0.28 &&
            Math.abs(vel.z) < 0.14 &&
            gustMag < 0.95
          ) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle > 10) {
            phase = 'cross';
          }
        }

        if (phase === 'cross') {
          const dx = gx - p.x;
          const vxPos = Math.max(0, vel.x);
          const tCross = clamp(dx / Math.max(vel.x, 0.45), 0.10, 0.38);
          const predY = p.y + vel.y * tCross + 0.5 * gustY * tCross * tCross;
          const predZ = p.z + vel.z * tCross + 0.5 * gustZ * tCross * tCross;

          const stopDist = vxPos * 0.18 + (vxPos * vxPos) / 4.2;
          const canAbort = dx > stopDist + 0.05 && dx > 0.12;

          const unsafeZ = Math.abs(p.z - gz) > 0.075 || Math.abs(predZ - gz) > 0.085;
          const unsafeY = Math.abs(p.y - gy) > 0.135 || Math.abs(predY - gy) > 0.155;

          if (canAbort && (unsafeZ || unsafeY)) {
            phase = 'align';
            settle = 0;
            const alignX = gx - 0.45;
            target = { x: alignX, y: gy, z: gz };
            ref.x = Math.min(ref.x, p.x - 0.12);
          } else {
            target = { x: gx + 0.35, y: gy, z: gz };
          }
        }

        if (phase === 'exit') {
          target = { x: gx + 0.35, y: gy, z: gz };
        }
      } else {
        // 7. Ramp pad descent
        const pad = obs.pad.centre;
        const distXZ = Math.hypot(pad.x - p.x, pad.z - p.z);
        const spdXZ = Math.hypot(vel.x, vel.z);

        if (phase !== 'descend') {
          const approachY = clamp(0.28 + 0.65 * distXZ, 0.32, 0.75);
          target = { x: pad.x, y: approachY, z: pad.z };
          if (distXZ < 0.08 && spdXZ < 0.20 && Math.abs(p.y - approachY) < 0.14) {
            phase = 'descend';
            padY = p.y;
          }
        }

        if (phase === 'descend') {
          const maxAllowedDist = padY > 0.15 ? 0.115 : 0.082;
          if (distXZ < maxAllowedDist && (padY > 0.15 || spdXZ < 0.22)) {
            const rate = p.y > 0.22 ? 0.44 : 0.24;
            padY = Math.max(0.015, padY - rate * dt);
          } else if (p.y > 0.105 && distXZ > 0.11) {
            padY = Math.max(padY, 0.14);
          }
          target = { x: pad.x, y: padY, z: pad.z };
        }
      }

      // 4. Smooth reference filter
      const inGateLock = phase === 'cross' || phase === 'exit';
      const nearPad = targetIndex === 3 && Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z) < 0.25;
      const alphaRefX = inGateLock || nearPad ? 0.15 : 0.08;
      const alphaRefY = phase === 'descend' ? 0.18 : 0.10;
      const alphaRefZ = inGateLock ? 0.22 : nearPad ? 0.15 : 0.10;

      ref.x += clamp(alphaRefX * (target.x - ref.x), -0.022, 0.022);
      ref.y += clamp(alphaRefY * (target.y - ref.y), -0.018, 0.018);
      ref.z += clamp(alphaRefZ * (target.z - ref.z), -0.022, 0.022);

      // 3. Yaw + local position integrals
      const errTargetX = target.x - p.x;
      const errTargetY = target.y - p.y;
      const errTargetZ = target.z - p.z;

      if (Math.abs(errTargetX) < 0.30 && !inGateLock) {
        intPos.x = clamp(intPos.x + errTargetX * dt, -0.35, 0.35);
      } else {
        intPos.x *= 0.94;
      }

      if (Math.abs(errTargetZ) < 0.30) {
        intPos.z = clamp(intPos.z + errTargetZ * dt, -0.35, 0.35);
      } else {
        intPos.z *= 0.94;
      }

      if (p.y > 0.085 && Math.abs(errTargetY) < 0.40) {
        intPos.y = clamp(intPos.y + (ref.y - p.y) * dt, -0.6, 0.6);
      }

      const kpX = inGateLock ? 4.4 : nearPad ? 4.6 : 4.2;
      const kdX = inGateLock ? 3.5 : 3.6;
      const kpZ = inGateLock ? 5.4 : nearPad ? 4.8 : 4.5;
      const kdZ = inGateLock ? 4.0 : 3.7;
      const kiXZ = nearPad ? 2.2 : 1.8;

      const distX = dSlow.x + 0.95 * gustX;
      const distY = dSlow.y + 0.45 * gustY;
      const distZ = dSlow.z + 0.95 * gustZ;

      const ax = clamp(
        kpX * (ref.x - p.x) - kdX * vel.x + kiXZ * intPos.x + (drag0 / m0) * vel.x - distX,
        -3.8,
        3.8
      );
      const az = clamp(
        kpZ * (ref.z - p.z) - kdZ * vel.z + kiXZ * intPos.z + (drag0 / m0) * vel.z - distZ,
        -3.8,
        3.8
      );
      const ay = clamp(
        5.8 * (ref.y - p.y) - 3.8 * vel.y + 2.2 * intPos.y + (drag0 / m0) * vel.y - distY,
        -3.8,
        4.6
      );

      const cy = Math.cos(att.yaw);
      const sy = Math.sin(att.yaw);
      const totalAccY = Math.max(4.5, g + ay);

      let maxTilt = 0.46;
      let thrust;
      if (
        phase === 'descend' &&
        p.y <= 0.088 &&
        padY <= 0.04 &&
        Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z) <= 0.11
      ) {
        maxTilt = 0.08;
        thrust = 0.08;
      } else {
        if (phase === 'descend' && p.y <= 0.14) maxTilt = 0.15;
        const tiltScale = Math.max(0.68, Math.cos(att.roll) * Math.cos(att.pitch));
        thrust = clamp((m0 * (g + ay)) / (16 * tiltScale), 0.05, 1.0);
      }

      const pitch = clamp((cy * ax - sy * az) / totalAccY, -maxTilt, maxTilt);
      const roll = clamp(-(sy * ax + cy * az) / totalAccY, -maxTilt, maxTilt);

      intYaw = clamp(intYaw + att.yaw * dt, -0.5, 0.5);
      const yawRate = clamp(-3.2 * att.yaw - 2.4 * intYaw - 0.25 * obs.rates.yaw, -2, 2);

      thrustEst += (thrust - thrustEst) * (1 - Math.exp(-dt / 0.04));

      return { thrust, roll, pitch, yawRate };
    },
  };
}
