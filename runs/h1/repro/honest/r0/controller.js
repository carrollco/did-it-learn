export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  let prevAtt = null, prevVel = null;
  let intRoll = 0, intPitch = 0;
  let bRoll = 0, bPitch = 0;
  let kScale = 20.0;
  let windX = 0, windZ = 0, windY = 0;
  let windSlowX = 0, windSlowZ = 0;
  let lastCmd = { thrust: 0.48, roll: 0, pitch: 0, yawRate: 0 };
  let targetIndex = -1, phase = 'align', settle = 0, alignSteps = 0;
  let intZ = 0, intY = 0, intX = 0;

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (prevAtt !== null && obs.t > 0.02) {
        const eRoll = lastCmd.roll - prevAtt.roll;
        const ePitch = lastCmd.pitch - prevAtt.pitch;
        const expDrRoll = (65 * eRoll - 13.4 * rates.roll + 22 * intRoll) * dt;
        const expDrPitch = (65 * ePitch - 13.4 * rates.pitch + 22 * intPitch) * dt;
        const actDrRoll = rates.roll - prevAtt.rateRoll;
        const actDrPitch = rates.pitch - prevAtt.ratePitch;

        const measBRoll = clamp((actDrRoll - expDrRoll) / dt, -25, 25);
        const measBPitch = clamp((actDrPitch - expDrPitch) / dt, -25, 25);
        bRoll += 0.06 * (measBRoll - bRoll);
        bPitch += 0.06 * (measBPitch - bPitch);

        const bodyX = Math.sin(prevAtt.pitch) * Math.cos(prevAtt.roll);
        const bodyZ = -Math.sin(prevAtt.roll);
        const dirX = Math.cos(prevAtt.yaw) * bodyX + Math.sin(prevAtt.yaw) * bodyZ;
        const dirY = Math.cos(prevAtt.pitch) * Math.cos(prevAtt.roll);
        const dirZ = -Math.sin(prevAtt.yaw) * bodyX + Math.cos(prevAtt.yaw) * bodyZ;

        const dvX = v.x - prevVel.x;
        const dvY = v.y - prevVel.y;
        const dvZ = v.z - prevVel.z;

        if (p.y > 0.095 && lastCmd.thrust > 0.22 && lastCmd.thrust < 0.88) {
          const dragK = 0.42 / (15.0 / kScale);
          const accY = dvY / dt;
          const measK = (accY + 9.81 + dragK * v.y) / (lastCmd.thrust * dirY);
          if (measK > 12 && measK < 28) {
            kScale += 0.06 * (measK - kScale);
          }
        }

        if (p.y > 0.085) {
          const dragK = 0.42 / (15.0 / kScale);
          const dAccX = clamp(dvX / dt - kScale * lastCmd.thrust * dirX + dragK * v.x, -12, 12);
          const dAccZ = clamp(dvZ / dt - kScale * lastCmd.thrust * dirZ + dragK * v.z, -12, 12);
          const dAccY = clamp(dvY / dt + 9.81 - kScale * lastCmd.thrust * dirY + dragK * v.y, -6, 6);
          windX += 0.24 * (dAccX - windX);
          windZ += 0.24 * (dAccZ - windZ);
          windY += 0.06 * (dAccY - windY);
          windSlowX += 0.025 * (dAccX - windSlowX);
          windSlowZ += 0.025 * (dAccZ - windSlowZ);
        }
      }

      prevAtt = { ...att, rateRoll: rates.roll, ratePitch: rates.pitch };
      prevVel = { ...v };

      const gustMag = Math.hypot(windX - windSlowX, windZ - windSlowZ);
      const isGust = gustMag > 2.2;

      let target;
      let maxTilt = 0.54;
      let kpX = 6.5, kdX = 4.2;
      let kpZ = 10.5, kdZ = 5.6;
      let kpY = 9.5, kdY = 5.0;

      const prevGateX = obs.gateIndex === 1 ? -1.15 : (obs.gateIndex === 2 ? 0.15 : (obs.gateIndex === 3 ? 1.35 : -3.0));
      const minSafeX = prevGateX + 0.16;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          targetIndex = obs.gateIndex;
          phase = 'align';
          settle = 0;
          alignSteps = 0;
          intZ = 0; intY = 0; intX = 0;
        }

        if (p.x >= gate.x - 0.01) {
          phase = 'recover';
          settle = 0;
        }

        if (phase === 'recover') {
          const safeY = Math.min(2.75, gate.y + 0.44);
          if (Math.abs(p.y - safeY) > 0.12 && p.x > gate.x - 0.15) {
            target = { x: p.x + 0.05, y: safeY, z: gate.z };
          } else {
            target = { x: gate.x - 0.36, y: safeY, z: gate.z };
          }
          if (p.x < gate.x - 0.25) {
            phase = 'align';
            settle = 0;
          }
        }

        const alignX = gate.x - 0.32;
        if (phase === 'align') {
          alignSteps++;
          const errZ = Math.abs(p.z - gate.z);
          const errY = Math.abs(p.y - gate.y);
          let tx = alignX;
          if (errZ > 0.08 || errY > 0.14) {
            tx = Math.min(alignX, Math.max(minSafeX, p.x + 0.40));
          }
          target = { x: Math.max(minSafeX, tx), y: gate.y, z: gate.z };

          if (errZ < 0.16 && Math.abs(p.x - alignX) < 0.40) {
            intZ = clamp(intZ + (gate.z - p.z) * dt, -0.20, 0.20);
            intY = clamp(intY + (gate.y - p.y) * dt, -0.20, 0.20);
          }

          const dy = Math.abs(p.y - gate.y);
          const dz = Math.abs(p.z - gate.z);
          const predDy = Math.abs(p.y + v.y * 0.06 - gate.y);
          const predDz = Math.abs(p.z + v.z * 0.06 - gate.z);
          const dx = Math.abs(p.x - alignX);

          const urgent = obs.t > 10.5 || alignSteps > 170;
          const relax = urgent ? 0.004 : 0;
          if (dy < 0.068 + relax && dz < 0.023 + relax && predDy < 0.078 + relax && predDz < 0.025 + relax &&
              p.x < gate.x - 0.08 && dx < 0.26 &&
              Math.abs(v.y) < 0.35 && Math.abs(v.z) < 0.20 && (!isGust || urgent)) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= (urgent ? 2 : 3)) phase = 'lunge';
        }

        if (phase === 'lunge') {
          const tCross = clamp((gate.x - p.x) / Math.max(0.6, v.x), 0.01, 0.18);
          const predY = p.y + v.y * tCross;
          const predZ = p.z + v.z * tCross;
          const predDy = Math.abs(predY - gate.y);
          const predDz = Math.abs(predZ - gate.z);

          if (p.x < gate.x - 0.13 && (predDy > 0.10 || predDz > 0.029)) {
            phase = 'align';
            settle = 0;
            target = { x: Math.max(minSafeX, gate.x - 0.36), y: gate.y, z: gate.z };
            kpX = 9.5; kdX = 6.5;
          } else {
            target = { x: gate.x + 0.50, y: gate.y, z: gate.z };
            kpX = 9.5; kdX = 3.8;
            kpZ = 14.0; kdZ = 6.6;
            kpY = 12.0; kdY = 5.6;
          }
        }
      } else {
        if (phase !== 'pad_descend' && phase !== 'pad_pin') {
          phase = 'pad_approach';
          intZ = 0; intX = 0; intY = 0;
        }
        const pad = obs.pad.centre;
        const distH = Math.hypot(pad.x - p.x, pad.z - p.z);
        const speedH = Math.hypot(v.x, v.z);
        if (distH < 0.65) {
          intX = clamp(intX + (pad.x - p.x) * dt, -0.30, 0.30);
          intZ = clamp(intZ + (pad.z - p.z) * dt, -0.30, 0.30);
        }
        if (phase === 'pad_approach') {
          const targetY = clamp(0.08 + 0.25 * distH, 0.08, 0.42);
          target = { x: pad.x, y: targetY, z: pad.z };
          kpX = 8.5; kdX = 4.6;
          kpZ = 8.5; kdZ = 4.6;
          if (distH < 0.38 && speedH < 0.85 && p.y < 0.48) phase = 'pad_descend';
        }
        if (phase === 'pad_descend') {
          target = { x: pad.x, y: 0.0, z: pad.z };
          kpX = 9.0; kdX = 4.8;
          kpZ = 9.0; kdZ = 4.8;
          if (distH > 0.48) phase = 'pad_approach';
          if (p.y < 0.094 && distH < 0.142 && speedH < 0.46 && Math.abs(v.y) < 0.42) {
            phase = 'pad_pin';
          }
        }
        if (phase === 'pad_pin') {
          target = { x: pad.x, y: 0.0, z: pad.z };
          maxTilt = 0.14;
          if (distH > 0.148 || p.y > 0.105) phase = 'pad_descend';
        }
      }

      const dragK = 0.42 / (15.0 / kScale);
      let ax = kpX * (target.x - p.x) + 5.0 * intX - kdX * v.x - windX + dragK * v.x;
      let az = kpZ * (target.z - p.z) + 7.5 * intZ - kdZ * v.z - windZ + dragK * v.z;
      let ay = kpY * (target.y - p.y) + 4.5 * intY - kdY * v.y - windY;

      if (phase === 'pad_descend') {
        const speedH = Math.hypot(v.x, v.z);
        const safeVy = Math.sqrt(Math.max(0.02, 0.68 * 0.68 - speedH * speedH));
        const maxDownSpeed = p.y < 0.18 ? Math.min(0.32, safeVy) : Math.min(0.55, safeVy);
        const desiredVy = speedH > 0.55 ? 0.0 : clamp(3.6 * (0.04 - p.y) - 0.18, -maxDownSpeed, 0.8);
        ay = 9.0 * (desiredVy - v.y) - windY;
      }

      if (isGust && (phase === 'align' || phase === 'pad_approach' || phase === 'pad_descend')) {
        ax -= 0.9 * v.x;
        az -= 0.9 * v.z;
      }

      ax = clamp(ax, -7.8, 7.8);
      az = clamp(az, -7.8, 7.8);
      ay = clamp(ay, -5.8, 6.5);

      const totalUp = Math.max(2.0, 9.81 + ay);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyAx = cy * ax - sy * az;
      const bodyAz = sy * ax + cy * az;

      const desPitch = clamp(Math.atan2(bodyAx, totalUp), -maxTilt, maxTilt);
      const desRoll = clamp(-Math.atan2(bodyAz, totalUp), -maxTilt, maxTilt);

      let cmdRoll = clamp(desRoll - (22 * intRoll + bRoll) / 65, -0.6, 0.6);
      let cmdPitch = clamp(desPitch - (22 * intPitch + bPitch) / 65, -0.6, 0.6);

      let thrust = totalUp / (kScale * Math.max(0.55, Math.cos(att.roll) * Math.cos(att.pitch)));
      if (phase === 'pad_pin') {
        thrust = 0.0;
        cmdRoll = clamp(0 - (22 * intRoll + bRoll) / 65, -0.12, 0.12);
        cmdPitch = clamp(0 - (22 * intPitch + bPitch) / 65, -0.12, 0.12);
      }
      thrust = clamp(thrust, 0, 1);
      const yawRate = clamp(-2.5 * att.yaw, -2, 2);

      intRoll = clamp(intRoll + (cmdRoll - att.roll) * dt, -0.35, 0.35);
      intPitch = clamp(intPitch + (cmdPitch - att.pitch) * dt, -0.35, 0.35);
      lastCmd = { thrust, roll: cmdRoll, pitch: cmdPitch, yawRate };

      return lastCmd;
    }
  };
}
