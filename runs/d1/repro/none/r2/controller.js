export default function makeController() {
  const GATES = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];
  let targetIndex = -1;
  let phase = 'align';
  let settle = 0;

  let px = null, py = 0.07, pz = null;
  let vx = 0, vy = 0, vz = 0;
  let wx = 0, wz = 0;
  let wBaseX = 0, wBaseZ = 0, baseCount = 0;

  let KT = 21.0, estMass = 0.72;
  let uT = 0.48;
  let lastThrust = 0.48, lastRollDes = 0, lastPitchDes = 0;
  let prevDirX = 0, prevDirY = 1, prevDirZ = 0;
  let refRoll = 0, refPitch = 0;
  let trimRoll = 0, trimPitch = 0;
  let intYaw = 0, intY = 0, intX = 0, intZ = 0;
  let stepCount = 0;

  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;
      const dt = 0.01;

      const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
      const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);

      const bodyX = sp * cr, bodyZ = -sr;
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = cp * cr;
      const dirZ = -sy * bodyX + cy * bodyZ;

      if (px === null) {
        px = p.x; py = p.y; pz = p.z;
        vx = v.x; vy = v.y; vz = v.z;
        prevDirX = dirX; prevDirY = dirY; prevDirZ = dirZ;
      } else {
        uT += (lastThrust - uT) * (1 - Math.exp(-dt / 0.04));
        const avgDirX = 0.5 * (prevDirX + dirX);
        const avgDirY = 0.5 * (prevDirY + dirY);
        const avgDirZ = 0.5 * (prevDirZ + dirZ);
        const cDrag = 0.42 / estMass;

        const ayPred = KT * uT * avgDirY - cDrag * vy - 9.81;
        const vyPred = vy + ayPred * dt;
        const pyPred = Math.max(0.07, py + vyPred * dt);
        const epY = p.y - pyPred;
        const evY = v.y - vyPred;
        py = pyPred + 0.65 * epY + 0.002 * evY;
        vy = vyPred + 8.0 * epY + 0.40 * evY;

        if (p.y > 0.11 && p.y < 2.85 && uT * avgDirY > 0.25) {
          const ayMeas = (v.y - (vyPred - ayPred * dt)) / dt;
          const ayErr = ayMeas - ayPred;
          const gain = obs.t < 2.0 ? 0.08 : 0.015;
          KT = clamp(KT + gain * ayErr / (uT * avgDirY), 16.0, 27.5);
          estMass = clamp(15.2 / KT, 0.60, 0.85);
        }

        const AT = KT * uT;
        const axKnown = AT * avgDirX - cDrag * vx;
        const azKnown = AT * avgDirZ - cDrag * vz;

        const vxPred = vx + (axKnown + wx) * dt;
        const pxPred = px + vxPred * dt;
        const epX = p.x - pxPred, evX = v.x - vxPred;
        px = pxPred + 0.65 * epX + 0.002 * evX;
        vx = vxPred + 8.8 * epX + 0.46 * evX;
        if (p.y > 0.095) wx = clamp(wx + 32.0 * epX + 27.0 * evX, -4.5, 4.5);

        const vzPred = vz + (azKnown + wz) * dt;
        const pzPred = pz + vzPred * dt;
        const epZ = p.z - pzPred, evZ = v.z - vzPred;
        pz = pzPred + 0.65 * epZ + 0.002 * evZ;
        vz = vzPred + 8.8 * epZ + 0.46 * evZ;
        if (p.y > 0.095) wz = clamp(wz + 32.0 * epZ + 27.0 * evZ, -4.5, 4.5);

        if (obs.t >= 0.25 && obs.t < 1.98 && p.y > 0.10) {
          baseCount++;
          wBaseX += (wx - wBaseX) / baseCount;
          wBaseZ += (wz - wBaseZ) / baseCount;
        }

        refRoll += (lastRollDes - refRoll) * (dt / 0.085);
        refPitch += (lastPitchDes - refPitch) * (dt / 0.085);
        if (p.y > 0.10) {
          const trimGain = obs.t < 2.0 ? 2.4 : 0.6;
          trimRoll = clamp(trimRoll + trimGain * (att.roll - refRoll) * dt, -0.22, 0.22);
          trimPitch = clamp(trimPitch + trimGain * (att.pitch - refPitch) * dt, -0.22, 0.22);
        }

        prevDirX = dirX; prevDirY = dirY; prevDirZ = dirZ;
      }
      stepCount++;

      let activeGateIdx = obs.gateIndex;
      if (activeGateIdx > 0 && px < GATES[activeGateIdx - 1].x + 0.18) {
        activeGateIdx = activeGateIdx - 1;
      }

      let target, desVx = 0, desVy = 0, desVz = 0;
      let brakingX = false;
      if (activeGateIdx < 3) {
        const gate = GATES[activeGateIdx];
        if (targetIndex !== activeGateIdx) {
          targetIndex = activeGateIdx;
          phase = obs.gateIndex > activeGateIdx ? 'cross' : 'align';
          settle = 0;
          intX = 0; intZ = 0;
        }
        const dy = Math.abs(py - gate.y);
        const dz = Math.abs(pz - gate.z);
        const dtCross = clamp((gate.x - px) / Math.max(0.22, vx), 0.03, 0.35);
        const dyPred = Math.abs(py + vy * dtCross - gate.y);
        const dzPred = Math.abs(pz + vz * dtCross - gate.z);
        const offCenter = dz > 0.058 || dy > 0.125 || dzPred > 0.068 || dyPred > 0.14 || Math.abs(vz) > 0.30;

        if (phase === 'cross') {
          if (px < gate.x - 0.01 && px > gate.x - 0.42 && offCenter) {
            phase = 'align';
            settle = 0;
            brakingX = true;
          } else {
            target = { x: gate.x + 0.34, y: gate.y, z: gate.z };
            desVx = 0.60;
          }
        }
        if (phase === 'align') {
          const alignX = gate.x - 0.38;
          target = { x: alignX, y: gate.y, z: gate.z };
          const xStop = px + 0.22 * Math.max(0, vx);
          if (px > gate.x - 0.26 || xStop > gate.x - 0.16 || (offCenter && xStop > gate.x - 0.28)) {
            brakingX = true;
            desVx = px > gate.x - 0.22 ? -0.35 : -0.20;
            target.x = Math.min(alignX, px - 0.18);
          }
          if (Math.abs(px - alignX) < 0.18 && px < gate.x - 0.18 && dy < 0.085 && dz < 0.048 && Math.hypot(vy, vz) < 0.24 && vx < 0.25) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle > 5) phase = 'cross';
        }
      } else {
        if (targetIndex !== 3) {
          targetIndex = 3;
          phase = 'pad_approach';
          intX = 0; intZ = 0;
        }
        const pad = obs.pad.centre;
        const padTargetX = pad.x - clamp(0.012 * wx, -0.035, 0.035);
        const padTargetZ = pad.z - clamp(0.012 * wz, -0.035, 0.035);
        const distXZ = Math.hypot(pad.x - px, pad.z - pz);
        const speedXZ = Math.hypot(vx, vz);
        if (phase === 'pad_approach') {
          target = { x: padTargetX, y: 0.36, z: padTargetZ };
          if (distXZ < 0.095 && speedXZ < 0.22 && py < 0.48 && Math.abs(vy) < 0.28 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.30) {
            phase = 'descend';
            intY = clamp(intY, -0.15, 0.25);
          }
        }
        if (phase === 'descend') {
          if (distXZ > 0.115 || (py > 0.105 && speedXZ > 0.26)) {
            target = { x: padTargetX, y: Math.max(py, 0.16), z: padTargetZ };
            desVy = 0;
          } else {
            const stepY = py > 0.16 ? 0.055 : 0.030;
            target = { x: padTargetX, y: Math.max(0.05, py - stepY), z: padTargetZ };
            desVy = py > 0.16 ? -0.26 : -0.11;
          }
        }
      }

      const ex = target.x - px;
      const ey = target.y - py;
      const ez = target.z - pz;

      if (Math.abs(ex) < 0.25) intX = clamp(intX + ex * dt, -0.30, 0.30);
      if (Math.abs(ez) < 0.25) intZ = clamp(intZ + ez * dt, -0.30, 0.30);
      intY = clamp(intY + ey * dt, -0.45, 0.45);

      const kpX = brakingX ? 7.5 : (activeGateIdx < 3 && phase === 'cross' ? 4.8 : 4.8);
      const kdX = brakingX ? 5.4 : (activeGateIdx < 3 && phase === 'cross' ? 3.8 : 4.0);
      const kpZ = activeGateIdx < 3 ? 8.8 : 5.5;
      const kdZ = activeGateIdx < 3 ? 5.4 : 4.2;

      const axFb = clamp(kpX * ex + kdX * (desVx - vx) + 1.5 * intX, brakingX ? -5.2 : -4.0, 3.6);
      const azFb = clamp(kpZ * ez + kdZ * (desVz - vz) + 2.2 * intZ, -4.4, 4.4);
      const ayFb = clamp(6.2 * ey + 4.4 * (desVy - vy) + 2.5 * intY, -3.5, 4.8);

      const cDrag = 0.42 / estMass;
      const Fx = axFb + cDrag * vx - wx;
      const Fy = Math.max(2.5, 9.81 + ayFb + cDrag * vy);
      const Fz = azFb + cDrag * vz - wz;
      const Fmag = Math.hypot(Fx, Fy, Fz);

      const FbodyX = cy * Fx - sy * Fz;
      const FbodyZ = sy * Fx + cy * Fz;

      let maxTilt = brakingX ? 0.54 : 0.48;
      if (activeGateIdx === 3 && phase === 'descend') {
        if (py < 0.115) maxTilt = 0.10;
        else if (py < 0.17) maxTilt = 0.16;
      }

      const rollDes = clamp(-Math.asin(clamp(FbodyZ / Fmag, -0.55, 0.55)), -maxTilt, maxTilt);
      const pitchDes = clamp(Math.atan2(FbodyX, Fy), -maxTilt, maxTilt);

      const roll = clamp(rollDes - trimRoll + 0.32 * (rollDes - att.roll) - 0.035 * rates.roll, -0.56, 0.56);
      const pitch = clamp(pitchDes - trimPitch + 0.32 * (pitchDes - att.pitch) - 0.035 * rates.pitch, -0.56, 0.56);

      let thrust = Fy / (KT * Math.max(0.65, dirY));
      if (activeGateIdx === 3 && phase === 'descend' && py <= 0.088 && Math.hypot(obs.pad.centre.x - px, obs.pad.centre.z - pz) <= 0.115 && Math.hypot(vx, vy, vz) < 0.36 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.15) {
        thrust = 0;
        lastRollDes = 0;
        lastPitchDes = 0;
        lastThrust = 0;
        return { thrust: 0, roll: 0, pitch: 0, yawRate: 0 };
      }
      thrust = clamp(thrust, 0.05, 1.0);

      if (p.y > 0.10) intYaw = clamp(intYaw + att.yaw * dt, -0.4, 0.4);
      const yawRate = clamp(-3.2 * att.yaw - 0.25 * rates.yaw - 3.5 * intYaw, -2.0, 2.0);

      lastThrust = thrust;
      lastRollDes = rollDes;
      lastPitchDes = pitchDes;

      return { thrust, roll, pitch, yawRate };
    },
  };
}
