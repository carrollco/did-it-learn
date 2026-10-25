export default function makeController() {
  const dt = 0.01;
  const m0 = 0.72;
  const drag = 0.42 / m0;
  const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

  let init = false;
  let vfX = 0, vfY = 0, vfZ = 0;
  let refX = 0, refY = 0, refZ = 0;

  let dFastX = 0, dFastY = 0, dFastZ = 0;
  let dSlowX = 0, dSlowY = 0, dSlowZ = 0;
  let thrustLag = 0;
  let lastThrust = 0.45;

  let intX = 0, intY = 0, intZ = 0, yawInt = 0;

  let targetIndex = -1;
  let phase = 0; // 0 = align, 1 = cross
  let settle = 0;
  let hasActiveGate = false;
  let activeGateX = 0, activeGateY = 0, activeGateZ = 0;
  let hasPrevGate = false;
  let prevGateX = -10, prevGateY = 0, prevGateZ = 0;
  let hasLockedGate = false;
  let lockedGateX = 0, lockedGateY = 0, lockedGateZ = 0;
  let padY = 0.60;

  const out = { thrust: 0.45, roll: 0, pitch: 0, yawRate: 0 };

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;
      const px = p.x, py = p.y, pz = p.z;

      const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
      const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyX = sp * cr;
      const bodyZ = -sr;
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = cp * cr;
      const dirZ = -sy * bodyX + cy * bodyZ;

      if (!init) {
        init = true;
        vfX = v.x; vfY = v.y; vfZ = v.z;
        refX = px; refY = py > 0.25 ? py : 0.25; refZ = pz;
      } else {
        const prevVfX = vfX, prevVfY = vfY, prevVfZ = vfZ;
        const av = 0.35;
        vfX += av * (v.x - vfX);
        vfY += av * (v.y - vfY);
        vfZ += av * (v.z - vfZ);

        thrustLag += (lastThrust - thrustLag) * 0.22;

        if (obs.t > 0.12 && py > 0.10) {
          const measAx = (vfX - prevVfX) * 100;
          const measAy = (vfY - prevVfY) * 100;
          const measAz = (vfZ - prevVfZ) * 100;

          const specThrust = (16 / m0) * thrustLag;
          const expAy = specThrust * dirY - 9.81 - drag * vfY;
          const rawDy = clamp(measAy - expAy, -6, 6);
          dFastY += 0.16 * (rawDy - dFastY);
          dSlowY += 0.025 * (rawDy - dSlowY);

          const effSpecThrust = clamp(specThrust + dSlowY / (dirY > 0.7 ? dirY : 0.7), 4, 24);
          const expAx = effSpecThrust * dirX - drag * vfX;
          const expAz = effSpecThrust * dirZ - drag * vfZ;

          const rawDx = clamp(measAx - expAx, -8, 8);
          const rawDz = clamp(measAz - expAz, -8, 8);
          dFastX += 0.22 * (rawDx - dFastX);
          dFastZ += 0.22 * (rawDz - dFastZ);
          dSlowX += 0.02 * (rawDx - dSlowX);
          dSlowZ += 0.02 * (rawDz - dSlowZ);
        }
      }

      if (hasLockedGate && (px > lockedGateX + 0.25 || px < lockedGateX - 0.15)) {
        hasLockedGate = false;
      }

      if (obs.gateIndex !== targetIndex) {
        if (hasActiveGate && obs.gateIndex > targetIndex) {
          hasPrevGate = true;
          prevGateX = activeGateX;
          prevGateY = activeGateY;
          prevGateZ = activeGateZ;
          if (px <= activeGateX + 0.25) {
            hasLockedGate = true;
            lockedGateX = activeGateX;
            lockedGateY = activeGateY;
            lockedGateZ = activeGateZ;
          }
          if (refX < activeGateX + 0.35) refX = activeGateX + 0.35;
        }
        targetIndex = obs.gateIndex;
        phase = 0;
        settle = 0;
        if (obs.gateIndex === 3) {
          padY = clamp(py, 0.35, 0.65);
        }
      }

      if (hasPrevGate && px < prevGateX + 0.18) {
        hasLockedGate = true;
        lockedGateX = prevGateX;
        lockedGateY = prevGateY;
        lockedGateZ = prevGateZ;
      }

      let targetX = px, targetY = py, targetZ = pz;
      let emergencyBrakeX = false;
      let exitBoostX = false;

      if (hasLockedGate) {
        targetX = lockedGateX + 0.42;
        targetY = lockedGateY;
        targetZ = lockedGateZ;
        if (px < lockedGateX + 0.12 && vfX < 0.30) {
          refX = lockedGateX + 0.55;
          exitBoostX = true;
        }
      } else if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        const gx = gate.x, gy = gate.y, gz = gate.z;
        hasActiveGate = true;
        activeGateX = gx; activeGateY = gy; activeGateZ = gz;

        const eyA = Math.abs(gy - py);
        const ezA = Math.abs(gz - pz);
        const alignX = (eyA > 0.20 || ezA > 0.14) ? gx - 0.48 : gx - 0.40;

        if (phase === 0) {
          if (px > gx - 0.03) {
            if (eyA < 0.075 && ezA < 0.06) {
              targetX = gx - 0.42; targetY = gy; targetZ = gz;
            } else {
              targetX = px > gx + 0.25 ? px : gx + 0.25;
              targetY = gy; targetZ = gz;
            }
            settle = 0;
          } else {
            targetX = alignX; targetY = gy; targetZ = gz;
            if (px > gx - 0.26 && (ezA > 0.075 || eyA > 0.14 || vfX > 0.20)) {
              targetX = gx - 0.50;
              refX = targetX;
              emergencyBrakeX = true;
            }
            const exA = Math.abs((gx - 0.40) - px);
            const spd = Math.hypot(vfX, vfY, vfZ);
            const gustZ = Math.abs(dFastZ - dSlowZ);
            const gustX = Math.abs(dFastX - dSlowX);
            if (exA < 0.12 && eyA < 0.08 && ezA < 0.055 && spd < 0.28 && gustZ < 0.95 && gustX < 1.1) {
              settle++;
            } else if (settle > 0) {
              settle--;
            }
            if (settle >= 10) {
              phase = 1;
            }
          }
        }

        if (phase === 1) {
          const dxGate = gx - px;
          const stopDist = (vfX > 0 ? (vfX * vfX) / 8.0 : 0) + (vfX > 0 ? vfX * 0.04 : 0);
          if (dxGate > (stopDist > 0.055 ? stopDist : 0.055)) {
            const tPred = clamp(dxGate / (vfX > 0.28 ? vfX : 0.28), 0.05, 0.28);
            const predY = (py - gy) + vfY * tPred + 0.22 * (dFastY - dSlowY) * tPred * tPred;
            const predZ = (pz - gz) + vfZ * tPred + 0.22 * (dFastZ - dSlowZ) * tPred * tPred;
            if (ezA > 0.072 || Math.abs(predZ) > 0.080 || Math.abs(vfZ) > 0.32 || eyA > 0.14 || Math.abs(predY) > 0.16) {
              phase = 0;
              settle = 0;
              targetX = gx - 0.50; targetY = gy; targetZ = gz;
              refX = targetX;
              emergencyBrakeX = true;
            }
          }
          if (phase === 1) {
            targetX = gx + 0.36; targetY = gy; targetZ = gz;
          }
        }
      } else {
        const pad = obs.pad.centre;
        const errH = Math.hypot(pad.x - px, pad.z - pz);
        const spdH = Math.hypot(vfX, vfZ);
        const gustH = Math.hypot(dFastX - dSlowX, dFastZ - dSlowZ);
        if (errH < 0.11 && spdH < 0.24 && gustH < 1.1) {
          padY = padY - 0.0045 > 0.015 ? padY - 0.0045 : 0.015;
        } else if (errH < 0.22 && spdH < 0.38) {
          const floorY = 0.22;
          if (padY > floorY) padY = padY - 0.0020 > floorY ? padY - 0.0020 : floorY;
          else if (py > 0.10) padY = padY + 0.0030 < floorY ? padY + 0.0030 : floorY;
        } else {
          const safeY = 0.32;
          if (padY > 0.58) padY = 0.58;
          if (padY < safeY && py > 0.10) padY = padY + 0.0040 < safeY ? padY + 0.0040 : safeY;
        }
        targetX = pad.x; targetY = padY; targetZ = pad.z;
      }

      const isCross = (obs.gateIndex < 3 && phase === 1) || hasLockedGate;
      const maxStepX = (obs.gateIndex < 3 && phase === 1 && px < activeGateX - 0.04) ? 0.017 : (isCross ? 0.025 : 0.021);
      const maxStepZ = isCross ? 0.025 : 0.022;
      const maxStepY = 0.018;
      const alphaRef = isCross ? 0.13 : 0.09;

      if (!emergencyBrakeX && !exitBoostX) {
        refX += clamp((targetX - refX) * alphaRef, -maxStepX, maxStepX);
        if (obs.gateIndex < 3 && phase === 1 && px < activeGateX - 0.04 && refX > px + 0.22) {
          refX = px + 0.22;
        }
        if (hasPrevGate && refX < prevGateX + 0.30) {
          refX = prevGateX + 0.30;
        }
      }
      refY += clamp((targetY - refY) * alphaRef, -maxStepY, maxStepY);
      refZ += clamp((targetZ - refZ) * alphaRef, -maxStepZ, maxStepZ);

      const rawEx = targetX - px;
      const rawEy = targetY - py;
      const rawEz = targetZ - pz;

      if (py > 0.085) {
        if (Math.abs(rawEx) < 0.35 && !emergencyBrakeX) intX = clamp(intX + rawEx * dt, -0.45, 0.45);
        else intX *= 0.94;

        if (Math.abs(rawEz) < 0.35) intZ = clamp(intZ + rawEz * dt, -0.45, 0.45);
        else intZ *= 0.95;

        if (Math.abs(rawEy) < 0.45) intY = clamp(intY + rawEy * dt, -0.60, 0.60);
        else intY *= 0.97;

        yawInt = clamp(yawInt + att.yaw * dt, -0.6, 0.6);
      }

      const ex = refX - px;
      const ey = refY - py;
      const ez = refZ - pz;

      const distX = clamp(0.80 * dFastX + 0.20 * dSlowX, -5.2, 5.2);
      const distZ = clamp(0.80 * dFastZ + 0.20 * dSlowZ, -5.2, 5.2);
      const distY = clamp(0.25 * dFastY + 0.75 * dSlowY, -4.5, 4.5);

      const kpZ = isCross ? 6.2 : 5.0;
      const kdZ = isCross ? 4.2 : 3.7;
      const maxNegAx = emergencyBrakeX ? 4.8 : 3.6;
      const maxPosAx = exitBoostX ? 5.2 : 3.8;

      const ax = clamp(5.0 * ex - 3.7 * vfX + 1.8 * intX + drag * vfX - distX, -maxNegAx, maxPosAx);
      const az = clamp(kpZ * ez - kdZ * vfZ + 1.8 * intZ + drag * vfZ - distZ, -4.2, 4.2);
      const ay = clamp(6.0 * ey - 3.8 * vfY + 2.2 * intY + drag * vfY - distY, -3.5, 4.8);

      const aFwd = cy * ax - sy * az;
      const aRight = sy * ax + cy * az;

      const nearPadTouchdown = obs.gateIndex === 3 && py < 0.115 && Math.hypot(obs.pad.centre.x - px, obs.pad.centre.z - pz) < 0.14;
      const lowAlt = obs.gateIndex === 3 && py < 0.17;
      const maxTilt = nearPadTouchdown ? 0.13 : (lowAlt ? 0.26 : ((emergencyBrakeX || exitBoostX) ? 0.52 : 0.46));

      const pitch = clamp(Math.atan2(aFwd, 9.81), -maxTilt, maxTilt);
      const roll = clamp(Math.atan2(-aRight, 9.81), -maxTilt, maxTilt);

      let thrust = clamp((m0 * (9.81 + ay)) / (16 * (dirY > 0.68 ? dirY : 0.68)), 0, 1);
      if (nearPadTouchdown && py <= 0.082 && Math.hypot(vfX, vfY, vfZ) < 0.22) {
        if (thrust > 0.10) thrust = 0.10;
      }
      lastThrust = thrust;

      out.thrust = thrust;
      out.roll = roll;
      out.pitch = pitch;
      out.yawRate = clamp(-3.2 * att.yaw - 2.5 * yawInt - 0.25 * rates.yaw, -2, 2);
      return out;
    },
  };
}
