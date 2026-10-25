export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01;
  const kThrust = 21.4;
  const kDrag = 0.58;
  const motorAlpha = 1 - Math.exp(-dt / 0.04);

  let initialized = false;
  let vfX = 0, vfY = 0, vfZ = 0;
  let aFiltX = 0, aFiltY = 0, aFiltZ = 0;
  let distFastX = 0, distFastY = 0, distFastZ = 0;
  let distSlowX = 0, distSlowY = 0, distSlowZ = 0;
  let thrustLag = 0;
  let lastThrust = 0.45;

  let intX = 0, intY = 0, intZ = 0, intYaw = 0;
  let refX = 0, refY = 0, refZ = 0;

  let activeGateIdx = 0;
  let activeGateX = 0, activeGateY = 0, activeGateZ = 0;
  let phase = 0; // 0: align, 1: cross, 2: exit_lock, 3: pad
  let settle = 0;
  let padY = 0.85;

  const out = { thrust: 0.45, roll: 0, pitch: 0, yawRate: 0 };

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (!initialized) {
        initialized = true;
        vfX = v.x; vfY = v.y; vfZ = v.z;
        refX = p.x; refY = p.y > 0.35 ? p.y : 0.35; refZ = p.z;
        if (obs.nextGate) {
          activeGateIdx = obs.gateIndex;
          const gc = obs.nextGate.centre;
          activeGateX = gc.x; activeGateY = gc.y; activeGateZ = gc.z;
        }
      }

      // 1. Filter velocity & estimate acceleration
      const prevVx = vfX, prevVy = vfY, prevVz = vfZ;
      vfX += 0.38 * (v.x - vfX);
      vfY += 0.38 * (v.y - vfY);
      vfZ += 0.38 * (v.z - vfZ);

      const axMeas = (vfX - prevVx) * 100;
      const ayMeas = (vfY - prevVy) * 100;
      const azMeas = (vfZ - prevVz) * 100;

      aFiltX += 0.35 * (axMeas - aFiltX);
      aFiltY += 0.35 * (ayMeas - aFiltY);
      aFiltZ += 0.35 * (azMeas - aFiltZ);

      // 2. Fast/slow acceleration disturbance observer
      thrustLag += (lastThrust - thrustLag) * motorAlpha;
      const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
      const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyX = sp * cr, bodyZ = -sr;
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = cp * cr;
      const dirZ = -sy * bodyX + cy * bodyZ;

      const axPred = kThrust * thrustLag * dirX - kDrag * vfX;
      const ayPred = kThrust * thrustLag * dirY - kDrag * vfY - 9.81;
      const azPred = kThrust * thrustLag * dirZ - kDrag * vfZ;

      if (p.y > 0.095 && obs.t > 0.06) {
        const rx = clamp(axMeas - axPred, -7, 7);
        const ry = clamp(ayMeas - ayPred, -6, 6);
        const rz = clamp(azMeas - azPred, -7, 7);

        distFastX += 0.18 * (rx - distFastX);
        distFastY += 0.18 * (ry - distFastY);
        distFastZ += 0.18 * (rz - distFastZ);

        distSlowX += 0.025 * (rx - distSlowX);
        distSlowY += 0.025 * (ry - distSlowY);
        distSlowZ += 0.025 * (rz - distSlowZ);
      }

      // 5 & 6. Gate navigation with gate exit lock and predictive cross abort
      let targetX = 0, targetY = 0, targetZ = 0;

      if (activeGateIdx < 3 && obs.gateIndex > activeGateIdx) {
        if (p.x >= activeGateX + 0.18) {
          activeGateIdx = obs.gateIndex;
          if (activeGateIdx < 3 && obs.nextGate) {
            const gc = obs.nextGate.centre;
            activeGateX = gc.x; activeGateY = gc.y; activeGateZ = gc.z;
            phase = 0;
            settle = 0;
            intX = 0; intZ = 0;
          } else {
            phase = 3;
            padY = clamp(p.y, 0.55, 1.05);
            intX = 0; intZ = 0;
          }
        } else {
          phase = 2; // exit_lock
        }
      }

      if (activeGateIdx < 3) {
        const alignX = activeGateX - 0.42;
        const exitX = activeGateX + 0.35;
        targetY = activeGateY;
        targetZ = activeGateZ;

        if (phase === 0) {
          targetX = alignX;
          const ex = Math.abs(targetX - p.x);
          const ey = Math.abs(activeGateY - p.y);
          const ez = Math.abs(activeGateZ - p.z);
          const spd = Math.hypot(vfX, vfY, vfZ);
          const gustTransient = Math.hypot(distFastY - distSlowY, distFastZ - distSlowZ);
          if (ex < 0.09 && ey < 0.065 && ez < 0.048 && spd < 0.25 && gustTransient < 0.9) {
            settle++;
          } else if (settle > 0) {
            settle--;
          }
          if (settle >= 12) {
            phase = 1;
            settle = 0;
          }
        } else if (phase === 1) {
          targetX = exitX;
          if (p.x >= activeGateX - 0.08) {
            phase = 2;
          } else {
            const dx = activeGateX - p.x;
            const tCross = clamp(dx / Math.max(vfX, 0.35), 0.06, 0.32);
            const yPred = p.y + vfY * tCross + 0.5 * aFiltY * tCross * tCross;
            const zPred = p.z + vfZ * tCross + 0.5 * aFiltZ * tCross * tCross;
            const eyNow = Math.abs(p.y - activeGateY), ezNow = Math.abs(p.z - activeGateZ);
            const eyPred = Math.abs(yPred - activeGateY), ezPred = Math.abs(zPred - activeGateZ);
            const canStop = p.x + Math.max(0, vfX) * 0.09 < activeGateX - 0.03;

            if (canStop && (ezNow > 0.082 || ezPred > 0.088 || eyNow > 0.15 || eyPred > 0.165)) {
              phase = 0;
              settle = 0;
              targetX = alignX;
              if (refX > p.x - 0.12) refX = p.x - 0.12;
            }
          }
        } else {
          targetX = exitX;
        }
      } else {
        // 7. Ramp pad descent
        const pad = obs.pad.centre;
        const distXZ = Math.hypot(pad.x - p.x, pad.z - p.z);
        const spdXZ = Math.hypot(vfX, vfZ);
        const gustXZ = Math.hypot(distFastX - distSlowX, distFastZ - distSlowZ);

        if (distXZ > 0.25) {
          padY = Math.max(0.45, padY - 0.40 * dt);
        } else if (distXZ < 0.085 && spdXZ < 0.24 && (p.y > 0.22 || gustXZ < 0.9)) {
          const rate = p.y < 0.22 ? 0.26 : 0.48;
          padY = Math.max(0.015, padY - rate * dt);
        } else if (distXZ < 0.15 && spdXZ < 0.32) {
          padY = Math.max(0.14, padY - 0.22 * dt);
        } else if (p.y < 0.18 && p.y > 0.085 && distXZ > 0.12) {
          padY = Math.min(0.22, padY + 0.25 * dt);
        }
        targetX = pad.x;
        targetY = padY;
        targetZ = pad.z;
      }

      // 4. Smooth reference filter
      const inCross = phase === 1 || phase === 2;
      const maxStepX = (inCross ? 1.6 : 1.75) * dt;
      const maxStepY = 1.4 * dt;
      const maxStepZ = 1.6 * dt;
      const alphaRef = inCross ? 0.14 : 0.095;
      refX += clamp(alphaRef * (targetX - refX), -maxStepX, maxStepX);
      refY += clamp(alphaRef * (targetY - refY), -maxStepY, maxStepY);
      refZ += clamp(alphaRef * (targetZ - refZ), -maxStepZ, maxStepZ);

      // 3. Yaw + local pos integrals
      const exTarget = targetX - p.x;
      const eyTarget = targetY - p.y;
      const ezTarget = targetZ - p.z;
      const distHoriz = Math.hypot(exTarget, ezTarget);

      if (distHoriz < 0.38 && p.y > 0.085) {
        intX = clamp(intX + exTarget * dt, -0.35, 0.35);
        intZ = clamp(intZ + ezTarget * dt, -0.35, 0.35);
      } else {
        intX *= 0.985;
        intZ *= 0.985;
      }

      if (Math.abs(eyTarget) < 0.40 && p.y > 0.08) {
        intY = clamp(intY + eyTarget * dt, -0.45, 0.65);
      }

      intYaw = clamp(intYaw + att.yaw * dt, -0.6, 0.6);

      // Control law with disturbance rejection
      const distX = 0.68 * distFastX + 0.32 * distSlowX;
      const distY = 0.35 * distFastY + 0.65 * distSlowY;
      const distZ = 0.68 * distFastZ + 0.32 * distSlowZ;

      const exRef = refX - p.x;
      const eyRef = refY - p.y;
      const ezRef = refZ - p.z;

      const axCmd = clamp(5.2 * exRef - 3.8 * vfX + 1.8 * intX + kDrag * vfX - distX, -4.2, 4.2);
      const azCmd = clamp(5.6 * ezRef - 4.0 * vfZ + 2.0 * intZ + kDrag * vfZ - distZ, -4.2, 4.2);
      const ayCmd = clamp(6.2 * eyRef - 4.2 * vfY + 2.2 * intY + kDrag * vfY - distY, -4.0, 5.0);

      let maxTilt = 0.45;
      if (activeGateIdx === 3) {
        if (p.y <= 0.105) maxTilt = 0.15;
        else if (p.y < 0.18) maxTilt = 0.28;
      }

      const denom = 9.81 + clamp(ayCmd, -3.0, 3.0);
      let pitch = clamp((cy * axCmd - sy * azCmd) / denom, -maxTilt, maxTilt);
      let roll = clamp(-(sy * axCmd + cy * azCmd) / denom, -maxTilt, maxTilt);
      let thrust = clamp((9.81 + ayCmd) / (kThrust * Math.max(0.7, cr * cp)), 0, 1);

      if (activeGateIdx === 3) {
        const pad = obs.pad.centre;
        const distXZ = Math.hypot(pad.x - p.x, pad.z - p.z);
        if (distXZ < 0.11 && p.y <= 0.082 && padY <= 0.04 && Math.hypot(vfX, vfY, vfZ) < 0.35) {
          if (thrust > 0.15) thrust = 0.15;
          pitch = clamp(pitch, -0.12, 0.12);
          roll = clamp(roll, -0.12, 0.12);
        }
      }

      out.thrust = thrust;
      out.roll = roll;
      out.pitch = pitch;
      out.yawRate = clamp(-2.8 * att.yaw - 1.8 * intYaw - 0.15 * rates.yaw, -2, 2);
      lastThrust = thrust;

      return out;
    },
  };
}
