export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01;
  const g = 9.81;
  const mNom = 0.725;
  const fNom = 15.4;
  const dragCoeff = 0.42 / mNom;

  let initialized = false;
  let vf = { x: 0, y: 0, z: 0 };
  let prevVf = { x: 0, y: 0, z: 0 };
  let dFast = { x: 0, y: 0, z: 0 };
  let dSlow = { x: 0, y: 0, z: 0 };
  let motorEst = 0.45;
  let lastThrust = 0.45;

  let intYaw = 0;
  let intPos = { x: 0, y: 0, z: 0 };
  let ref = { x: 0, y: 0, z: 0 };

  let activeGateIdx = 0;
  let activeGate = null;
  let phase = 'align';
  let settle = 0;
  let padY = 0.65;

  return {
    step(obs) {
      const p = obs.pos;
      const v = obs.vel;
      const att = obs.att;
      const rates = obs.rates;

      if (!initialized) {
        initialized = true;
        vf.x = v.x; vf.y = v.y; vf.z = v.z;
        prevVf.x = v.x; prevVf.y = v.y; prevVf.z = v.z;
        ref.x = p.x; ref.y = Math.max(0.35, p.y); ref.z = p.z;
        if (obs.nextGate) {
          activeGate = { x: obs.nextGate.centre.x, y: obs.nextGate.centre.y, z: obs.nextGate.centre.z };
        }
      }

      // 1. Filter velocity
      const alphaV = 0.38;
      vf.x += alphaV * (v.x - vf.x);
      vf.y += alphaV * (v.y - vf.y);
      vf.z += alphaV * (v.z - vf.z);

      // 2. Fast/slow accel disturbance observer
      motorEst += (lastThrust - motorEst) * (1 - Math.exp(-dt / 0.04));
      if (obs.t > 0.06 && p.y > 0.095) {
        const aMeasX = (vf.x - prevVf.x) / dt;
        const aMeasY = (vf.y - prevVf.y) / dt;
        const aMeasZ = (vf.z - prevVf.z) / dt;

        const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
        const bodyZ = -Math.sin(att.roll);
        const dirX = Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ;
        const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
        const dirZ = -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ;

        const aThrust = (fNom / mNom) * motorEst;
        const aExpX = aThrust * dirX - dragCoeff * vf.x;
        const aExpY = aThrust * dirY - g - dragCoeff * vf.y;
        const aExpZ = aThrust * dirZ - dragCoeff * vf.z;

        const rawX = clamp(aMeasX - aExpX, -9, 9);
        const rawY = clamp(aMeasY - aExpY, -9, 9);
        const rawZ = clamp(aMeasZ - aExpZ, -9, 9);

        const alphaFast = 0.24;
        const alphaSlow = 0.022;
        dFast.x += alphaFast * (rawX - dFast.x);
        dFast.y += alphaFast * (rawY - dFast.y);
        dFast.z += alphaFast * (rawZ - dFast.z);

        dSlow.x += alphaSlow * (rawX - dSlow.x);
        dSlow.y += alphaSlow * (rawY - dSlow.y);
        dSlow.z += alphaSlow * (rawZ - dSlow.z);
      }
      prevVf.x = vf.x; prevVf.y = vf.y; prevVf.z = vf.z;

      // 5 & 6 & 7. Navigation with gate exit lock, predictive cross abort, ramp pad descent
      if (phase === 'exit' && activeGate) {
        if (p.x > activeGate.x + 0.22 || obs.gateIndex === activeGateIdx) {
          if (obs.gateIndex > activeGateIdx) {
            activeGateIdx = obs.gateIndex;
            if (obs.nextGate) {
              activeGate = { x: obs.nextGate.centre.x, y: obs.nextGate.centre.y, z: obs.nextGate.centre.z };
              phase = 'align';
              settle = 0;
            } else {
              activeGate = null;
              phase = 'padAlign';
              padY = 0.60;
              settle = 0;
            }
          } else if (p.x < activeGate.x - 0.18) {
            phase = 'align';
            settle = 0;
          }
        }
      } else if (obs.gateIndex > activeGateIdx) {
        // Gate just crossed: enter gate exit lock
        phase = 'exit';
      }

      let target = { x: p.x, y: 1.0, z: p.z };
      const alignOffset = 0.42;

      if (phase === 'exit' && activeGate) {
        target = { x: activeGate.x + 0.36, y: activeGate.y, z: activeGate.z };
      } else if (activeGateIdx < 3 && activeGate) {
        const gx = activeGate.x, gy = activeGate.y, gz = activeGate.z;
        if (phase === 'align') {
          // If drone was blown past or near gate plane while unaligned, stay clear in y/z first
          if (p.x > gx - 0.12 && (Math.abs(p.y - gy) > 0.16 || Math.abs(p.z - gz) > 0.09)) {
            target = { x: gx - alignOffset, y: p.y, z: p.z };
          } else {
            target = { x: gx - alignOffset, y: gy, z: gz };
          }
          const ex = Math.abs((gx - alignOffset) - p.x);
          const ey = Math.abs(gy - p.y);
          const ez = Math.abs(gz - p.z);
          const gustTransient = Math.hypot(dFast.y - dSlow.y, dFast.z - dSlow.z);
          if (p.x < gx - 0.20 && ex < 0.13 && ey < 0.06 && ez < 0.042 &&
              Math.abs(vf.y) < 0.18 && Math.abs(vf.z) < 0.14 && gustTransient < 1.6) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= 9) {
            phase = 'cross';
            settle = 0;
          }
        }
        if (phase === 'cross') {
          const lockDist = 0.11;
          if (p.x >= gx - lockDist) {
            // Committed to gate exit lock
            phase = 'exit';
            target = { x: gx + 0.36, y: gy, z: gz };
          } else {
            // Predictive cross abort check
            const vxCross = Math.max(0.25, vf.x);
            const tToGate = clamp((gx - p.x) / vxCross, 0.04, 0.40);
            const predY = p.y + vf.y * tToGate + 0.25 * (dFast.y - dSlow.y) * tToGate * tToGate;
            const predZ = p.z + vf.z * tToGate + 0.25 * (dFast.z - dSlow.z) * tToGate * tToGate;
            const curEy = Math.abs(p.y - gy);
            const curEz = Math.abs(p.z - gz);
            if (Math.abs(predY - gy) > 0.135 || Math.abs(predZ - gz) > 0.072 || curEy > 0.115 || curEz > 0.062) {
              // Abort cross!
              phase = 'align';
              settle = 0;
              target = { x: gx - alignOffset, y: gy, z: gz };
              ref.x = Math.min(ref.x, gx - 0.28);
            } else {
              target = { x: gx + 0.36, y: gy, z: gz };
            }
          }
        }
      } else {
        // Pad approach and ramp pad descent
        const px = obs.pad.centre.x, pz = obs.pad.centre.z;
        const hErr = Math.hypot(px - p.x, pz - p.z);
        const hSpd = Math.hypot(vf.x, vf.z);
        if (phase !== 'descend') {
          target = { x: px, y: padY, z: pz };
          if (hErr < 0.18 && hSpd < 0.35 && Math.abs(p.y - padY) < 0.25) {
            phase = 'descend';
          }
        }
        if (phase === 'descend') {
          if (p.y < 0.22 && hErr > 0.125) {
            // Hold altitude briefly if pushed near edge of landing radius right above floor
            padY = Math.max(padY, 0.13);
          } else if (padY > 0.20) {
            padY -= 0.52 * dt;
          } else {
            padY = Math.max(0.01, padY - 0.22 * dt);
          }
          target = { x: px, y: padY, z: pz };
        }
      }

      // 4. Smooth ref filter
      const maxStepX = (phase === 'cross' || phase === 'exit') ? 1.8 * dt : 1.55 * dt;
      const maxStepY = (phase === 'descend') ? 0.8 * dt : 1.35 * dt;
      const maxStepZ = 1.55 * dt;
      const alphaRef = (phase === 'cross' || phase === 'exit') ? 0.16 : 0.11;

      const dRefX = clamp(alphaRef * (target.x - ref.x), -maxStepX, maxStepX);
      const dRefY = clamp(alphaRef * (target.y - ref.y), -maxStepY, maxStepY);
      const dRefZ = clamp(alphaRef * (target.z - ref.z), -maxStepZ, maxStepZ);

      ref.x += dRefX;
      ref.y += dRefY;
      ref.z += dRefZ;

      const vRefX = dRefX / dt;
      const vRefY = dRefY / dt;
      const vRefZ = dRefZ / dt;

      // 3. Yaw + local pos integrals
      intYaw = clamp(intYaw + att.yaw * dt, -0.6, 0.6);

      const distToTargetH = Math.hypot(target.x - p.x, target.z - p.z);
      if (distToTargetH < 0.32 && phase !== 'cross' && phase !== 'exit') {
        intPos.x = clamp(intPos.x + (target.x - p.x) * dt, -0.35, 0.35);
        intPos.z = clamp(intPos.z + (target.z - p.z) * dt, -0.35, 0.35);
      } else {
        intPos.x *= 0.96;
        intPos.z *= 0.96;
      }

      if (Math.abs(target.y - p.y) < 0.35 && phase !== 'descend') {
        intPos.y = clamp(intPos.y + (target.y - p.y) * dt, -0.4, 0.4);
      } else {
        intPos.y *= 0.98;
      }

      // Position PD + local integral + disturbance observer compensation
      const inGateLine = (phase === 'cross' || phase === 'exit');
      const kpX = inGateLine ? 4.8 : 4.4;
      const kdX = inGateLine ? 3.6 : 3.4;
      const kpZ = inGateLine ? 5.6 : 4.6;
      const kdZ = inGateLine ? 4.0 : 3.5;

      const exRef = ref.x - p.x;
      const eyRef = ref.y - p.y;
      const ezRef = ref.z - p.z;

      const distX = dSlow.x + 0.85 * (dFast.x - dSlow.x);
      const distY = 0.45 * dFast.y + 0.55 * dSlow.y;
      const distZ = dSlow.z + 0.85 * (dFast.z - dSlow.z);

      const ax = clamp(
        kpX * exRef + kdX * (0.55 * vRefX - vf.x) + 1.6 * intPos.x + dragCoeff * vf.x - distX,
        -4.2, 4.2
      );
      const az = clamp(
        kpZ * ezRef + kdZ * (0.55 * vRefZ - vf.z) + 1.6 * intPos.z + dragCoeff * vf.z - distZ,
        -4.2, 4.2
      );
      const ay = clamp(
        6.0 * eyRef + 3.8 * (0.6 * vRefY - vf.y) + 2.0 * intPos.y + dragCoeff * vf.y - distY,
        -4.0, 5.0
      );

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const totalUp = Math.max(4.0, g + ay);
      let pitch = clamp(Math.atan2(cy * ax - sy * az, totalUp), -0.48, 0.48);
      let roll = clamp(Math.atan2(-(sy * ax + cy * az), totalUp), -0.48, 0.48);

      const tiltCos = Math.max(0.65, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = clamp((mNom * (g + ay)) / (fNom * tiltCos), 0, 1);

      // Soft touchdown hold on pad
      if (phase === 'descend' && p.y <= 0.088 && Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z) <= 0.125) {
        roll = clamp(roll, -0.11, 0.11);
        pitch = clamp(pitch, -0.11, 0.11);
        if (p.y <= 0.076 && padY <= 0.03) {
          thrust = Math.min(thrust, 0.18);
        }
      }

      const yawRate = clamp(-3.2 * att.yaw - 1.8 * intYaw - 0.25 * rates.yaw, -2, 2);
      lastThrust = thrust;
      return { thrust, roll, pitch, yawRate };
    },
  };
}
