// Adaptive flight controller with disturbance observer, anti-windup local integral,
// post-gate departure lock, and predictive gate corridor safety checks.
export default function makeController() {
  let targetIndex = -1, phase = 'align', settle = 0;
  const estMass = 0.725;
  let distX = 0, distY = 0, distZ = 0;
  let fastDistX = 0, fastDistZ = 0;
  let ix = 0, iy = 0, iz = 0, iyaw = 0;
  let prevVel = null, prevAccCmd = { x: 0, y: 9.81, z: 0 };
  let posF = null, velF = null;

  const GATES = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];

  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  return {
    step(obs) {
      const dt = 0.01;
      const pRaw = obs.pos, vRaw = obs.vel, att = obs.att, rates = obs.rates;

      if (!posF) {
        posF = { x: pRaw.x, y: pRaw.y, z: pRaw.z };
        velF = { x: vRaw.x, y: vRaw.y, z: vRaw.z };
      } else {
        posF.x = 0.75 * pRaw.x + 0.25 * (posF.x + velF.x * dt);
        posF.y = 0.75 * pRaw.y + 0.25 * (posF.y + velF.y * dt);
        posF.z = 0.75 * pRaw.z + 0.25 * (posF.z + velF.z * dt);
        velF.x = 0.60 * vRaw.x + 0.40 * velF.x;
        velF.y = 0.60 * vRaw.y + 0.40 * velF.y;
        velF.z = 0.60 * vRaw.z + 0.40 * velF.z;
      }

      const p = posF, v = velF;

      if (prevVel && p.y > 0.12) {
        const axMeas = (v.x - prevVel.x) / dt;
        const ayMeas = (v.y - prevVel.y) / dt;
        const azMeas = (v.z - prevVel.z) / dt;

        const errAx = axMeas - prevAccCmd.x;
        const errAy = ayMeas - (prevAccCmd.y - 9.81);
        const errAz = azMeas - prevAccCmd.z;

        distX = clamp(distX + 0.035 * (errAx - distX), -5.0, 5.0);
        distY = clamp(distY + 0.040 * (errAy - distY), -4.5, 4.5);
        distZ = clamp(distZ + 0.035 * (errAz - distZ), -5.0, 5.0);

        fastDistX = clamp(fastDistX + 0.14 * (errAx - fastDistX), -6.5, 6.5);
        fastDistZ = clamp(fastDistZ + 0.14 * (errAz - fastDistZ), -6.5, 6.5);
      }
      prevVel = { x: v.x, y: v.y, z: v.z };

      let target;
      let kpX = 4.2, kdX = 3.4;
      let kpZ = 4.8, kdZ = 3.8;
      let kpY = 6.2, kdY = 4.0;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          targetIndex = obs.gateIndex;
          phase = 'align';
          settle = 0;
          ix = 0; iz = 0;
        }

        const dy = p.y - gate.y;
        const dz = p.z - gate.z;

        if (phase === 'align') {
          target = { x: gate.x - 0.42, y: gate.y, z: gate.z };
          // If drone is near or past the current gate plane while unaligned, climb over safely
          if (p.x > gate.x - 0.12 && (Math.abs(dy) > 0.16 || Math.abs(dz) > 0.085)) {
            target = { x: gate.x - 0.50, y: Math.min(2.4, gate.y + 0.65), z: gate.z };
          }
          // Prevent backing into or clipping the previous gate frame if near or behind its plane
          if (obs.gateIndex > 0) {
            const prevGate = GATES[obs.gateIndex - 1];
            if (p.x < prevGate.x + 0.24) {
              target = { x: prevGate.x + 0.40, y: prevGate.y, z: prevGate.z };
              kpX = 5.2; kdX = 3.6;
              kpZ = 7.8; kdZ = 5.2;
              kpY = 7.5; kdY = 4.6;
            }
          }

          const distAlign = Math.hypot(target.x - p.x, dy, dz);
          const speed = Math.hypot(v.x, v.y, v.z);
          if (
            distAlign < 0.13 &&
            speed < 0.42 &&
            Math.abs(dy) < 0.08 &&
            Math.abs(dz) < 0.052 &&
            Math.abs(v.z) < 0.17 &&
            Math.abs(v.y) < 0.20
          ) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= 6) {
            phase = 'cross';
            ix = 0; iz = 0;
          }
        }

        if (phase === 'cross') {
          const dxGate = gate.x - p.x;
          const tCross = clamp(dxGate / Math.max(0.25, v.x), 0.0, 0.35);
          const predY = p.y + v.y * tCross - gate.y;
          const predZ = p.z + v.z * tCross - gate.z;

          const unsafeNow = Math.abs(dy) > 0.14 || Math.abs(dz) > 0.070;
          const unsafePred = Math.abs(predY) > 0.15 || Math.abs(predZ) > 0.075;

          // Only abort when far enough in front of the gate to stop before crossing the plane
          if (dxGate > 0.11 && (unsafeNow || unsafePred)) {
            phase = 'align';
            settle = 0;
            ix = 0; iz = 0;
            target = { x: gate.x - 0.42, y: gate.y, z: gate.z };
            kpX = 5.2; kdX = 4.4;
            kpZ = 7.8; kdZ = 5.2;
          } else {
            target = { x: gate.x + 0.42, y: gate.y, z: gate.z };
            kpZ = 8.0; kdZ = 5.2;
            kpY = 7.5; kdY = 4.6;
            kpX = 3.8; kdX = 3.2;
          }
        }
      } else {
        const pad = obs.pad.centre;
        if (targetIndex !== 3) {
          targetIndex = 3;
          phase = 'approach';
          settle = 0;
          ix = 0; iz = 0;
        }
        const horizErr = Math.hypot(pad.x - p.x, pad.z - p.z);
        const horizVel = Math.hypot(v.x, v.z);

        if (phase === 'approach') {
          target = { x: pad.x, y: 0.48, z: pad.z };
          if (p.x < GATES[2].x + 0.22) {
            target = { x: GATES[2].x + 0.40, y: GATES[2].y, z: GATES[2].z };
            kpZ = 7.8; kdZ = 5.2;
            kpY = 7.5; kdY = 4.6;
          }
          if (horizErr < 0.12 && horizVel < 0.28 && p.y < 0.65) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= 5 || obs.t > 21.5) {
            phase = 'descend';
            ix = 0; iz = 0;
          }
        }

        if (phase === 'descend') {
          target = { x: pad.x, y: 0.02, z: pad.z };
          if (horizErr > 0.14 && obs.t < 23.2) {
            target.y = Math.max(p.y, 0.22);
          } else {
            const maxDescendStep = p.y > 0.18 ? 0.16 : 0.08;
            target.y = Math.max(0.015, p.y - maxDescendStep);
          }
          kpX = 4.8; kdX = 3.8;
          kpZ = 4.8; kdZ = 3.8;
        }
      }

      const ex = target.x - p.x;
      const ey = target.y - p.y;
      const ez = target.z - p.z;

      if (Math.abs(ex) < 0.25 && Math.abs(v.x) < 0.35) ix = clamp(ix + ex * dt, -0.35, 0.35);
      else ix *= 0.95;
      if (Math.abs(ez) < 0.25 && Math.abs(v.z) < 0.35) iz = clamp(iz + ez * dt, -0.35, 0.35);
      else iz *= 0.95;
      if (Math.abs(ey) < 0.25 && Math.abs(v.y) < 0.35) iy = clamp(iy + ey * dt, -0.35, 0.35);
      else iy *= 0.95;

      const compX = 0.55 * distX + 0.45 * fastDistX;
      const compZ = 0.55 * distZ + 0.45 * fastDistZ;
      const compY = distY;

      const dragCompX = (0.42 / estMass) * v.x;
      const dragCompZ = (0.42 / estMass) * v.z;

      const ax = clamp(kpX * ex - kdX * v.x + 1.5 * ix - compX + 0.5 * dragCompX, -4.2, 4.2);
      const az = clamp(kpZ * ez - kdZ * v.z + 1.5 * iz - compZ + 0.5 * dragCompZ, -4.2, 4.2);
      const ay = clamp(kpY * ey - kdY * v.y + 1.5 * iy - compY, -4.0, 5.2);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const axBody = cy * ax - sy * az;
      const azBody = sy * ax + cy * az;

      const gEff = Math.max(5.5, 9.81 + ay);
      let pitch = clamp(Math.atan2(axBody, gEff), -0.50, 0.50);
      let roll = clamp(Math.atan2(-azBody, gEff), -0.50, 0.50);

      const tiltCos = Math.max(0.65, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = (estMass * (9.81 + ay)) / (16 * tiltCos);

      if (obs.gateIndex === 3 && phase === 'descend' && p.y <= 0.092) {
        const padErr = Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z);
        if (padErr < 0.135) {
          pitch = clamp(pitch * 0.45, -0.12, 0.12);
          roll = clamp(roll * 0.45, -0.12, 0.12);
          if (p.y <= 0.078 && Math.hypot(v.x, v.z) < 0.35) {
            thrust = 0.05;
          } else {
            thrust = Math.min(thrust, 0.32);
          }
        }
      }

      thrust = clamp(thrust, 0, 1);

      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
      const dirZ = -sy * bodyX + cy * bodyZ;
      const totalForce = thrust * 16;
      prevAccCmd = {
        x: (totalForce * dirX - 0.42 * v.x) / estMass,
        y: (totalForce * dirY - 0.42 * v.y) / estMass,
        z: (totalForce * dirZ - 0.42 * v.z) / estMass,
      };

      iyaw = clamp(iyaw - att.yaw * dt, -0.8, 0.8);
      const yawRate = clamp(-3.5 * att.yaw - 0.3 * rates.yaw + 2.5 * iyaw, -2, 2);

      return { thrust, roll, pitch, yawRate };
    },
  };
}
