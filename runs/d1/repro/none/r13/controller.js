export default function makeController() {
  let targetIndex = -1, phase = 'climb';
  let ix = 0, iy = 0, iz = 0;
  let vxEst = 0, vyEst = 0, vzEst = 0;
  let axDist = 0, ayDist = 0, azDist = 0;
  let lastCmdAx = 0, lastCmdAy = 0, lastCmdAz = 0;
  let settle = 0, landTimer = 0, postGateClear = 0;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  const GATES_KNOWN = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att;
      const dt = 0.01;

      if (obs.t === 0) {
        vxEst = v.x; vyEst = v.y; vzEst = v.z;
      } else {
        const rawAx = (v.x - vxEst) / dt;
        const rawAy = (v.y - vyEst) / dt;
        const rawAz = (v.z - vzEst) / dt;
        vxEst = v.x; vyEst = v.y; vzEst = v.z;

        const alpha = 0.13;
        axDist += alpha * (clamp(rawAx - lastCmdAx, -16, 16) - axDist);
        ayDist += alpha * (clamp(rawAy - lastCmdAy, -16, 16) - ayDist);
        azDist += alpha * (clamp(rawAz - lastCmdAz, -16, 16) - azDist);
      }

      let target, targetVel = { x: 0, y: 0, z: 0 };

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          targetIndex = obs.gateIndex;
          phase = obs.gateIndex > 0 ? 'clear_prev' : 'align';
          settle = 0;
          postGateClear = 0;
        }

        if (phase === 'clear_prev') {
          const prevG = GATES_KNOWN[obs.gateIndex - 1];
          postGateClear++;
          const safeToLeave = p.x >= prevG.x + 0.20 && (p.x >= prevG.x + 0.26 || v.x > 0.20);
          if (!safeToLeave && postGateClear < 35) {
            target = { x: prevG.x + 0.35, y: prevG.y, z: prevG.z };
            targetVel = { x: 0.85, y: 0, z: 0 };
          } else {
            phase = 'align';
          }
        }

        const dy = Math.abs(p.y - gate.y);
        const dz = Math.abs(p.z - gate.z);

        if (phase !== 'clear_prev') {
          if (p.x > gate.x - 0.04 && phase !== 'cross') {
            phase = 'recover';
          }
          if (phase === 'recover') {
            const safeZ = gate.z + (p.z >= gate.z ? 0.50 : -0.50);
            if (p.x > gate.x - 0.36) {
              target = { x: gate.x - 0.56, y: gate.y + 0.36, z: clamp(safeZ, -2.6, 2.6) };
            } else {
              phase = 'align';
              settle = 0;
            }
          }

          if (phase === 'align') {
            const pullBack = clamp(Math.max(dy - 0.10, dz - 0.08, 0) * 1.15, 0, 0.38);
            const approachX = gate.x - 0.35 - pullBack;
            target = { x: approachX, y: gate.y, z: gate.z };
            const alignedYZ = dy < 0.058 && dz < 0.046 && Math.abs(v.y) < 0.28 && Math.abs(v.z) < 0.25;
            const nearX = p.x > gate.x - 0.48 && p.x < gate.x - 0.14 && v.x > -0.18 && v.x < 0.65;
            if (alignedYZ && nearX) settle++;
            else settle = Math.max(0, settle - 1);
            if (settle >= 4) phase = 'cross';
          }

          if (phase === 'cross') {
            const distToPlane = gate.x - p.x;
            const speedX = Math.max(0.45, v.x);
            const tCross = clamp(distToPlane / speedX, 0, 0.30);
            const predY = p.y + v.y * tCross;
            const predZ = p.z + v.z * tCross;
            if (distToPlane > 0.035 && (dy > 0.12 || dz > 0.068 || Math.abs(predY - gate.y) > 0.135 || Math.abs(predZ - gate.z) > 0.076)) {
              phase = 'align';
              settle = 0;
              target = { x: gate.x - 0.42, y: gate.y, z: gate.z };
              targetVel = { x: -0.25, y: 0, z: 0 };
            } else {
              target = { x: gate.x + 0.35, y: gate.y, z: gate.z };
              targetVel = { x: 0.95, y: 0, z: 0 };
            }
          }
        }
      } else {
        const pad = obs.pad.centre;
        const prevG = GATES_KNOWN[2];
        if (targetIndex !== 3) {
          targetIndex = 3;
          phase = 'clear_gate3';
          postGateClear = 0;
        }
        if (phase === 'clear_gate3') {
          postGateClear++;
          const safeToLeave = p.x >= prevG.x + 0.20 && (p.x >= prevG.x + 0.26 || v.x > 0.20);
          if (!safeToLeave && postGateClear < 35) {
            target = { x: prevG.x + 0.35, y: prevG.y, z: prevG.z };
            targetVel = { x: 0.85, y: 0, z: 0 };
          } else {
            phase = 'pad_approach';
          }
        }

        const distXZ = Math.hypot(pad.x - p.x, pad.z - p.z);
        const speedXZ = Math.hypot(v.x, v.z);
        if (phase === 'pad_approach') {
          target = { x: pad.x, y: 0.50, z: pad.z };
          if (distXZ < 0.15 && speedXZ < 0.38 && p.y < 0.80) {
            phase = 'descend';
          }
        }
        if (phase === 'descend') {
          const targetY = distXZ < 0.125 ? 0.035 : 0.20;
          target = { x: pad.x, y: targetY, z: pad.z };
          if (p.y <= 0.092 && distXZ <= 0.140 && speedXZ < 0.42 && Math.abs(v.y) < 0.38) {
            phase = 'touchdown';
            landTimer = 0;
          } else if (distXZ > 0.23 && p.y > 0.15) {
            phase = 'pad_approach';
          }
        }
        if (phase === 'touchdown') {
          landTimer++;
          target = { x: pad.x, y: 0.02, z: pad.z };
          if (distXZ > 0.146 || p.y > 0.11) {
            phase = 'descend';
          } else if (landTimer > 1) {
            return { thrust: 0.06, roll: 0, pitch: 0, yawRate: clamp(-2 * att.yaw, -2, 2) };
          }
        }
      }

      const ex = target.x - p.x;
      const ey = target.y - p.y;
      const ez = target.z - p.z;

      if (Math.abs(ex) < 0.6) ix = clamp(ix + ex * dt, -0.8, 0.8);
      if (Math.abs(ey) < 0.8) iy = clamp(iy + ey * dt, -1.2, 1.8);
      if (Math.abs(ez) < 0.6) iz = clamp(iz + ez * dt, -0.8, 0.8);

      const isGate = obs.gateIndex < 3;
      const kpZ = isGate ? 8.2 : 5.2;
      const kdZ = isGate ? 5.2 : 3.9;
      const kpY = isGate ? 7.8 : 6.0;
      const kdY = isGate ? 4.8 : 3.9;

      const maxA = phase === 'descend' && p.y < 0.25 ? 2.4 : 4.3;
      let desAx = 5.2 * ex + 3.9 * (targetVel.x - v.x) + 2.5 * ix - 0.82 * axDist;
      let desAz = kpZ * ez + kdZ * (targetVel.z - v.z) + 3.0 * iz - 0.88 * azDist;
      let desAy = kpY * ey + kdY * (targetVel.y - v.y) + 3.4 * iy - 0.68 * ayDist;

      if (phase === 'descend' && p.y < 0.30) {
        const maxSink = -0.36;
        if (v.y < maxSink) desAy += 8.0 * (maxSink - v.y);
      }

      desAx = clamp(desAx, -maxA, maxA);
      desAz = clamp(desAz, -maxA, maxA);
      desAy = clamp(desAy, -4.4, 5.8);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const totalUp = Math.max(2.0, 9.81 + desAy);
      const bodyAx = cy * desAx - sy * desAz;
      const bodyAz = sy * desAx + cy * desAz;

      const maxTilt = phase === 'descend' && p.y < 0.18 ? 0.16 : 0.52;
      const pitch = clamp(Math.atan2(bodyAx, totalUp), -maxTilt, maxTilt);
      const roll = clamp(-Math.atan2(bodyAz, totalUp), -maxTilt, maxTilt);

      const curTiltCos = Math.max(0.7, Math.cos(att.roll) * Math.cos(att.pitch));
      const thrust = clamp((0.725 * totalUp) / (16 * curTiltCos), 0.05, 1.0);

      const bX = Math.sin(att.pitch) * Math.cos(att.roll), bZ = -Math.sin(att.roll);
      const dirX = cy * bX + sy * bZ;
      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
      const dirZ = -sy * bX + cy * bZ;
      const estAccMag = (thrust * 16) / 0.725;
      lastCmdAx = estAccMag * dirX - (0.42 / 0.725) * v.x;
      lastCmdAy = estAccMag * dirY - 9.81 - (0.42 / 0.725) * v.y;
      lastCmdAz = estAccMag * dirZ - (0.42 / 0.725) * v.z;

      return { thrust, roll, pitch, yawRate: clamp(-2.5 * att.yaw, -2, 2) };
    },
  };
}
