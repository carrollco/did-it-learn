export default function makeController() {
  let targetIndex = -1, phase = 'align', settle = 0;
  let distX = 0, distY = 0, distZ = 0;
  let intX = 0, intY = 0, intZ = 0;
  let lastPos = null, estV = { x: 0, y: 0, z: 0 }, estA = { x: 0, y: 0, z: 0 };
  let estThrustAcc = 9.81;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;
      const dt = 0.01;
      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const dirX = Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ;
      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
      const dirZ = -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ;

      if (!lastPos) {
        lastPos = { x: p.x, y: p.y, z: p.z };
        estV = { x: v.x, y: v.y, z: v.z };
      } else {
        for (const axis of ['x', 'y', 'z']) {
          const posV = (p[axis] - lastPos[axis]) / dt;
          lastPos[axis] = p[axis];
          const prevV = estV[axis];
          estV[axis] = 0.60 * (estV[axis] + estA[axis] * dt) + 0.30 * posV + 0.10 * v[axis];
          const realA = (estV[axis] - prevV) / dt;
          const modelA = axis === 'x' ? estThrustAcc * dirX : (axis === 'y' ? estThrustAcc * dirY - 9.81 : estThrustAcc * dirZ);
          if (obs.t > 0.08 && p.y > 0.085) {
            if (axis === 'x') distX = clamp(distX + 0.28 * (realA - modelA - distX), -9.5, 9.5);
            if (axis === 'y') distY = clamp(distY + 0.22 * (realA - modelA - distY), -5.5, 5.5);
            if (axis === 'z') distZ = clamp(distZ + 0.38 * (realA - modelA - distZ), -9.5, 9.5);
          }
          estA[axis] = modelA + (axis === 'x' ? distX : (axis === 'y' ? distY : distZ));
        }
      }

      let target, targetVel = { x: 0, y: 0, z: 0 };
      let kpX = 8.0, kdX = 4.6, kpZ = 18.5, kdZ = 6.8, kpY = 10.5, kdY = 4.8;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          targetIndex = obs.gateIndex;
          phase = 'align';
          settle = 0;
          intX *= 0.25; intZ *= 0.25;
        }
        const dx = gate.x - p.x;
        const dy = Math.abs(p.y - gate.y);
        const dz = Math.abs(p.z - gate.z);

        const tLook = clamp(dx / Math.max(0.6, estV.x), 0.05, 0.14);
        const predZ = p.z + estV.z * tLook + 0.20 * estA.z * tLook * tLook;
        const predY = p.y + estV.y * tLook + 0.20 * estA.y * tLook * tLook;
        const predDz = Math.abs(predZ - gate.z);
        const predDy = Math.abs(predY - gate.y);

        if (dx < 0) {
          phase = 'align';
          settle = 0;
          if (dy < 0.10 && dz < 0.022) {
            target = { x: gate.x - 0.28, y: gate.y, z: gate.z };
          } else {
            target = { x: gate.x - 0.35, y: Math.min(2.75, gate.y + 0.42), z: gate.z };
          }
        } else {
          if (phase === 'align') {
            const stageX = (dy < 0.08 && dz < 0.032) ? gate.x - 0.22 : gate.x - 0.34;
            target = { x: stageX, y: gate.y, z: gate.z };
            if (dx > 0.36 && dz < 0.20 && dy < 0.28) {
              targetVel.x = clamp(1.35 * (dx - 0.34), 0, 1.35);
            }
            if (dx > 0.05 && dx < 0.46 && dy < 0.070 && dz < 0.026 && predDz < 0.028 && predDy < 0.085 && Math.abs(estV.z) < 0.18 && Math.abs(estV.y) < 0.30) {
              settle++;
            } else {
              settle = Math.max(0, settle - 1);
            }
            if (settle >= 2) phase = 'cross';
          }
          if (phase === 'cross') {
            const stopDist = (estV.x > 0 ? (estV.x * estV.x) / 11.5 + estV.x * 0.03 : 0);
            const canAbort = (dx - 0.06) > stopDist;
            if (dx > 0.06 && canAbort && (dz > 0.028 || predDz > 0.030 || dy > 0.095 || predDy > 0.105)) {
              phase = 'align';
              settle = 0;
              target = { x: gate.x - 0.26, y: gate.y, z: gate.z };
              targetVel = { x: -0.15, y: 0, z: 0 };
              kpX = 11.0; kdX = 5.8;
            } else {
              target = { x: gate.x + 0.35, y: gate.y, z: gate.z };
              targetVel = { x: 1.55, y: 0, z: 0 };
              kpZ = 22.0; kdZ = 7.4;
              kpY = 12.5; kdY = 5.2;
            }
          }
        }
      } else {
        const pad = obs.pad.centre;
        if (targetIndex !== 3) {
          targetIndex = 3;
          phase = 'approach';
          intX *= 0.25; intZ *= 0.25;
        }
        const horizErr = Math.hypot(pad.x - p.x, pad.z - p.z);
        const horizSpd = Math.hypot(estV.x, estV.z);
        kpX = 11.0; kdX = 5.6;
        kpZ = 14.0; kdZ = 6.0;
        if (phase === 'approach') {
          const approachY = clamp(0.18 + 0.42 * horizErr, 0.18, 0.65);
          target = { x: pad.x, y: approachY, z: pad.z };
          if (horizErr > 0.20) {
            const ux = (pad.x - p.x) / horizErr, uz = (pad.z - p.z) / horizErr;
            const spd = clamp(1.35 * (horizErr - 0.12), 0, 1.35);
            targetVel.x = ux * spd;
            targetVel.z = uz * spd;
          }
          if (horizErr < 0.12 && horizSpd < 0.35 && p.y < 0.28) {
            phase = 'descend';
          }
        }
        if (phase === 'descend') {
          if (horizErr > 0.135 || horizSpd > 0.48) {
            phase = 'approach';
            target = { x: pad.x, y: 0.20, z: pad.z };
          } else {
            target = { x: pad.x, y: 0.05, z: pad.z };
            targetVel = { x: 0, y: p.y > 0.11 ? -0.14 : -0.06, z: 0 };
          }
        }
      }

      const ex = clamp(target.x - p.x, -0.8, 0.8);
      const ey = clamp(target.y - p.y, -0.8, 0.8);
      const ez = clamp(target.z - p.z, -0.8, 0.8);

      if (Math.abs(target.x - p.x) < 0.30) intX = clamp(intX + ex * dt, -0.45, 0.45);
      if (Math.abs(target.y - p.y) < 0.30) intY = clamp(intY + ey * dt, -0.45, 0.45);
      if (Math.abs(target.z - p.z) < 0.30) intZ = clamp(intZ + ez * dt, -0.45, 0.45);

      const ax = clamp(kpX * ex + kdX * (targetVel.x - estV.x) + 3.5 * intX - distX, -7.5, 7.5);
      const az = clamp(kpZ * ez + kdZ * (targetVel.z - estV.z) + 5.5 * intZ - distZ, -8.0, 8.0);
      const ay = clamp(kpY * ey + kdY * (targetVel.y - estV.y) + 3.0 * intY - distY, -5.0, 6.8);

      const totalAy = Math.max(2.5, 9.81 + ay);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyAx = cy * ax - sy * az;
      const bodyAz = sy * ax + cy * az;

      let maxTilt = obs.t < 0.22 ? 0.38 : 0.56;
      let ka = 1.35, kLead = 0.07;
      if (obs.gateIndex === 3 && p.y < 0.24) {
        const horizErr = Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z);
        const horizSpd = Math.hypot(estV.x, estV.z);
        if (p.y < 0.115 && horizErr < 0.11 && horizSpd < 0.28) {
          maxTilt = 0.155;
          ka = 0.45; kLead = 0.04;
        } else {
          maxTilt = 0.32;
          ka = 0.85; kLead = 0.05;
        }
      }

      const pitchRef = clamp(Math.atan2(bodyAx, totalAy), -maxTilt, maxTilt);
      const rollRef = clamp(-Math.atan2(bodyAz, totalAy), -maxTilt, maxTilt);

      const pitch = clamp(pitchRef + 0.65 * ka * (pitchRef - att.pitch) - 0.65 * kLead * rates.pitch, -maxTilt, maxTilt);
      const roll = clamp(rollRef + ka * (rollRef - att.roll) - kLead * rates.roll, -maxTilt, maxTilt);

      const cosTilt = Math.max(0.55, Math.cos(att.roll) * Math.cos(att.pitch));
      const reqThrustAcc = totalAy / cosTilt;

      let thrust = clamp(reqThrustAcc / 19.5, 0, 0.82);
      if (obs.gateIndex === 3 && p.y <= 0.082 && Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z) < 0.11 && Math.hypot(estV.x, estV.y, estV.z) < 0.24 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.15) {
        thrust = 0;
      }
      estThrustAcc += 0.25 * (thrust * 19.5 - estThrustAcc);

      return { thrust, roll, pitch, yawRate: clamp(-2.5 * att.yaw, -2, 2) };
    },
  };
}
