// Smooth trajectory-generating flight controller with reference filter,
// dual-rate disturbance observer, centered gate-corridor tracking, and vertical landing column.
export default function makeController() {
  let targetIndex = -1, phase = 'align', settle = 0;
  const estMass = 0.725;
  let distX = 0, distY = 0, distZ = 0;
  let fastDistX = 0, fastDistY = 0, fastDistZ = 0;
  let ix = 0, iy = 0, iz = 0, iyaw = 0;
  let prevVel = null, prevAccCmd = { x: 0, y: 9.81, z: 0 };
  let posF = null, velF = null;
  let refPos = null, refVel = null;
  let descendY = 0.48;

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
        refPos = { x: pRaw.x, y: pRaw.y, z: pRaw.z };
        refVel = { x: 0, y: 0, z: 0 };
      } else {
        posF.x = 0.72 * pRaw.x + 0.28 * (posF.x + velF.x * dt);
        posF.y = 0.72 * pRaw.y + 0.28 * (posF.y + velF.y * dt);
        posF.z = 0.72 * pRaw.z + 0.28 * (posF.z + velF.z * dt);
        velF.x = 0.58 * vRaw.x + 0.42 * velF.x;
        velF.y = 0.58 * vRaw.y + 0.42 * velF.y;
        velF.z = 0.58 * vRaw.z + 0.42 * velF.z;
      }

      const p = posF, v = velF;

      if (prevVel && p.y > 0.12) {
        const axMeas = (v.x - prevVel.x) / dt;
        const ayMeas = (v.y - prevVel.y) / dt;
        const azMeas = (v.z - prevVel.z) / dt;

        const errAx = axMeas - prevAccCmd.x;
        const errAy = ayMeas - (prevAccCmd.y - 9.81);
        const errAz = azMeas - prevAccCmd.z;

        distX = clamp(distX + 0.042 * (errAx - distX), -5.5, 5.5);
        distY = clamp(distY + 0.045 * (errAy - distY), -4.5, 4.5);
        distZ = clamp(distZ + 0.042 * (errAz - distZ), -5.5, 5.5);

        fastDistX = clamp(fastDistX + 0.16 * (errAx - fastDistX), -6.5, 6.5);
        fastDistY = clamp(fastDistY + 0.12 * (errAy - fastDistY), -5.0, 5.0);
        fastDistZ = clamp(fastDistZ + 0.16 * (errAz - fastDistZ), -6.5, 6.5);
      }
      prevVel = { x: v.x, y: v.y, z: v.z };

      let rawTarget;
      let maxSpeed = { x: 1.40, y: 1.20, z: 1.40 };
      let maxAcc = { x: 3.0, y: 2.8, z: 3.0 };
      let kpX = 4.8, kdX = 3.9;
      let kpZ = 5.8, kdZ = 4.4;
      let kpY = 6.6, kdY = 4.3;

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
          const alignX = gate.x - 0.34;
          rawTarget = { x: alignX, y: gate.y, z: gate.z };

          if (p.x > gate.x - 0.12 && (Math.abs(dy) > 0.16 || Math.abs(dz) > 0.085)) {
            rawTarget = { x: gate.x - 0.48, y: Math.min(2.4, gate.y + 0.65), z: gate.z };
          }

          if (obs.gateIndex > 0) {
            const prevGate = GATES[obs.gateIndex - 1];
            if (p.x < prevGate.x + 0.20) {
              rawTarget = { x: prevGate.x + 0.36, y: prevGate.y, z: prevGate.z };
              kpX = 5.0; kdX = 3.8;
              kpZ = 8.0; kdZ = 5.2;
              kpY = 7.6; kdY = 4.6;
            } else if (p.x < prevGate.x + 0.58) {
              const u = clamp((p.x - (prevGate.x + 0.18)) / (alignX - (prevGate.x + 0.18)), 0, 1);
              const s = u * u * (3 - 2 * u);
              rawTarget = {
                x: alignX,
                y: prevGate.y + (gate.y - prevGate.y) * s,
                z: prevGate.z + (gate.z - prevGate.z) * s,
              };
            }
          } else if (p.y < 0.45) {
            const s = clamp((p.y - 0.07) / 0.55, 0.25, 1.0);
            rawTarget = {
              x: -2.1 + (alignX - (-2.1)) * s,
              y: gate.y,
              z: -1.65 + (gate.z - (-1.65)) * s,
            };
          }

          const distAlign = Math.hypot(alignX - p.x, dy, dz);
          const speed = Math.hypot(v.x, v.y, v.z);
          if (
            distAlign < 0.18 &&
            speed < 0.52 &&
            Math.abs(dy) < 0.058 &&
            Math.abs(dz) < 0.040 &&
            Math.abs(v.z) < 0.16 &&
            Math.abs(v.y) < 0.18
          ) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= 3) {
            phase = 'cross';
            ix = 0; iz = 0;
          }
        }

        if (phase === 'cross') {
          const dxGate = gate.x - p.x;
          const tCross = clamp(dxGate / Math.max(0.25, v.x), 0.0, 0.35);
          const predY = p.y + v.y * tCross - gate.y;
          const predZ = p.z + v.z * tCross - gate.z;

          const unsafeNow = Math.abs(dy) > 0.13 || Math.abs(dz) > 0.062;
          const unsafePred = Math.abs(predY) > 0.14 || Math.abs(predZ) > 0.068;

          if (dxGate > 0.11 && (unsafeNow || unsafePred)) {
            phase = 'align';
            settle = 0;
            ix = 0; iz = 0;
            rawTarget = { x: gate.x - 0.34, y: gate.y, z: gate.z };
            refPos.x = p.x; refPos.y = p.y; refPos.z = p.z;
            refVel.x = v.x * 0.5; refVel.y = v.y * 0.5; refVel.z = v.z * 0.5;
            kpX = 5.4; kdX = 4.4;
            kpZ = 8.4; kdZ = 5.4;
          } else {
            rawTarget = { x: gate.x + 0.38, y: gate.y, z: gate.z };
            maxSpeed.x = 1.10;
            kpZ = 9.0; kdZ = 5.8;
            kpY = 8.4; kdY = 5.2;
            kpX = 4.2; kdX = 3.5;
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
          if (p.x < GATES[2].x + 0.20) {
            rawTarget = { x: GATES[2].x + 0.36, y: GATES[2].y, z: GATES[2].z };
            kpZ = 8.2; kdZ = 5.4;
            kpY = 7.6; kdY = 4.8;
          } else {
            const u = clamp((p.x - (GATES[2].x + 0.18)) / (pad.x - (GATES[2].x + 0.18)), 0, 1);
            const s = u * u * (3 - 2 * u);
            rawTarget = {
              x: pad.x,
              y: 0.48 + (1 - s) * 0.42,
              z: GATES[2].z + (pad.z - GATES[2].z) * Math.min(1, u * 1.15),
            };
          }
          kpX = 5.2; kdX = 4.4;
          kpZ = 5.6; kdZ = 4.6;

          if (horizErr < 0.055 && horizVel < 0.15 && p.y < 0.56) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= 8 || obs.t > 21.0) {
            phase = 'descend';
            descendY = p.y;
          }
        }

        if (phase === 'descend') {
          const descendRate = descendY > 0.20 ? 0.38 : 0.18;
          if (horizErr < 0.11 || obs.t > 23.0) {
            descendY = Math.max(0.02, descendY - descendRate * dt);
          }
          rawTarget = { x: pad.x, y: descendY, z: pad.z };
          maxSpeed.y = descendRate;
          kpX = 6.6; kdX = 5.0;
          kpZ = 6.6; kdZ = 5.0;
          kpY = 6.8; kdY = 4.6;
        }
      }

      for (const axis of ['x', 'y', 'z']) {
        const leash = axis === 'y' ? 0.30 : 0.35;
        refPos[axis] = clamp(refPos[axis], p[axis] - leash, p[axis] + leash);

        const dist = rawTarget[axis] - refPos[axis];
        const desVel = clamp(3.8 * dist, -maxSpeed[axis], maxSpeed[axis]);
        const acc = clamp((desVel - refVel[axis]) * 8.0, -maxAcc[axis], maxAcc[axis]);
        refVel[axis] += acc * dt;
        refPos[axis] += refVel[axis] * dt;
      }

      const blendRef = (obs.gateIndex < 3 && phase === 'cross') ? 0.30 : 0.60;
      const targetX = blendRef * refPos.x + (1 - blendRef) * rawTarget.x;
      const targetY = blendRef * refPos.y + (1 - blendRef) * rawTarget.y;
      const targetZ = blendRef * refPos.z + (1 - blendRef) * rawTarget.z;

      const ex = targetX - p.x;
      const ey = targetY - p.y;
      const ez = targetZ - p.z;

      const errRawX = rawTarget.x - p.x;
      const errRawY = rawTarget.y - p.y;
      const errRawZ = rawTarget.z - p.z;

      if (Math.abs(errRawX) < 0.28 && Math.abs(v.x) < 0.35) ix = clamp(ix + errRawX * dt, -0.45, 0.45);
      else ix *= 0.96;
      if (Math.abs(errRawZ) < 0.28 && Math.abs(v.z) < 0.35) iz = clamp(iz + errRawZ * dt, -0.45, 0.45);
      else iz *= 0.96;
      if (Math.abs(errRawY) < 0.28 && Math.abs(v.y) < 0.35) iy = clamp(iy + errRawY * dt, -0.45, 0.45);
      else iy *= 0.96;

      const compX = 0.55 * distX + 0.45 * fastDistX;
      const compZ = 0.55 * distZ + 0.45 * fastDistZ;
      const compY = 0.65 * distY + 0.35 * fastDistY;

      const dragCompX = (0.42 / estMass) * v.x;
      const dragCompZ = (0.42 / estMass) * v.z;

      const vffScale = blendRef * 0.45;
      const ax = clamp(kpX * ex + kdX * (vffScale * refVel.x - v.x) + 1.8 * ix - compX + 0.5 * dragCompX, -4.2, 4.2);
      const az = clamp(kpZ * ez + kdZ * (vffScale * refVel.z - v.z) + 2.0 * iz - compZ + 0.5 * dragCompZ, -4.2, 4.2);
      const ay = clamp(kpY * ey + kdY * (vffScale * refVel.y - v.y) + 1.8 * iy - compY, -4.0, 5.2);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const axBody = cy * ax - sy * az;
      const azBody = sy * ax + cy * az;

      const gEff = Math.max(5.5, 9.81 + ay);
      let pitch = clamp(Math.atan2(axBody, gEff), -0.48, 0.48);
      let roll = clamp(Math.atan2(-azBody, gEff), -0.48, 0.48);

      const tiltCos = Math.max(0.65, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = (estMass * (9.81 + ay)) / (16 * tiltCos);

      if (obs.gateIndex === 3 && phase === 'descend' && p.y <= 0.092) {
        const padErr = Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z);
        if (padErr < 0.125) {
          pitch = clamp(pitch * 0.45, -0.12, 0.12);
          roll = clamp(roll * 0.45, -0.12, 0.12);
          if (p.y <= 0.078 && Math.hypot(v.x, v.z) < 0.32) {
            thrust = 0.04;
          } else {
            thrust = Math.min(thrust, 0.30);
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
