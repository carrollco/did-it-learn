export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01;
  const GATES = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];

  let initialized = false;
  let zx = 0, zz = 0;
  let integralY = 0;

  let targetIndex = -1;
  let phase = 'align';
  let settle = 0;
  let padTimer = 0;

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att;

      const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
      const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyX = sp * cr, bodyZ = -sr;
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = Math.max(0.5, cp * cr);
      const dirZ = -sy * bodyX + cy * bodyZ;

      const tauD = 0.058;
      if (!initialized) {
        zx = v.x;
        zz = v.z;
        initialized = true;
      }

      // Disturbance estimates from decoupled Q-filter DOB
      const dx = p.y > 0.09 ? clamp((v.x - zx) / tauD, -7.0, 7.0) : 0;
      const dz = p.y > 0.09 ? clamp((v.z - zz) / tauD, -7.0, 7.0) : 0;

      let target;
      let tvx = 0, tvz = 0;
      let nearGate = false;

      // Post-gate exit tunnel: after crossing gate i-1, stay aligned in y/z until x > prevGate.x + 0.24
      const prevGate = obs.gateIndex > 0 ? GATES[obs.gateIndex - 1] : null;
      const inExitTunnel = prevGate && p.x < prevGate.x + 0.24;

      if (inExitTunnel) {
        target = { x: prevGate.x + 0.36, y: prevGate.y, z: prevGate.z };
        tvx = 0.45;
        nearGate = true;
      } else if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        nearGate = Math.abs(gate.x - p.x) < 0.48;
        if (targetIndex !== obs.gateIndex) {
          targetIndex = obs.gateIndex;
          phase = 'align';
          settle = 0;
        }
        if (phase === 'align') {
          const alignX = gate.x - 0.36;
          if (obs.gateIndex === 0 && p.y < 0.72) {
            const u = clamp((p.y - 0.12) / 0.60, 0, 1);
            target = {
              x: -2.05 + u * (alignX - (-2.05)),
              y: gate.y,
              z: -1.60 + u * (gate.z - (-1.60)),
            };
          } else {
            target = { x: alignX, y: gate.y, z: gate.z };
            if (p.x > gate.x - 0.33 && v.x > 0.08) {
              // Strong braking if forward of align point while in align/abort state
              target.x = gate.x - 0.55;
              tvx = -0.45;
            }
          }
          const absZ = Math.abs(gate.z - p.z);
          const absY = Math.abs(gate.y - p.y);
          const absX = Math.abs(alignX - p.x);
          if (absZ < 0.052 && absY < 0.09 && absX < 0.11 && Math.abs(v.z) < 0.18 && Math.abs(v.y) < 0.26 && Math.abs(v.x) < 0.32) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= 6) phase = 'cross';
        }
        if (phase === 'cross') {
          if (p.x < gate.x - 0.015) {
            const tCross = clamp((gate.x - p.x) / Math.max(0.25, v.x), 0.02, 0.35);
            const zPred = p.z + v.z * tCross;
            const yPred = p.y + v.y * tCross;
            const zLim = p.x < gate.x - 0.10 ? 0.068 : 0.088;
            if (Math.abs(p.z - gate.z) > 0.076 || Math.abs(zPred - gate.z) > zLim ||
                Math.abs(p.y - gate.y) > 0.15 || Math.abs(yPred - gate.y) > 0.17) {
              phase = 'align';
              settle = 0;
              target = { x: gate.x - 0.55, y: gate.y, z: gate.z };
              tvx = -0.45;
            } else {
              target = { x: Math.min(p.x + 0.32, gate.x + 0.34), y: gate.y, z: gate.z };
              tvx = 0.55;
            }
          } else {
            target = { x: gate.x + 0.34, y: gate.y, z: gate.z };
            tvx = 0.55;
          }
        }
      } else {
        const pad = obs.pad.centre;
        if (targetIndex !== 3) {
          targetIndex = 3;
          phase = 'approach';
          settle = 0;
        }
        const distXZ = Math.hypot(pad.x - p.x, pad.z - p.z);
        const speedXZ = Math.hypot(v.x, v.z);
        if (phase === 'approach') {
          const h = clamp(0.26 + 0.9 * distXZ, 0.26, 0.95);
          target = { x: pad.x, y: h, z: pad.z };
          if (distXZ < 0.08 && speedXZ < 0.20 && p.y < 0.38 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.22) {
            settle++;
            if (settle >= 4) phase = 'descend';
          } else {
            settle = 0;
          }
        }
        if (phase === 'descend') {
          target = { x: pad.x, y: Math.max(0.035, p.y - 0.055), z: pad.z };
          if ((distXZ > 0.095 || Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > 0.26) && p.y > 0.095) {
            target.y = Math.max(p.y, 0.15);
          }
          const predSlideXZ = Math.hypot(pad.x - (p.x + v.x * 0.08), pad.z - (p.z + v.z * 0.08));
          if (p.y <= 0.084 && distXZ <= 0.105 && predSlideXZ <= 0.115 && Math.hypot(v.x, v.y, v.z) < 0.38 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.22) {
            padTimer++;
            if (padTimer >= 2) {
              return { thrust: 0, roll: 0, pitch: 0, yawRate: 0 };
            }
          } else {
            padTimer = 0;
          }
        }
      }

      const ey = target.y - p.y;
      integralY = clamp(integralY + ey * dt, -0.6, 2.2);
      const ayDyn = clamp(6.2 * ey - 4.0 * v.y, -3.2, 4.5);
      const ay = clamp(ayDyn + 2.8 * integralY, -3.5, 5.0);

      const isPad = obs.gateIndex === 3;
      const kpZ = nearGate ? 10.2 : (isPad ? 6.8 : 5.8);
      const kdZ = nearGate ? 5.5 : (isPad ? 4.4 : 4.0);
      const kpX = isPad ? 6.8 : 5.6;
      const kdX = isPad ? 4.4 : 4.0;

      // Clamp positive x position error during align so approach speed cannot overshoot into gate frame
      const exRaw = target.x - p.x;
      const ex = (obs.gateIndex < 3 && phase === 'align') ? clamp(exRaw, -0.75, 0.56) : clamp(exRaw, -0.85, 0.85);
      const ez = clamp(target.z - p.z, -0.85, 0.85);

      const ax = clamp(kpX * ex + kdX * (tvx - v.x) - dx + 0.58 * v.x, -4.6, 4.2);
      const az = clamp(kpZ * ez + kdZ * (tvz - v.z) - dz + 0.58 * v.z, -5.2, 5.2);

      const specVert = clamp(9.81 + ayDyn, 4.0, 14.0);
      const aTiltX = specVert * (dirX / dirY);
      const aTiltZ = specVert * (dirZ / dirY);
      if (p.y > 0.09) {
        zx += (0.75 * aTiltX + 0.25 * ax - 0.58 * v.x + dx) * dt;
        zz += (0.75 * aTiltZ + 0.25 * az - 0.58 * v.z + dz) * dt;
      } else {
        zx = v.x;
        zz = v.z;
      }

      let maxTilt = nearGate ? 0.54 : 0.48;
      if (isPad) {
        if (p.y < 0.14) maxTilt = 0.24;
        else if (p.y < 0.35) maxTilt = 0.28;
      }
      const pitchDes = clamp((cy * ax - sy * az) / 9.81, -maxTilt, maxTilt);
      const rollDes = clamp(-(sy * ax + cy * az) / 9.81, -maxTilt, maxTilt);
      const pitch = clamp(pitchDes + 0.18 * (pitchDes - att.pitch), -maxTilt, maxTilt);
      const roll = clamp(rollDes + 0.18 * (rollDes - att.roll), -maxTilt, maxTilt);
      const thrust = clamp((0.72 * (9.81 + ay)) / (16 * Math.max(0.7, cr * cp)), 0, 1);

      return { thrust, roll, pitch, yawRate: clamp(-1.8 * att.yaw, -1.2, 1.2) };
    },
  };
}
