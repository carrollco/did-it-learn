export default function makeController() {
  const dt = 0.01;
  const g = 9.81;
  const nominalMass = 0.725;
  const maxTotalThrust = 16.0;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  let initialized = false;
  let vf = { x: 0, y: 0, z: 0 };
  let prevVf = { x: 0, y: 0, z: 0 };
  let prevCmdAcc = { x: 0, y: 0, z: 0 };
  let dSlow = { x: 0, y: 0, z: 0 };
  let dFast = { x: 0, y: 0, z: 0 };

  let refPos = { x: 0, y: 0, z: 0 };
  let refVel = { x: 0, y: 0, z: 0 };

  let intPos = { x: 0, y: 0, z: 0 };
  let intYaw = 0;

  let targetIndex = -1;
  let phase = 'align';
  let settle = 0;
  let lockedGate = null;
  let padY = 0.95;

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att;

      if (!initialized) {
        initialized = true;
        vf = { x: v.x, y: v.y, z: v.z };
        prevVf = { x: v.x, y: v.y, z: v.z };
        refPos = { x: p.x, y: Math.max(p.y, 0.25), z: p.z };
      }

      // 1. Filter velocity
      const alphaV = 0.52;
      vf.x += alphaV * (v.x - vf.x);
      vf.y += alphaV * (v.y - vf.y);
      vf.z += alphaV * (v.z - vf.z);

      // 2. Fast/slow accel disturbance observer
      const measAcc = {
        x: (vf.x - prevVf.x) / dt,
        y: (vf.y - prevVf.y) / dt,
        z: (vf.z - prevVf.z) / dt,
      };
      prevVf = { x: vf.x, y: vf.y, z: vf.z };

      if (obs.t > 0.06 && p.y > 0.11) {
        const rawDist = {
          x: clamp(measAcc.x - prevCmdAcc.x, -14, 14),
          y: clamp(measAcc.y - prevCmdAcc.y, -10, 10),
          z: clamp(measAcc.z - prevCmdAcc.z, -14, 14),
        };
        const alphaSlow = 0.025;
        const alphaFast = 0.34;
        for (const k of ['x', 'y', 'z']) {
          dSlow[k] = clamp(dSlow[k] + alphaSlow * (rawDist[k] - dSlow[k]), -5.5, 5.5);
          dFast[k] = clamp(dFast[k] + alphaFast * (rawDist[k] - dFast[k]), -9.5, 9.5);
        }
      } else {
        dFast.x *= 0.85; dFast.y *= 0.85; dFast.z *= 0.85;
      }

      // 3. Navigation: gate exit lock, predictive cross abort, ramp pad descent
      let rawTarget = { x: p.x, y: 0.95, z: p.z };

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          // Gate exit lock: stay locked on passed gate's y,z and push forward until safely clear of gate plane
          if (targetIndex >= 0 && lockedGate && (p.x < lockedGate.x + 0.34 || vf.x < 0.05)) {
            rawTarget = { x: lockedGate.x + 0.50, y: lockedGate.y, z: lockedGate.z };
          } else {
            targetIndex = obs.gateIndex;
            lockedGate = { x: gate.x, y: gate.y, z: gate.z };
            phase = 'align';
            settle = 0;
            intPos = { x: 0, y: 0, z: 0 };
          }
        }

        if (targetIndex === obs.gateIndex) {
          // Also protect against backing into the PREVIOUS gate if blown backwards right after exit lock release
          if (p.x > gate.x + 0.04 && phase !== 'cross') {
            phase = 'recover';
          }
          if (phase === 'recover') {
            const dz = p.z - gate.z, dy = p.y - gate.y;
            if (p.x > gate.x - 0.35 && Math.abs(dz) < 0.55 && Math.abs(dy) < 0.55) {
              const safeY = clamp(gate.y + (dy >= 0 ? 0.62 : -0.62), 0.35, 2.65);
              rawTarget = { x: p.x, y: safeY, z: gate.z };
              if (Math.abs(p.y - gate.y) > 0.48 || Math.abs(p.z - gate.z) > 0.58) {
                rawTarget = { x: gate.x - 0.55, y: safeY, z: gate.z };
              }
            } else {
              phase = 'align';
              settle = 0;
            }
          }

          if (phase === 'align') {
            const ey = Math.abs(gate.y - p.y);
            const ez = Math.abs(gate.z - p.z);
            const alignX = gate.x - clamp(0.45 + 0.65 * Math.max(0, ey - 0.08) + 0.85 * Math.max(0, ez - 0.06), 0.45, 0.82);
            rawTarget = { x: alignX, y: gate.y, z: gate.z };
            const ex = Math.abs((gate.x - 0.45) - p.x);
            const vy = Math.abs(vf.y), vz = Math.abs(vf.z);
            const predEy = Math.abs((p.y + vf.y * 0.16 + 0.5 * (dFast.y - dSlow.y) * 0.025) - gate.y);
            const predEz = Math.abs((p.z + vf.z * 0.16 + 0.5 * (dFast.z - dSlow.z) * 0.025) - gate.z);

            if (ex < 0.18 && ey < 0.068 && ez < 0.055 && predEy < 0.085 && predEz < 0.070 && vy < 0.24 && vz < 0.19 && Math.abs(vf.x) < 0.36) {
              settle++;
            } else {
              settle = Math.max(0, settle - 1);
            }
            if (settle >= 10) {
              phase = 'cross';
            }
          }

          if (phase === 'cross') {
            const dx = gate.x - p.x;
            // Conservative stopping distance so an abort never skids into the gate plane!
            const stopDist = 0.09 + Math.max(0, vf.x) * 0.22 + (Math.max(0, vf.x) ** 2) / 5.2;
            if (dx > stopDist && dx > 0.14) {
              const vxEff = Math.max(vf.x, 0.35);
              const tCross = clamp(dx / vxEff, 0.05, 0.36);
              const netDistY = dFast.y - dSlow.y;
              const netDistZ = dFast.z - dSlow.z;
              const predY = p.y + vf.y * tCross + 0.5 * netDistY * tCross * tCross;
              const predZ = p.z + vf.z * tCross + 0.5 * netDistZ * tCross * tCross;
              const curEy = Math.abs(p.y - gate.y);
              const curEz = Math.abs(p.z - gate.z);
              const predEy = Math.abs(predY - gate.y);
              const predEz = Math.abs(predZ - gate.z);

              if (predEy > 0.145 || predEz > 0.078 || curEy > 0.145 || curEz > 0.078) {
                phase = 'align';
                settle = 0;
                rawTarget = { x: gate.x - 0.52, y: gate.y, z: gate.z };
                refPos.x = Math.min(refPos.x, p.x - 0.05);
                refVel.x = Math.min(refVel.x, -0.15);
              } else {
                rawTarget = { x: gate.x + 0.45, y: gate.y, z: gate.z };
              }
            } else {
              rawTarget = { x: gate.x + 0.45, y: gate.y, z: gate.z };
            }
          }
        }
      } else {
        const pad = obs.pad.centre;
        if (targetIndex !== 3) {
          if (lockedGate && (p.x < lockedGate.x + 0.34 || vf.x < 0.05)) {
            rawTarget = { x: lockedGate.x + 0.50, y: lockedGate.y, z: lockedGate.z };
          } else {
            targetIndex = 3;
            phase = 'pad_approach';
            padY = Math.max(0.65, Math.min(1.05, p.y));
            intPos = { x: 0, y: 0, z: 0 };
          }
        }
        if (targetIndex === 3) {
          const distH = Math.hypot(pad.x - p.x, pad.z - p.z);
          const speedH = Math.hypot(vf.x, vf.z);
          if (phase === 'pad_approach') {
            rawTarget = { x: pad.x, y: padY, z: pad.z };
            if (distH < 0.35) {
              padY = Math.max(0.32, padY - 0.45 * dt);
              rawTarget.y = padY;
            }
            if (distH < 0.11 && speedH < 0.25) {
              phase = 'descend';
            }
          }
          if (phase === 'descend') {
            if (distH < 0.13) {
              const rate = p.y > 0.22 ? 0.55 : 0.25;
              padY = Math.max(0.035, padY - rate * dt);
            } else if (distH > 0.16 && p.y > 0.12) {
              padY = Math.min(0.30, Math.max(padY, p.y));
            }
            rawTarget = { x: pad.x, y: padY, z: pad.z };
          }
        }
      }

      // 4. Smooth ref filter
      const inExitLock = targetIndex !== obs.gateIndex;
      const isCrossOrExit = phase === 'cross' || inExitLock;
      const wn = isCrossOrExit ? 7.5 : 5.5;
      const zeta = 0.95;
      const maxVel = isCrossOrExit ? 1.40 : 1.65;
      const maxAcc = 4.2;
      for (const k of ['x', 'y', 'z']) {
        const err = rawTarget[k] - refPos[k];
        let desVel = clamp(wn * err, -maxVel, maxVel);
        let acc = clamp(2 * zeta * wn * (desVel - refVel[k]), -maxAcc, maxAcc);
        refVel[k] = clamp(refVel[k] + acc * dt, -maxVel, maxVel);
        refPos[k] += refVel[k] * dt;
      }

      // 5. Local position integrals
      const locErr = {
        x: rawTarget.x - p.x,
        y: rawTarget.y - p.y,
        z: rawTarget.z - p.z,
      };
      for (const k of ['x', 'y', 'z']) {
        if (Math.abs(locErr[k]) < 0.35) {
          intPos[k] = clamp(intPos[k] + locErr[k] * dt, -0.55, 0.55);
        } else {
          intPos[k] *= 0.98;
        }
      }

      // 6. Position PD + Feedforward + Disturbance Rejection + Local Integrals
      const kp = isCrossOrExit ? { x: 4.8, y: 9.0, z: 9.8 } : { x: 4.5, y: 6.8, z: 5.8 };
      const kd = isCrossOrExit ? { x: 3.6, y: 5.0, z: 5.5 } : { x: 3.6, y: 4.2, z: 4.3 };
      const ki = { x: 1.8, y: 2.6, z: 1.9 };

      const distComp = {
        x: dSlow.x + 0.72 * (dFast.x - dSlow.x),
        y: dSlow.y + 0.52 * (dFast.y - dSlow.y),
        z: dSlow.z + 0.82 * (dFast.z - dSlow.z),
      };

      const ex = (0.60 * (refPos.x - p.x) + 0.40 * (rawTarget.x - p.x));
      const ey = (0.60 * (refPos.y - p.y) + 0.40 * (rawTarget.y - p.y));
      const ez = (0.60 * (refPos.z - p.z) + 0.40 * (rawTarget.z - p.z));

      const ax = clamp(kp.x * ex + kd.x * (0.45 * refVel.x - vf.x) + ki.x * intPos.x - distComp.x, -4.5, 4.5);
      const az = clamp(kp.z * ez + kd.z * (0.45 * refVel.z - vf.z) + ki.z * intPos.z - distComp.z, -4.5, 4.5);
      let ay = clamp(kp.y * ey + kd.y * (0.45 * refVel.y - vf.y) + ki.y * intPos.y - distComp.y, -4.5, 5.5);

      if (targetIndex === 3 && phase === 'descend' && p.y <= 0.088 && Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z) < 0.13) {
        ay = Math.min(ay, -2.5);
      }

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      // Smoothly taper maxTilt near floor to avoid floor crash (floor crash triggers if tilt > 0.36 at y <= 0.07)
      let maxTilt = 0.48;
      if (targetIndex === 3 && phase === 'descend') {
        if (p.y < 0.11) maxTilt = 0.15;
        else if (p.y < 0.25) maxTilt = 0.28;
      }
      const totalAy = Math.max(2.0, g + ay);
      const pitch = clamp(Math.atan2(cy * ax - sy * az, totalAy), -maxTilt, maxTilt);
      const roll = clamp(Math.atan2(-(sy * ax + cy * az), totalAy), -maxTilt, maxTilt);

      const thrust = clamp(
        (nominalMass * (g + ay)) / (maxTotalThrust * Math.max(0.65, Math.cos(att.roll) * Math.cos(att.pitch))),
        0,
        1
      );

      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll), bodyZ = -Math.sin(att.roll);
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
      const dirZ = -sy * bodyX + cy * bodyZ;
      const estForceMag = thrust * maxTotalThrust;
      const dragCoeff = 0.42 / nominalMass;
      prevCmdAcc = {
        x: (estForceMag * dirX) / nominalMass - dragCoeff * vf.x,
        y: (estForceMag * dirY) / nominalMass - g - dragCoeff * vf.y,
        z: (estForceMag * dirZ) / nominalMass - dragCoeff * vf.z,
      };

      // 7. Yaw + integral
      intYaw = clamp(intYaw + att.yaw * dt, -0.5, 0.5);
      const yawRate = clamp(-2.5 * att.yaw - 1.2 * intYaw - 0.15 * obs.rates.yaw, -2, 2);

      return { thrust, roll, pitch, yawRate };
    },
  };
}
