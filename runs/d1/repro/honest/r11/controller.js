export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01;
  const GATES = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];

  let inited = false;
  let vx = 0, vy = 0, vz = 0;
  let prevVx = 0, prevVy = 0, prevVz = 0;
  let prevAtt = { roll: 0, pitch: 0, yaw: 0 };
  let motorThrust = 0.45, lastThrust = 0.45;

  // Fast and slow acceleration disturbance observers
  let dFast = { x: 0, y: 0, z: 0 };
  let dSlow = { x: 0, y: 0, z: 0 };

  // Integrals: yaw + local position
  let intYaw = 0;
  let intX = 0, intY = 0, intZ = 0;

  // Smooth reference filter state
  let ref = { x: 0, y: 0, z: 0 };
  let refVel = { x: 0, y: 0, z: 0 };

  // Navigation state
  let activeGate = 0;
  let phase = 'align'; // 'align' | 'cross' | 'exit' | 'pad_approach' | 'pad_descend'
  let settle = 0;
  let padY = 0.55;

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (!inited) {
        inited = true;
        vx = v.x; vy = v.y; vz = v.z;
        prevVx = vx; prevVy = vy; prevVz = vz;
        prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };
        ref = { x: p.x, y: Math.max(p.y, 0.35), z: p.z };
      } else {
        // 1. Filter velocity
        const vAlpha = 0.42;
        vx += vAlpha * (v.x - vx);
        vy += vAlpha * (v.y - vy);
        vz += vAlpha * (v.z - vz);
      }

      // Update motor lag model (nominal 40ms lag)
      motorThrust += (lastThrust - motorThrust) * (1 - Math.exp(-dt / 0.04));

      // 2. Fast/slow acceleration disturbance observer
      if (obs.t > 0.02 && p.y > 0.075) {
        const axMeas = (vx - prevVx) / dt;
        const ayMeas = (vy - prevVy) / dt;
        const azMeas = (vz - prevVz) / dt;

        const r = 0.5 * (att.roll + prevAtt.roll);
        const pt = 0.5 * (att.pitch + prevAtt.pitch);
        const yw = 0.5 * (att.yaw + prevAtt.yaw);
        const bx = Math.sin(pt) * Math.cos(r), bz = -Math.sin(r);
        const dirX = Math.cos(yw) * bx + Math.sin(yw) * bz;
        const dirY = Math.cos(pt) * Math.cos(r);
        const dirZ = -Math.sin(yw) * bx + Math.cos(yw) * bz;

        const kThrust = 15.2 / 0.725;
        const drag = 0.42 / 0.725;
        const fMag = kThrust * motorThrust;

        const axNom = fMag * dirX - drag * vx;
        const ayNom = fMag * dirY - 9.81 - drag * vy;
        const azNom = fMag * dirZ - drag * vz;

        const dxRaw = clamp(axMeas - axNom, -9, 9);
        const dyRaw = clamp(ayMeas - ayNom, -7, 7);
        const dzRaw = clamp(azMeas - azNom, -9, 9);

        const aFast = 0.28;
        const aSlow = obs.t < 0.7 ? 0.08 : 0.032;
        dFast.x += aFast * (dxRaw - dFast.x);
        dFast.y += aFast * (dyRaw - dFast.y);
        dFast.z += aFast * (dzRaw - dFast.z);

        dSlow.x += aSlow * (dxRaw - dSlow.x);
        dSlow.y += aSlow * (dyRaw - dSlow.y);
        dSlow.z += aSlow * (dzRaw - dSlow.z);
      }

      prevVx = vx; prevVy = vy; prevVz = vz;
      prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };

      // 5 & 6. Navigation with gate exit lock and predictive cross abort
      let target;
      let emergencyBrake = false;

      if (activeGate < 3) {
        const gate = GATES[activeGate];

        // If simulator incremented gateIndex, lock exit until safely past gate plane
        if (obs.gateIndex > activeGate && phase !== 'exit') {
          phase = 'exit';
          settle = 0;
        }

        if (phase === 'exit') {
          target = { x: gate.x + 0.36, y: gate.y, z: gate.z };
          if (p.x < gate.x + 0.18) {
            ref.x = Math.max(ref.x, p.x + 0.06);
            refVel.x = Math.max(refVel.x, 0.35);
          }
          const xPredExit = p.x + vx * 0.22;
          if (p.x > gate.x + 0.19 && xPredExit > gate.x + 0.17 && vx > -0.05) {
            activeGate = obs.gateIndex;
            if (activeGate < 3) {
              phase = 'align';
              settle = 0;
              intX *= 0.2; intZ *= 0.2;
            } else {
              phase = 'pad_approach';
              padY = Math.min(p.y, 0.68);
              intX *= 0.2; intZ *= 0.2;
            }
          }
        }

        if (activeGate < 3 && phase !== 'exit') {
          const g = GATES[activeGate];
          const baseAlignX = g.x - 0.40;
          const errYZ = Math.hypot(p.y - g.y, p.z - g.z);
          const stageOffset = errYZ > 0.16 ? clamp(0.50 * (errYZ - 0.16), 0, 0.35) : 0;
          let alignX = baseAlignX - stageOffset;
          if (activeGate > 0) {
            alignX = Math.max(alignX, GATES[activeGate - 1].x + 0.32);
          }

          if (phase === 'align') {
            target = { x: alignX, y: g.y, z: g.z };
            if (p.x > baseAlignX + 0.08 && vx > 0.15) {
              emergencyBrake = true;
            }
            const ex = Math.abs(p.x - baseAlignX);
            const ey = Math.abs(p.y - g.y);
            const ez = Math.abs(p.z - g.z);
            const gustZ = Math.abs(dFast.z - dSlow.z);
            const gustY = Math.abs(dFast.y - dSlow.y);
            const zPred = Math.abs(p.z + vz * 0.24 + 0.03 * (dFast.z - dSlow.z) - g.z);
            const yPred = Math.abs(p.y + vy * 0.24 + 0.03 * (dFast.y - dSlow.y) - g.y);

            if (ex < 0.11 && ey < 0.075 && ez < 0.050 &&
                Math.abs(vx) < 0.25 && Math.abs(vy) < 0.20 && Math.abs(vz) < 0.14 &&
                zPred < 0.062 && yPred < 0.095 && gustZ < 1.15 && gustY < 1.15) {
              settle++;
            } else {
              settle = Math.max(0, settle - 1);
            }
            if (settle >= 8) {
              phase = 'cross';
              settle = 0;
            }
          }

          if (phase === 'cross') {
            target = { x: g.x + 0.34, y: g.y, z: g.z };
            const dxGate = g.x - p.x;
            const stopDist = Math.max(0.065, (Math.max(0, vx) * Math.max(0, vx)) / 8.8 + 0.03);

            // Predictive cross abort while still able to stop before gate plane
            if (dxGate > stopDist) {
              const tCross = clamp(dxGate / Math.max(vx, 0.32), 0.10, 0.45);
              const tLook = Math.min(tCross, 0.32);
              const gZ = dFast.z - dSlow.z;
              const gY = dFast.y - dSlow.y;
              const zPred = p.z + vz * tLook + 0.55 * gZ * tLook * tLook;
              const yPred = p.y + vy * tLook + 0.50 * gY * tLook * tLook;

              if (Math.abs(zPred - g.z) > 0.076 || Math.abs(p.z - g.z) > 0.072 ||
                  Math.abs(yPred - g.y) > 0.145 || Math.abs(p.y - g.y) > 0.135 ||
                  (dxGate > 0.13 && Math.abs(gZ) > 1.45)) {
                phase = 'align';
                settle = 0;
                target = { x: baseAlignX, y: g.y, z: g.z };
                ref.x = Math.min(p.x - 0.22, baseAlignX);
                refVel.x = -0.35;
                emergencyBrake = true;
              }
            } else if (p.x > g.x + 0.06 && obs.gateIndex === activeGate) {
              // Missed gate outside frame: return to align
              phase = 'align';
              settle = 0;
              ref.x = baseAlignX;
              refVel.x = 0;
            }
          }
        }
      }

      // 7. Ramp pad descent
      if (activeGate >= 3) {
        const pad = obs.pad.centre;
        const distXZ = Math.hypot(pad.x - p.x, pad.z - p.z);
        const speedXZ = Math.hypot(vx, vz);
        const tilt = Math.max(Math.abs(att.roll), Math.abs(att.pitch));
        const gustXZ = Math.hypot(dFast.x - dSlow.x, dFast.z - dSlow.z);

        if (phase === 'pad_approach') {
          const approachY = clamp(0.28 + 0.45 * distXZ, 0.28, 0.72);
          padY += clamp(approachY - padY, -0.55 * dt, 0.45 * dt);
          target = { x: pad.x, y: padY, z: pad.z };
          if (distXZ < 0.12 && speedXZ < 0.24 && p.y < 0.42 && tilt < 0.16 && gustXZ < 1.2) {
            phase = 'pad_descend';
            padY = p.y;
          }
        }

        if (phase === 'pad_descend') {
          if (distXZ > 0.165) {
            // Pushed well outside pad radius: climb back to approach
            phase = 'pad_approach';
            padY = Math.max(p.y, 0.26);
          } else if (distXZ > 0.125 || tilt > 0.20 || (p.y < 0.20 && gustXZ > 1.5)) {
            // Pause or lift slightly if gust tilts drone or pushes toward pad edge
            padY = clamp(padY + 0.25 * dt, 0.16, 0.30);
          } else if (distXZ < 0.115 && speedXZ < 0.30 && tilt < 0.16) {
            const rate = p.y > 0.18 ? 0.36 : 0.18;
            padY = Math.max(0.03, padY - rate * dt);
          }
          target = { x: pad.x, y: padY, z: pad.z };
        }
      }

      // Guard against ever being blown backward into a previously crossed gate
      if (obs.gateIndex > 0) {
        const prevG = GATES[obs.gateIndex - 1];
        if (p.x < prevG.x + 0.22 && p.x + vx * 0.25 < prevG.x + 0.16) {
          target = { x: Math.max(target.x, prevG.x + 0.38), y: prevG.y, z: prevG.z };
          ref.x = Math.max(ref.x, prevG.x + 0.32);
        }
      }

      // 4. Smooth reference filter
      const isCross = (phase === 'cross' || phase === 'exit');
      const tauRef = isCross ? 0.13 : 0.15;
      const maxSpeed = isCross ? 1.05 : 1.65;
      let desVx = (target.x - ref.x) / tauRef;
      let desVy = (target.y - ref.y) / tauRef;
      let desVz = (target.z - ref.z) / tauRef;
      const desSpd = Math.hypot(desVx, desVy, desVz);
      if (desSpd > maxSpeed) {
        const s = maxSpeed / desSpd;
        desVx *= s; desVy *= s; desVz *= s;
      }
      const vRefAlpha = 0.36;
      refVel.x += vRefAlpha * (desVx - refVel.x);
      refVel.y += vRefAlpha * (desVy - refVel.y);
      refVel.z += vRefAlpha * (desVz - refVel.z);
      ref.x += refVel.x * dt;
      ref.y += refVel.y * dt;
      ref.z += refVel.z * dt;

      // 3. Yaw + local position integrals
      if (p.y > 0.08) {
        intYaw = clamp(intYaw + att.yaw * dt, -0.6, 0.6);
      }
      const distTargetXZ = Math.hypot(target.x - p.x, target.z - p.z);
      const localWeight = 1 / (1 + (distTargetXZ / 0.35) * (distTargetXZ / 0.35));
      if (p.y > 0.08) {
        intX = clamp(intX + (ref.x - p.x) * localWeight * dt, -0.45, 0.45);
        intZ = clamp(intZ + (ref.z - p.z) * localWeight * dt, -0.45, 0.45);
        intY = clamp(intY + (ref.y - p.y) * dt, -0.85, 0.85);
      }

      // Position & velocity control with fast/slow disturbance rejection
      const kpX = emergencyBrake ? 6.5 : (isCross ? 5.2 : 4.8);
      const kdX = emergencyBrake ? 4.8 : (isCross ? 3.8 : 3.6);
      const kpZ = isCross ? 6.8 : 5.2;
      const kdZ = isCross ? 4.5 : 3.8;

      const distX = dSlow.x + 0.88 * (dFast.x - dSlow.x);
      const distY = dSlow.y + 0.80 * (dFast.y - dSlow.y);
      const distZ = dSlow.z + (isCross ? 1.05 : 0.92) * (dFast.z - dSlow.z);

      const drag = 0.42 / 0.725;
      const axCmd = kpX * (ref.x - p.x) + kdX * (0.65 * refVel.x - vx) + 1.8 * intX;
      const azCmd = kpZ * (ref.z - p.z) + kdZ * (0.65 * refVel.z - vz) + 1.9 * intZ;
      const ayCmd = 6.4 * (ref.y - p.y) + 4.1 * (0.65 * refVel.y - vy) + 2.2 * intY;

      const maxFx = emergencyBrake ? 5.5 : 4.5;
      const fx = clamp(axCmd + drag * vx - distX, -maxFx, 4.5);
      const fz = clamp(azCmd + drag * vz - distZ, -4.6, 4.6);
      const fy = clamp(9.81 + ayCmd + drag * vy - distY, 3.8, 15.5);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const fBx = cy * fx - sy * fz;
      const fBz = sy * fx + cy * fz;

      const maxPitch = emergencyBrake ? 0.55 : 0.48;
      let pitch = clamp(Math.atan2(fBx, fy), -maxPitch, 0.48);
      let roll = clamp(-Math.atan2(fBz, Math.hypot(fBx, fy)), -0.48, 0.48);

      const kThrust = 15.2 / 0.725;
      const tiltCos = Math.max(0.65, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = clamp(fy / (kThrust * tiltCos), 0, 1);

      // Progressive tilt protection near ground on pad & soft touchdown hold
      if (activeGate >= 3 && p.y < 0.22) {
        const padDist = Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z);
        const maxPadTilt = clamp(0.125 + (p.y - 0.085) * 1.35, 0.125, 0.30);
        pitch = clamp(pitch, -maxPadTilt, maxPadTilt);
        roll = clamp(roll, -maxPadTilt, maxPadTilt);
        if (p.y <= 0.082 && padDist <= 0.125 && Math.abs(vy) < 0.22) {
          thrust = Math.min(thrust, 0.18);
        }
      }

      const yawRate = clamp(-3.2 * att.yaw - 2.6 * intYaw - 0.35 * rates.yaw, -2, 2);
      lastThrust = thrust;
      return { thrust, roll, pitch, yawRate };
    },
  };
}
