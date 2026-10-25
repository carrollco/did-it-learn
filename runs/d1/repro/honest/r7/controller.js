export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01;
  const g = 9.81;
  const mNom = 0.725;
  const dragNom = 0.42 / mNom;

  let init = false;
  let vf = { x: 0, y: 0, z: 0 };
  let prevVf = { x: 0, y: 0, z: 0 };
  let prevAtt = { roll: 0, pitch: 0, yaw: 0 };
  let thrustEst = 0.45;
  let lastThrust = 0.45;

  // Fast/slow acceleration disturbance observer
  let dFast = { x: 0, y: 0, z: 0 };
  let dSlow = { x: 0, y: 0, z: 0 };

  // Integrals: yaw + local position
  let yawInt = 0;
  let intPos = { x: 0, y: 0, z: 0 };

  // Smooth reference filter
  let ref = { x: 0, y: 0, z: 0 };

  // Navigation state
  let targetIndex = 0;
  let phase = 'align';
  let settle = 0;
  let prevGate = null;
  let exitLock = false;
  let padY = 0.68;
  let touchedDown = false;
  let abortBoost = 0;

  return {
    step(obs) {
      const p = obs.pos;
      const vRaw = obs.vel;
      const att = obs.att;
      const rates = obs.rates;

      if (!init) {
        init = true;
        vf = { x: vRaw.x, y: vRaw.y, z: vRaw.z };
        prevVf = { x: vRaw.x, y: vRaw.y, z: vRaw.z };
        prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };
        ref = { x: p.x, y: Math.max(0.20, p.y), z: p.z };
        targetIndex = obs.gateIndex;
      } else {
        // 1. Filter velocity
        const alphaV = 0.42;
        vf.x += alphaV * (vRaw.x - vf.x);
        vf.y += alphaV * (vRaw.y - vf.y);
        vf.z += alphaV * (vRaw.z - vf.z);

        // 2. Fast/slow accel disturbance observer
        thrustEst += (lastThrust - thrustEst) * (dt / 0.04);
        const rMid = 0.5 * (prevAtt.roll + att.roll);
        const pMid = 0.5 * (prevAtt.pitch + att.pitch);
        const yMid = 0.5 * (prevAtt.yaw + att.yaw);
        const bx = Math.sin(pMid) * Math.cos(rMid);
        const bz = -Math.sin(rMid);
        const cy = Math.cos(yMid), sy = Math.sin(yMid);
        const dirX = cy * bx + sy * bz;
        const dirY = Math.cos(pMid) * Math.cos(rMid);
        const dirZ = -sy * bx + cy * bz;

        const fNom = (16.0 * thrustEst) / mNom;
        const aModX = fNom * dirX - dragNom * vf.x;
        const aModY = fNom * dirY - dragNom * vf.y - g;
        const aModZ = fNom * dirZ - dragNom * vf.z;

        const aMeasX = (vf.x - prevVf.x) / dt;
        const aMeasY = (vf.y - prevVf.y) / dt;
        const aMeasZ = (vf.z - prevVf.z) / dt;

        if ((p.y > 0.078 || obs.t > 0.06) && !touchedDown) {
          const rawX = clamp(aMeasX - aModX, -8.0, 8.0);
          const rawY = clamp(aMeasY - aModY, -6.0, 6.0);
          const rawZ = clamp(aMeasZ - aModZ, -8.0, 8.0);

          const aFast = 0.26;
          const aSlow = obs.t < 0.7 ? 0.08 : 0.04;
          dFast.x += aFast * (rawX - dFast.x);
          dFast.z += aFast * (rawZ - dFast.z);
          dSlow.x += aSlow * (rawX - dSlow.x);
          dSlow.z += aSlow * (rawZ - dSlow.z);

          if (p.y > 0.085) {
            dFast.y += 0.22 * (rawY - dFast.y);
            dSlow.y += 0.07 * (rawY - dSlow.y);
          }
        }

        prevVf.x = vf.x;
        prevVf.y = vf.y;
        prevVf.z = vf.z;
        prevAtt.roll = att.roll;
        prevAtt.pitch = att.pitch;
        prevAtt.yaw = att.yaw;
      }

      if (abortBoost > 0) abortBoost--;

      // Detect gate transition and engage gate exit lock
      if (obs.gateIndex !== targetIndex) {
        targetIndex = obs.gateIndex;
        exitLock = true;
        phase = 'align';
        settle = 0;
        abortBoost = 0;
        if (prevGate) {
          ref.x = Math.max(ref.x, p.x + 0.12);
          ref.y = prevGate.y;
          ref.z = prevGate.z;
        }
        if (obs.gateIndex === 3) {
          padY = Math.min(0.72, Math.max(0.50, p.y));
        }
      }

      // Update gate exit lock status
      if (exitLock && prevGate) {
        if (p.x > prevGate.x + 0.20 && vf.x > -0.05) {
          exitLock = false;
        }
      } else {
        exitLock = false;
      }

      let target;
      let maxRefStep = 1.45 * dt;
      let alphaRef = 0.13;

      if (exitLock && prevGate) {
        // 5. Gate exit lock: stay locked on exit line until clear of gate plane
        target = { x: prevGate.x + 0.34, y: prevGate.y, z: prevGate.z };
        ref.x = Math.max(ref.x, p.x + 0.06);
        maxRefStep = 1.25 * dt;
        alphaRef = 0.18;
      } else if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        prevGate = { x: gate.x, y: gate.y, z: gate.z };
        const errYZ = Math.hypot(p.y - gate.y, p.z - gate.z);
        const alignX = errYZ > 0.18 ? gate.x - 0.50 : gate.x - 0.40;

        if (phase === 'align') {
          target = { x: alignX, y: gate.y, z: gate.z };
          if (p.x > alignX - 0.05 && vf.x > 0.22 && errYZ > 0.12) {
            ref.x = Math.min(ref.x, alignX);
          }
          const ex = Math.abs(p.x - (gate.x - 0.40));
          const ey = Math.abs(p.y - gate.y);
          const ez = Math.abs(p.z - gate.z);
          const spd = Math.hypot(vf.x, vf.y, vf.z);
          const gustDelta = Math.hypot(dFast.x - dSlow.x, dFast.z - dSlow.z);

          if (p.x < gate.x - 0.18 && ex < 0.105 && ey < 0.068 && ez < 0.052 &&
              spd < 0.28 && Math.abs(vf.z) < 0.11 && Math.abs(vf.y) < 0.14 && gustDelta < 0.85) {
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
          target = { x: gate.x + 0.34, y: gate.y, z: gate.z };
          maxRefStep = 0.78 * dt;
          alphaRef = 0.14;

          // 6. Predictive cross abort (only when we can safely stop before the gate plane)
          const dxGate = gate.x - p.x;
          const vxPos = Math.max(0, vf.x);
          const axGust = Math.max(0, dFast.x - dSlow.x);
          const stopDist = Math.max(0.14, vxPos * 0.15 + (vxPos * vxPos) / 6.5 + 0.02 * axGust);
          if (dxGate > stopDist) {
            const tLook = clamp(dxGate / Math.max(vf.x, 0.35), 0.08, 0.32);
            const gustY = dFast.y - dSlow.y;
            const gustZ = dFast.z - dSlow.z;
            const yPred = p.y + vf.y * tLook + 0.3 * gustY * tLook * tLook;
            const zPred = p.z + vf.z * tLook + 0.3 * gustZ * tLook * tLook;
            const eyNow = Math.abs(p.y - gate.y);
            const ezNow = Math.abs(p.z - gate.z);
            const eyPred = Math.abs(yPred - gate.y);
            const ezPred = Math.abs(zPred - gate.z);

            const unsafeZ = ezNow > 0.072 || ezPred > 0.080 || (dxGate > 0.18 && Math.abs(gustZ) > 1.25);
            const unsafeY = eyNow > 0.130 || eyPred > 0.145;
            if (unsafeZ || unsafeY) {
              phase = 'align';
              settle = 0;
              abortBoost = 25;
              target = { x: gate.x - 0.44, y: gate.y, z: gate.z };
              ref.x = gate.x - 0.52;
              intPos.x *= 0.3;
            }
          } else {
            // Committed to gate exit lock: keep ref centered in gate opening
            ref.y = gate.y;
            ref.z = gate.z;
          }
        }
      } else {
        // 7. Ramp pad descent
        const pad = obs.pad.centre;
        const errXZ = Math.hypot(p.x - pad.x, p.z - pad.z);
        const spdXZ = Math.hypot(vf.x, vf.z);

        if (!touchedDown) {
          if (errXZ < 0.30) {
            if (p.y > 0.24) {
              const rate = errXZ < 0.15 ? 0.52 : 0.30;
              padY = Math.max(0.20, padY - rate * dt);
            } else if (errXZ < 0.115 && spdXZ < 0.24) {
              padY = Math.max(0.045, padY - 0.24 * dt);
            } else if (errXZ > 0.14) {
              padY = Math.min(0.26, padY + 0.15 * dt);
            }
          } else {
            padY = Math.max(0.50, Math.min(0.70, padY));
          }
          if (p.y <= 0.088 && errXZ < 0.125 && Math.hypot(vf.x, vf.y, vf.z) < 0.38) {
            touchedDown = true;
          }
        }
        if (touchedDown) {
          if (errXZ > 0.14 || p.y > 0.10) {
            touchedDown = false;
            padY = 0.18;
          } else {
            padY = 0.02;
          }
        }
        target = { x: pad.x, y: padY, z: pad.z };
        maxRefStep = 1.35 * dt;
        alphaRef = 0.13;
      }

      // 4. Smooth ref filter
      const dx = target.x - ref.x;
      const dy = target.y - ref.y;
      const dz = target.z - ref.z;
      const dMag = Math.hypot(dx, dy, dz);
      if (dMag > 1e-6) {
        const step = Math.min(maxRefStep, dMag * alphaRef);
        const s = step / dMag;
        ref.x += dx * s;
        ref.y += dy * s;
        ref.z += dz * s;
      }

      // 3. Yaw + local pos integrals (per-axis local check so crossing in X never wipes Z/Y integral)
      yawInt = clamp(yawInt + att.yaw * dt, -0.6, 0.6);
      if (Math.abs(target.x - p.x) < 0.35) {
        intPos.x = clamp(intPos.x + (ref.x - p.x) * dt, -0.55, 0.55);
      } else {
        intPos.x *= 0.96;
      }
      if (Math.abs(target.z - p.z) < 0.30) {
        intPos.z = clamp(intPos.z + (ref.z - p.z) * dt, -0.55, 0.55);
      } else {
        intPos.z *= 0.96;
      }
      if (Math.abs(target.y - p.y) < 0.28 && p.y > 0.09 && !touchedDown) {
        intPos.y = clamp(intPos.y + (ref.y - p.y) * dt, -0.45, 0.45);
      } else {
        intPos.y *= 0.96;
      }

      // Position control with disturbance rejection
      const ex = ref.x - p.x;
      const ey = ref.y - p.y;
      const ez = ref.z - p.z;

      const distX = 0.75 * dFast.x + 0.25 * dSlow.x;
      const distY = 0.35 * dFast.y + 0.65 * dSlow.y;
      const distZ = 0.75 * dFast.z + 0.25 * dSlow.z;

      const axLim = abortBoost > 0 ? 5.6 : 4.4;
      const ax = clamp(
        (abortBoost > 0 ? 6.2 : 5.0) * ex - (abortBoost > 0 ? 4.2 : 3.6) * vf.x + 2.2 * intPos.x - distX + dragNom * vf.x,
        -axLim, 4.4
      );
      const az = clamp(
        5.2 * ez - 3.6 * vf.z + 2.2 * intPos.z - distZ + dragNom * vf.z,
        -4.4, 4.4
      );
      const ay = clamp(
        6.0 * ey - 3.8 * vf.y + 2.0 * intPos.y - distY + dragNom * vf.y,
        -3.5, 4.8
      );

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const ux = cy * ax - sy * az;
      const uz = sy * ax + cy * az;
      const uy = Math.max(4.0, g + ay);

      const maxTilt = touchedDown ? 0.13 : (abortBoost > 0 ? 0.55 : 0.48);
      const pitch = clamp(Math.atan2(ux, uy), -maxTilt, maxTilt);
      const roll = clamp(-Math.atan2(uz, Math.hypot(ux, uy)), -maxTilt, maxTilt);

      const cosTilt = Math.max(0.7, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = (mNom * (g + ay)) / (16.0 * cosTilt);
      if (touchedDown) {
        thrust = Math.min(thrust, 0.16);
      }
      thrust = clamp(thrust, 0, 1);
      lastThrust = thrust;

      const yawRate = clamp(-3.2 * att.yaw - 4.5 * yawInt - 0.15 * rates.yaw, -2, 2);

      return { thrust, roll, pitch, yawRate };
    },
  };
}
