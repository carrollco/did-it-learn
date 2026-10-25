export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01, g = 9.81;
  const mNom = 0.725, fMax = 15.5, specScale = fMax / mNom, dragCoeff = 0.42 / mNom;

  let init = false;
  let vf = { x: 0, y: 0, z: 0 }, prevVf = { x: 0, y: 0, z: 0 };
  let prevAtt = { roll: 0, pitch: 0, yaw: 0 };
  let thrustEst = 0, lastThrust = 0.45;
  let dSlow = { x: 0, y: 0, z: 0 }, dFast = { x: 0, y: 0, z: 0 };
  let intPos = { x: 0, y: 0, z: 0 }, yawInt = 0;
  let ref = { x: 0, y: 0, z: 0 }, refVel = { x: 0, y: 0, z: 0 };

  let gateIdx = 0, phase = 'align', settle = 0, phaseTime = 0;
  let lastGate = null, lockedGate = null, exitTimer = 0;
  let descendY = 0.60;

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (!init) {
        init = true;
        vf = { x: v.x, y: v.y, z: v.z };
        prevVf = { x: v.x, y: v.y, z: v.z };
        prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };
        ref = { x: p.x, y: Math.max(p.y, 0.15), z: p.z };
        thrustEst = (mNom * g) / fMax;
        lastThrust = thrustEst;
      }

      // 1. Filter velocity
      prevVf.x = vf.x; prevVf.y = vf.y; prevVf.z = vf.z;
      const alphaV = 0.42;
      vf.x += alphaV * (v.x - vf.x);
      vf.y += alphaV * (v.y - vf.y);
      vf.z += alphaV * (v.z - vf.z);

      // 2. Fast/slow acceleration disturbance observer
      thrustEst += (lastThrust - thrustEst) * (1 - Math.exp(-dt / 0.038));
      const bx = Math.sin(prevAtt.pitch) * Math.cos(prevAtt.roll);
      const bz = -Math.sin(prevAtt.roll);
      const cy0 = Math.cos(prevAtt.yaw), sy0 = Math.sin(prevAtt.yaw);
      const dirX = cy0 * bx + sy0 * bz;
      const dirY = Math.cos(prevAtt.pitch) * Math.cos(prevAtt.roll);
      const dirZ = -sy0 * bx + cy0 * bz;

      const specT = specScale * thrustEst;
      const aModX = specT * dirX - dragCoeff * prevVf.x;
      const aModY = specT * dirY - g - dragCoeff * prevVf.y;
      const aModZ = specT * dirZ - dragCoeff * prevVf.z;

      const aMeasX = (vf.x - prevVf.x) / dt;
      const aMeasY = (vf.y - prevVf.y) / dt;
      const aMeasZ = (vf.z - prevVf.z) / dt;

      if (p.y > 0.088 && obs.t > 0.03) {
        const dRawX = clamp(aMeasX - aModX, -9, 9);
        const dRawY = clamp(aMeasY - aModY, -6, 6);
        const dRawZ = clamp(aMeasZ - aModZ, -9, 9);

        const aSlow = obs.t < 0.55 ? 0.09 : 0.032;
        const aSlowY = obs.t < 0.55 ? 0.10 : 0.042;
        dSlow.x += aSlow * (dRawX - dSlow.x);
        dSlow.y += aSlowY * (dRawY - dSlow.y);
        dSlow.z += aSlow * (dRawZ - dSlow.z);

        dFast.x += 0.24 * (dRawX - dFast.x);
        dFast.y += 0.15 * (dRawY - dFast.y);
        dFast.z += 0.24 * (dRawZ - dFast.z);
      }

      const distX = dSlow.x + 0.88 * (dFast.x - dSlow.x);
      const distY = dSlow.y + 0.50 * (dFast.y - dSlow.y);
      const distZ = dSlow.z + 0.88 * (dFast.z - dSlow.z);

      // 3. Gate navigation with gate exit lock & predictive cross abort
      if (obs.gateIndex > gateIdx) {
        if (lastGate) {
          lockedGate = { x: lastGate.x, y: lastGate.y, z: lastGate.z };
          phase = 'exit';
          exitTimer = 0;
        } else {
          phase = obs.gateIndex < 3 ? 'align' : 'pad';
        }
        gateIdx = obs.gateIndex;
        settle = 0;
        phaseTime = 0;
        intPos.x = 0;
        intPos.z = 0;
      }
      if (obs.nextGate) {
        lastGate = obs.nextGate.centre;
      }

      let target;
      let lockYZ = false;

      if (phase === 'exit' && lockedGate) {
        exitTimer += dt;
        target = { x: lockedGate.x + 0.30, y: lockedGate.y, z: lockedGate.z };
        lockYZ = true;
        if (p.x >= lockedGate.x + 0.20 || exitTimer > 0.50) {
          phase = obs.gateIndex < 3 ? 'align' : 'pad';
          settle = 0;
          phaseTime = 0;
          intPos.x = 0;
          intPos.z = 0;
        }
      }

      if (phase !== 'exit') {
        if (obs.gateIndex < 3) {
          const gate = obs.nextGate.centre;
          phaseTime += dt;
          const errYZ = Math.hypot(p.y - gate.y, p.z - gate.z);
          const baseAlignX = gate.x - 0.42;
          const alignX = errYZ > 0.22 ? gate.x - 0.52 : baseAlignX;

          if (phase === 'align') {
            if (p.x >= gate.x - 0.04 && (Math.abs(p.y - gate.y) > 0.16 || Math.abs(p.z - gate.z) > 0.085)) {
              const safeY = Math.min(2.6, gate.y + 0.48);
              target = { x: p.y > gate.y + 0.38 ? alignX : p.x, y: safeY, z: gate.z };
            } else {
              target = { x: alignX, y: gate.y, z: gate.z };
              const dxA = Math.abs(p.x - baseAlignX);
              const dy = Math.abs(p.y - gate.y);
              const dz = Math.abs(p.z - gate.z);
              const dyPred = Math.abs(p.y + vf.y * 0.20 - gate.y);
              const dzPred = Math.abs(p.z + vf.z * 0.20 - gate.z);
              const gustSpike = Math.hypot(dFast.x - dSlow.x, dFast.z - dSlow.z);

              if (
                p.x < gate.x - 0.18 &&
                dxA < 0.16 &&
                dy < 0.085 &&
                dz < 0.050 &&
                dyPred < 0.10 &&
                dzPred < 0.060 &&
                Math.hypot(vf.y, vf.z) < 0.22 &&
                Math.abs(vf.x) < 0.28 &&
                gustSpike < 1.35
              ) {
                settle++;
              } else {
                settle = Math.max(0, settle - 1);
              }

              const urgent = phaseTime > 3.2 && settle >= 4 && dy < 0.10 && dz < 0.060;
              if (settle >= 9 || urgent) {
                phase = 'cross';
                settle = 0;
                phaseTime = 0;
              }
            }
          }

          if (phase === 'cross') {
            target = { x: gate.x + 0.32, y: gate.y, z: gate.z };
            lockYZ = true;
            const dx = gate.x - p.x;
            if (dx > 0) {
              const tCross = clamp(dx / Math.max(vf.x, 0.35), 0.02, 0.38);
              const yPred = p.y + vf.y * tCross + 0.20 * (dFast.y - dSlow.y) * tCross * tCross;
              const zPred = p.z + vf.z * tCross + 0.20 * (dFast.z - dSlow.z) * tCross * tCross;
              const dy = Math.abs(p.y - gate.y);
              const dz = Math.abs(p.z - gate.z);
              const dyPred = Math.abs(yPred - gate.y);
              const dzPred = Math.abs(zPred - gate.z);

              const vPos = Math.max(0, vf.x);
              const commitDx = 0.07 + vPos * 0.11 + (vPos * vPos) / 8.5;
              if (dx > commitDx && (dzPred > 0.078 || dz > 0.084 || dyPred > 0.155 || dy > 0.165)) {
                // Predictive cross abort while we can still stop before gate.x
                phase = 'align';
                settle = 0;
                phaseTime = 0;
                target = { x: alignX, y: gate.y, z: gate.z };
                ref.x = Math.min(ref.x, p.x - 0.18);
                lockYZ = true;
              }
            }
          }
        } else {
          // 7. Ramp pad descent
          const pad = obs.pad.centre;
          const padErr = Math.hypot(p.x - pad.x, p.z - pad.z);
          const padVel = Math.hypot(vf.x, vf.z);

          if (phase !== 'descend') {
            descendY = 0.60;
            target = { x: pad.x, y: descendY, z: pad.z };
            if (padErr < 0.14 && padVel < 0.30 && Math.abs(p.y - descendY) < 0.22) {
              phase = 'descend';
              descendY = p.y;
            }
          }

          if (phase === 'descend') {
            if (padErr < 0.115 && padVel < 0.35) {
              const rate = p.y > 0.24 ? 0.48 : 0.20;
              descendY = Math.max(0.025, descendY - rate * dt);
            } else if (padErr > 0.135 && p.y > 0.11) {
              descendY = Math.min(0.35, Math.max(descendY, p.y));
            }
            target = { x: pad.x, y: descendY, z: pad.z };
          }
        }
      }

      // 4. Smooth ref filter (3D coordinated step)
      const prevRefX = ref.x, prevRefY = ref.y, prevRefZ = ref.z;
      const maxSpeed = phase === 'cross' || phase === 'exit' ? 2.1 : (phase === 'descend' ? 1.2 : 1.75);
      const alphaRef = phase === 'cross' || phase === 'exit' ? 0.18 : 0.12;

      const rdx = target.x - ref.x;
      const rdy = target.y - ref.y;
      const rdz = target.z - ref.z;
      const rDist = Math.hypot(rdx, rdy, rdz);
      if (rDist > 1e-6) {
        const stepMag = Math.min(rDist * alphaRef, maxSpeed * dt);
        const scale = stepMag / rDist;
        ref.x += rdx * scale;
        ref.y = lockYZ || phase === 'descend' ? target.y : ref.y + rdy * scale;
        ref.z = lockYZ ? target.z : ref.z + rdz * scale;
      }

      if (obs.gateIndex < 3 && phase === 'align') {
        const gate = obs.nextGate.centre;
        if (p.x > gate.x - 0.32 && vf.x > 0.05) {
          ref.x = Math.min(ref.x, gate.x - 0.45);
        }
      }

      refVel.x = (ref.x - prevRefX) / dt;
      refVel.y = (ref.y - prevRefY) / dt;
      refVel.z = (ref.z - prevRefZ) / dt;

      // 5. Local position integrals + yaw integral
      const distToTargetXZ = Math.hypot(target.x - p.x, target.z - p.z);
      if (p.y > 0.085 && distToTargetXZ < 0.42 && phase !== 'cross') {
        intPos.x = clamp(intPos.x + (ref.x - p.x) * dt, -0.45, 0.45);
        intPos.z = clamp(intPos.z + (ref.z - p.z) * dt, -0.45, 0.45);
      } else {
        intPos.x *= 0.96;
        intPos.z *= 0.96;
      }

      if (p.y > 0.085 && Math.abs(target.y - p.y) < 0.42) {
        intPos.y = clamp(intPos.y + (ref.y - p.y) * dt, -0.45, 0.45);
      } else {
        intPos.y *= 0.98;
      }

      if (p.y > 0.085) {
        yawInt = clamp(yawInt + att.yaw * dt, -0.4, 0.4);
      }

      // Position PD + local I + disturbance cancellation
      const nearGate = obs.gateIndex < 3 && Math.abs(obs.nextGate.centre.x - p.x) < 0.35;
      const tightYZ = phase === 'cross' || phase === 'exit' || nearGate;
      const kpX = 6.5, kdX = 4.6, kiX = 3.2;
      const kpZ = tightYZ ? 9.0 : 6.8, kdZ = tightYZ ? 5.4 : 4.6, kiZ = 3.6;
      const kpY = tightYZ ? 7.8 : 7.0, kdY = 4.7, kiY = 3.0;

      const ex = ref.x - p.x;
      const ey = ref.y - p.y;
      const ez = ref.z - p.z;

      let aDesX = kpX * ex + kdX * (0.40 * refVel.x - vf.x) + kiX * intPos.x;
      if (obs.gateIndex < 3 && phase === 'align') {
        const gateX = obs.nextGate.centre.x;
        const marginX = gateX - p.x;
        if (marginX < 0.34 && vf.x > -0.1) {
          aDesX -= 6.5 * (0.34 - marginX) + 3.5 * Math.max(0, vf.x);
        }
      }
      aDesX = clamp(aDesX, -5.5, 4.2);
      const aDesY = clamp(kpY * ey + kdY * (0.35 * refVel.y - vf.y) + kiY * intPos.y, -4.0, 5.0);
      const aDesZ = clamp(kpZ * ez + kdZ * (0.40 * refVel.z - vf.z) + kiZ * intPos.z, -5.0, 5.0);

      const ux = aDesX + dragCoeff * vf.x - distX;
      const uy = Math.max(2.0, aDesY + g + dragCoeff * vf.y - distY);
      const uz = aDesZ + dragCoeff * vf.z - distZ;

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const uFwd = cy * ux - sy * uz;
      const uLeft = sy * ux + cy * uz;

      const onPadGround = phase === 'descend' && p.y <= 0.092 && Math.hypot(p.x - obs.pad.centre.x, p.z - obs.pad.centre.z) < 0.125;
      const maxTilt = onPadGround ? 0.12 : (phase === 'descend' && p.y < 0.18 ? 0.22 : 0.52);

      const pitch = clamp(Math.atan2(uFwd, uy), -maxTilt, maxTilt);
      const roll = clamp(Math.atan2(-uLeft, Math.hypot(uFwd, uy)), -maxTilt, maxTilt);

      const cosTilt = Math.max(0.55, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = clamp((uy / cosTilt) / specScale, 0, 1);
      if (onPadGround && descendY <= 0.04) {
        thrust = Math.min(thrust, 0.22);
      }

      const yawRate = clamp(-3.2 * att.yaw - 2.2 * yawInt - 0.20 * rates.yaw, -2, 2);

      prevAtt.roll = att.roll;
      prevAtt.pitch = att.pitch;
      prevAtt.yaw = att.yaw;
      lastThrust = thrust;

      return { thrust, roll, pitch, yawRate };
    },
  };
}
