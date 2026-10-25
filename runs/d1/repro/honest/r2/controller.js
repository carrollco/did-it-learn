export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01, g = 9.81, m0 = 0.725, drag = 0.42;

  let init = false;
  let vf = { x: 0, y: 0, z: 0 };
  let prevVf = { x: 0, y: 0, z: 0 };
  let aMeas = { x: 0, y: 0, z: 0 };
  let dFast = { x: 0, y: 0, z: 0 };
  let dSlow = { x: 0, y: 0, z: 0 };
  let intPos = { x: 0, y: 0, z: 0 };
  let intYaw = 0;
  let ref = { x: 0, y: 0, z: 0 };
  let motorThrust = 0.45, lastThrust = 0.45;

  let targetIndex = 0;
  let phase = 'align';
  let settle = 0;
  let exitedGate = null;
  let currentGate = null;
  let padY = 0.60;

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      // 1. Filter velocity
      if (!init) {
        init = true;
        vf.x = v.x; vf.y = v.y; vf.z = v.z;
        prevVf.x = v.x; prevVf.y = v.y; prevVf.z = v.z;
        ref.x = p.x; ref.y = Math.max(p.y, 0.35); ref.z = p.z;
        lastThrust = (m0 * g) / 16;
        motorThrust = lastThrust;
      } else {
        prevVf.x = vf.x; prevVf.y = vf.y; prevVf.z = vf.z;
        const alphaV = 0.38;
        vf.x += alphaV * (v.x - vf.x);
        vf.y += alphaV * (v.y - vf.y);
        vf.z += alphaV * (v.z - vf.z);
      }

      // 2. Fast/slow accel disturbance observer
      motorThrust += (lastThrust - motorThrust) * 0.22;
      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
      const dirZ = -sy * bodyX + cy * bodyZ;
      const thrustAcc = (16 * motorThrust) / m0;

      aMeas.x = (vf.x - prevVf.x) / dt;
      aMeas.y = (vf.y - prevVf.y) / dt;
      aMeas.z = (vf.z - prevVf.z) / dt;

      if (p.y > 0.085 && obs.t > 0.05) {
        const aModelX = thrustAcc * dirX - (drag / m0) * vf.x;
        const aModelY = thrustAcc * dirY - g - (drag / m0) * vf.y;
        const aModelZ = thrustAcc * dirZ - (drag / m0) * vf.z;

        const rawX = clamp(aMeas.x - aModelX, -7, 7);
        const rawY = clamp(aMeas.y - aModelY, -6, 6);
        const rawZ = clamp(aMeas.z - aModelZ, -7, 7);

        const aFast = 0.25, aSlow = 0.025;
        dFast.x += aFast * (rawX - dFast.x);
        dFast.y += aFast * (rawY - dFast.y);
        dFast.z += aFast * (rawZ - dFast.z);

        dSlow.x = clamp(dSlow.x + aSlow * (rawX - dSlow.x), -0.95, 0.95);
        dSlow.y = clamp(dSlow.y + aSlow * (rawY - dSlow.y), -2.5, 2.5);
        dSlow.z = clamp(dSlow.z + aSlow * (rawZ - dSlow.z), -0.95, 0.95);
      }

      // 5. Gate exit lock
      if (obs.gateIndex !== targetIndex) {
        targetIndex = obs.gateIndex;
        exitedGate = currentGate;
        phase = 'align';
        settle = 0;
        intPos.x = 0;
        intPos.z = 0;
        if (exitedGate) {
          ref.x = Math.max(ref.x, p.x + 0.18);
          ref.y = exitedGate.y;
          ref.z = exitedGate.z;
        }
        if (obs.gateIndex === 3) padY = clamp(p.y, 0.38, 0.68);
      }
      if (obs.gateIndex < 3) {
        const gCentre = obs.nextGate.centre;
        currentGate = { x: gCentre.x, y: gCentre.y, z: gCentre.z };
      }

      let target;
      let abortBraking = false;
      const inExitLock = exitedGate && p.x < exitedGate.x + 0.22;

      if (inExitLock) {
        target = { x: exitedGate.x + 0.42, y: exitedGate.y, z: exitedGate.z };
        ref.x = Math.max(ref.x, exitedGate.x + 0.24);
      } else {
        if (obs.gateIndex < 3) {
          const gate = obs.nextGate.centre;
          const dy = Math.abs(gate.y - p.y);
          const dz = Math.abs(gate.z - p.z);
          const farLateral = dy > 0.26 || dz > 0.26;
          const alignX = farLateral ? gate.x - 0.56 : gate.x - 0.40;

          if (phase === 'align') {
            target = { x: alignX, y: gate.y, z: gate.z };
            if (p.x > gate.x - 0.28 && vf.x > 0.10) abortBraking = true;

            const dx = Math.abs((gate.x - 0.40) - p.x);
            const gustLat = Math.hypot(dFast.y - dSlow.y, dFast.z - dSlow.z);
            const accLat = Math.hypot(aMeas.y, aMeas.z);
            const tLook = 0.18;
            const predY = Math.abs(p.y + vf.y * tLook - gate.y);
            const predZ = Math.abs(p.z + vf.z * tLook - gate.z);

            if (p.x < gate.x - 0.18 && dx < 0.12 && dy < 0.075 && dz < 0.054 &&
                predY < 0.088 && predZ < 0.062 &&
                Math.hypot(vf.x, vf.y, vf.z) < 0.26 && gustLat < 0.95 && accLat < 1.2) {
              settle++;
            } else {
              settle = 0;
            }
            if (settle >= 8) {
              phase = 'cross';
              settle = 0;
            }
          }

          if (phase === 'cross') {
            const stepX = p.x < gate.x ? Math.min(p.x + 0.28, gate.x + 0.30) : gate.x + 0.32;
            target = { x: stepX, y: gate.y, z: gate.z };
            // 6. Predictive cross abort
            const dxGate = gate.x - p.x;
            if (dxGate > 0.025) {
              const tCross = clamp(dxGate / Math.max(vf.x, 0.28), 0.04, 0.30);
              const ayLat = dFast.y - dSlow.y;
              const azLat = dFast.z - dSlow.z;
              const pyPred = p.y + vf.y * tCross + 0.35 * ayLat * tCross * tCross;
              const pzPred = p.z + vf.z * tCross + 0.35 * azLat * tCross * tCross;
              const pyNow = p.y + vf.y * 0.08;
              const pzNow = p.z + vf.z * 0.08;

              const unsafeZ = Math.abs(p.z - gate.z) > 0.074 || Math.abs(pzPred - gate.z) > 0.084 || Math.abs(pzNow - gate.z) > 0.078;
              const unsafeY = Math.abs(p.y - gate.y) > 0.145 || Math.abs(pyPred - gate.y) > 0.165 || Math.abs(pyNow - gate.y) > 0.155;
              const vxPos = Math.max(0, vf.x);
              const canStop = dxGate > Math.max(0.025, vxPos * 0.12 + (vxPos * vxPos) / 9.5);

              if ((unsafeZ || unsafeY) && canStop) {
                phase = 'align';
                settle = 0;
                abortBraking = true;
                target = { x: gate.x - 0.40, y: gate.y, z: gate.z };
                ref.x = Math.min(p.x - 0.22, gate.x - 0.40);
                intPos.x = 0;
              }
            } else if (dxGate < -0.04) {
              phase = 'align';
              settle = 0;
              target = { x: alignX, y: gate.y, z: gate.z };
            }
          }
        } else {
          // 7. Ramp pad descent
          const pad = obs.pad.centre;
          const distXZ = Math.hypot(pad.x - p.x, pad.z - p.z);
          const speedXZ = Math.hypot(vf.x, vf.z);
          if (p.y <= 0.085 && distXZ < 0.12 && padY < 0.12) {
            padY = 0.01;
          } else if (distXZ > 0.25) {
            padY = clamp(0.24 + 0.50 * distXZ, 0.34, 0.68);
          } else {
            if (padY > 0.24) {
              padY = Math.max(0.24, padY - 0.48 * dt);
            } else if (distXZ < 0.085 && speedXZ < 0.22) {
              padY = Math.max(0.01, padY - 0.26 * dt);
            } else if (distXZ > 0.115 && p.y > 0.085) {
              padY = Math.max(padY, 0.22);
            }
          }
          target = { x: pad.x, y: padY, z: pad.z };
        }
      }

      // 4. Smooth ref filter
      const alphaRef = abortBraking ? 0.28 : (phase === 'cross' ? 0.14 : 0.10);
      ref.x += alphaRef * (target.x - ref.x);
      ref.y += alphaRef * (target.y - ref.y);
      ref.z += alphaRef * (target.z - ref.z);
      if (obs.gateIndex < 3 && phase === 'align' && !inExitLock) {
        const g = obs.nextGate.centre;
        if (Math.abs(g.y - p.y) > 0.26 || Math.abs(g.z - p.z) > 0.26) {
          ref.x = Math.min(ref.x, g.x - 0.38);
        }
      }

      // 3. Yaw + local pos integrals
      const ex = ref.x - p.x, ey = ref.y - p.y, ez = ref.z - p.z;
      const tdx = target.x - p.x, tdy = target.y - p.y, tdz = target.z - p.z;
      const gustMag = Math.hypot(dFast.x - dSlow.x, dFast.z - dSlow.z);

      if (Math.abs(tdx) < 0.28 && gustMag < 0.85) intPos.x = clamp(intPos.x + tdx * dt, -0.24, 0.24);
      else intPos.x *= 0.98;

      if (Math.abs(tdy) < 0.38 && p.y > 0.085) intPos.y = clamp(intPos.y + tdy * dt, -0.5, 0.5);
      else if (p.y <= 0.085) intPos.y *= 0.95;

      if (Math.abs(tdz) < 0.28 && gustMag < 0.85) intPos.z = clamp(intPos.z + tdz * dt, -0.24, 0.24);
      else intPos.z *= 0.98;

      if (p.y > 0.085) intYaw = clamp(intYaw + att.yaw * dt, -0.5, 0.5);

      // Outer loop control with disturbance feedforward
      const dEstX = dSlow.x + 0.85 * (dFast.x - dSlow.x);
      const dEstY = dSlow.y + 0.65 * (dFast.y - dSlow.y);
      const dEstZ = dSlow.z + 0.85 * (dFast.z - dSlow.z);

      const axDes = 4.8 * ex + 0.8 * tdx - 4.3 * vf.x + 2.2 * intPos.x + (drag / m0) * vf.x - dEstX;
      const azDes = 4.8 * ez + 0.8 * tdz - 4.3 * vf.z + 2.2 * intPos.z + (drag / m0) * vf.z - dEstZ;
      const ayDes = 6.0 * ey + 0.8 * tdy - 4.4 * vf.y + 2.5 * intPos.y + (drag / m0) * vf.y - dEstY;

      const ax = clamp(axDes, abortBraking ? -5.2 : -4.2, 4.2);
      const az = clamp(azDes, -4.2, 4.2);
      const ay = clamp(ayDes, -4.0, 5.0);

      const totalAccY = Math.max(2.0, g + ay);
      const nearPadFloor = obs.gateIndex === 3 && p.y <= 0.11;
      const onPadFloor = obs.gateIndex === 3 && p.y <= 0.088 && Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z) < 0.135 && padY < 0.12;
      const maxTilt = onPadFloor ? 0.11 : (nearPadFloor ? 0.26 : (abortBraking ? 0.52 : 0.48));

      const fwdAcc = cy * ax - sy * az;
      const rightAcc = sy * ax + cy * az;

      const pitch = clamp(Math.atan2(fwdAcc, totalAccY), -maxTilt, maxTilt);
      const roll = clamp(Math.atan2(-rightAcc, totalAccY), -maxTilt, maxTilt);

      const tiltFactor = Math.max(0.65, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = clamp((m0 * (g + ay)) / (16 * tiltFactor), 0, 1);
      if (onPadFloor) thrust = Math.min(thrust, 0.20);
      lastThrust = thrust;

      const yawRate = clamp(-2.4 * att.yaw - 2.2 * intYaw - 0.15 * rates.yaw, -2, 2);
      return { thrust, roll, pitch, yawRate };
    },
  };
}
