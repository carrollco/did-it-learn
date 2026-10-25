export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const DT = 0.01;
  const G = 9.81;
  const K_THRUST = 21.5; // Nominal max specific thrust (15.5 N / 0.72 kg)
  const K_DRAG = 0.58;   // Nominal linear drag / mass (0.42 / 0.72)

  let init = false;
  let vf = { x: 0, y: 0, z: 0 };
  let aModelF = { x: 0, y: 0, z: 0 };
  let dFast = { x: 0, y: 0, z: 0 };
  let dSlow = { x: 0, y: 0, z: 0 };
  let uLag = 0;
  let lastThrust = 0.45;

  let yawInt = 0;
  let posInt = { x: 0, y: 0, z: 0 };

  let ref = { x: 0, y: 0, z: 0 };
  let refVel = { x: 0, y: 0, z: 0 };

  let gateIdx = 0;
  let phase = 'align'; // 'align', 'cross', 'exit', 'pad'
  let activeGate = null;
  let prevGate = null;
  let settle = 0;
  let padY = 1.0;

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (!init) {
        init = true;
        vf = { x: v.x, y: v.y, z: v.z };
        ref = { x: p.x, y: Math.max(p.y, 0.25), z: p.z };
        if (obs.nextGate) activeGate = { ...obs.nextGate.centre };
      }

      // 1. Filter velocity & 2. Fast/slow accel disturbance observer
      const prevVf = { x: vf.x, y: vf.y, z: vf.z };
      const alphaV = 0.35;
      vf.x += alphaV * (v.x - vf.x);
      vf.y += alphaV * (v.y - vf.y);
      vf.z += alphaV * (v.z - vf.z);

      uLag += (lastThrust - uLag) * (1 - Math.exp(-DT / 0.04));

      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
      const dirZ = -sy * bodyX + cy * bodyZ;

      const aModX = K_THRUST * uLag * dirX - K_DRAG * vf.x;
      const aModY = K_THRUST * uLag * dirY - G - K_DRAG * vf.y;
      const aModZ = K_THRUST * uLag * dirZ - K_DRAG * vf.z;

      aModelF.x += alphaV * (aModX - aModelF.x);
      aModelF.y += alphaV * (aModY - aModelF.y);
      aModelF.z += alphaV * (aModZ - aModelF.z);

      if (obs.t > 0.04 && p.y > 0.105) {
        const aMeasX = (vf.x - prevVf.x) / DT;
        const aMeasY = (vf.y - prevVf.y) / DT;
        const aMeasZ = (vf.z - prevVf.z) / DT;

        const dRawX = clamp(aMeasX - aModelF.x, -6.5, 6.5);
        const dRawY = clamp(aMeasY - aModelF.y, -5.0, 5.0);
        const dRawZ = clamp(aMeasZ - aModelF.z, -6.5, 6.5);

        const alphaFast = 0.25, alphaSlow = 0.035;
        dFast.x = clamp(dFast.x + alphaFast * (dRawX - dFast.x), -5.5, 5.5);
        dFast.y = clamp(dFast.y + alphaFast * (dRawY - dFast.y), -4.5, 4.5);
        dFast.z = clamp(dFast.z + alphaFast * (dRawZ - dFast.z), -5.5, 5.5);

        dSlow.x = clamp(dSlow.x + alphaSlow * (dRawX - dSlow.x), -4.5, 4.5);
        dSlow.y = clamp(dSlow.y + alphaSlow * (dRawY - dSlow.y), -4.5, 4.5);
        dSlow.z = clamp(dSlow.z + alphaSlow * (dRawZ - dSlow.z), -4.5, 4.5);
      }

      // 5. Gate exit lock & 6. Predictive cross abort & 7. Ramp pad descent
      if (gateIdx < 3 && obs.gateIndex > gateIdx) {
        if (phase !== 'exit') {
          phase = 'exit';
        }
      }

      let target;
      let maxRefSpeedXZ = 1.55;
      let maxRefSpeedY = 1.10;

      if (phase === 'exit') {
        const g = activeGate;
        target = { x: g.x + 0.34, y: g.y, z: g.z };
        maxRefSpeedXZ = 1.30;
        if (p.x >= g.x + 0.20 && vf.x > -0.05) {
          prevGate = { ...g };
          gateIdx = obs.gateIndex;
          settle = 0;
          if (gateIdx < 3 && obs.nextGate) {
            activeGate = { ...obs.nextGate.centre };
            phase = 'align';
          } else {
            phase = 'pad';
            padY = clamp(p.y, 0.45, 0.98);
          }
        }
      } else if (gateIdx < 3) {
        if (obs.nextGate) activeGate = { ...obs.nextGate.centre };
        const g = activeGate;

        if (phase === 'align') {
          const climbHold = gateIdx === 0 && p.y < 0.70;
          const alignX = climbHold ? g.x - 0.54 : g.x - 0.41;
          if (climbHold) maxRefSpeedXZ = 0.80;
          target = { x: alignX, y: g.y, z: g.z };
          const ex = Math.abs(p.x - (g.x - 0.41));
          const ey = Math.abs(p.y - g.y);
          const ez = Math.abs(p.z - g.z);
          const gustTransient = Math.hypot(dFast.x - dSlow.x, dFast.z - dSlow.z);
          if (
            !climbHold &&
            ex < 0.12 &&
            ey < 0.072 &&
            ez < 0.048 &&
            Math.abs(vf.x) < 0.25 &&
            Math.abs(vf.y) < 0.18 &&
            Math.abs(vf.z) < 0.15 &&
            gustTransient < 1.4
          ) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= 7) {
            phase = 'cross';
            settle = 0;
          }
        }

        if (phase === 'cross') {
          target = { x: g.x + 0.34, y: g.y, z: g.z };
          maxRefSpeedXZ = 1.10;

          // Predictive cross abort
          const dxGate = g.x - p.x;
          if (dxGate > 0.07) {
            const tPred = clamp(dxGate / Math.max(vf.x, 0.42), 0.05, 0.25);
            const ayGust = dFast.y - dSlow.y;
            const azGust = dFast.z - dSlow.z;
            const yPred = p.y + vf.y * tPred + 0.40 * ayGust * tPred * tPred;
            const zPred = p.z + vf.z * tPred + 0.40 * azGust * tPred * tPred;
            const dyNow = Math.abs(p.y - g.y);
            const dzNow = Math.abs(p.z - g.z);
            const dyPred = Math.abs(yPred - g.y);
            const dzPred = Math.abs(zPred - g.z);

            const canStop = p.x + Math.max(0, vf.x) * 0.12 < g.x - 0.02;
            if (
              canStop &&
              (dyNow > 0.142 || dzNow > 0.076 || dyPred > 0.156 || dzPred > 0.086)
            ) {
              phase = 'align';
              settle = 0;
              target = { x: g.x - 0.41, y: g.y, z: g.z };
              ref.x = Math.min(ref.x, p.x - 0.05);
              refVel.x = Math.min(refVel.x, 0);
            }
          }
        }

        // Guard against being blown back into previous gate plane
        if (prevGate && p.x < prevGate.x + 0.15) {
          target = { x: prevGate.x + 0.34, y: prevGate.y, z: prevGate.z };
        }
      } else {
        // 7. Ramp pad descent
        phase = 'pad';
        const pad = obs.pad.centre;
        const distXZ = Math.hypot(p.x - pad.x, p.z - pad.z);
        const speedXZ = Math.hypot(vf.x, vf.z);

        if (distXZ > 0.25) {
          if (padY > 0.35) padY = Math.max(0.35, padY - 0.65 * DT);
        } else if (distXZ < 0.135 && speedXZ < 0.32) {
          const rate = padY > 0.20 ? 0.62 : 0.28;
          padY = Math.max(-0.02, padY - rate * DT);
        } else {
          if (padY > 0.13) padY = Math.max(0.13, padY - 0.36 * DT);
        }
        target = { x: pad.x, y: padY, z: pad.z };
        maxRefSpeedXZ = 1.40;
      }

      // 4. Smooth ref filter
      const rxErr = target.x - ref.x;
      const ryErr = target.y - ref.y;
      const rzErr = target.z - ref.z;
      const kRef = 4.3;
      let vxDes = rxErr * kRef;
      let vzDes = rzErr * kRef;
      const spdXZ = Math.hypot(vxDes, vzDes);
      if (spdXZ > maxRefSpeedXZ) {
        vxDes *= maxRefSpeedXZ / spdXZ;
        vzDes *= maxRefSpeedXZ / spdXZ;
      }
      const vyDes = clamp(ryErr * kRef, -maxRefSpeedY, maxRefSpeedY);

      const alphaRefVel = 0.23;
      refVel.x += alphaRefVel * (vxDes - refVel.x);
      refVel.y += alphaRefVel * (vyDes - refVel.y);
      refVel.z += alphaRefVel * (vzDes - refVel.z);

      ref.x += refVel.x * DT;
      ref.y += refVel.y * DT;
      ref.z += refVel.z * DT;

      // 3. Yaw + local pos integrals (integrate when ref is settled near target, gentle decay during slews)
      if (obs.t > 0.03 && p.y > 0.09) {
        yawInt = clamp(yawInt + att.yaw * DT, -0.6, 0.6);
      }

      if (Math.abs(target.x - ref.x) < 0.14 && Math.abs(ref.x - p.x) < 0.25 && p.y > 0.08) {
        posInt.x = clamp(posInt.x + (ref.x - p.x) * DT, -0.45, 0.45);
      } else if (phase !== 'cross' && phase !== 'exit') {
        posInt.x *= 0.992;
      }

      if (Math.abs(target.z - ref.z) < 0.14 && Math.abs(ref.z - p.z) < 0.25 && p.y > 0.08) {
        posInt.z = clamp(posInt.z + (ref.z - p.z) * DT, -0.45, 0.45);
      } else {
        posInt.z *= 0.992;
      }

      if (phase === 'pad' && padY < 0.18) {
        posInt.y *= 0.96;
      } else if (Math.abs(target.y - ref.y) < 0.16 && Math.abs(ref.y - p.y) < 0.25 && p.y > 0.085) {
        posInt.y = clamp(posInt.y + (ref.y - p.y) * DT, -0.5, 0.6);
      }

      // Combined disturbance estimate
      const dX = 0.72 * dFast.x + 0.28 * dSlow.x;
      const dY = 0.45 * dFast.y + 0.55 * dSlow.y;
      const dZ = 0.72 * dFast.z + 0.28 * dSlow.z;

      const ex = ref.x - p.x;
      const ey = ref.y - p.y;
      const ez = ref.z - p.z;

      const ax = clamp(
        7.0 * ex + 4.7 * (0.65 * refVel.x - vf.x) + 2.8 * posInt.x - dX + K_DRAG * vf.x,
        -4.5,
        4.5
      );
      const az = clamp(
        7.0 * ez + 4.7 * (0.65 * refVel.z - vf.z) + 2.8 * posInt.z - dZ + K_DRAG * vf.z,
        -4.5,
        4.5
      );
      const ay = clamp(
        7.0 * ey + 4.5 * (0.65 * refVel.y - vf.y) + 2.4 * posInt.y - dY + K_DRAG * vf.y,
        -3.5,
        3.5
      );

      // Desired tilt & thrust (preserve motor headroom for attitude torque)
      const effAccY = clamp(G + ay + dY, 7.5, 11.5);
      let pitch = clamp(Math.atan2(cy * ax - sy * az, effAccY), -0.48, 0.48);
      let roll = clamp(Math.atan2(-(sy * ax + cy * az), effAccY), -0.48, 0.48);

      const tiltCos = Math.max(0.68, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = clamp((G + ay) / (K_THRUST * tiltCos), 0.05, 0.79);

      // Soft landing hold when touching pad
      if (phase === 'pad') {
        const pad = obs.pad.centre;
        const padErr = Math.hypot(p.x - pad.x, p.z - pad.z);
        if (p.y <= 0.088 && padErr < 0.135 && Math.hypot(vf.x, vf.y, vf.z) < 0.38) {
          pitch = clamp(pitch * 0.5, -0.10, 0.10);
          roll = clamp(roll * 0.5, -0.10, 0.10);
          if (p.y <= 0.078 && padY <= 0.02) {
            thrust = Math.min(thrust, 0.10);
            pitch = clamp(pitch * 0.5, -0.05, 0.05);
            roll = clamp(roll * 0.5, -0.05, 0.05);
          }
        }
      }

      const yawRate = clamp(-3.2 * att.yaw - 2.4 * yawInt - 0.15 * rates.yaw, -2, 2);
      lastThrust = thrust;

      return { thrust, roll, pitch, yawRate };
    },
  };
}
