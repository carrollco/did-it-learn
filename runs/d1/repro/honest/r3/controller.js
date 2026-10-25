export default function makeController() {
  const dt = 0.01;
  const g = 9.81;
  const massNom = 0.725;
  const dragNom = 0.42 / massNom;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  let initialized = false;
  let vf = { x: 0, y: 0, z: 0 };
  let prevVf = { x: 0, y: 0, z: 0 };
  let aExpFilt = { x: 0, y: 0, z: 0 };
  let dFast = { x: 0, y: 0, z: 0 };
  let dSlow = { x: 0, y: 0, z: 0 };
  let motorEst = (massNom * g) / 16;

  let intYaw = 0;
  let intPos = { x: 0, y: 0, z: 0 };
  let ref = { x: 0, y: 0, z: 0 };

  let activeGateIndex = 0;
  let lastGate = null;
  let lockedGate = null;
  let phase = 'align';
  let settle = 0;
  let rampY = 0.60;

  return {
    step(obs) {
      const p = obs.pos;
      const v = obs.vel;
      const att = obs.att;
      const rates = obs.rates;

      if (!initialized) {
        initialized = true;
        vf = { x: v.x, y: v.y, z: v.z };
        prevVf = { x: v.x, y: v.y, z: v.z };
        ref = { x: p.x, y: 0.48, z: p.z };
        activeGateIndex = obs.gateIndex;
        if (obs.nextGate) lastGate = { ...obs.nextGate.centre };
      } else {
        prevVf.x = vf.x;
        prevVf.y = vf.y;
        prevVf.z = vf.z;
        const alphaV = 0.32;
        vf.x += alphaV * (v.x - vf.x);
        vf.y += alphaV * (v.y - vf.y);
        vf.z += alphaV * (v.z - vf.z);

        const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
        const bodyZ = -Math.sin(att.roll);
        const dirX = Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ;
        const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
        const dirZ = -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ;
        const accNom = (16 * motorEst) / massNom;
        const accHoriz = Math.max(5.0, accNom + dSlow.y);

        const aExpX = accHoriz * dirX - dragNom * vf.x;
        const aExpY = accNom * dirY - g - dragNom * vf.y;
        const aExpZ = accHoriz * dirZ - dragNom * vf.z;

        aExpFilt.x += alphaV * (aExpX - aExpFilt.x);
        aExpFilt.y += alphaV * (aExpY - aExpFilt.y);
        aExpFilt.z += alphaV * (aExpZ - aExpFilt.z);

        if (p.y > 0.082) {
          const aMeasX = (vf.x - prevVf.x) / dt;
          const aMeasY = (vf.y - prevVf.y) / dt;
          const aMeasZ = (vf.z - prevVf.z) / dt;

          const rawX = clamp(aMeasX - aExpFilt.x, -8, 8);
          const rawY = clamp(aMeasY - aExpFilt.y, -6, 6);
          const rawZ = clamp(aMeasZ - aExpFilt.z, -8, 8);

          const alphaFast = 0.22;
          const alphaSlow = 0.035;
          dFast.x += alphaFast * (rawX - dFast.x);
          dFast.y += alphaFast * (rawY - dFast.y);
          dFast.z += alphaFast * (rawZ - dFast.z);

          dSlow.x += alphaSlow * (rawX - dSlow.x);
          dSlow.y += alphaSlow * (rawY - dSlow.y);
          dSlow.z += alphaSlow * (rawZ - dSlow.z);
        }
      }

      // Whenever a gate is crossed, engage gate exit lock on that gate until well past its plane
      if (obs.gateIndex !== activeGateIndex) {
        if (lastGate) {
          lockedGate = { ...lastGate };
          ref.x = Math.max(ref.x, lockedGate.x + 0.34);
          ref.y = lockedGate.y;
          ref.z = lockedGate.z;
        }
        activeGateIndex = obs.gateIndex;
        phase = 'align';
        settle = 0;
        intPos.x = 0;
        intPos.z = 0;
      }
      if (obs.nextGate) {
        lastGate = { ...obs.nextGate.centre };
      }

      if (lockedGate && p.x >= lockedGate.x + 0.24) {
        lockedGate = null;
        phase = 'align';
        settle = 0;
      }

      let target;
      if (lockedGate) {
        target = { x: lockedGate.x + 0.38, y: lockedGate.y, z: lockedGate.z };
        ref.x = Math.max(ref.x, lockedGate.x + 0.34);
      } else if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (phase === 'align') {
          const alignX = obs.gateIndex === 0 && p.y < 0.55 ? gate.x - 0.56 : gate.x - 0.42;
          target = { x: alignX, y: gate.y, z: gate.z };
          const dist = Math.hypot(target.x - p.x, target.y - p.y, target.z - p.z);
          const gustTransient = Math.hypot(dFast.x - dSlow.x, dFast.z - dSlow.z);
          if (
            dist < 0.115 &&
            Math.abs(gate.y - p.y) < 0.08 &&
            Math.abs(gate.z - p.z) < 0.048 &&
            Math.abs(gate.z - (p.z + vf.z * 0.18)) < 0.052 &&
            Math.abs(vf.x) < 0.25 &&
            Math.abs(vf.y) < 0.18 &&
            Math.abs(vf.z) < 0.12 &&
            gustTransient < 1.1
          ) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle > 9) {
            phase = 'cross';
            settle = 0;
            intPos.x = 0;
          }
        }
        if (phase === 'cross') {
          const dx = gate.x - p.x;
          // Predictive cross abort active while in front of the gate commit zone (dx > 0.13)
          if (dx > 0.13) {
            const tHit = clamp(dx / Math.max(vf.x, 0.40), 0.09, 0.28);
            const gustY = dFast.y - dSlow.y;
            const gustZ = dFast.z - dSlow.z;
            const predY = p.y + vf.y * tHit + 0.5 * gustY * tHit * tHit;
            const predZ = p.z + vf.z * tHit + 0.5 * gustZ * tHit * tHit;
            const dy = Math.abs(p.y - gate.y);
            const dz = Math.abs(p.z - gate.z);
            const pdy = Math.abs(predY - gate.y);
            const pdz = Math.abs(predZ - gate.z);
            if (
              dy > 0.13 ||
              pdy > 0.145 ||
              dz > 0.062 ||
              pdz > 0.072 ||
              Math.abs(vf.z) > 0.21 ||
              (dx > 0.16 && Math.abs(gustZ) > 1.2)
            ) {
              phase = 'align';
              settle = 0;
              target = { x: gate.x - 0.44, y: gate.y, z: gate.z };
              ref.x = Math.min(target.x, p.x - 0.24);
              intPos.x = 0;
            } else {
              target = { x: gate.x + 0.36, y: gate.y, z: gate.z };
            }
          } else {
            // Inside commit zone (dx <= 0.13): engage gate exit lock
            lockedGate = { x: gate.x, y: gate.y, z: gate.z };
            target = { x: gate.x + 0.38, y: gate.y, z: gate.z };
            ref.x = Math.max(ref.x, gate.x + 0.34);
          }
        }
      } else {
        const pad = obs.pad.centre;
        const hErr = Math.hypot(pad.x - p.x, pad.z - p.z);
        const hSpd = Math.hypot(vf.x, vf.z);
        if (phase !== 'descend') {
          target = { x: pad.x, y: 0.60, z: pad.z };
          if (hErr < 0.105 && Math.abs(0.60 - p.y) < 0.16 && hSpd < 0.24) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle > 6) {
            phase = 'descend';
            rampY = p.y;
          }
        }
        if (phase === 'descend') {
          if (hErr < 0.13 && hSpd < 0.40) {
            const rate = rampY > 0.20 ? 0.58 : 0.28;
            rampY = Math.max(0.02, rampY - rate * dt);
          } else if (hErr > 0.145 && p.y < 0.20) {
            rampY = Math.min(0.22, rampY + 0.20 * dt);
          }
          target = { x: pad.x, y: rampY, z: pad.z };
        }
      }

      // Smooth ref filter
      const stepLimX = phase === 'cross' && !lockedGate ? 0.0085 : 0.036;
      const alphaRef = phase === 'cross' || lockedGate ? 0.15 : 0.115;
      ref.x += clamp((target.x - ref.x) * alphaRef, -0.036, stepLimX);
      ref.y += clamp((target.y - ref.y) * 0.13, -0.036, 0.036);
      ref.z += clamp((target.z - ref.z) * alphaRef, -0.036, 0.036);

      // Local pos integrals
      const exRaw = target.x - p.x;
      const eyRaw = target.y - p.y;
      const ezRaw = target.z - p.z;

      if (Math.abs(exRaw) < 0.26 && Math.abs(vf.x) < 0.4 && phase !== 'cross' && !lockedGate) {
        intPos.x = clamp(intPos.x + exRaw * dt, -0.25, 0.25);
      } else {
        intPos.x *= 0.94;
      }
      if (Math.abs(ezRaw) < 0.26 && Math.abs(vf.z) < 0.4) {
        intPos.z = clamp(intPos.z + ezRaw * dt, -0.25, 0.25);
      } else {
        intPos.z *= 0.94;
      }
      if (Math.abs(eyRaw) < 0.45 && p.y > 0.085) {
        intPos.y = clamp(intPos.y + eyRaw * dt, -0.45, 0.45);
      }

      const ex = ref.x - p.x;
      const ey = ref.y - p.y;
      const ez = ref.z - p.z;

      const inGateCorridor = phase === 'cross' || lockedGate !== null;
      const kpZ = inGateCorridor ? 8.4 : 5.5;
      const kdZ = inGateCorridor ? 4.8 : 4.0;
      const kpX = inGateCorridor ? 5.8 : 5.0;
      const kdX = 3.8;

      const distX = dSlow.x + 1.05 * (dFast.x - dSlow.x);
      const distY = 0.70 * dSlow.y + 0.30 * dFast.y;
      const distZ = dSlow.z + 1.20 * (dFast.z - dSlow.z);

      let axDes = kpX * ex - kdX * vf.x + 1.4 * intPos.x + dragNom * vf.x - distX;
      if (lockedGate && p.x < lockedGate.x + 0.18) {
        axDes = Math.max(axDes, 3.2 * (0.70 - vf.x) + dragNom * vf.x - dSlow.x);
      }
      const azDes = kpZ * ez - kdZ * vf.z + 1.6 * intPos.z + dragNom * vf.z - distZ;
      const ayDes = 6.2 * ey - 4.0 * vf.y + 2.0 * intPos.y + dragNom * vf.y - distY;

      const ax = clamp(axDes, -4.4, 4.2);
      const az = clamp(azDes, -4.4, 4.4);
      const ay = clamp(ayDes, -4.0, 5.0);

      const cy = Math.cos(att.yaw);
      const sy = Math.sin(att.yaw);
      const totalUp = clamp(g + 0.35 * ay, 8.5, 11.5);

      const maxTilt = phase === 'descend' && p.y < 0.16 ? 0.15 : 0.46;
      const pitch = clamp(Math.atan2(cy * ax - sy * az, totalUp), -maxTilt, maxTilt);
      const roll = clamp(Math.atan2(-(sy * ax + cy * az), totalUp), -maxTilt, maxTilt);

      const tiltCos = Math.max(0.7, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = clamp((massNom * (g + ay)) / (16 * tiltCos), 0, 0.88);
      if (phase === 'descend' && p.y <= 0.082 && Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z) < 0.12) {
        thrust = Math.min(thrust, 0.18);
      }

      motorEst += (thrust - motorEst) * (1 - Math.exp(-dt / 0.04));

      // Yaw + integral
      intYaw = clamp(intYaw + att.yaw * dt, -0.5, 0.5);
      const yawRate = clamp(-3.2 * att.yaw - 2.0 * intYaw - 0.3 * rates.yaw, -2, 2);

      return { thrust, roll, pitch, yawRate };
    },
  };
}
