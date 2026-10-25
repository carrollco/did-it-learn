export default function makeController() {
  const dt = 0.01;
  const g = 9.81;
  const nomMass = 0.72;
  const dragCoeff = 0.42;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  let initialized = false;
  let vf = { x: 0, y: 0, z: 0 };
  let prevVf = { x: 0, y: 0, z: 0 };
  let distSlow = { x: 0, y: 0, z: 0 };
  let distFast = { x: 0, y: 0, z: 0 };
  let motorThrust = 0.45;
  let lastThrust = 0.45;

  let yawInt = 0;
  let posInt = { x: 0, y: 0, z: 0 };
  let ref = { x: 0, y: 0, z: 0 };

  let targetIndex = -1;
  let phase = 'align';
  let settle = 0;
  let currentGate = null;
  let lockedGate = null;
  let lastPassedGate = null;
  let exitTimer = 0;
  let descentY = 0.75;

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
        ref = { x: p.x, y: Math.max(p.y, 0.35), z: p.z };
        motorThrust = (nomMass * g) / 16;
        lastThrust = motorThrust;
      }

      // 1. Filter velocity
      const alphaV = 0.42;
      vf.x += alphaV * (v.x - vf.x);
      vf.y += alphaV * (v.y - vf.y);
      vf.z += alphaV * (v.z - vf.z);

      // 2. Fast/slow acceleration disturbance observer
      const aMeas = {
        x: (vf.x - prevVf.x) / dt,
        y: (vf.y - prevVf.y) / dt,
        z: (vf.z - prevVf.z) / dt,
      };
      prevVf.x = vf.x;
      prevVf.y = vf.y;
      prevVf.z = vf.z;

      motorThrust += (lastThrust - motorThrust) * (1 - Math.exp(-dt / 0.04));
      const specThrust = (16 * motorThrust) / nomMass;
      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const dirX = Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ;
      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
      const dirZ = -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ;

      const dragAcc = dragCoeff / nomMass;
      const aModel = {
        x: specThrust * dirX - dragAcc * vf.x,
        y: specThrust * dirY - g - dragAcc * vf.y,
        z: specThrust * dirZ - dragAcc * vf.z,
      };

      if (obs.t > 0.05 && p.y > 0.08) {
        const rawDistX = clamp(aMeas.x - aModel.x, -9, 9);
        const rawDistY = clamp(aMeas.y - aModel.y, -6, 6);
        const rawDistZ = clamp(aMeas.z - aModel.z, -9, 9);

        const alphaSlow = 0.04;
        const alphaFast = 0.26;
        distSlow.x += alphaSlow * (rawDistX - distSlow.x);
        distSlow.y += alphaSlow * (rawDistY - distSlow.y);
        distSlow.z += alphaSlow * (rawDistZ - distSlow.z);

        distFast.x += alphaFast * (rawDistX - distFast.x);
        distFast.y += alphaFast * (rawDistY - distFast.y);
        distFast.z += alphaFast * (rawDistZ - distFast.z);
      }

      // 5 & 6 & 7. Navigation state machine: gate exit lock, predictive cross abort, ramp pad descent
      let target;

      if (targetIndex !== obs.gateIndex) {
        const passed = lockedGate || currentGate;
        if (targetIndex >= 0 && passed) {
          lastPassedGate = { x: passed.x, y: passed.y, z: passed.z };
          lockedGate = { x: passed.x, y: passed.y, z: passed.z };
        }
        if (targetIndex >= 0 && lockedGate && p.x < lockedGate.x + 0.28) {
          targetIndex = obs.gateIndex;
          phase = 'exitLock';
          exitTimer = 0;
          ref.x = Math.max(ref.x, p.x + 0.05);
        } else {
          targetIndex = obs.gateIndex;
          phase = obs.gateIndex < 3 ? 'align' : 'padHover';
          settle = 0;
          lockedGate = null;
          if (obs.gateIndex === 3) {
            descentY = Math.max(0.65, Math.min(0.9, p.y));
            posInt.x *= 0.3;
            posInt.z *= 0.3;
          }
        }
      }

      if (obs.gateIndex < 3) {
        const g0 = obs.nextGate.centre;
        currentGate = { x: g0.x, y: g0.y, z: g0.z };
      }

      if (phase === 'exitLock' && lockedGate) {
        exitTimer += dt;
        ref.x = Math.max(ref.x, p.x + 0.05);
        target = { x: lockedGate.x + 0.40, y: lockedGate.y, z: lockedGate.z };
        if (p.x >= lockedGate.x + 0.26 || (exitTimer > 0.45 && p.x >= lockedGate.x + 0.18)) {
          phase = obs.gateIndex < 3 ? 'align' : 'padHover';
          lockedGate = null;
          settle = 0;
          if (obs.gateIndex === 3) {
            descentY = Math.max(0.65, Math.min(0.9, p.y));
            posInt.x *= 0.3;
            posInt.z *= 0.3;
          }
        }
      }

      if (phase !== 'exitLock') {
        if (obs.gateIndex < 3) {
          const gate = obs.nextGate.centre;
          const errY = Math.abs(gate.y - p.y);
          const errZ = Math.abs(gate.z - p.z);
          const yzReady = errY < 0.16 && errZ < 0.095;
          const alignX = yzReady ? gate.x - 0.43 : gate.x - 0.58;

          if (phase === 'align') {
            target = { x: alignX, y: gate.y, z: gate.z };
            if (p.x > gate.x - 0.38 && (vf.x > 0.10 || !yzReady)) {
              ref.x = Math.min(ref.x, alignX);
            }
            const errX = Math.abs(gate.x - 0.43 - p.x);
            const spd = Math.hypot(vf.x, vf.y, vf.z);
            const gustMag = Math.hypot(distFast.x - distSlow.x, distFast.z - distSlow.z);
            if (errX < 0.11 && errY < 0.08 && errZ < 0.052 && spd < 0.32 && gustMag < 1.05) {
              settle++;
            } else {
              settle = Math.max(0, settle - 1);
            }
            if (settle >= 10) {
              phase = 'cross';
              lockedGate = { x: gate.x, y: gate.y, z: gate.z };
            }
          }

          if (phase === 'cross') {
            lockedGate = { x: gate.x, y: gate.y, z: gate.z };
            target = { x: gate.x + 0.38, y: gate.y, z: gate.z };

            // Predictive cross abort ONLY when drone can physically stop well before the gate
            const dxGate = gate.x - p.x;
            const vxPos = Math.max(0, vf.x);
            const stopDist = vxPos * 0.18 + (vxPos * vxPos) / 6.5;
            if (dxGate > Math.max(0.20, stopDist + 0.05)) {
              const tCross = clamp(dxGate / Math.max(vf.x, 0.25), 0.05, 0.45);
              const zPred = p.z + vf.z * tCross + 0.30 * (distFast.z - distSlow.z) * tCross * tCross;
              const yPred = p.y + vf.y * tCross + 0.30 * (distFast.y - distSlow.y) * tCross * tCross;
              if (
                Math.abs(zPred - gate.z) > 0.075 ||
                errZ > 0.068 ||
                Math.abs(yPred - gate.y) > 0.14 ||
                errY > 0.13
              ) {
                phase = 'align';
                settle = 0;
                target = { x: gate.x - 0.52, y: gate.y, z: gate.z };
                ref.x = target.x;
              }
            }
          }
        } else {
          // Pad approach and ramp descent
          const pad = obs.pad.centre;
          const errXZ = Math.hypot(pad.x - p.x, pad.z - p.z);
          const spdXZ = Math.hypot(vf.x, vf.z);
          if (phase === 'padHover') {
            target = { x: pad.x, y: 0.72, z: pad.z };
            if (errXZ < 0.085 && spdXZ < 0.22 && Math.abs(p.y - 0.72) < 0.15) {
              phase = 'descend';
              descentY = p.y;
            }
          }
          if (phase === 'descend') {
            const tolXZ = p.y < 0.25 ? 0.095 : 0.115;
            if (errXZ < tolXZ) {
              const rate = p.y < 0.22 ? 0.26 : 0.50;
              descentY = Math.max(0.035, descentY - rate * dt);
            } else if (errXZ > 0.125) {
              descentY = Math.min(0.45, Math.max(descentY, p.y + 0.08));
            }
            target = { x: pad.x, y: descentY, z: pad.z };
          }
        }

        // Guard against being blown backwards into the just-passed gate
        if (lastPassedGate && p.x < lastPassedGate.x + 0.26) {
          target = {
            x: Math.max(target.x, lastPassedGate.x + 0.42),
            y: lastPassedGate.y,
            z: lastPassedGate.z,
          };
        }
      }

      // 4. Smooth reference filter
      const alphaRef = phase === 'cross' || phase === 'exitLock' ? 0.16 : 0.10;
      const maxStep = phase === 'cross' || phase === 'exitLock' ? 0.032 : 0.024;
      ref.x += clamp((target.x - ref.x) * alphaRef, -maxStep, maxStep);
      ref.y += clamp((target.y - ref.y) * alphaRef, -maxStep, maxStep);
      ref.z += clamp((target.z - ref.z) * alphaRef, -maxStep, maxStep);

      // 3. Local position integrals & Yaw integral
      const distToTargetXZ = Math.hypot(target.x - p.x, target.z - p.z);
      if (distToTargetXZ < 0.38 && phase !== 'cross' && phase !== 'exitLock') {
        posInt.x = clamp(posInt.x + (target.x - p.x) * dt, -0.35, 0.35);
        posInt.z = clamp(posInt.z + (target.z - p.z) * dt, -0.35, 0.35);
      } else {
        posInt.x *= 0.985;
        posInt.z *= 0.985;
      }

      if (Math.abs(target.y - p.y) < 0.45) {
        posInt.y = clamp(posInt.y + (target.y - p.y) * dt, -0.6, 1.2);
      } else {
        posInt.y *= 0.99;
      }

      // Blend filtered reference and direct target error
      const ex = 0.75 * (ref.x - p.x) + 0.25 * (target.x - p.x);
      const ey = 0.70 * (ref.y - p.y) + 0.30 * (target.y - p.y);
      const ez = 0.70 * (ref.z - p.z) + 0.30 * (target.z - p.z);

      const dCompX = distSlow.x + 0.85 * (distFast.x - distSlow.x);
      const dCompY = distSlow.y + 0.50 * (distFast.y - distSlow.y);
      const dCompZ = distSlow.z + 0.90 * (distFast.z - distSlow.z);

      const nearGate = obs.gateIndex < 3 && p.x > obs.nextGate.centre.x - 0.38;
      const brakingBeforeGate = phase === 'align' && nearGate;
      const kpX = phase === 'cross' || phase === 'exitLock' ? 5.2 : brakingBeforeGate ? 6.2 : 4.5;
      const kdX = phase === 'cross' || phase === 'exitLock' ? 3.8 : brakingBeforeGate ? 4.4 : 3.5;
      const kpZ = phase === 'cross' || phase === 'exitLock' || nearGate ? 6.2 : 4.5;
      const kdZ = phase === 'cross' || phase === 'exitLock' || nearGate ? 4.2 : 3.5;

      const ax = clamp(kpX * ex - kdX * vf.x + 1.8 * posInt.x + dragAcc * vf.x - dCompX, brakingBeforeGate ? -5.5 : -4.5, 4.0);
      const az = clamp(kpZ * ez - kdZ * vf.z + 1.8 * posInt.z + dragAcc * vf.z - dCompZ, -4.5, 4.5);
      const ay = clamp(6.0 * ey - 3.8 * vf.y + 2.2 * posInt.y + dragAcc * vf.y - dCompY, -4.0, 5.0);

      const cy = Math.cos(att.yaw);
      const sy = Math.sin(att.yaw);
      const totalY = Math.max(4.0, g + ay);
      let maxTilt = brakingBeforeGate ? 0.54 : 0.48;
      if (obs.gateIndex === 3 && p.y < 0.20) {
        maxTilt = p.y < 0.11 ? 0.14 : 0.24;
      }
      let pitch = clamp(Math.atan2(cy * ax - sy * az, totalY), -maxTilt, maxTilt);
      let roll = clamp(Math.atan2(-(sy * ax + cy * az), totalY), -maxTilt, maxTilt);

      const tiltCos = Math.max(0.7, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = clamp((nomMass * (g + ay)) / (16 * tiltCos), 0, 1);

      // Soft touchdown hold on pad
      if (obs.gateIndex === 3 && phase === 'descend') {
        const pad = obs.pad.centre;
        const padErr = Math.hypot(pad.x - p.x, pad.z - p.z);
        if (p.y <= 0.092 && padErr <= 0.13) {
          pitch = clamp(pitch, -0.12, 0.12);
          roll = clamp(roll, -0.12, 0.12);
          if (p.y <= 0.078 && padErr <= 0.105 && Math.hypot(vf.x, vf.y, vf.z) < 0.35) {
            thrust = 0;
            pitch = 0;
            roll = 0;
          } else {
            thrust = Math.min(thrust, 0.18);
          }
        }
      }

      yawInt = clamp(yawInt + att.yaw * dt, -0.5, 0.5);
      const yawRate = clamp(-2.6 * att.yaw - 1.6 * yawInt - 0.15 * rates.yaw, -2, 2);

      lastThrust = thrust;
      return { thrust, roll, pitch, yawRate };
    },
  };
}
