export default function makeController() {
  const dt = 0.01;
  const g = 9.81;
  const mNom = 0.725;
  const dragCoeff = 0.42;
  const maxTotalThrust = 16.0;

  let init = false;
  let vFilt = { x: 0, y: 0, z: 0 };
  let prevVFilt = { x: 0, y: 0, z: 0 };
  let dFast = { x: 0, y: 0, z: 0 };
  let dSlow = { x: 0, y: 0, z: 0 };
  let motorThrust = 0.45;
  let lastThrust = 0.45;

  let yawInt = 0;
  let posInt = { x: 0, y: 0, z: 0 };

  let ref = { x: 0, y: 0, z: 0 };
  let refVel = { x: 0, y: 0, z: 0 };

  let activeGateIdx = 0;
  let lastGateCentre = null;
  let phase = 'align';
  let settle = 0;
  let padY = 0.95;
  let touchCount = 0;

  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  return {
    step(obs) {
      const p = obs.pos;
      const v = obs.vel;
      const att = obs.att;
      const rates = obs.rates;

      if (!init) {
        init = true;
        vFilt = { x: v.x, y: v.y, z: v.z };
        prevVFilt = { x: v.x, y: v.y, z: v.z };
        ref = { x: p.x, y: Math.max(0.25, p.y), z: p.z };
        motorThrust = (mNom * g) / maxTotalThrust;
        lastThrust = motorThrust;
        activeGateIdx = obs.gateIndex;
        if (obs.nextGate) lastGateCentre = { ...obs.nextGate.centre };
      }

      // 1. Filter velocity
      const alphaV = 0.38;
      vFilt.x += alphaV * (v.x - vFilt.x);
      vFilt.y += alphaV * (v.y - vFilt.y);
      vFilt.z += alphaV * (v.z - vFilt.z);

      // 2. Fast/slow acceleration disturbance observer
      const aMeas = {
        x: (vFilt.x - prevVFilt.x) / dt,
        y: (vFilt.y - prevVFilt.y) / dt,
        z: (vFilt.z - prevVFilt.z) / dt,
      };
      prevVFilt.x = vFilt.x;
      prevVFilt.y = vFilt.y;
      prevVFilt.z = vFilt.z;

      const lagAlpha = 1 - Math.exp(-dt / 0.04);
      motorThrust += (lastThrust - motorThrust) * lagAlpha;

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
      const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
      const bodyX = sp * cr, bodyZ = -sr;
      const dir = {
        x: cy * bodyX + sy * bodyZ,
        y: cp * cr,
        z: -sy * bodyX + cy * bodyZ,
      };

      const specThrust = (maxTotalThrust / mNom) * motorThrust;
      const kDrag = dragCoeff / mNom;
      const aModel = {
        x: specThrust * dir.x - kDrag * vFilt.x,
        y: specThrust * dir.y - kDrag * vFilt.y - g,
        z: specThrust * dir.z - kDrag * vFilt.z,
      };

      if (obs.t > 0.04 && p.y > 0.09) {
        const alphaFast = 0.20;
        const alphaSlow = 0.025;
        for (const axis of ['x', 'y', 'z']) {
          const raw = clamp(aMeas[axis] - aModel[axis], -9.0, 9.0);
          dFast[axis] += alphaFast * (raw - dFast[axis]);
          dSlow[axis] += alphaSlow * (raw - dSlow[axis]);
        }
      }

      // 5. Gate exit lock across gateIndex transition:
      // Keep flying straight out of the just-crossed gate until p.x clears its plane by 0.20m.
      if (obs.gateIndex > activeGateIdx) {
        if (lastGateCentre && p.x < lastGateCentre.x + 0.20) {
          phase = 'exitHold';
        } else {
          const exitedX = lastGateCentre ? lastGateCentre.x : p.x;
          activeGateIdx = obs.gateIndex;
          if (obs.nextGate) lastGateCentre = { ...obs.nextGate.centre };
          phase = activeGateIdx < 3 ? 'align' : 'padApproach';
          settle = 0;
          ref.x = Math.max(ref.x, p.x, exitedX + 0.20);
          refVel.x = Math.max(0, refVel.x);
          posInt = { x: 0, y: posInt.y * 0.5, z: 0 };
          if (activeGateIdx === 3) {
            padY = clamp(p.y, 0.65, 1.0);
            touchCount = 0;
          }
        }
      }

      let target;
      const alignDist = 0.42;
      const exitDist = 0.30;

      if (phase === 'exitHold' && lastGateCentre) {
        target = { x: lastGateCentre.x + exitDist, y: lastGateCentre.y, z: lastGateCentre.z };
      } else if (activeGateIdx < 3) {
        const gate = obs.nextGate.centre;
        lastGateCentre = { ...gate };

        const gustZ = dFast.z - dSlow.z;
        const gustY = dFast.y - dSlow.y;

        if (phase === 'align') {
          const dy = Math.abs(gate.y - p.y);
          const dz = Math.abs(gate.z - p.z);
          // Stay further back in x until z and y are reasonably aligned with the gate
          const standOff = obs.t < 0.45 ? 0.78 : (dz > 0.12 || dy > 0.20 ? 0.58 : alignDist);
          target = { x: gate.x - standOff, y: gate.y, z: gate.z };

          const dx = Math.abs(gate.x - alignDist - p.x);
          const spd = Math.hypot(vFilt.x, vFilt.y, vFilt.z);
          const latOk = dy < 0.07 && dz < 0.048 && Math.abs(vFilt.y) < 0.20 && Math.abs(vFilt.z) < 0.15;
          const gustOk = Math.hypot(gustY, gustZ) < 0.90;
          if (dx < 0.10 && p.x < gate.x - 0.24 && latOk && spd < 0.35 && gustOk) settle++;
          else settle = Math.max(0, settle - 1);

          if (settle >= 10) {
            phase = 'cross';
            settle = 0;
            posInt.x = 0;
          }
        }

        if (phase === 'cross') {
          // 5. Gate exit lock & 6. Predictive cross abort
          const distToPlane = gate.x - p.x;
          if (distToPlane <= 0.13) {
            phase = 'lock';
          } else {
            const vx = Math.max(0.22, vFilt.x);
            const tCross = clamp(distToPlane / vx, 0.04, 0.55);
            const predY = p.y + vFilt.y * tCross + 0.45 * gustY * tCross * tCross;
            const predZ = p.z + vFilt.z * tCross + 0.45 * gustZ * tCross * tCross;
            const curDy = Math.abs(p.y - gate.y);
            const curDz = Math.abs(p.z - gate.z);
            const predDy = Math.abs(predY - gate.y);
            const predDz = Math.abs(predZ - gate.z);

            if (curDz > 0.062 || predDz > 0.070 || curDy > 0.12 || predDy > 0.145 || (distToPlane > 0.17 && Math.abs(gustZ) > 1.15)) {
              phase = 'align';
              settle = 0;
              target = { x: gate.x - 0.48, y: gate.y, z: gate.z };
              ref.x = Math.min(ref.x, gate.x - 0.46);
              refVel.x = -0.6;
            }
          }
        }

        if (phase === 'cross' || phase === 'lock') {
          target = { x: gate.x + exitDist, y: gate.y, z: gate.z };
        }
      } else {
        // 7. Ramp pad descent
        const pad = obs.pad.centre;
        const errXZ = Math.hypot(pad.x - p.x, pad.z - p.z);
        const velXZ = Math.hypot(vFilt.x, vFilt.z);

        if (phase === 'padApproach') {
          if (errXZ < 0.45) {
            padY = Math.max(0.38, padY - 0.55 * dt);
          }
          if (errXZ < 0.13 && velXZ < 0.28) {
            phase = 'descend';
          }
        }

        if (phase === 'descend') {
          if (errXZ < 0.125) {
            const rate = p.y > 0.25 ? 0.50 : 0.34;
            padY = Math.max(0.0, padY - rate * dt);
          } else if (errXZ < 0.15) {
            padY = Math.max(0.0, padY - 0.20 * dt);
          } else {
            padY = Math.min(0.35, Math.max(padY, p.y + 0.02));
          }
        }

        target = { x: pad.x, y: padY, z: pad.z };
      }

      // 4. Smooth ref filter
      const inGate = phase === 'cross' || phase === 'lock' || phase === 'exitHold';
      const maxSpeedX = inGate ? 1.15 : (phase === 'align' ? 0.90 : 1.45);
      const maxSpeedYZ = inGate ? 1.40 : 1.75;
      const tauRefX = inGate ? 5.5 : (phase === 'align' ? 3.4 : 4.8);
      const tauRefYZ = inGate ? 5.5 : 4.8;
      for (const axis of ['x', 'y', 'z']) {
        const mSpd = axis === 'x' ? maxSpeedX : maxSpeedYZ;
        const tRef = axis === 'x' ? tauRefX : tauRefYZ;
        const err = target[axis] - ref[axis];
        const desVel = clamp(err * tRef, -mSpd, mSpd);
        refVel[axis] += 0.32 * (desVel - refVel[axis]);
        ref[axis] += refVel[axis] * dt;
      }
      if (inGate) {
        ref.y = target.y;
        ref.z = target.z;
        refVel.y = 0;
        refVel.z = 0;
      }

      // 3. Yaw + local pos integrals
      const distTarget = Math.hypot(target.x - p.x, target.y - p.y, target.z - p.z);
      if (distTarget < 0.40 && !inGate) {
        posInt.x = clamp(posInt.x + (target.x - p.x) * dt, -0.35, 0.35);
        posInt.z = clamp(posInt.z + (target.z - p.z) * dt, -0.35, 0.35);
      } else if (inGate) {
        posInt.x *= 0.95;
        posInt.z = clamp(posInt.z + (target.z - p.z) * dt, -0.25, 0.25);
      } else {
        posInt.x *= 0.98;
        posInt.z *= 0.98;
      }
      if (Math.abs(target.y - p.y) < 0.45) {
        posInt.y = clamp(posInt.y + (target.y - p.y) * dt, -0.45, 0.45);
      }

      // Position & velocity PD + feedforward + disturbance rejection
      const nearGatePlane = obs.nextGate !== null && p.x > obs.nextGate.centre.x - 0.26;
      const kpX = inGate ? 5.2 : 4.8;
      const kdX = inGate ? 3.9 : 3.8;
      const kpZ = inGate || nearGatePlane ? 7.5 : 5.4;
      const kdZ = inGate || nearGatePlane ? 4.8 : 4.0;
      const kpY = inGate || nearGatePlane ? 6.8 : 6.0;
      const kdY = inGate || nearGatePlane ? 4.2 : 3.8;

      const distBlendX = 0.80 * dFast.x + 0.20 * dSlow.x;
      const distBlendY = 0.55 * dFast.y + 0.45 * dSlow.y;
      const distBlendZ = 0.85 * dFast.z + 0.15 * dSlow.z;

      const ffX = phase === 'align' ? 0.40 : 0.65;
      let ax = kpX * (ref.x - p.x) + kdX * (refVel.x * ffX - vFilt.x) + 1.8 * posInt.x + kDrag * vFilt.x - distBlendX;
      let az = kpZ * (ref.z - p.z) + kdZ * (refVel.z * 0.65 - vFilt.z) + 2.0 * posInt.z + kDrag * vFilt.z - distBlendZ;
      let ay = kpY * (ref.y - p.y) + kdY * (refVel.y * 0.65 - vFilt.y) + 2.2 * posInt.y + kDrag * vFilt.y - distBlendY;

      // Guard against overshooting into gate plane during align while laterally unaligned
      if (obs.nextGate !== null && phase === 'align') {
        const gate = obs.nextGate.centre;
        const predX = p.x + 0.40 * Math.max(0, vFilt.x);
        if (predX > gate.x - 0.28 && (Math.abs(p.z - gate.z) > 0.06 || Math.abs(p.y - gate.y) > 0.12)) {
          ax = Math.min(ax, -3.8 * vFilt.x - 4.5 * (predX - (gate.x - 0.32)));
        }
      }

      ax = clamp(ax, -4.2, 4.2);
      az = clamp(az, -4.2, 4.2);
      ay = clamp(ay, -4.5, 5.5);

      const upY = Math.max(2.0, g + ay);
      const bodyAccX = cy * ax - sy * az;
      const bodyAccZ = sy * ax + cy * az;

      let maxTilt = 0.48;
      if (activeGateIdx === 3 && phase === 'descend' && p.y < 0.16) {
        maxTilt = 0.13;
      }

      let pitch = clamp(Math.atan2(bodyAccX, upY), -maxTilt, maxTilt);
      let roll = clamp(-Math.atan2(bodyAccZ, Math.hypot(bodyAccX, upY)), -maxTilt, maxTilt);

      const tiltCos = Math.max(0.7, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = clamp((mNom * upY) / (maxTotalThrust * tiltCos), 0, 1);

      // Soft touchdown hold on pad
      if (activeGateIdx === 3 && phase === 'descend') {
        const errXZ = Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z);
        if (p.y <= 0.092 && errXZ <= 0.14 && Math.hypot(vFilt.x, vFilt.y, vFilt.z) < 0.45) {
          touchCount++;
          posInt.y *= 0.85;
          const settleScale = clamp(1 - touchCount * 0.09, 0.12, 1.0);
          thrust = Math.min(thrust, 0.20 * settleScale);
          pitch = clamp(pitch, -0.11, 0.11);
          roll = clamp(roll, -0.11, 0.11);
        } else {
          touchCount = Math.max(0, touchCount - 1);
        }
      }

      yawInt = clamp(yawInt + att.yaw * dt, -0.5, 0.5);
      const yawRate = clamp(-3.2 * att.yaw - 1.6 * yawInt - 0.25 * rates.yaw, -2, 2);

      lastThrust = thrust;
      return { thrust, roll, pitch, yawRate };
    },
  };
}
