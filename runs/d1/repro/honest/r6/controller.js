export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01;
  const g = 9.81;
  const massNom = 0.725;
  const dragCoeff = 0.42 / massNom;

  let initialized = false;
  let vf = { x: 0, y: 0, z: 0 };
  let prevVf = { x: 0, y: 0, z: 0 };
  let aModFilt = { x: 0, y: 0, z: 0 };
  let ref = { x: 0, y: 0, z: 0 };
  let dFast = { x: 0, y: 0, z: 0 };
  let dSlow = { x: 0, y: 0, z: 0 };
  let posInt = { x: 0, y: 0, z: 0 };
  let yawInt = 0;
  let attLag = { roll: 0, pitch: 0 };
  let attBias = { roll: 0, pitch: 0 };
  let lastDesAtt = { roll: 0, pitch: 0 };
  let thrustEst = 0.45;
  let lastThrust = 0.45;

  let activeIndex = 0;
  let activeGate = null;
  let phase = 'align';
  let settle = 0;
  let padY = 0.65;

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (!initialized) {
        initialized = true;
        vf = { x: v.x, y: v.y, z: v.z };
        prevVf = { x: v.x, y: v.y, z: v.z };
        ref = { x: p.x, y: p.y, z: p.z };
        if (obs.nextGate) {
          activeIndex = obs.gateIndex;
          activeGate = { x: obs.nextGate.centre.x, y: obs.nextGate.centre.y, z: obs.nextGate.centre.z };
        }
      }

      // 1. Filter velocity
      const alphaV = 0.28;
      prevVf.x = vf.x; prevVf.y = vf.y; prevVf.z = vf.z;
      vf.x += alphaV * (v.x - vf.x);
      vf.y += alphaV * (v.y - vf.y);
      vf.z += alphaV * (v.z - vf.z);

      // Track inner-loop attitude response and weak-motor attitude bias
      const alphaAttLag = dt / 0.065;
      attLag.roll += alphaAttLag * (lastDesAtt.roll - attLag.roll);
      attLag.pitch += alphaAttLag * (lastDesAtt.pitch - attLag.pitch);
      if (p.y > 0.095) {
        attBias.roll = clamp(attBias.roll + 0.08 * ((att.roll - attLag.roll) - attBias.roll), -0.22, 0.22);
        attBias.pitch = clamp(attBias.pitch + 0.08 * ((att.pitch - attLag.pitch) - attBias.pitch), -0.22, 0.22);
      }

      // 2. Fast/slow acceleration disturbance observer
      thrustEst += (lastThrust - thrustEst) * (dt / 0.04);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
      const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
      const bodyX = sp * cr, bodyZ = -sr;
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = Math.max(0.5, cp * cr);
      const dirZ = -sy * bodyX + cy * bodyZ;

      const aMeasX = (vf.x - prevVf.x) / dt;
      const aMeasY = (vf.y - prevVf.y) / dt;
      const aMeasZ = (vf.z - prevVf.z) / dt;

      const thrustAccNom = (16.0 / massNom) * thrustEst;
      const aModInstY = thrustAccNom * dirY - g - dragCoeff * v.y;
      const vertAcc = clamp(g + aMeasY + dragCoeff * vf.y, 3.0, 20.0);
      const aModInstX = vertAcc * (dirX / dirY) - dragCoeff * v.x;
      const aModInstZ = vertAcc * (dirZ / dirY) - dragCoeff * v.z;

      aModFilt.x += alphaV * (aModInstX - aModFilt.x);
      aModFilt.y += alphaV * (aModInstY - aModFilt.y);
      aModFilt.z += alphaV * (aModInstZ - aModFilt.z);

      if (p.y > 0.095) {
        const rawX = clamp(aMeasX - aModFilt.x, -4.5, 4.5);
        const rawY = clamp(aMeasY - aModFilt.y, -4.5, 4.5);
        const rawZ = clamp(aMeasZ - aModFilt.z, -4.5, 4.5);

        const aFast = 0.22, aSlow = 0.025;
        dFast.x = clamp(dFast.x + aFast * (rawX - dFast.x), -3.2, 3.2);
        dFast.y = clamp(dFast.y + aFast * (rawY - dFast.y), -3.5, 3.5);
        dFast.z = clamp(dFast.z + aFast * (rawZ - dFast.z), -3.2, 3.2);

        dSlow.x = clamp(dSlow.x + aSlow * (rawX - dSlow.x), -1.5, 1.5);
        dSlow.y = clamp(dSlow.y + aSlow * (rawY - dSlow.y), -3.0, 3.0);
        dSlow.z = clamp(dSlow.z + aSlow * (rawZ - dSlow.z), -1.5, 1.5);
      }

      // 5. Gate exit lock & 6. Predictive cross abort & 7. Ramp pad descent
      let target;
      const alignDist = 0.44;
      const exitDist = 0.32;
      const exitLockClearance = 0.20;

      if (activeIndex < 3) {
        if (obs.gateIndex > activeIndex) {
          if (p.x >= activeGate.x + exitLockClearance) {
            activeIndex = obs.gateIndex;
            settle = 0;
            posInt.x *= 0.2;
            posInt.z *= 0.2;
            if (activeIndex < 3 && obs.nextGate) {
              activeGate = { x: obs.nextGate.centre.x, y: obs.nextGate.centre.y, z: obs.nextGate.centre.z };
              phase = 'align';
            } else {
              phase = 'pad_align';
              padY = 0.65;
            }
          } else {
            phase = 'exit_lock';
          }
        } else if (obs.nextGate) {
          activeGate = { x: obs.nextGate.centre.x, y: obs.nextGate.centre.y, z: obs.nextGate.centre.z };
        }
      }

      if (activeIndex < 3) {
        const gate = activeGate;
        if (phase === 'align') {
          const climbHold = (activeIndex === 0 && p.y < 0.65) ? 0.18 : 0.0;
          target = { x: gate.x - alignDist - climbHold, y: gate.y, z: gate.z };
          const errX = Math.abs((gate.x - alignDist) - p.x);
          const errY = Math.abs(gate.y - p.y);
          const errZ = Math.abs(gate.z - p.z);
          const speed = Math.hypot(vf.x, vf.y, vf.z);
          const gustTransient = Math.hypot(dFast.x - dSlow.x, dFast.z - dSlow.z);
          if (errX < 0.11 && errY < 0.075 && errZ < 0.055 && speed < 0.28 && Math.abs(vf.z) < 0.12 && gustTransient < 0.9) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= 12) {
            phase = 'cross';
            settle = 0;
          }
        } else if (phase === 'cross') {
          target = { x: gate.x + exitDist, y: gate.y, z: gate.z };
          const dx = gate.x - p.x;
          if (dx > 0.03) {
            const vx = Math.max(vf.x, 0.25);
            const tCross = clamp(dx / vx, 0.0, 0.45);
            const predY = p.y + vf.y * tCross;
            const predZ = p.z + vf.z * tCross + 0.35 * (dFast.z - dSlow.z) * tCross * tCross;
            const curErrY = Math.abs(p.y - gate.y);
            const curErrZ = Math.abs(p.z - gate.z);
            const predErrY = Math.abs(predY - gate.y);
            const predErrZ = Math.abs(predZ - gate.z);
            if (curErrY > 0.145 || predErrY > 0.155 || curErrZ > 0.074 || predErrZ > 0.080) {
              phase = 'align';
              settle = 0;
              target = { x: gate.x - alignDist, y: gate.y, z: gate.z };
              ref.x = Math.min(p.x - 0.22, gate.x - alignDist);
              posInt.x = 0;
            }
          }
        } else if (phase === 'exit_lock') {
          target = { x: gate.x + exitDist, y: gate.y, z: gate.z };
        }
      } else {
        const pad = obs.pad.centre;
        const errXZ = Math.hypot(pad.x - p.x, pad.z - p.z);
        const predErrXZ = Math.hypot(pad.x - (p.x + vf.x * 0.22), pad.z - (p.z + vf.z * 0.22));
        const speedXZ = Math.hypot(vf.x, vf.z);
        if (phase === 'pad_align') {
          target = { x: pad.x, y: padY, z: pad.z };
          if (errXZ < 0.085 && speedXZ < 0.20 && Math.abs(p.y - padY) < 0.15) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= 10) {
            phase = 'descend';
            padY = p.y;
          }
        } else {
          if (p.y > 0.105 && (errXZ > 0.095 || predErrXZ > 0.105 || speedXZ > 0.28)) {
            padY = Math.min(0.45, Math.max(padY, p.y));
          } else {
            const rate = p.y > 0.25 ? 0.52 : 0.26;
            padY = Math.max(0.015, padY - rate * dt);
          }
          target = { x: pad.x, y: padY, z: pad.z };
        }
      }

      // 4. Smooth ref filter
      const maxRateXZ = phase === 'cross' ? 1.15 : (phase === 'exit_lock' ? 1.45 : 1.6);
      const maxRateY = 1.4;
      const alphaRef = phase === 'cross' || phase === 'exit_lock' ? 0.11 : 0.08;
      const stepX = clamp((target.x - ref.x) * alphaRef, -maxRateXZ * dt, maxRateXZ * dt);
      const stepY = clamp((target.y - ref.y) * alphaRef, -maxRateY * dt, maxRateY * dt);
      const stepZ = clamp((target.z - ref.z) * alphaRef, -maxRateXZ * dt, maxRateXZ * dt);
      ref.x += stepX;
      ref.y += stepY;
      ref.z += stepZ;
      const refVx = stepX / dt;
      const refVy = stepY / dt;
      const refVz = stepZ / dt;

      // 3. Yaw + local pos integrals
      const distTargetXZ = Math.hypot(target.x - p.x, target.z - p.z);
      if (p.y > 0.09) {
        if (distTargetXZ < 0.35 && phase !== 'cross' && phase !== 'exit_lock') {
          posInt.x = clamp(posInt.x + (target.x - p.x) * dt, -0.35, 0.35);
          posInt.z = clamp(posInt.z + (target.z - p.z) * dt, -0.35, 0.35);
        } else {
          posInt.x *= 0.985;
          posInt.z *= 0.985;
        }
        posInt.y = clamp(posInt.y + (ref.y - p.y) * dt, -0.6, 1.8);
        yawInt = clamp(yawInt + att.yaw * dt, -0.5, 0.5);
      }

      const ex = ref.x - p.x;
      const ey = ref.y - p.y;
      const ez = ref.z - p.z;

      const distSlowWeight = 0.35;
      const distFastWeight = 0.65;
      const distX = distSlowWeight * dSlow.x + distFastWeight * dFast.x;
      const distY = 0.5 * dSlow.y + 0.3 * dFast.y;
      const distZ = distSlowWeight * dSlow.z + distFastWeight * dFast.z;

      const maxAx = phase === 'cross' ? 2.4 : 3.8;
      const minAx = (phase === 'align' && activeIndex < 3 && p.x > activeGate.x - alignDist) ? -4.8 : -3.8;

      const ax = clamp(
        4.4 * ex + 3.5 * (0.35 * refVx - vf.x) + 1.8 * posInt.x + dragCoeff * vf.x - distX,
        minAx, maxAx
      );
      const az = clamp(
        4.8 * ez + 3.7 * (0.35 * refVz - vf.z) + 1.8 * posInt.z + dragCoeff * vf.z - distZ,
        -4.2, 4.2
      );
      const ay = clamp(
        6.0 * ey + 3.8 * (0.4 * refVy - vf.y) + 2.6 * posInt.y + dragCoeff * vf.y - distY,
        -4.0, 5.0
      );

      let maxTilt = 0.56;
      if (phase === 'descend' && p.y <= 0.11) {
        maxTilt = 0.14;
      }

      const desPitch = clamp((cy * ax - sy * az) / g, -0.50, 0.45);
      const desRoll = clamp(-(sy * ax + cy * az) / g, -0.45, 0.45);
      lastDesAtt.pitch = desPitch;
      lastDesAtt.roll = desRoll;

      const pitch = clamp(desPitch - 0.85 * attBias.pitch, -maxTilt, maxTilt);
      const roll = clamp(desRoll - 0.85 * attBias.roll, -maxTilt, maxTilt);

      let thrust = (massNom * (g + ay)) / (16.0 * Math.max(0.65, cr * cp));
      if (phase === 'descend' && p.y <= 0.085 && Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z) < 0.135) {
        thrust = Math.min(thrust, 0.22);
      }
      thrust = clamp(thrust, 0, 1);
      lastThrust = thrust;

      const yawRate = clamp(-3.2 * att.yaw - 2.4 * yawInt - 0.25 * rates.yaw, -2, 2);

      return { thrust, roll, pitch, yawRate };
    },
  };
}
