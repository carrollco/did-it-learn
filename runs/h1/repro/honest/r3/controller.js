export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const GATES_X = [-1.15, 0.15, 1.35];
  const GATES_Y = [1.05, 1.40, 1.05];
  const GATES_Z = [-0.85, 0.25, 1.05];

  let stepCount = 0;
  let prevPos = null, prevVel = null, prevAtt = null, prevRates = null;
  let intEst = { roll: 0, pitch: 0 };
  let lastAction = { thrust: 0.5, roll: 0, pitch: 0, yawRate: 0 };
  let uFilt = 0.0;
  let cmdAlphaFilt = { roll: 0, pitch: 0, yaw: 0 };
  let biasPerThrust = { roll: 0, pitch: 0, yaw: 0 };
  let biasTrim = { roll: 0, pitch: 0 };
  let thrustScale = 20.0;
  let windFast = { x: 0, z: 0 };
  let windSlow = { x: 0, z: 0 };
  let integralY = 0;

  let gateIdx = -1;
  let phase = 'align';
  let centeredCount = 0;

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;
      const mAlpha = 1 - Math.exp(-dt / 0.038);

      if (stepCount > 0) {
        for (const axis of ['roll', 'pitch']) {
          const e1 = lastAction[axis] - prevAtt[axis];
          intEst[axis] = clamp(intEst[axis] + e1 * 0.005, -0.35, 0.35);
          const e2 = lastAction[axis] - 0.5 * (prevAtt[axis] + att[axis]);
          intEst[axis] = clamp(intEst[axis] + e2 * 0.005, -0.35, 0.35);
        }
        uFilt += (lastAction.thrust - uFilt) * mAlpha;

        for (const axis of ['roll', 'pitch']) {
          const avgAtt = 0.5 * (prevAtt[axis] + att[axis]);
          const avgRate = 0.5 * (prevRates[axis] + rates[axis]);
          const cmdAlpha = 65 * (lastAction[axis] - avgAtt) - 13 * avgRate + 22 * intEst[axis];
          cmdAlphaFilt[axis] += (cmdAlpha - cmdAlphaFilt[axis]) * mAlpha;
          const actualAlpha = (rates[axis] - prevRates[axis]) / dt;
          const expectedAlpha = cmdAlphaFilt[axis] - 0.4 * avgRate;
          const alphaErr = actualAlpha - expectedAlpha - biasPerThrust[axis] * uFilt - biasTrim[axis];
          if (p.y > 0.085 && prevPos.y > 0.085 && uFilt > 0.15) {
            biasPerThrust[axis] = clamp(biasPerThrust[axis] + 0.08 * alphaErr * uFilt, -32, 32);
            biasTrim[axis] = clamp(biasTrim[axis] + 0.025 * alphaErr, -8, 8);
          }
        }

        const avgYawRate = 0.5 * (prevRates.yaw + rates.yaw);
        const cmdYawAlpha = 8 * (lastAction.yawRate - avgYawRate);
        cmdAlphaFilt.yaw += (cmdYawAlpha - cmdAlphaFilt.yaw) * mAlpha;
        const actualYawAlpha = (rates.yaw - prevRates.yaw) / dt;
        const yawAlphaErr = actualYawAlpha - (cmdAlphaFilt.yaw - 0.4 * avgYawRate) - biasPerThrust.yaw * uFilt;
        if (p.y > 0.085 && prevPos.y > 0.085 && uFilt > 0.15) {
          biasPerThrust.yaw = clamp(biasPerThrust.yaw + 0.06 * yawAlphaErr * uFilt, -10, 10);
        }

        const r = 0.5 * (prevAtt.roll + att.roll);
        const pt = 0.5 * (prevAtt.pitch + att.pitch);
        const yw = 0.5 * (prevAtt.yaw + att.yaw);
        const bx = Math.sin(pt) * Math.cos(r), bz = -Math.sin(r);
        const dir = {
          x: Math.cos(yw) * bx + Math.sin(yw) * bz,
          y: Math.cos(pt) * Math.cos(r),
          z: -Math.sin(yw) * bx + Math.cos(yw) * bz,
        };

        const dragCoeff = 0.02745 * thrustScale;
        const avgVel = {
          x: 0.5 * (prevVel.x + v.x),
          y: 0.5 * (prevVel.y + v.y),
          z: 0.5 * (prevVel.z + v.z),
        };

        if (p.y > 0.085 && prevPos.y > 0.085) {
          const ayMeas = (v.y - prevVel.y) / dt;
          const uy = uFilt * dir.y;
          if (lastAction.thrust > 0.15 && lastAction.thrust < 0.88 && uy > 0.2) {
            const ayPred = thrustScale * uy - dragCoeff * avgVel.y - 9.81;
            const tsGain = stepCount < 120 ? 0.12 : 0.07;
            thrustScale = clamp(thrustScale + tsGain * (ayMeas - ayPred) * uy, 14.5, 27.5);
          }
          const axMeas = (v.x - prevVel.x) / dt;
          const azMeas = (v.z - prevVel.z) / dt;
          const wxMeas = clamp(axMeas - (thrustScale * uFilt * dir.x - dragCoeff * avgVel.x), -11, 11);
          const wzMeas = clamp(azMeas - (thrustScale * uFilt * dir.z - dragCoeff * avgVel.z), -11, 11);
          windFast.x += 0.22 * (wxMeas - windFast.x);
          windFast.z += 0.22 * (wzMeas - windFast.z);
          windSlow.x += 0.035 * (wxMeas - windSlow.x);
          windSlow.z += 0.035 * (wzMeas - windSlow.z);
        }
      }

      const gustX = windFast.x - windSlow.x;
      const gustZ = windFast.z - windSlow.z;
      const gustMag = Math.hypot(gustX, gustZ);

      let targetX = p.x, targetY = 1.0, targetZ = p.z;
      let desVx = 0, desVy = 0, desVz = 0;
      let kpX = 4.6, kdX = 3.8, kpZ = 6.2, kdZ = 4.5, kpY = 6.5, kdY = 4.5;
      let maxAxLo = -5.5, maxAxHi = 4.5, maxAz = 5.5;
      let maxTilt = 0.54;
      let pinPad = false;
      let useVelCapX = false;
      let unalignedNearGate = false;
      let maxVxCap = 1.30;

      const prevGateIdx = obs.gateIndex - 1;
      const nearPrevGate = prevGateIdx >= 0 && p.x < GATES_X[prevGateIdx] + 0.09;

      if (nearPrevGate) {
        targetX = GATES_X[prevGateIdx] + 0.20;
        targetY = GATES_Y[prevGateIdx];
        targetZ = GATES_Z[prevGateIdx];
        desVx = 0.95;
        kpX = 4.5; kdX = 3.5;
        kpZ = 8.0; kdZ = 5.2;
        kpY = 7.5; kdY = 4.8;
      } else if (obs.gateIndex < 3) {
        const g = obs.nextGate.centre;
        if (gateIdx !== obs.gateIndex) {
          gateIdx = obs.gateIndex;
          phase = 'align';
          centeredCount = 0;
        }

        const alignDist = 0.30;
        if (phase === 'align') {
          useVelCapX = true;
          const errZ = Math.abs(g.z - p.z);
          const errY = Math.abs(g.y - p.y);
          const extraStandOff = clamp(1.5 * Math.max(0, errZ - 0.020) + 0.4 * Math.max(0, errY - 0.07), 0, 0.09);
          targetX = g.x - (alignDist + extraStandOff);
          targetY = g.y;
          targetZ = g.z;
          if (errZ > 0.25) {
            maxVxCap = 1.08;
            maxAxLo = -6.5;
          }

          if ((errZ > 0.022 || errY > 0.09 || Math.abs(gustZ) > 1.10) && (p.x + 0.16 * Math.max(0, v.x) > g.x - 0.25)) {
            unalignedNearGate = true;
            maxAxLo = -6.8;
            maxTilt = 0.58;
            kpZ = 7.5; kdZ = 5.0;
          }

          const zPred = p.z + 0.09 * v.z;
          const xReady = p.x > g.x - 0.40 && p.x < g.x - 0.14 && v.x > -0.25 && v.x < 0.45;
          const yReady = errY < 0.085 && Math.abs(v.y) < 0.35;
          const zReady = errZ < 0.018 && Math.abs(g.z - zPred) < 0.020 && Math.abs(v.z) < 0.15;
          const gustCalm = gustMag < 1.10;

          if (xReady && yReady && zReady && gustCalm) {
            centeredCount++;
          } else {
            centeredCount = Math.max(0, centeredCount - 1);
          }
          if (centeredCount >= 3) {
            phase = 'lunge';
          }
        }

        if (phase === 'lunge') {
          const dxGate = g.x - p.x;
          const tCross = clamp(dxGate / Math.max(0.55, v.x), 0.02, 0.24);
          const zAtGate = p.z + v.z * tCross;
          const yAtGate = p.y + v.y * tCross;
          const offCenter = dxGate > 0.11 && (Math.abs(g.z - zAtGate) > 0.025 || Math.abs(g.z - p.z) > 0.024 || Math.abs(g.y - yAtGate) > 0.11);
          const gustHit = dxGate > 0.14 && Math.abs(gustZ) > 1.15;

          if (offCenter || gustHit) {
            phase = 'align';
            centeredCount = 0;
            useVelCapX = true;
            unalignedNearGate = true;
            targetX = g.x - 0.35;
            targetY = g.y;
            targetZ = g.z;
            maxAxLo = -6.8;
            maxTilt = 0.58;
            kpZ = 8.5; kdZ = 5.4;
          } else {
            targetX = g.x + 0.26;
            targetY = g.y;
            targetZ = g.z;
            desVx = 1.40;
            kpX = 5.2; kdX = 3.4;
            kpZ = 8.8; kdZ = 5.5;
            kpY = 7.5; kdY = 4.8;
            maxAz = 6.2;
            maxTilt = 0.56;
          }
        }
      } else {
        const pad = obs.pad.centre;
        if (gateIdx !== 3) {
          gateIdx = 3;
          phase = 'pad_align';
          centeredCount = 0;
        }
        targetX = pad.x;
        targetZ = pad.z;
        useVelCapX = true;
        const distXZ = Math.hypot(pad.x - p.x, pad.z - p.z);
        const speedXZ = Math.hypot(v.x, v.z);

        if (phase === 'pad_align') {
          targetY = clamp(0.18 + 0.55 * (distXZ - 0.08), 0.18, 0.85);
          desVy = p.y > 0.30 ? -0.25 : 0;
          if (distXZ < 0.115 && p.y < 0.26 && speedXZ < 0.28) {
            phase = 'pad_descend';
          }
        }
        if (phase === 'pad_descend') {
          targetY = 0.04;
          desVy = p.y > 0.12 ? -0.28 : -0.20;
          maxTilt = 0.28;
          if ((distXZ > 0.138 || speedXZ > 0.40) && p.y > 0.092) {
            phase = 'pad_align';
          }
          if (p.y <= 0.092 && distXZ <= 0.138 && speedXZ < 0.42) {
            phase = 'pad_pin';
          }
        }
        if (phase === 'pad_pin') {
          if (distXZ > 0.142 || p.y > 0.100) {
            phase = 'pad_align';
          } else {
            pinPad = true;
          }
        }
      }

      const dragCoeff = 0.02745 * thrustScale;

      let axDes;
      if (useVelCapX) {
        const vxRef = unalignedNearGate ? clamp(2.4 * (targetX - p.x) - 0.25, -1.5, 0.4) : clamp(1.9 * (targetX - p.x), -1.35, maxVxCap);
        const gainVx = unalignedNearGate ? 5.8 : 4.8;
        axDes = clamp(gainVx * (vxRef - v.x), maxAxLo, maxAxHi);
      } else {
        axDes = clamp(kpX * (targetX - p.x) + kdX * (desVx - v.x), maxAxLo, maxAxHi);
      }
      let azDes = clamp(kpZ * (targetZ - p.z) + kdZ * (desVz - v.z), -maxAz, maxAz);

      const ey = targetY - p.y;
      if (p.y > 0.085 && !pinPad && Math.abs(ey) < 0.14 && Math.abs(v.y) < 0.35) {
        integralY = clamp(integralY + ey * dt, -0.25, 0.25);
      }
      let ayDes = clamp(kpY * ey + kdY * (desVy - v.y) + 1.5 * integralY, -4.5, 5.0);

      let windCompX = windFast.x + 0.58 * gustX;
      const windCompZ = windFast.z + 0.58 * gustZ;
      if (unalignedNearGate && v.x > -0.05) {
        windCompX = Math.max(-0.6, windCompX);
      }

      const Tx = axDes + dragCoeff * v.x - windCompX;
      const Tz = azDes + dragCoeff * v.z - windCompZ;
      const Ty = Math.max(2.5, 9.81 + ayDes + dragCoeff * v.y);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bx = cy * Tx - sy * Tz;
      const bz = sy * Tx + cy * Tz;

      let pitchDes = clamp(Math.atan2(bx, Ty), -maxTilt, maxTilt);
      let rollDes = clamp(Math.atan2(-bz, Math.hypot(bx, Ty)), -maxTilt, maxTilt);
      let thrust = clamp(Ty / (thrustScale * Math.max(0.55, Math.cos(att.roll) * Math.cos(att.pitch))), 0.05, 0.98);

      if (pinPad) {
        thrust = 0.02;
        pitchDes = clamp(0.35 * (targetX - p.x) - 0.18 * v.x, -0.08, 0.08);
        rollDes = clamp(-(0.35 * (targetZ - p.z) - 0.18 * v.z), -0.08, 0.08);
      }

      const rollTorqueBias = biasPerThrust.roll * thrust + biasTrim.roll;
      const pitchTorqueBias = biasPerThrust.pitch * thrust + biasTrim.pitch;
      const rollCmd = clamp(rollDes - (22 * intEst.roll + rollTorqueBias) / 65, -0.6, 0.6);
      const pitchCmd = clamp(pitchDes - (22 * intEst.pitch + pitchTorqueBias) / 65, -0.6, 0.6);
      const yawRateCmd = clamp(-3.5 * att.yaw - 0.4 * rates.yaw - (biasPerThrust.yaw * thrust) / 8, -2, 2);

      prevPos = { x: p.x, y: p.y, z: p.z };
      prevVel = { x: v.x, y: v.y, z: v.z };
      prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };
      prevRates = { roll: rates.roll, pitch: rates.pitch, yaw: rates.yaw };
      lastAction = { thrust, roll: rollCmd, pitch: pitchCmd, yawRate: yawRateCmd };
      stepCount++;

      return lastAction;
    },
  };
}
