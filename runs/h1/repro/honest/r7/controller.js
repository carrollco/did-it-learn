export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const simInt = { roll: 0, pitch: 0 };
  const uLag = { roll: 0, pitch: 0, yaw: 0 };
  const biasPerThrust = { roll: 0, pitch: 0, yaw: 0 };
  const torqueBias = { roll: 0, pitch: 0, yaw: 0 };
  let tLag = 0.45;
  let thrustScale = 20.5;
  const windFast = { x: 0, z: 0 };
  const windMid = { x: 0, z: 0 };
  const windSlow = { x: 0, z: 0 };

  let prevObs = null;
  let lastAction = { thrust: 0.48, roll: 0, pitch: 0, yawRate: 0 };
  let targetIndex = -1;
  let phase = 'align';
  let settle = 0;
  let prevGate = null;
  let currentGate = null;
  let pinned = false;

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const dir = {
        x: cy * bodyX + sy * bodyZ,
        y: Math.cos(att.pitch) * Math.cos(att.roll),
        z: -sy * bodyX + cy * bodyZ,
      };

      if (prevObs) {
        const pAtt = prevObs.att, pRates = prevObs.rates, pVel = prevObs.vel;
        const pCy = Math.cos(pAtt.yaw), pSy = Math.sin(pAtt.yaw);
        const pBodyX = Math.sin(pAtt.pitch) * Math.cos(pAtt.roll);
        const pBodyZ = -Math.sin(pAtt.roll);
        const pDir = {
          x: pCy * pBodyX + pSy * pBodyZ,
          y: Math.cos(pAtt.pitch) * Math.cos(pAtt.roll),
          z: -pSy * pBodyX + pCy * pBodyZ,
        };

        const alphaLag = 1 - Math.exp(-dt / 0.038);
        for (const axis of ['roll', 'pitch']) {
          const err = lastAction[axis] - pAtt[axis];
          simInt[axis] = clamp(simInt[axis] + err * dt, -0.35, 0.35);
          const uCmd = 65 * err - 13 * pRates[axis] + 22 * simInt[axis];
          const uEff = uLag[axis] + 0.5 * alphaLag * (uCmd - uLag[axis]);
          uLag[axis] += alphaLag * (uCmd - uLag[axis]);
          const alphaObs = (rates[axis] - pRates[axis]) / dt + 0.4 * 0.5 * (rates[axis] + pRates[axis]);
          if (tLag > 0.15 && p.y > 0.08) {
            const bMeas = (alphaObs - uEff) / Math.max(0.2, tLag);
            biasPerThrust[axis] = clamp(biasPerThrust[axis] + 0.12 * (bMeas - biasPerThrust[axis]), -45, 45);
          }
          torqueBias[axis] = biasPerThrust[axis] * tLag;
        }
        {
          const uCmdYaw = 8 * (lastAction.yawRate - pRates.yaw);
          const uEffYaw = uLag.yaw + 0.5 * alphaLag * (uCmdYaw - uLag.yaw);
          uLag.yaw += alphaLag * (uCmdYaw - uLag.yaw);
          const alphaObsYaw = (rates.yaw - pRates.yaw) / dt + 0.4 * 0.5 * (rates.yaw + pRates.yaw);
          if (tLag > 0.15 && p.y > 0.08) {
            const bMeasYaw = (alphaObsYaw - uEffYaw) / Math.max(0.2, tLag);
            biasPerThrust.yaw = clamp(biasPerThrust.yaw + 0.10 * (bMeasYaw - biasPerThrust.yaw), -25, 25);
          }
          torqueBias.yaw = biasPerThrust.yaw * tLag;
        }

        const tEff = tLag + 0.5 * alphaLag * (lastAction.thrust - tLag);
        tLag += alphaLag * (lastAction.thrust - tLag);

        const dragOverMass = 0.0275 * thrustScale;
        const airborne = p.y > 0.09 && prevObs.pos.y > 0.09 && p.y < 2.88;
        if (airborne) {
          const ayObs = (v.y - pVel.y) / dt;
          const uVert = tEff * pDir.y;
          if (uVert > 0.20) {
            const sMeas = (ayObs + 9.81 + dragOverMass * 0.5 * (v.y + pVel.y)) / uVert;
            thrustScale = clamp(thrustScale + 0.10 * (sMeas - thrustScale), 14.8, 27.0);
          }
          const axObs = (v.x - pVel.x) / dt;
          const azObs = (v.z - pVel.z) / dt;
          const wxMeas = clamp(axObs - thrustScale * tEff * pDir.x + dragOverMass * 0.5 * (v.x + pVel.x), -12, 12);
          const wzMeas = clamp(azObs - thrustScale * tEff * pDir.z + dragOverMass * 0.5 * (v.z + pVel.z), -12, 12);
          windFast.x += 0.28 * (wxMeas - windFast.x);
          windFast.z += 0.28 * (wzMeas - windFast.z);
          windMid.x += 0.08 * (wxMeas - windMid.x);
          windMid.z += 0.08 * (wzMeas - windMid.z);
          if (Math.hypot(windFast.x - windSlow.x, windFast.z - windSlow.z) < 1.8 || obs.t < 1.8) {
            windSlow.x += 0.04 * (wxMeas - windSlow.x);
            windSlow.z += 0.04 * (wzMeas - windSlow.z);
          } else {
            windSlow.x += 0.008 * (wxMeas - windSlow.x);
            windSlow.z += 0.008 * (wzMeas - windSlow.z);
          }
        }
      }

      const dragOverMass = 0.0275 * thrustScale;
      const gustDelta = Math.hypot(windFast.x - windMid.x, windFast.z - windMid.z);
      const gustActive = Math.hypot(windFast.x - windSlow.x, windFast.z - windSlow.z) > 1.6 || gustDelta > 0.9;

      let target = { x: p.x, y: 1.0, z: p.z };
      let vxFf = 0, vzFf = 0, vyFf = 0;
      let kpX = 5.0, kdX = 3.8, kpZ = 6.5, kdZ = 4.4, kpY = 6.5, kdY = 4.2;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          if (currentGate) prevGate = currentGate;
          currentGate = { x: gate.x, y: gate.y, z: gate.z };
          targetIndex = obs.gateIndex;
          phase = 'align';
          settle = 0;
        }

        if (prevGate && p.x < prevGate.x + 0.12) {
          target = { x: prevGate.x + 0.32, y: prevGate.y, z: prevGate.z };
          vxFf = 0.8;
          kpZ = 8.5; kdZ = 5.0; kpY = 7.5; kdY = 4.5;
        } else {
          const dz = p.z - gate.z;
          const dy = p.y - gate.y;
          const dx = gate.x - p.x;

          if (phase === 'align') {
            const alignX = gate.x - 0.30;
            target = { x: alignX, y: gate.y, z: gate.z };
            if (Math.abs(dz) > 0.028 || Math.abs(dy) > 0.10) {
              target.x = Math.min(alignX, gate.x - 0.34);
            }
            kpX = 5.5; kdX = 4.2;
            kpZ = 7.8; kdZ = 4.8;
            kpY = 7.2; kdY = 4.5;

            const zPred = dz + v.z * 0.18;
            const yPred = dy + v.y * 0.18;
            const centered =
              p.x > gate.x - 0.45 &&
              p.x < gate.x - 0.06 &&
              Math.abs(dz) < 0.017 &&
              Math.abs(zPred) < 0.020 &&
              Math.abs(v.z) < 0.08 &&
              Math.abs(dy) < 0.065 &&
              Math.abs(yPred) < 0.08 &&
              Math.abs(v.y) < 0.18 &&
              gustDelta < 0.85;

            if (centered) settle++;
            else settle = Math.max(0, settle - 1);

            if (settle >= 4) {
              phase = 'lunge';
            }
          }

          if (phase === 'lunge') {
            const tCross = clamp(dx / Math.max(0.35, v.x), 0.02, 0.25);
            const zCross = dz + v.z * tCross;
            const yCross = dy + v.y * tCross;
            const canBrake = dx > 0.08 && (v.x * v.x) / (2 * 5.0) < dx - 0.04;
            const unsafe =
              Math.abs(dz) > 0.025 ||
              Math.abs(zCross) > 0.027 ||
              Math.abs(dy) > 0.10 ||
              Math.abs(yCross) > 0.11 ||
              (gustActive && dx > 0.14 && (Math.abs(dz) > 0.016 || Math.abs(v.z) > 0.10));

            if (dx > 0.05 && unsafe && canBrake) {
              phase = 'align';
              settle = 0;
              target = { x: gate.x - 0.32, y: gate.y, z: gate.z };
              kpX = 6.5; kdX = 5.0;
              kpZ = 8.5; kdZ = 5.2;
            } else {
              target = { x: gate.x + 0.38, y: gate.y, z: gate.z };
              vxFf = 1.35;
              kpX = 5.5; kdX = 3.8;
              kpZ = 9.5; kdZ = 5.4;
              kpY = 8.0; kdY = 4.8;
            }
          }
        }
      } else {
        if (targetIndex !== 3) {
          if (currentGate) prevGate = currentGate;
          targetIndex = 3;
          phase = 'pad_align';
          settle = 0;
        }
        const pad = obs.pad.centre;
        if (prevGate && p.x < prevGate.x + 0.10) {
          target = { x: prevGate.x + 0.30, y: prevGate.y, z: prevGate.z };
          vxFf = 0.7;
        } else {
          const distH = Math.hypot(pad.x - p.x, pad.z - p.z);
          const speedH = Math.hypot(v.x, v.z);
          const curTilt = Math.max(Math.abs(att.roll), Math.abs(att.pitch));
          target = { x: pad.x, y: 0.20, z: pad.z };
          kpX = 6.4; kdX = 4.5;
          kpZ = 6.4; kdZ = 4.5;
          kpY = 7.0; kdY = 4.5;

          if (distH > 0.28) {
            target.y = clamp(0.20 + 0.70 * (distH - 0.18), 0.20, 0.90);
          } else if (phase === 'pad_align') {
            target.y = 0.20;
            const tiltOk = curTilt < 0.26 || obs.t > 13.4;
            if (distH < 0.10 && speedH < 0.26 && p.y < 0.27 && tiltOk) {
              settle++;
              if (settle >= 1) phase = 'descend';
            } else {
              settle = 0;
            }
          }

          if (phase === 'descend') {
            if ((distH > 0.135 && p.y > 0.10) || (curTilt > 0.285 && p.y > 0.11 && obs.t < 13.8)) {
              phase = 'pad_align';
              settle = 0;
              target.y = 0.20;
            } else {
              target.y = 0.04;
              vyFf = p.y > 0.11 ? -0.40 : -0.25;
              kpX = 6.8; kdX = 4.8;
              kpZ = 6.8; kdZ = 4.8;
              if (distH < 0.125 && p.y <= 0.086 && Math.hypot(v.x, v.y, v.z) < 0.45 && curTilt < 0.175) {
                pinned = true;
              }
            }
          }

          if (pinned) {
            if (distH > 0.14 || p.y > 0.093) {
              pinned = false;
              phase = 'descend';
            } else {
              const axPin = clamp(5.5 * (pad.x - p.x) - 3.8 * v.x - 0.65 * windFast.x, -1.3, 1.3);
              const azPin = clamp(5.5 * (pad.z - p.z) - 3.8 * v.z - 0.65 * windFast.z, -1.3, 1.3);
              const bxPin = cy * axPin - sy * azPin;
              const bzPin = sy * axPin + cy * azPin;
              const desPitchPin = clamp(bxPin / 9.81, -0.12, 0.12);
              const desRollPin = clamp(-bzPin / 9.81, -0.12, 0.12);
              const thrustPin = 0.08;
              const kpAtt = 82, kdAtt = 15.5;
              const uRoll = kpAtt * (desRollPin - att.roll) - kdAtt * rates.roll + 0.4 * rates.roll - torqueBias.roll;
              const uPitch = kpAtt * (desPitchPin - att.pitch) - kdAtt * rates.pitch + 0.4 * rates.pitch - torqueBias.pitch;
              const rollCmdPin = clamp(att.roll + (uRoll + 13 * rates.roll - 22 * simInt.roll) / 65, -0.25, 0.25);
              const pitchCmdPin = clamp(att.pitch + (uPitch + 13 * rates.pitch - 22 * simInt.pitch) / 65, -0.25, 0.25);
              const uYaw = -16 * att.yaw - 5.5 * rates.yaw + 0.4 * rates.yaw - torqueBias.yaw;
              const yawCmdPin = clamp(rates.yaw + uYaw / 8, -2, 2);
              prevObs = obs;
              lastAction = { thrust: thrustPin, roll: rollCmdPin, pitch: pitchCmdPin, yawRate: yawCmdPin };
              return lastAction;
            }
          }
        }
      }

      const windCmdX = windFast.x + 0.55 * (windFast.x - windMid.x);
      const windCmdZ = windFast.z + 0.55 * (windFast.z - windMid.z);

      const ex = target.x - p.x;
      const ez = target.z - p.z;
      const ey = target.y - p.y;

      const vxDes = clamp(kpX * ex / kdX + vxFf, -2.2, 2.2);
      const vzDes = clamp(kpZ * ez / kdZ + vzFf, -2.0, 2.0);
      const vyDes = clamp(kpY * ey / kdY + vyFf, -1.6, 1.8);

      const axDes = clamp(kdX * (vxDes - v.x), -5.8, 5.8);
      const azDes = clamp(kdZ * (vzDes - v.z), -5.8, 5.8);
      const ayDes = clamp(kdY * (vyDes - v.y), -4.5, 5.5);

      const ayThrust = Math.max(3.5, 9.81 + ayDes + dragOverMass * v.y);
      const axThrust = clamp(axDes - windCmdX + dragOverMass * v.x, -6.5, 6.5);
      const azThrust = clamp(azDes - windCmdZ + dragOverMass * v.z, -6.5, 6.5);

      const bx = cy * axThrust - sy * azThrust;
      const bz = sy * axThrust + cy * azThrust;

      let maxTilt = 0.56;
      if (obs.gateIndex === 3 && p.y < 0.105) {
        maxTilt = 0.15;
      } else if (obs.gateIndex === 3 && p.y < 0.13) {
        maxTilt = 0.28;
      }
      const desPitch = clamp(Math.atan2(bx, ayThrust), -maxTilt, maxTilt);
      const desRoll = clamp(Math.atan2(-bz * Math.cos(desPitch), ayThrust), -maxTilt, maxTilt);

      const kpAtt = 82, kdAtt = 15.5;
      const uDesRoll = kpAtt * (desRoll - att.roll) - kdAtt * rates.roll + 0.4 * rates.roll - torqueBias.roll;
      const uDesPitch = kpAtt * (desPitch - att.pitch) - kdAtt * rates.pitch + 0.4 * rates.pitch - torqueBias.pitch;

      const roll = clamp(att.roll + (uDesRoll + 13 * rates.roll - 22 * simInt.roll) / 65, -0.6, 0.6);
      const pitch = clamp(att.pitch + (uDesPitch + 13 * rates.pitch - 22 * simInt.pitch) / 65, -0.6, 0.6);

      const uDesYaw = -16 * att.yaw - 5.5 * rates.yaw + 0.4 * rates.yaw - torqueBias.yaw;
      const yawRate = clamp(rates.yaw + uDesYaw / 8, -2, 2);

      const thrust = clamp(ayThrust / (thrustScale * Math.max(0.55, dir.y)), 0, 1);

      prevObs = obs;
      lastAction = { thrust, roll, pitch, yawRate };
      return lastAction;
    },
  };
}
