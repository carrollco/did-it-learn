export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const alphaLag = 1 - Math.exp(-0.01 / 0.04);
  const gates = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];

  let prevPos = null, prevVel = null, prevAtt = null, prevRates = null, prevAction = null;
  let simIntRoll = 0, simIntPitch = 0;
  let lagAlphaRoll = 0, lagAlphaPitch = 0, lagAlphaYaw = 0, lagThrust = 0.45;
  let biasPerThrustRoll = 0, biasPerThrustPitch = 0;
  let biasResRoll = 0, biasResPitch = 0, yawBias = 0;
  let thrustScale = 20.0;
  let windFastX = 0, windFastZ = 0, windSlowX = 0, windSlowZ = 0;
  let innovX = 0, innovZ = 0;
  let intZ = 0, intY = 0;
  let targetIndex = -1, phase = "align", settle = 0, pinned = false;

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, a = obs.att, r = obs.rates;

      if (prevAction !== null) {
        const midRoll = 0.5 * (prevAtt.roll + a.roll);
        const midPitch = 0.5 * (prevAtt.pitch + a.pitch);
        const midYaw = 0.5 * (prevAtt.yaw + a.yaw);
        const midRateRoll = 0.5 * (prevRates.roll + r.roll);
        const midRatePitch = 0.5 * (prevRates.pitch + r.pitch);
        const midRateYaw = 0.5 * (prevRates.yaw + r.yaw);
        const midVelX = 0.5 * (prevVel.x + v.x);
        const midVelY = 0.5 * (prevVel.y + v.y);
        const midVelZ = 0.5 * (prevVel.z + v.z);

        const errRoll = prevAction.roll - midRoll;
        const errPitch = prevAction.pitch - midPitch;
        simIntRoll = clamp(simIntRoll + errRoll * 0.01, -0.35, 0.35);
        simIntPitch = clamp(simIntPitch + errPitch * 0.01, -0.35, 0.35);

        const cmdAlphaRoll = 65 * errRoll - 13 * midRateRoll + 22 * simIntRoll;
        const cmdAlphaPitch = 65 * errPitch - 13 * midRatePitch + 22 * simIntPitch;
        const cmdAlphaYaw = 8 * (prevAction.yawRate - midRateYaw);

        lagAlphaRoll += (cmdAlphaRoll - lagAlphaRoll) * alphaLag;
        lagAlphaPitch += (cmdAlphaPitch - lagAlphaPitch) * alphaLag;
        lagAlphaYaw += (cmdAlphaYaw - lagAlphaYaw) * alphaLag;
        lagThrust += (prevAction.thrust - lagThrust) * alphaLag;

        const measAlphaRoll = (r.roll - prevRates.roll) / 0.01 + 0.4 * midRateRoll;
        const measAlphaPitch = (r.pitch - prevRates.pitch) / 0.01 + 0.4 * midRatePitch;
        const measAlphaYaw = (r.yaw - prevRates.yaw) / 0.01 + 0.4 * midRateYaw;

        if (obs.t > 0.02 && Math.max(Math.abs(prevAction.roll), Math.abs(prevAction.pitch)) < 0.58) {
          const predBiasRoll = biasPerThrustRoll * lagThrust + biasResRoll;
          const predBiasPitch = biasPerThrustPitch * lagThrust + biasResPitch;
          const diffRoll = measAlphaRoll - lagAlphaRoll - predBiasRoll;
          const diffPitch = measAlphaPitch - lagAlphaPitch - predBiasPitch;
          if (lagThrust > 0.16) {
            biasPerThrustRoll = clamp(biasPerThrustRoll + 0.16 * diffRoll, -32, 32);
            biasPerThrustPitch = clamp(biasPerThrustPitch + 0.16 * diffPitch, -32, 32);
          }
          biasResRoll = clamp(1 * biasResRoll + 0.14 * diffRoll, -14, 14);
          biasResPitch = clamp(1 * biasResPitch + 0.14 * diffPitch, -14, 14);
          yawBias = clamp(yawBias + 0.12 * (measAlphaYaw - lagAlphaYaw - yawBias), -10, 10);
        }

        const airborne = p.y > 0.088 && prevPos.y > 0.088 && p.y < 2.88 &&
          Math.abs(p.x) < 2.78 && Math.abs(p.z) < 2.78;
        if (airborne) {
          const dirY = Math.cos(midPitch) * Math.cos(midRoll);
          const massEst = clamp(15.2 / thrustScale, 0.6, 0.95);
          const dragCoeff = 0.42 / massEst;
          const ayPred = thrustScale * lagThrust * dirY - dragCoeff * midVelY - 9.81;
          const ayMeas = (v.y - prevVel.y) / 0.01;
          const uY = lagThrust * dirY;
          if (uY > 0.18) {
            thrustScale = clamp(thrustScale + 0.14 * (ayMeas - ayPred) * uY, 14.8, 27.2);
          }

          const bodyX = Math.sin(midPitch) * Math.cos(midRoll);
          const bodyZ = -Math.sin(midRoll);
          const cy = Math.cos(midYaw), sy = Math.sin(midYaw);
          const dirX = cy * bodyX + sy * bodyZ;
          const dirZ = -sy * bodyX + cy * bodyZ;

          const axPred = thrustScale * lagThrust * dirX - dragCoeff * midVelX;
          const azPred = thrustScale * lagThrust * dirZ - dragCoeff * midVelZ;
          const wxMeas = (v.x - prevVel.x) / 0.01 - axPred;
          const wzMeas = (v.z - prevVel.z) / 0.01 - azPred;

          innovX = 0.45 * innovX + 0.55 * (wxMeas - windFastX);
          innovZ = 0.45 * innovZ + 0.55 * (wzMeas - windFastZ);
          const shock = Math.hypot(innovX, innovZ);
          const wGain = clamp(0.26 + 0.05 * shock, 0.26, 0.48);

          windFastX = clamp(windFastX + wGain * (wxMeas - windFastX), -11, 11);
          windFastZ = clamp(windFastZ + wGain * (wzMeas - windFastZ), -11, 11);
          windSlowX = clamp(windSlowX + 0.025 * (wxMeas - windSlowX), -11, 11);
          windSlowZ = clamp(windSlowZ + 0.025 * (wzMeas - windSlowZ), -11, 11);
        }
      }

      const massEst = clamp(15.2 / thrustScale, 0.6, 0.95);
      const dragCoeff = 0.42 / massEst;
      const gustShock = Math.hypot(innovX, innovZ);
      const gustDelta = Math.hypot(windFastX - windSlowX, windFastZ - windSlowZ);

      const bXNow = Math.sin(a.pitch) * Math.cos(a.roll);
      const bZNow = -Math.sin(a.roll);
      const cy = Math.cos(a.yaw), sy = Math.sin(a.yaw);
      const dirXNow = cy * bXNow + sy * bZNow;
      const dirZNow = -sy * bXNow + cy * bZNow;
      const dirYNow = Math.max(0.55, Math.cos(a.pitch) * Math.cos(a.roll));
      const axCurr = thrustScale * lagThrust * dirXNow - dragCoeff * v.x + windFastX;
      const azCurr = thrustScale * lagThrust * dirZNow - dragCoeff * v.z + windFastZ;

      let tx = p.x, ty = p.y, tz = p.z;
      let vxDes = 0, vyDes = 0, vzDes = 0;
      let kpX = 6.5, kdX = 4.5, kpZ = 8.8, kdZ = 5.5, kpY = 8.2, kdY = 4.7;
      let clampEx = 0.9, clampEz = 0.8, clampEy = 0.85;

      if (obs.gateIndex < 3) {
        pinned = false;
        const g = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          targetIndex = obs.gateIndex;
          phase = "align";
          settle = 0;
          intZ = 0;
          intY = 0;
        }

        const ez = g.z - p.z;
        const ey = g.y - p.y;

        if (p.x > g.x - 0.50 && p.x < g.x + 0.05 && gustDelta < 2.2) {
          if (Math.abs(ez) < 0.10) intZ = clamp(intZ + ez * 0.01, -0.14, 0.14);
          if (Math.abs(ey) < 0.16) intY = clamp(intY + ey * 0.01, -0.18, 0.18);
        }

        // Follow-through guard after previous gate so we never back into previous gate frame
        const prevG = obs.gateIndex > 0 ? gates[obs.gateIndex - 1] : null;
        if (prevG && p.x < prevG.x + 0.12) {
          tx = prevG.x + 0.28;
          vxDes = 0.85;
          ty = prevG.y + 0.35 * (g.y - prevG.y);
          tz = prevG.z + 0.25 * (g.z - prevG.z);
        } else if (phase === "align") {
          const extraStandoff = clamp(
            1 * Math.max(0, Math.abs(ez) - 0.025) + 0.30 * Math.max(0, Math.abs(ey) - 0.08),
            0,
            0.36
          );
          const standoff = 0.26 + extraStandoff;
          tx = g.x - standoff;
          ty = g.y;
          tz = g.z;
          kpX = 6.6; kdX = 4.6;

          const zPred = p.z + v.z * 0.14;
          const yPred = p.y + v.y * 0.14;
          const late = obs.t > 10.2;
          const centeredZ = Math.abs(ez) < (late ? 0.021 : 0.018) &&
            Math.abs(g.z - zPred) < (late ? 0.024 : 0.021) &&
            Math.abs(v.z) < 0.13;
          const centeredY = Math.abs(ey) < 0.075 &&
            Math.abs(g.y - yPred) < 0.090 &&
            Math.abs(v.y) < 0.28;
          const inFront = p.x > g.x - 0.42 && p.x < g.x - 0.02 && v.x > -0.18;
          const calmGust = gustShock < (late ? 2.8 : 2.1);

          if (centeredZ && centeredY && inFront && calmGust) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }

          if (settle >= (late ? 2 : 3)) {
            phase = "lunge";
          } else if (p.x + Math.max(0, v.x) * 0.20 > g.x - 0.06 && (!centeredZ || !centeredY)) {
            tx = g.x - 0.36;
            kpX = 8.5; kdX = 5.8;
          }
        }

        if (phase === "lunge") {
          const dtCross = clamp((g.x - p.x) / Math.max(0.45, v.x), 0.02, 0.30);
          const zCross = p.z + v.z * dtCross;
          const yCross = p.y + v.y * dtCross;
          const stopDist = Math.max(0, v.x) * 0.14 + (v.x > 0 ? (v.x * v.x) / 7.5 : 0) + 0.05;
          const canStop = (g.x - p.x) > stopDist;

          if (canStop && (Math.abs(g.z - zCross) > 0.026 || Math.abs(g.y - yCross) > 0.115)) {
            phase = "align";
            settle = 0;
            tx = g.x - 0.32;
            ty = g.y;
            tz = g.z;
            kpX = 8.5; kdX = 5.8;
          } else {
            tx = g.x + 0.40;
            vxDes = 1.4;
            ty = g.y;
            tz = g.z;
            kpX = 7.8; kdX = 4.3;
            kpZ = 10.2; kdZ = 5.9;
            kpY = 9.0; kdY = 5.0;
          }
        }
      } else {
        intZ = 0;
        intY = 0;
        const pad = obs.pad.centre;
        tx = pad.x;
        tz = pad.z;
        const distH = Math.hypot(pad.x - p.x, pad.z - p.z);
        const speedH = Math.hypot(v.x, v.z);
        const speedTot = Math.hypot(v.x, v.y, v.z);

        kpX = 7.4; kdX = 4.9;
        kpZ = 7.4; kdZ = 4.9;
        kpY = 7.8; kdY = 4.8;
        clampEx = 0.90; clampEz = 0.90;

        if (p.y <= 0.093 && distH <= 0.138 && speedTot < 0.47) {
          pinned = true;
        }
        if (pinned && (distH > 0.146 || p.y > 0.094)) {
          pinned = false;
        }

        if (distH > 0.13) {
          ty = clamp(0.12 + 0.72 * distH, 0.12, 0.75);
          if (distH < 0.35) vyDes = -0.22;
        } else if (p.y < 0.22 && speedH > 0.38) {
          ty = 0.14;
        } else {
          ty = 0.03;
          vyDes = -0.34;
          kpY = 7.2; kdY = 5.0;
        }
      }

      const ex = tx - p.x;
      const ey = ty - p.y;
      const ez = tz - p.z;

      let axDes = kpX * clamp(ex, -clampEx, clampEx) + kdX * (vxDes - v.x) - 0.18 * axCurr;
      let azDes = kpZ * clamp(ez, -clampEz, clampEz) + kdZ * (vzDes - v.z) + 2.5 * intZ - 0.18 * azCurr;
      let ayDes = kpY * clamp(ey, -clampEy, clampEy) + kdY * (vyDes - v.y) + 2.0 * intY;

      if (gustDelta > 1.2) {
        axDes += clamp(-0.18 * innovX, -1.5, 1.5);
        azDes += clamp(-0.18 * innovZ, -1.5, 1.5);
      }

      axDes = clamp(axDes, -5.0, 5.0);
      azDes = clamp(azDes, -5.2, 5.2);
      ayDes = clamp(ayDes, -4.2, 5.4);

      if (p.y < 0.28 && v.y < -0.36) {
        ayDes = Math.max(ayDes, -7.2 * (v.y + 0.28));
      }
      if (p.y > 2.55 && v.y > 0.20) {
        ayDes = Math.min(ayDes, -6.0 * (v.y - 0.10));
      }

      const axThrust = axDes + dragCoeff * v.x - windFastX;
      const azThrust = azDes + dragCoeff * v.z - windFastZ;
      const ayThrust = clamp(9.81 + ayDes + dragCoeff * v.y, 2.2, 22.0);

      const bodyXRatio = cy * axThrust - sy * azThrust;
      const bodyZRatio = sy * axThrust + cy * azThrust;

      let pitchDes = clamp(Math.atan2(bodyXRatio, ayThrust), -0.50, 0.50);
      let rollDes = clamp(-Math.atan2(bodyZRatio, Math.hypot(bodyXRatio, ayThrust)), -0.50, 0.50);

      if (obs.gateIndex === 3 && p.y < 0.18) {
        pitchDes = clamp(pitchDes, -0.22, 0.22);
        rollDes = clamp(rollDes, -0.22, 0.22);
      }

      let thrust = clamp(ayThrust / (thrustScale * dirYNow), 0.05, 1.0);

      let totBiasRoll, totBiasPitch;
      if (pinned) {
        thrust = 0.06;
        pitchDes = clamp(pitchDes * 0.25, -0.07, 0.07);
        rollDes = clamp(rollDes * 0.25, -0.07, 0.07);
        totBiasRoll = biasPerThrustRoll * thrust;
        totBiasPitch = biasPerThrustPitch * thrust;
      } else {
        totBiasRoll = biasPerThrustRoll * thrust + biasResRoll;
        totBiasPitch = biasPerThrustPitch * thrust + biasResPitch;
      }

      const roll = clamp(rollDes - (2.5 * r.roll + 22 * simIntRoll + totBiasRoll) / 65, -0.6, 0.6);
      const pitch = clamp(pitchDes - (2.5 * r.pitch + 22 * simIntPitch + totBiasPitch) / 65, -0.6, 0.6);
      const yawRate = clamp(-3.0 * a.yaw - 0.35 * r.yaw - yawBias / 8, -2, 2);

      prevPos = { x: p.x, y: p.y, z: p.z };
      prevVel = { x: v.x, y: v.y, z: v.z };
      prevAtt = { roll: a.roll, pitch: a.pitch, yaw: a.yaw };
      prevRates = { roll: r.roll, pitch: r.pitch, yaw: r.yaw };
      prevAction = { thrust, roll, pitch, yawRate };

      return prevAction;
    },
  };
}
