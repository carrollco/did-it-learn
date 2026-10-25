export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  const GATES = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];

  let stepCount = 0;
  let prevPos = null, prevVel = null, prevAtt = null, prevRates = null;
  let lastAction = { thrust: 0.0, roll: 0, pitch: 0, yawRate: 0 };

  const simInt = { roll: 0, pitch: 0 };
  const uLag = { roll: 0, pitch: 0 };
  const biasRatio = { roll: 0, pitch: 0 };
  const biasResid = { roll: 0, pitch: 0 };

  let kT = 20.5;
  let laggedThrust = 0.0;
  let intX = 0;
  let intY = 0;
  let intZ = 0;

  const windFast = { x: 0, z: 0 };
  const windMid = { x: 0, z: 0 };
  const windSlow = { x: 0, z: 0 };

  let targetGate = -1;
  let phase = 'align';
  let settle = 0;
  let padPinned = false;

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;
      const dt = 0.01;

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const dir = {
        x: cy * bodyX + sy * bodyZ,
        y: Math.cos(att.pitch) * Math.cos(att.roll),
        z: -sy * bodyX + cy * bodyZ,
      };

      if (stepCount > 0) {
        const tauM = 0.040;
        const alphaM = 1 - Math.exp(-dt / tauM);
        laggedThrust += (lastAction.thrust - laggedThrust) * alphaM;

        for (const axis of ['roll', 'pitch']) {
          const prevI = simInt[axis];
          const midAtt = 0.5 * (prevAtt[axis] + att[axis]);
          simInt[axis] = clamp(simInt[axis] + (lastAction[axis] - midAtt) * dt, -0.35, 0.35);

          const uCmd = 65 * (lastAction[axis] - prevAtt[axis]) - 13 * prevRates[axis] + 22 * prevI;
          const uDiff = Math.abs(uCmd - uLag[axis]);
          uLag[axis] += (uCmd - uLag[axis]) * alphaM;

          const alphaMeas = (rates[axis] - prevRates[axis]) / dt;
          const alphaExp = uLag[axis] - 0.4 * 0.5 * (rates[axis] + prevRates[axis]);
          const biasInst = alphaMeas - alphaExp;

          if (stepCount >= 10 && laggedThrust > 0.24 && p.y > 0.09 && (stepCount < 45 || uDiff < 9.0)) {
            const gainR = stepCount < 45 ? 0.14 : 0.035;
            const gainRes = stepCount < 45 ? 0.15 : 0.04;
            const rMeas = clamp(biasInst / laggedThrust, -30, 30);
            biasRatio[axis] += (rMeas - biasRatio[axis]) * gainR;
            const resMeas = clamp(biasInst - biasRatio[axis] * laggedThrust, -14, 14);
            biasResid[axis] += (resMeas - biasResid[axis]) * gainRes;
          }
        }

        const massEst = clamp(15.2 / kT, 0.6, 0.95);
        const dragCoeff = 0.42 / massEst;

        const prevCy = Math.cos(prevAtt.yaw), prevSy = Math.sin(prevAtt.yaw);
        const prevBx = Math.sin(prevAtt.pitch) * Math.cos(prevAtt.roll);
        const prevBz = -Math.sin(prevAtt.roll);
        const prevDir = {
          x: prevCy * prevBx + prevSy * prevBz,
          y: Math.cos(prevAtt.pitch) * Math.cos(prevAtt.roll),
          z: -prevSy * prevBx + prevCy * prevBz,
        };

        if (stepCount >= 14 && p.y > 0.11 && p.y < 2.88) {
          const uy = laggedThrust * prevDir.y;
          if (uy > 0.28) {
            const ayMeas = (v.y - prevVel.y) / dt;
            const fyMeas = ayMeas + 9.81 + dragCoeff * 0.5 * (v.y + prevVel.y);
            const kTMeas = clamp(fyMeas / uy, 15.0, 27.0);
            const gain = stepCount < 40 ? 0.16 : 0.07;
            kT = clamp(kT + (kTMeas - kT) * gain, 15.0, 27.0);
          }
        }

        if (stepCount >= 8 && p.y > 0.09 && p.y < 2.88) {
          const axMeas = (v.x - prevVel.x) / dt;
          const azMeas = (v.z - prevVel.z) / dt;
          const axProp = kT * laggedThrust * prevDir.x;
          const azProp = kT * laggedThrust * prevDir.z;

          const wxInst = clamp(axMeas - axProp + dragCoeff * 0.5 * (v.x + prevVel.x), -12, 12);
          const wzInst = clamp(azMeas - azProp + dragCoeff * 0.5 * (v.z + prevVel.z), -12, 12);

          windFast.x += (wxInst - windFast.x) * 0.28;
          windFast.z += (wzInst - windFast.z) * 0.28;
          windMid.x += (wxInst - windMid.x) * 0.10;
          windMid.z += (wzInst - windMid.z) * 0.10;
          windSlow.x += (wxInst - windSlow.x) * 0.035;
          windSlow.z += (wzInst - windSlow.z) * 0.035;
        }
      }

      stepCount++;
      prevPos = { ...p };
      prevVel = { ...v };
      prevAtt = { ...att };
      prevRates = { ...rates };

      const massEst = clamp(15.2 / kT, 0.6, 0.95);
      const dragCoeff = 0.42 / massEst;
      const gustX = windFast.x - windSlow.x;
      const gustZ = windFast.z - windSlow.z;
      const gustTransient = Math.hypot(windFast.x - windMid.x, windFast.z - windMid.z);

      const axCur = kT * laggedThrust * dir.x - dragCoeff * v.x + windFast.x;
      const azCur = kT * laggedThrust * dir.z - dragCoeff * v.z + windFast.z;
      const vxLead = v.x + 0.042 * axCur;
      const vzLead = v.z + 0.045 * azCur;

      let axDes = 0, ayDes = 0, azDes = 0;
      let maxTilt = 0.54;

      // Stay centered on prevGate until safely past it so a headwind gust never pushes us into prevGate from behind
      const prevGate = obs.gateIndex > 0 ? GATES[obs.gateIndex - 1] : null;
      const clearingPrevGate = prevGate && p.x < prevGate.x + 0.16;

      if (obs.gateIndex < 3) {
        padPinned = false;
        const gate = obs.nextGate.centre;
        if (targetGate !== obs.gateIndex) {
          targetGate = obs.gateIndex;
          phase = 'align';
          settle = 0;
          intX = 0;
          intZ = 0;
        }

        const dxGate = gate.x - p.x;
        const dyGate = (clearingPrevGate ? prevGate.y : gate.y) - p.y;
        const dzGate = (clearingPrevGate ? prevGate.z : gate.z) - p.z;

        if (!clearingPrevGate && Math.abs(dzGate) < 0.08 && Math.abs(v.z) < 0.25 && Math.abs(dxGate) < 0.45) {
          intZ = clamp(intZ + dzGate * dt, -0.12, 0.12);
        } else {
          intZ *= 0.90;
        }

        if (Math.abs(dyGate) < 0.22) {
          intY = clamp(intY + dyGate * dt, -0.35, 0.35);
        }

        const vxForward = Math.max(0.45, v.x);
        const tCross = clamp(dxGate / vxForward, 0.02, 0.32);
        const zPred = p.z + v.z * tCross;
        const yPred = p.y + v.y * tCross;

        if (phase === 'align') {
          const alignX = gate.x - 0.24;
          const ex = alignX - p.x;
          const ey = dyGate;
          const ez = dzGate;

          const centeredZ = Math.abs(ez) < 0.019 && Math.abs(v.z) < 0.105 && Math.abs(zPred - gate.z) < 0.020;
          const centeredY = Math.abs(ey) < 0.095 && Math.abs(v.y) < 0.30 && Math.abs(yPred - gate.y) < 0.105;
          const inPositionX = p.x > gate.x - 0.45 && p.x < gate.x - 0.07 && v.x > -0.28 && v.x < 0.52;
          const calmGust = gustTransient < 1.45;

          if (centeredZ && centeredY && inPositionX && calmGust) {
            if (Math.abs(ez) < 0.013 && Math.abs(v.z) < 0.075 && Math.abs(ey) < 0.070) settle += 2;
            else settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }

          if (settle >= 3) {
            phase = 'lunge';
          } else {
            const kpX = p.x > gate.x - 0.38 ? 7.2 : 5.8;
            const kdX = p.x > gate.x - 0.38 ? 5.2 : 4.0;
            axDes = clamp(kpX * ex - kdX * vxLead, -5.2, 4.8);
            if (p.x > gate.x - 0.13 && v.x > 0) {
              axDes = -5.5;
            }
            if (prevGate && p.x < prevGate.x + 0.25 && v.x < 0.80) {
              axDes = Math.max(axDes, 3.8 * (0.85 - v.x));
            }

            azDes = clamp(10.5 * ez - 6.5 * vzLead + 10.0 * intZ, -5.8, 5.8);
            ayDes = clamp(9.2 * ey - 5.0 * v.y + 2.6 * intY, -4.5, 5.2);
          }
        }

        if (phase === 'lunge') {
          const stopDist = (v.x > 0 ? (v.x * v.x) / (2 * 5.0) : 0);
          const canStopBeforeGate = dxGate > stopDist + 0.05 && dxGate > 0.09;
          const willMiss = Math.abs(zPred - gate.z) > 0.027 || Math.abs(yPred - gate.y) > 0.115 || Math.abs(dzGate) > 0.029;

          if (canStopBeforeGate && willMiss) {
            phase = 'align';
            settle = 0;
            axDes = -5.5;
          } else {
            const targetX = gate.x + 0.35;
            const ex = targetX - p.x;
            axDes = clamp(8.0 * ex - 2.6 * (v.x - 1.65), 2.2, 5.0);
          }
          azDes = clamp(13.5 * dzGate - 6.8 * vzLead + 10.0 * intZ, -6.0, 6.0);
          ayDes = clamp(9.2 * dyGate - 5.0 * v.y + 2.6 * intY, -4.5, 5.2);
        }
      } else {
        const pad = obs.pad.centre;
        const ex = pad.x - p.x;
        const ez = (clearingPrevGate ? prevGate.z : pad.z) - p.z;
        const distH = Math.hypot(pad.x - p.x, pad.z - p.z);
        const speedH = Math.hypot(v.x, v.z);
        const speedTotal = Math.hypot(v.x, v.y, v.z);

        if ((distH < 0.144 && p.y <= 0.122 && speedTotal < 0.48 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.17) ||
            (padPinned && distH < 0.148 && p.y <= 0.108)) {
          padPinned = true;
          const axPin = clamp(5.5 * ex - 3.8 * v.x - 0.6 * windFast.x, -1.3, 1.3);
          const azPin = clamp(5.5 * ez - 3.8 * v.z - 0.6 * windFast.z, -1.3, 1.3);
          const fxB = cy * axPin - sy * azPin;
          const fzB = sy * axPin + cy * azPin;
          const desPitch = clamp(fxB / 9.81, -0.11, 0.11);
          const desRoll = clamp(-fzB / 9.81, -0.11, 0.11);

          const pinThrust = p.y > 0.088 ? 0.01 : 0.04;
          const out = { thrust: pinThrust, roll: desRoll, pitch: desPitch, yawRate: clamp(-1.5 * att.yaw, -0.6, 0.6) };
          for (const axis of ['roll', 'pitch']) {
            const totalBias = biasRatio[axis] * out.thrust + biasResid[axis];
            const uTarget = 75 * (out[axis] - att[axis]) - 14 * rates[axis] - totalBias;
            out[axis] = clamp(att[axis] + (uTarget + 13 * rates[axis] - 22 * simInt[axis]) / 65, -0.14, 0.14);
          }
          lastAction = out;
          return out;
        }
        padPinned = false;

        if (distH < 0.30 && speedH < 0.45) {
          intX = clamp(intX + ex * dt, -0.16, 0.16);
          intZ = clamp(intZ + ez * dt, -0.16, 0.16);
        } else {
          intX *= 0.90;
          intZ *= 0.90;
        }

        let targetY = clearingPrevGate ? prevGate.y : clamp(0.04 + 0.38 * Math.max(0, distH - 0.10), 0.04, 0.76);
        if (distH < 0.18 && speedH < 0.50) {
          targetY = -0.03;
        }

        const ey = targetY - p.y;
        if (p.y < 0.42) {
          intY = clamp(intY + ey * dt, -0.50, 0.35);
        }

        axDes = clamp(7.4 * ex - 4.9 * vxLead + 10.0 * intX, -4.6, 4.6);
        if (prevGate && p.x < prevGate.x + 0.22 && v.x < 0.75) {
          axDes = Math.max(axDes, 3.5 * (0.80 - v.x));
        }
        azDes = clamp(7.6 * ez - 5.0 * vzLead + 10.0 * intZ, -4.6, 4.6);

        const maxDownSpeed = p.y < 0.22 ? -0.42 : -1.18;
        let vyTarget = clamp(3.8 * ey, maxDownSpeed, 1.2);
        if (distH < 0.16 && speedH < 0.45 && p.y < 0.25) {
          vyTarget = -0.38;
        }
        ayDes = clamp(6.8 * (vyTarget - v.y) + 2.5 * intY, -4.6, 4.5);

        if (p.y < 0.22) {
          maxTilt = 0.25;
        }
      }

      const kGust = 0.45;
      const axProp = axDes + dragCoeff * v.x - windFast.x - kGust * gustX;
      const azProp = azDes + dragCoeff * v.z - windFast.z - kGust * gustZ;
      const ayProp = Math.max(1.2, 9.81 + ayDes + dragCoeff * v.y);

      const fxBody = cy * axProp - sy * azProp;
      const fzBody = sy * axProp + cy * azProp;

      const desPitch = clamp(Math.atan2(fxBody, ayProp), -maxTilt, maxTilt);
      const desRoll = clamp(Math.atan2(-fzBody, Math.hypot(fxBody, ayProp)), -maxTilt, maxTilt);

      const thrust = clamp(ayProp / (kT * Math.max(0.55, dir.y)), 0.02, 1.0);

      const cmd = { thrust, roll: desRoll, pitch: desPitch, yawRate: clamp(-1.8 * att.yaw - 0.2 * rates.yaw, -0.8, 0.8) };
      for (const axis of ['roll', 'pitch']) {
        const desAngle = cmd[axis];
        const totalBias = biasRatio[axis] * thrust + biasResid[axis];
        const alphaDes = 80 * (desAngle - att[axis]) - 15.5 * rates[axis] + 0.4 * rates[axis];
        const uTarget = alphaDes - totalBias;
        cmd[axis] = clamp(att[axis] + (uTarget + 13 * rates[axis] - 22 * simInt[axis]) / 65, -0.6, 0.6);
      }

      lastAction = cmd;
      return cmd;
    },
  };
}
