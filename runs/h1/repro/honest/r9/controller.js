export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01;
  const lagTau = 0.038;
  const lagAlpha = 1 - Math.exp(-dt / lagTau);

  let prevVel = null;
  let prevAtt = null;
  let prevRates = null;
  let lastAction = { thrust: 0.45, roll: 0, pitch: 0, yawRate: 0 };

  let thrustLag = 0.45;
  let thrustScale = 21.0;

  let simInt = { roll: 0, pitch: 0 };
  let alphaLag = { roll: 0, pitch: 0, yaw: 0 };
  let torqueBias = { roll: 0, pitch: 0, yaw: 0 };

  const hist = [];
  let windEst = { x: 0, z: 0 };
  let baseWind = { x: 0, z: 0 };
  let gustDelta = 0;

  let intY = 0;
  let intZ = 0;

  let prevRollDes = 0;
  let prevPitchDes = 0;

  let gateIdx = -1;
  let mode = 'align';
  let centeredCount = 0;
  let padPhase = 'approach';

  const gateCentres = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];

  return {
    step(obs) {
      const p = obs.pos;
      const v = obs.vel;
      const att = obs.att;
      const rates = obs.rates;

      const cy = Math.cos(att.yaw);
      const sy = Math.sin(att.yaw);
      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const dir = {
        x: cy * bodyX + sy * bodyZ,
        y: Math.cos(att.pitch) * Math.cos(att.roll),
        z: -sy * bodyX + cy * bodyZ,
      };

      if (prevRates !== null) {
        for (const axis of ['roll', 'pitch']) {
          const errPrev = lastAction[axis] - prevAtt[axis];
          simInt[axis] = clamp(simInt[axis] + errPrev * dt, -0.35, 0.35);
          const cmd = 65 * errPrev - 13.4 * prevRates[axis] + 22 * simInt[axis];
          alphaLag[axis] += lagAlpha * (cmd - alphaLag[axis]);
          const meas = (rates[axis] - prevRates[axis]) / dt;
          if (p.y > 0.09 && thrustLag > 0.18 && thrustLag < 0.72 && Math.abs(cmd) < 22) {
            const res = meas - (alphaLag[axis] + torqueBias[axis] * thrustLag);
            const gain = obs.t < 1.2 ? 0.08 : 0.04;
            torqueBias[axis] = clamp(torqueBias[axis] + gain * res / Math.max(0.25, thrustLag), -28, 28);
          }
        }
        const yawCmd = 8 * (lastAction.yawRate - prevRates.yaw) - 0.4 * prevRates.yaw;
        alphaLag.yaw += lagAlpha * (yawCmd - alphaLag.yaw);
        const yawMeas = (rates.yaw - prevRates.yaw) / dt;
        if (p.y > 0.09 && thrustLag > 0.18 && thrustLag < 0.72 && Math.abs(yawCmd) < 12) {
          const yawRes = yawMeas - (alphaLag.yaw + torqueBias.yaw * thrustLag);
          torqueBias.yaw = clamp(torqueBias.yaw + 0.06 * yawRes / Math.max(0.25, thrustLag), -8, 8);
        }
      }

      const massEst0 = clamp(15.3 / thrustScale, 0.6, 0.95);
      const drag0 = 0.42 / massEst0;

      if (prevVel !== null && p.y > 0.095) {
        const ayMeas = (v.y - prevVel.y) / dt;
        const uy = thrustLag * dir.y;
        if (uy > 0.22 && thrustLag < 0.73) {
          const ayPred = thrustScale * uy - drag0 * prevVel.y - 9.81;
          const gainTs = obs.t < 1.5 ? 0.09 : 0.035;
          thrustScale = clamp(thrustScale + gainTs * (ayMeas - ayPred) / uy, 14.8, 27.2);
        }

        const massEst = clamp(15.3 / thrustScale, 0.6, 0.95);
        const drag = 0.42 / massEst;
        const S = thrustScale * thrustLag;

        hist.push({
          vx: v.x,
          vz: v.z,
          axM: S * dir.x - drag * v.x,
          azM: S * dir.z - drag * v.z,
        });
        if (hist.length > 6) hist.shift();

        const n4 = Math.min(hist.length - 1, 4);
        if (n4 >= 1) {
          const cur = hist[hist.length - 1];
          const old4 = hist[hist.length - 1 - n4];
          let sumAx4 = 0, sumAz4 = 0;
          for (let i = hist.length - n4; i < hist.length; i++) {
            sumAx4 += hist[i].axM;
            sumAz4 += hist[i].azM;
          }
          const wx4 = (cur.vx - old4.vx) / (n4 * dt) - sumAx4 / n4;
          const wz4 = (cur.vz - old4.vz) / (n4 * dt) - sumAz4 / n4;

          const n2 = Math.min(hist.length - 1, 2);
          const old2 = hist[hist.length - 1 - n2];
          let sumAx2 = 0, sumAz2 = 0;
          for (let i = hist.length - n2; i < hist.length; i++) {
            sumAx2 += hist[i].axM;
            sumAz2 += hist[i].azM;
          }
          const wx2 = (cur.vx - old2.vx) / (n2 * dt) - sumAx2 / n2;
          const wz2 = (cur.vz - old2.vz) / (n2 * dt) - sumAz2 / n2;

          gustDelta = Math.hypot(wx2 - wx4, wz2 - wz4);
          const fastAlpha = gustDelta > 1.3 ? 0.65 : 0.38;
          const wxTarget = gustDelta > 1.3 ? (0.6 * wx2 + 0.4 * wx4) : wx4;
          const wzTarget = gustDelta > 1.3 ? (0.6 * wz2 + 0.4 * wz4) : wz4;

          windEst.x += fastAlpha * (wxTarget - windEst.x);
          windEst.z += fastAlpha * (wzTarget - windEst.z);

          if (obs.t < 1.95) {
            const bwGain = obs.t < 0.35 ? 0.20 : 0.06;
            baseWind.x += bwGain * (wx4 - baseWind.x);
            baseWind.z += bwGain * (wz4 - baseWind.z);
          } else if (Math.hypot(wx4 - baseWind.x, wz4 - baseWind.z) < 0.85) {
            baseWind.x += 0.015 * (wx4 - baseWind.x);
            baseWind.z += 0.015 * (wz4 - baseWind.z);
          }
        }
      }

      thrustLag += lagAlpha * (lastAction.thrust - thrustLag);
      prevVel = { x: v.x, y: v.y, z: v.z };
      prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };
      prevRates = { roll: rates.roll, pitch: rates.pitch, yaw: rates.yaw };

      const massEst = clamp(15.3 / thrustScale, 0.6, 0.95);
      const dragCoeff = 0.42 / massEst;
      const windAccX = windEst.x;
      const windAccZ = windEst.z;
      const gustMag = Math.hypot(windAccX - baseWind.x, windAccZ - baseWind.z);

      let axDes = 0, ayDes = 0, azDes = 0;
      let maxTilt = 0.48;
      let pinPad = false;

      const prevG = obs.gateIndex > 0 ? gateCentres[obs.gateIndex - 1] : null;
      const holdPrev = prevG !== null && (p.x - prevG.x < 0.09) && (v.x < 0.18 || p.x < prevG.x + 0.015);

      if (obs.gateIndex < 3) {
        const g = obs.nextGate.centre;
        if (gateIdx !== obs.gateIndex) {
          gateIdx = obs.gateIndex;
          mode = 'align';
          centeredCount = 0;
          intZ = 0;
        }

        const dx = g.x - p.x;
        const dy = (holdPrev ? prevG.y : g.y) - p.y;
        const dz = (holdPrev ? prevG.z : g.z) - p.z;
        const absDy = Math.abs(g.y - p.y);
        const absDz = Math.abs(g.z - p.z);

        if (absDz < 0.08 && Math.abs(dx) < 0.42 && !holdPrev) {
          intZ = clamp(intZ + dz * dt, -0.18, 0.18);
        } else {
          intZ *= 0.9;
        }

        if (Math.abs(dy) < 0.25) {
          intY = clamp(intY + dy * dt, -0.35, 0.35);
        }

        const tPred = clamp(dx / Math.max(v.x, 0.45), 0.04, 0.20);
        const zPredErr = Math.abs((p.z + v.z * tPred) - g.z);
        const yPredErr = Math.abs((p.y + v.y * tPred) - g.y);

        if (dx <= 0 && (absDz > 0.032 || absDy > 0.13)) {
          mode = 'recover';
          centeredCount = 0;
        } else if (mode === 'recover') {
          if (dx > 0.24 && absDz < 0.25) {
            mode = 'align';
            centeredCount = 0;
          }
        } else if (mode === 'align') {
          const centered =
            dx > 0.12 && dx < 0.36 &&
            absDz < 0.015 && zPredErr < 0.017 && Math.abs(v.z) < 0.085 &&
            absDy < 0.055 && yPredErr < 0.065 && Math.abs(v.y) < 0.20 &&
            v.x > -0.12 &&
            gustDelta < 1.1;

          if (centered) centeredCount++;
          else centeredCount = 0;

          if (centeredCount >= 3) {
            mode = 'lunge';
          }
        } else if (mode === 'lunge') {
          const stopDist = 0.075 + Math.max(0, v.x) * 0.085 + (Math.max(0, v.x) * Math.max(0, v.x)) / 7.2;
          const unaligned = zPredErr > 0.025 || absDz > 0.024 || yPredErr > 0.10 || absDy > 0.095 || gustDelta > 2.0;
          if (dx > stopDist && unaligned) {
            mode = 'align';
            centeredCount = 0;
          }
        }

        if (mode === 'recover') {
          const safeZ = g.z + (p.z >= g.z ? 0.54 : -0.54);
          const zClear = Math.abs(p.z - g.z) > 0.47;
          const tx = zClear ? (g.x - 0.32) : Math.max(p.x, g.x + 0.16);
          const tz = zClear && dx > 0.22 ? g.z : safeZ;
          axDes = clamp(6.5 * (tx - p.x) - 5.0 * v.x, -4.8, 4.8);
          azDes = clamp(10.0 * (tz - p.z) - 6.0 * v.z, -5.0, 5.0);
          ayDes = clamp(11.0 * dy - 5.8 * v.y + 3.2 * intY, -4.5, 5.0);
        } else if (mode === 'align') {
          const alignDist = gustMag > 2.2 ? 0.33 : 0.28;
          const tx = g.x - alignDist;
          const ex = tx - p.x;
          const vxCap = ex > 0.15 ? Math.min(1.15, 0.35 + 1.8 * ex) : 1.15;
          const vxDes = clamp(1.9 * ex, -1.1, vxCap);
          const kdx = gustMag > 1.5 ? 5.8 : 5.0;
          axDes = clamp(kdx * (vxDes - v.x) + 2.5 * clamp(ex, -0.35, 0.35), -5.0, 4.2);

          if (dx < 0.22 && (absDz > 0.022 || absDy > 0.085)) {
            axDes = Math.min(axDes, -3.0 * Math.max(0, v.x) - 1.8);
          }

          if (prevG !== null && p.x - prevG.x < 0.14 && v.x < 0.45) {
            axDes = Math.max(axDes, 3.0 * (0.55 - v.x));
          }
          if (holdPrev) {
            axDes = Math.max(axDes, 4.2 * (0.65 - v.x));
          }

          const nearGate = dx < 0.40 || holdPrev;
          const kpz = nearGate ? 16.5 : 11.5;
          const kdz = nearGate ? 7.6 : 6.2;
          azDes = clamp(kpz * dz - kdz * v.z + 4.5 * intZ, -5.5, 5.5);

          const kpy = nearGate ? 14.0 : 11.0;
          const kdy = nearGate ? 6.4 : 5.6;
          ayDes = clamp(kpy * dy - kdy * v.y + 3.5 * intY, -4.5, 5.2);
        } else {
          const tx = g.x + 0.34;
          const vxTarget = 1.25;
          axDes = clamp(5.0 * (tx - p.x) + 3.8 * (vxTarget - v.x), -4.0, 3.8);

          const kpz = 18.5;
          const kdz = 8.0;
          azDes = clamp(kpz * dz - kdz * v.z + 5.0 * intZ, -5.8, 5.8);

          const kpy = 15.0;
          const kdy = 6.6;
          ayDes = clamp(kpy * dy - kdy * v.y + 3.5 * intY, -4.5, 5.2);
        }
      } else {
        const pad = obs.pad.centre;
        const dPad = Math.hypot(pad.x - p.x, pad.z - p.z);
        const hSpeed = Math.hypot(v.x, v.z);
        const speed3D = Math.hypot(v.x, v.y, v.z);

        if (padPhase !== 'pin' && dPad < 0.115 && p.y <= 0.092 && speed3D < 0.42) {
          padPhase = 'pin';
        }
        if (padPhase === 'pin' && (dPad > 0.14 || p.y > 0.11)) {
          padPhase = 'descend';
        }

        if (padPhase === 'pin') {
          pinPad = true;
          maxTilt = 0.12;
          axDes = clamp(6.0 * (pad.x - p.x) - 4.0 * v.x, -1.5, 1.5);
          azDes = clamp(6.0 * (pad.z - p.z) - 4.0 * v.z, -1.5, 1.5);
          ayDes = -6.0;
        } else {
          let ty;
          if (dPad > 0.12 || hSpeed > 0.30) {
            ty = clamp(0.18 + 0.85 * dPad, 0.18, 0.95);
          } else {
            padPhase = 'descend';
            ty = 0.05;
          }

          const kpx = dPad < 0.25 ? 9.5 : 6.5;
          const kdx = gustMag > 1.5 ? 6.2 : 5.2;
          axDes = clamp(kpx * (pad.x - p.x) - kdx * v.x, -4.8, 4.8);
          const tzPad = holdPrev ? prevG.z : pad.z;
          azDes = clamp((holdPrev ? 16.5 : kpx) * (tzPad - p.z) - (holdPrev ? 7.6 : kdx) * v.z, -4.8, 4.8);

          if (p.x - prevG.x < 0.14 && v.x < 0.40) {
            axDes = Math.max(axDes, 3.0 * (0.50 - v.x));
          }
          if (holdPrev) {
            axDes = Math.max(axDes, 4.2 * (0.60 - v.x));
          }

          if (ty <= 0.10) {
            const vyTarget = p.y < 0.16 ? -0.18 : -0.35;
            ayDes = clamp(5.5 * (vyTarget - v.y), -3.0, 4.0);
            if (p.y < 0.18) maxTilt = 0.15;
          } else {
            const ey = (holdPrev ? prevG.y : ty) - p.y;
            intY = clamp(intY + ey * dt, -0.35, 0.35);
            ayDes = clamp(10.5 * ey - 5.5 * v.y + 3.0 * intY, -3.5, 4.8);
          }
        }
      }

      if (p.x > 2.35 && v.x > 0.3) {
        axDes = Math.min(axDes, -6.0 * v.x - 2.0);
        maxTilt = Math.max(maxTilt, 0.54);
      }

      const Tx = axDes + dragCoeff * v.x - (pinPad ? 0.25 * windAccX : windAccX);
      const Ty = clamp(9.81 + ayDes + dragCoeff * v.y, 2.5, 16.5);
      const Tz = azDes + dragCoeff * v.z - (pinPad ? 0.25 * windAccZ : windAccZ);

      const Tfwd = cy * Tx - sy * Tz;
      const Tlat = sy * Tx + cy * Tz;

      if (obs.t < 0.12) maxTilt = Math.min(maxTilt, 0.28 + 1.5 * obs.t);

      const pitchDes = clamp(Math.atan2(Tfwd, Ty), -maxTilt, maxTilt);
      const rollDes = clamp(-Math.atan2(Tlat, Math.hypot(Tfwd, Ty)), -maxTilt, maxTilt);

      let thrustCmd;
      if (pinPad) {
        thrustCmd = 0.11;
      } else {
        const cosTilt = Math.max(0.55, Math.cos(pitchDes) * Math.cos(rollDes));
        const Sdes = Ty / cosTilt;
        const attErr = Math.hypot(rollDes - att.roll, pitchDes - att.pitch);
        const maxThrust = attErr > 0.18 ? 0.68 : 0.74;
        thrustCmd = clamp(Sdes / thrustScale, 0.06, maxThrust);
      }

      const rollRateDes = clamp((rollDes - prevRollDes) / dt, -3.5, 3.5);
      const pitchRateDes = clamp((pitchDes - prevPitchDes) / dt, -3.5, 3.5);
      prevRollDes = rollDes;
      prevPitchDes = pitchDes;

      const kpAtt = 105;
      const kdAtt = 16.0;

      const alphaRollDes = kpAtt * (rollDes - att.roll) + 0.3 * kdAtt * rollRateDes - kdAtt * rates.roll;
      const alphaPitchDes = kpAtt * (pitchDes - att.pitch) + 0.3 * kdAtt * pitchRateDes - kdAtt * rates.pitch;

      const rollCmd = clamp(
        att.roll + (alphaRollDes + 13.4 * rates.roll - 22 * simInt.roll - torqueBias.roll * thrustCmd) / 65,
        -0.6,
        0.6
      );
      const pitchCmd = clamp(
        att.pitch + (alphaPitchDes + 13.4 * rates.pitch - 22 * simInt.pitch - torqueBias.pitch * thrustCmd) / 65,
        -0.6,
        0.6
      );

      const yawRateDes = -2.8 * att.yaw - 0.35 * rates.yaw;
      const yawRateCmd = clamp(yawRateDes - (torqueBias.yaw * thrustCmd) / 8, -2.0, 2.0);

      lastAction = { thrust: thrustCmd, roll: rollCmd, pitch: pitchCmd, yawRate: yawRateCmd };
      return lastAction;
    },
  };
}
