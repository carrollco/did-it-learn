export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  let kT = 21.5;
  let uThrust = 0.0;
  let lastThrust = 0.0;
  let vyHat = 0, dyHat = 0;
  let vxHat = 0, dxHat = 0;
  let vzHat = 0, dzHat = 0;
  let baseDx = 0, baseDz = 0;
  let izGate = 0, iyGate = 0;
  let init = false;

  let gateIdx = -1;
  let prevGate = null;
  let lastGateSeen = null;
  let phase = 'approach';
  let alignCount = 0;
  let yawInt = 0;

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (!init) {
        init = true;
        vxHat = v.x; vyHat = v.y; vzHat = v.z;
      }

      const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
      const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyX = sp * cr, bodyZ = -sr;
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = cp * cr;
      const dirZ = -sy * bodyX + cy * bodyZ;

      uThrust += (lastThrust - uThrust) * (1 - Math.exp(-dt / 0.038));
      const massHat = clamp(15.5 / kT, 0.60, 0.85);
      const dragCoeff = 0.42 / massHat;

      if (p.y > 0.09 && p.y < 2.90) {
        const ayModel = kT * uThrust * dirY - 9.81 - dragCoeff * v.y + dyHat;
        vyHat += ayModel * dt;
        const errVy = v.y - vyHat;
        vyHat += 0.48 * errVy;
        if (lastThrust > 0.20 && lastThrust < 0.92 && dirY > 0.82 && obs.t < 2.0) {
          kT = clamp(kT + 3.0 * errVy / (uThrust * dirY), 17.2, 26.8);
        }
        dyHat = clamp(dyHat + 3.2 * errVy, -2.0, 2.0);

        const specThrust = Math.max(0, kT * uThrust + dyHat);
        const axModel = specThrust * dirX - dragCoeff * v.x;
        const azModel = specThrust * dirZ - dragCoeff * v.z;

        vxHat += (axModel + dxHat) * dt;
        const errVx = v.x - vxHat;
        vxHat += 0.54 * errVx;

        vzHat += (azModel + dzHat) * dt;
        const errVz = v.z - vzHat;
        vzHat += 0.54 * errVz;

        const maxDist = (obs.t < 2.0 || obs.t > 13.5) ? 0.65 : 6.8;
        dxHat = clamp(dxHat + 10.5 * errVx, -maxDist, maxDist);
        dzHat = clamp(dzHat + 10.5 * errVz, -maxDist, maxDist);

        if (obs.t > 0.45 && obs.t < 1.98) {
          baseDx += 0.04 * (dxHat - baseDx);
          baseDz += 0.04 * (dzHat - baseDz);
        } else if (obs.t >= 2.0) {
          dxHat += 0.008 * (baseDx - dxHat);
          dzHat += 0.008 * (baseDz - dzHat);
        }
      } else {
        vxHat = v.x; vyHat = v.y; vzHat = v.z;
      }

      let tx = p.x, ty = 1.05, tz = p.z;
      let vxRef = 0, vyRef = 0, vzRef = 0;
      let maxTilt = 0.58;
      let touchDown = false;
      let emergencyBrakeX = false;

      if (gateIdx !== obs.gateIndex) {
        if (lastGateSeen) prevGate = { x: lastGateSeen.x, y: lastGateSeen.y, z: lastGateSeen.z };
        gateIdx = obs.gateIndex;
        phase = prevGate ? 'clear_prev' : 'approach';
        alignCount = 0;
        izGate = 0;
        iyGate = 0;
      }
      if (obs.nextGate) {
        lastGateSeen = obs.nextGate.centre;
      }

      if (phase === 'clear_prev' && prevGate) {
        ty = prevGate.y;
        tz = prevGate.z;
        const prevDz = Math.abs(p.z - prevGate.z);
        const prevDy = Math.abs(p.y - prevGate.y);
        if (p.x < prevGate.x - 0.005 && (prevDz > 0.045 || prevDy > 0.095 || Math.abs(v.z) > 0.16)) {
          tx = prevGate.x - 0.26;
          vxRef = 0;
          if (p.x + Math.max(0, v.x) * 0.22 > prevGate.x - 0.05) {
            emergencyBrakeX = true;
          }
        } else {
          tx = prevGate.x + 0.45;
          vxRef = 1.15;
        }
        if (p.x >= prevGate.x + 0.22) {
          phase = 'approach';
          alignCount = 0;
        }
      }

      if (obs.gateIndex < 3 && phase !== 'clear_prev') {
        const g = obs.nextGate.centre;
        const dy = Math.abs(p.y - g.y);
        const dz = Math.abs(p.z - g.z);
        const tCross = clamp((g.x - p.x) / Math.max(0.35, v.x), 0.03, 0.22);
        const predDy = Math.abs(p.y + v.y * tCross - g.y);
        const predDz = Math.abs(p.z + v.z * tCross - g.z);

        if (dy < 0.18 && dz < 0.18) {
          izGate = clamp(izGate + (g.z - p.z) * dt, -0.08, 0.08);
          iyGate = clamp(iyGate + (g.y - p.y) * dt, -0.08, 0.08);
        }

        if (p.x >= g.x - 0.005) {
          phase = 'recover';
          alignCount = 0;
        }

        const vxPos = Math.max(0, v.x);
        // True stopping distance including 0.14s pitch reversal latency
        const stopDist = 0.14 * vxPos + (vxPos * vxPos) / 10.0;
        const canStopBeforeGate = (p.x + stopDist < g.x - 0.035);

        if (phase === 'cross') {
          const unsafe = (dz > 0.040 || predDz > 0.050 || Math.abs(v.z) > 0.16 || dy > 0.105 || predDy > 0.125);
          if (canStopBeforeGate && unsafe) {
            phase = 'approach';
            alignCount = 0;
            emergencyBrakeX = true;
          }
        }

        if (phase === 'recover') {
          tx = g.x - 0.38;
          ty = g.y;
          tz = g.z + (p.z >= g.z ? 0.62 : -0.62);
          if (p.x < g.x - 0.24 && Math.abs(p.z - tz) < 0.15) {
            phase = 'approach';
          }
        } else if (phase === 'approach') {
          ty = g.y;
          tz = g.z;
          const yzErr = Math.hypot(p.y - g.y, p.z - g.z);
          const baseStage = (obs.gateIndex === 0 && obs.t < 1.9) ? 0.22 : 0.29;
          const stageX = g.x - clamp(baseStage + 0.32 * yzErr, baseStage, 0.56);
          tx = stageX;

          if (p.x + stopDist > g.x - 0.08 && canStopBeforeGate && (dz > 0.038 || dy > 0.095 || predDz > 0.048 || Math.abs(v.z) > 0.15)) {
            emergencyBrakeX = true;
          }

          const aligned = p.x > g.x - 0.42 && p.x < g.x - 0.04 &&
                          dy < 0.058 && dz < 0.030 &&
                          predDy < 0.075 && predDz < 0.040 &&
                          Math.abs(v.y) < 0.18 && Math.abs(v.z) < 0.11;
          if (aligned) alignCount++;
          else alignCount = Math.max(0, alignCount - 1);

          if (alignCount >= (obs.gateIndex === 0 && obs.t < 1.9 ? 2 : 4)) {
            phase = 'cross';
          }
        }

        if (phase === 'cross') {
          ty = g.y;
          tz = g.z;
          if (canStopBeforeGate) {
            const targetVx = (obs.gateIndex === 0 && obs.t < 1.95) ? 0.85 : 0.53;
            tx = p.x;
            vxRef = targetVx;
          } else {
            tx = g.x + 0.45;
            vxRef = 1.15;
          }
        }
      } else if (obs.gateIndex >= 3 && phase !== 'clear_prev') {
        const pad = obs.pad.centre;
        if (phase !== 'pad_approach' && phase !== 'pad_land') {
          phase = 'pad_approach';
        }
        tx = pad.x;
        tz = pad.z;
        const padErr = Math.hypot(p.x - pad.x, p.z - pad.z);
        const hSpeed = Math.hypot(v.x, v.z);
        const speed3D = Math.hypot(v.x, v.y, v.z);
        const curTilt = Math.max(Math.abs(att.roll), Math.abs(att.pitch));

        if (phase === 'pad_approach') {
          ty = clamp(0.24 + 0.55 * padErr, 0.24, 0.95);
          if (padErr < 0.095 && hSpeed < 0.22 && p.y < 0.34 && curTilt < 0.17) {
            phase = 'pad_land';
          }
        }
        if (phase === 'pad_land') {
          if (padErr > 0.118 || (p.y < 0.11 && curTilt > 0.22) || hSpeed > 0.35) {
            phase = 'pad_approach';
            ty = 0.28;
          } else {
            ty = 0.045;
            vyRef = clamp(-0.30 * (p.y - 0.055), -0.30, 0);
            if (p.y <= 0.11) maxTilt = 0.155;
            if (p.y <= 0.076 && padErr <= 0.095 && speed3D < 0.38 && curTilt < 0.165) {
              touchDown = true;
            }
          }
        }
      }

      const ex = clamp(tx - p.x, -0.70, 0.70);
      const ey = clamp(ty - p.y, -0.75, 0.75);
      const ez = clamp(tz - p.z, -0.65, 0.65);

      const isPad = obs.gateIndex >= 3 && phase !== 'clear_prev';
      let axDes = (isPad ? 6.4 : 6.8) * ex + (isPad ? 4.6 : 5.2) * (vxRef - vxHat);
      if (emergencyBrakeX) {
        axDes = Math.min(axDes, -12.0 * Math.max(0.32, v.x) + 12.0 * (tx - p.x));
      } else if (phase === 'clear_prev' && p.x >= prevGate.x - 0.005 && v.x < 0.55) {
        axDes = Math.max(axDes, 8.5 * (0.65 - v.x));
      }
      const azDes = isPad ? (6.8 * ez + 4.8 * (vzRef - vzHat)) : (13.2 * ez + 7.0 * (vzRef - vzHat) + 3.2 * izGate);
      const ayDes = 8.5 * ey + 5.2 * (vyRef - vyHat) + (isPad ? 0 : 2.5 * iyGate);

      const fxSpec = axDes + dragCoeff * vxHat - dxHat;
      const fzSpec = azDes + dragCoeff * vzHat - dzHat;
      const fySpec = clamp(9.81 + ayDes + dragCoeff * vyHat - dyHat, 3.5, 16.5);

      const fyActual = Math.max(5.5, fySpec);
      const uX = fxSpec / fyActual;
      const uZ = fzSpec / fyActual;

      const pitchDes = Math.atan(cy * uX - sy * uZ);
      const rollDes = Math.atan(-Math.cos(pitchDes) * (sy * uX + cy * uZ));

      const leadGain = isPad ? 0.28 : 0.48;
      const rateGain = isPad ? 0.035 : 0.05;
      let pitch = clamp(pitchDes + leadGain * (pitchDes - att.pitch) - rateGain * rates.pitch, -maxTilt, maxTilt);
      let roll = clamp(rollDes + leadGain * (rollDes - att.roll) - rateGain * rates.roll, -maxTilt, maxTilt);
      let thrust = clamp(fySpec / (kT * Math.max(0.55, cr * cp)), 0, 1);

      if (touchDown) {
        pitch = clamp(pitch, -0.13, 0.13);
        roll = clamp(roll, -0.13, 0.13);
        thrust = 0.14;
      }

      yawInt = clamp(yawInt + att.yaw * dt, -0.4, 0.4);
      const yawRate = clamp(-2.2 * att.yaw - 0.6 * yawInt, -2, 2);

      lastThrust = thrust;
      return { thrust, roll, pitch, yawRate };
    },
  };
}
