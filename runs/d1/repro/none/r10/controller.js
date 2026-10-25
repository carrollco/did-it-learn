export default function makeController() {
  let initialized = false;
  let kT = 21.0;
  let tNorm = 0.45;
  let vxEst = 0, vyEst = 0, vzEst = 0;
  let dxEst = 0, dyEst = 0, dzEst = 0;
  let yawInt = 0;
  let gateIdx = 0;
  let phase = 'align';
  let settle = 0;

  const GATES_X = [-1.15, 0.15, 1.35];
  const GATES_Y = [1.05, 1.40, 1.05];
  const GATES_Z = [-0.85, 0.25, 1.05];

  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (!initialized) {
        initialized = true;
        vxEst = v.x; vyEst = v.y; vzEst = v.z;
      } else {
        const evx = v.x - vxEst;
        const evy = v.y - vyEst;
        const evz = v.z - vzEst;
        vxEst += 0.55 * evx;
        vyEst += 0.50 * evy;
        vzEst += 0.60 * evz;

        if (p.y > 0.095) {
          dxEst = clamp(dxEst + 8.4 * evx, -11.0, 11.0);
          dyEst = clamp(dyEst + 4.8 * evy, -7.5, 7.5);
          dzEst = clamp(dzEst + 10.0 * evz, -12.0, 12.0);
        }
      }

      if (obs.gateIndex !== gateIdx) {
        gateIdx = obs.gateIndex;
        phase = 'clear_prev';
        settle = 0;
      }

      let tx, ty, tz, vxFf = 0, vyFf = 0, vzFf = 0;
      let kpX = 6.2, kdX = 4.8;
      let kpY = 7.5, kdY = 4.8;
      let kpZ = 7.8, kdZ = 5.2;
      let maxPosErrX = 0.45;
      let fastRollLead = 0;

      // 1. After crossing a gate, keep lateral alignment until 12 cm past gate plane while moderating X speed
      if (phase === 'clear_prev' && gateIdx > 0) {
        const prev = gateIdx - 1;
        if (p.x < GATES_X[prev] + 0.12) {
          tx = GATES_X[prev] + 0.16;
          ty = GATES_Y[prev];
          tz = GATES_Z[prev];
          vxFf = 0.32;
          maxPosErrX = 0.16;
          kpX = 7.2; kdX = 5.2;
          kpY = 9.5; kdY = 5.5;
          kpZ = 12.5; kdZ = 6.5;
          fastRollLead = 0.22;
        } else {
          phase = 'align';
          settle = 0;
        }
      }

      if (phase !== 'clear_prev') {
        if (obs.gateIndex < 3) {
          const g = obs.nextGate.centre;
          const errY = Math.abs(g.y - p.y);
          const errZ = Math.abs(g.z - p.z);

          if (phase !== 'cross' && p.x > g.x - 0.13) {
            phase = 'recover';
          }

          if (phase === 'recover') {
            tx = g.x - 0.48;
            if (p.x > g.x - 0.18 && (errY > 0.17 || errZ > 0.085)) {
              ty = p.y;
              tz = p.z;
            } else {
              ty = g.y;
              tz = g.z;
            }
            maxPosErrX = 0.42;
            kpX = 9.0; kdX = 5.8;
            if (p.x < g.x - 0.30) {
              phase = 'align';
              settle = 0;
            }
          } else if (phase === 'align') {
            ty = g.y;
            tz = g.z;
            kpZ = 9.2; kdZ = 5.8;

            if (errZ > 0.088 || errY > 0.16) {
              tx = obs.gateIndex === 0 ? g.x - 0.50 : g.x - 0.76;
              maxPosErrX = 0.22;
              if (p.x + vxEst * 0.40 > g.x - 0.40) {
                kpX = 9.5; kdX = 6.0;
                maxPosErrX = 0.50;
              }
            } else {
              tx = g.x - 0.32;
              maxPosErrX = 0.28;
            }

            // Prevent a strong -x gust from pushing us back into the previous gate plane
            if (obs.gateIndex > 0) {
              const prev = obs.gateIndex - 1;
              if (p.x + vxEst * 0.30 < GATES_X[prev] + 0.16) {
                tx = Math.max(tx, GATES_X[prev] + 0.28);
                kpX = 9.0; kdX = 5.6;
                maxPosErrX = 0.45;
              }
              if (p.x < GATES_X[prev] + 0.12) {
                ty = GATES_Y[prev];
                tz = GATES_Z[prev];
              }
            }

            const distYZ = Math.hypot(ty - p.y, tz - p.z);
            const velYZ = Math.hypot(vyEst, vzEst);
            if (Math.abs((g.x - 0.32) - p.x) < 0.15 && distYZ < 0.054 && errZ < 0.040 && velYZ < 0.19 && Math.abs(vxEst) < 0.25) {
              settle++;
            } else {
              settle = Math.max(0, settle - 1);
            }
            if (settle >= 5) {
              phase = 'cross';
            }
          }

          if (phase === 'cross') {
            const dxGate = g.x - p.x;
            const vxPos = Math.max(0, vxEst);
            const stopDist = vxPos * 0.075 + (vxPos * vxPos) / 9.2 + 0.038;
            const tCross = clamp(dxGate / Math.max(0.45, vxEst + 0.25), 0.015, 0.28);
            const yPred = p.y + vyEst * tCross;
            const zPred = p.z + vzEst * tCross;

            if (dxGate > stopDist && (Math.abs(yPred - g.y) > 0.14 || Math.abs(zPred - g.z) > 0.056 || errZ > 0.058)) {
              phase = 'align';
              settle = 0;
              tx = g.x - 0.38;
              ty = g.y;
              tz = g.z;
              kpX = 9.5; kdX = 6.0;
              maxPosErrX = 0.45;
            } else {
              tx = g.x + 0.18;
              ty = g.y;
              tz = g.z;
              vxFf = 0.84;
              maxPosErrX = 0.16;
              kpX = 7.8; kdX = 5.4;
              kpY = 10.5; kdY = 5.8;
              kpZ = 14.5; kdZ = 7.2;
              fastRollLead = 0.24;
            }
          }
        } else {
          const pad = obs.pad.centre;
          tx = pad.x;
          tz = pad.z;
          const distXZ = Math.hypot(tx - p.x, tz - p.z);
          const speedXZ = Math.hypot(vxEst, vzEst);
          const urgent = obs.t > 20.5;

          if (phase !== 'descend') {
            ty = (distXZ > 0.35 || vxEst > 0.55) ? Math.max(0.60, p.y - 0.12) : 0.38;
            maxPosErrX = 0.24;
            kpX = 7.5; kdX = 5.2;
            kpZ = 7.5; kdZ = 5.2;
            if ((distXZ < 0.115 && speedXZ < 0.24 && p.y < 0.54 && Math.abs(vyEst) < 0.24) || (urgent && distXZ < 0.135)) {
              phase = 'descend';
            }
          }

          if (phase === 'descend') {
            kpX = 8.5; kdX = 5.4;
            kpZ = 8.5; kdZ = 5.4;
            maxPosErrX = 0.24;
            if (!urgent && (distXZ > 0.125 || speedXZ > 0.28)) {
              ty = Math.max(0.15, p.y);
              vyFf = 0;
            } else {
              ty = Math.max(0.04, p.y - (urgent ? 0.12 : 0.09));
              const maxSink = p.y > 0.18 ? (urgent ? 0.40 : 0.32) : (urgent ? 0.18 : 0.14);
              vyFf = clamp((0.06 - p.y) * (urgent ? 1.9 : 1.5), -maxSink, 0.15);
            }
          }
        }
      }

      const cd = 0.58;
      const ex = clamp(tx - p.x, -maxPosErrX, maxPosErrX);
      const ez = clamp(tz - p.z, -0.45, 0.45);
      const ey = clamp(ty - p.y, -0.18, 0.45);

      let axDes = clamp(kpX * ex + kdX * (vxFf - vxEst) - dxEst + cd * vxEst, -5.2, 5.2);
      let azDes = clamp(kpZ * ez + kdZ * (vzFf - vzEst) - dzEst + cd * vzEst, -6.4, 6.4);
      const ayDes = clamp(kpY * ey + kdY * (vyFf - vyEst) - dyEst + cd * vyEst, -2.2, 6.0);

      if (p.x + vxEst * 0.45 > 2.45) axDes = Math.min(axDes, -5.2);
      if (p.x + vxEst * 0.45 < -2.45) axDes = Math.max(axDes, 5.2);
      if (p.z + vzEst * 0.45 > 2.45) azDes = Math.min(azDes, -5.2);
      if (p.z + vzEst * 0.45 < -2.45) azDes = Math.max(azDes, 5.2);

      const onPad = obs.gateIndex === 3 &&
        Math.hypot(p.x - obs.pad.centre.x, p.z - obs.pad.centre.z) < 0.115 &&
        p.y <= 0.088 &&
        Math.hypot(v.x, v.y, v.z) < 0.32;

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const gTotal = Math.max(6.5, 9.81 + ayDes);
      const aBodyX = cy * axDes - sy * azDes;
      const aBodyZ = sy * axDes + cy * azDes;

      const maxPitch = (obs.gateIndex === 3 && p.y < 0.11) ? 0.165 : 0.50;
      const maxRoll = (obs.gateIndex === 3 && p.y < 0.11) ? 0.165 : 0.57;
      const pitchTarget = onPad ? 0 : clamp(Math.atan2(aBodyX, gTotal), -maxPitch, maxPitch);
      const rollTarget = onPad ? 0 : clamp(Math.atan2(-aBodyZ, gTotal), -maxRoll, maxRoll);

      const pitch = pitchTarget;
      const roll = onPad ? 0 : clamp(rollTarget + fastRollLead * (rollTarget - att.roll), -maxRoll, maxRoll);

      const dirY = Math.max(0.65, Math.cos(att.roll) * Math.cos(att.pitch));
      const thrust = onPad ? 0.02 : clamp((9.81 + ayDes) / (kT * dirY), 0.12, 1.0);

      yawInt = clamp(yawInt + att.yaw * dt, -0.4, 0.4);
      const yawRate = clamp(-3.0 * att.yaw - 1.2 * yawInt - 0.2 * rates.yaw, -2, 2);

      tNorm += (thrust - tNorm) * 0.22;
      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const dirX = cy * bodyX + sy * bodyZ;
      const dirZ = -sy * bodyX + cy * bodyZ;
      const aThrust = kT * tNorm;
      vxEst += (aThrust * dirX - cd * vxEst + dxEst) * dt;
      vyEst += (aThrust * dirY - 9.81 - cd * vyEst + dyEst) * dt;
      vzEst += (aThrust * dirZ - cd * vzEst + dzEst) * dt;

      return { thrust, roll, pitch, yawRate };
    },
  };
}
