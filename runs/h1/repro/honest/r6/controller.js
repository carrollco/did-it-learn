export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  let stepCount = 0;
  let prevPos = null, prevVel = null, prevAtt = null, prevRates = null;
  let lastAction = { thrust: 0, roll: 0, pitch: 0, yawRate: 0 };

  const lagTau = 0.038;
  let thrustLag = 0;
  const uTorqueLag = { roll: 0, pitch: 0 };
  const simInt = { roll: 0, pitch: 0 };

  const torqueBias = { roll: 0, pitch: 0 };
  const torqueRes = { roll: 0, pitch: 0 };
  let thrustScale = 20.5;
  let windFast = { x: 0, z: 0 };
  let windSlow = { x: 0, z: 0 };

  let targetGate = -1;
  let phase = 'align';
  let centeredCount = 0;
  let prevTargetPitch = 0, prevTargetRoll = 0;

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (stepCount === 0) {
        prevPos = { x: p.x, y: p.y, z: p.z };
        prevVel = { x: v.x, y: v.y, z: v.z };
        prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };
        prevRates = { roll: rates.roll, pitch: rates.pitch, yaw: rates.yaw };
      } else {
        const alphaLag = 1 - Math.exp(-dt / lagTau);
        const prevThrustLag = thrustLag;
        thrustLag += (lastAction.thrust - thrustLag) * alphaLag;
        const avgThrustLag = 0.5 * (prevThrustLag + thrustLag);

        const wFactor = clamp(1 - (Math.abs(torqueBias.roll) + Math.abs(torqueBias.pitch)) * 0.5 / 184, 0.91, 1.0);

        for (const axis of ['roll', 'pitch']) {
          const e1 = lastAction[axis] - prevAtt[axis];
          simInt[axis] = clamp(simInt[axis] + e1 * 0.005, -0.35, 0.35);
          const u1 = 65 * e1 - 13 * prevRates[axis] + 22 * simInt[axis];

          const attMid = prevAtt[axis] + prevRates[axis] * 0.005;
          const rateMid = 0.5 * (prevRates[axis] + rates[axis]);
          const e2 = lastAction[axis] - attMid;
          simInt[axis] = clamp(simInt[axis] + e2 * 0.005, -0.35, 0.35);
          const u2 = 65 * e2 - 13 * rateMid + 22 * simInt[axis];

          const uCmd = 0.5 * (u1 + u2);
          const prevULag = uTorqueLag[axis];
          uTorqueLag[axis] += (uCmd - uTorqueLag[axis]) * alphaLag;
          const avgULag = 0.5 * (prevULag + uTorqueLag[axis]);

          const alphaMeas = (rates[axis] - prevRates[axis]) / dt + 0.4 * rateMid;
          const predAlpha = wFactor * avgULag + torqueBias[axis] * avgThrustLag;
          const errAlpha = alphaMeas - predAlpha;

          if (avgThrustLag > 0.15) {
            torqueBias[axis] = clamp(torqueBias[axis] + 0.12 * (errAlpha / Math.max(0.25, avgThrustLag)), -38, 38);
          }
          torqueRes[axis] = clamp(0.6 * torqueRes[axis] + 0.25 * errAlpha, -8, 8);
        }

        const rMid = 0.5 * (prevAtt.roll + att.roll);
        const pMid = 0.5 * (prevAtt.pitch + att.pitch);
        const yMid = 0.5 * (prevAtt.yaw + att.yaw);
        const bodyX = Math.sin(pMid) * Math.cos(rMid), bodyZ = -Math.sin(rMid);
        const dirX = Math.cos(yMid) * bodyX + Math.sin(yMid) * bodyZ;
        const dirY = Math.cos(pMid) * Math.cos(rMid);
        const dirZ = -Math.sin(yMid) * bodyX + Math.cos(yMid) * bodyZ;

        const kDrag = 0.42 * (thrustScale + 0.0435 * (Math.abs(torqueBias.roll) + Math.abs(torqueBias.pitch))) / 16;

        if (p.y > 0.095 && prevPos.y > 0.09) {
          const ayMeas = (v.y - prevVel.y) / dt;
          const vyMid = 0.5 * (v.y + prevVel.y);
          const uY = avgThrustLag * dirY;
          if (uY > 0.18) {
            const neededThrustAcc = ayMeas + 9.81 + kDrag * vyMid;
            const errScale = neededThrustAcc - thrustScale * uY;
            thrustScale = clamp(thrustScale + 0.08 * (errScale / Math.max(0.22, uY)), 14.8, 27.2);
          }

          const axMeas = (v.x - prevVel.x) / dt;
          const azMeas = (v.z - prevVel.z) / dt;
          const vxMid = 0.5 * (v.x + prevVel.x);
          const vzMid = 0.5 * (v.z + prevVel.z);

          const specThrust = thrustScale * avgThrustLag;
          const wxInstant = clamp(axMeas - specThrust * dirX + kDrag * vxMid, -12, 12);
          const wzInstant = clamp(azMeas - specThrust * dirZ + kDrag * vzMid, -12, 12);

          windFast.x += 0.28 * (wxInstant - windFast.x);
          windFast.z += 0.28 * (wzInstant - windFast.z);
          windSlow.x += 0.07 * (wxInstant - windSlow.x);
          windSlow.z += 0.07 * (wzInstant - windSlow.z);
        }
      }

      stepCount++;

      const kDrag = 0.42 * (thrustScale + 0.0435 * (Math.abs(torqueBias.roll) + Math.abs(torqueBias.pitch))) / 16;
      const gustX = windFast.x - windSlow.x;
      const gustZ = windFast.z - windSlow.z;
      const gustMag = Math.hypot(gustX, gustZ);

      let tx, ty, tz, vxDes = 0, vyDes = 0, vzDes = 0;
      let kpX = 6.0, kdX = 4.6, kpZ = 8.5, kdZ = 5.5, kpY = 8.0, kdY = 5.0;
      let pinning = false;

      if (obs.gateIndex < 3) {
        const g = obs.nextGate.centre;
        if (targetGate !== obs.gateIndex) {
          targetGate = obs.gateIndex;
          phase = 'align';
          centeredCount = 0;
        }

        const dy0 = Math.abs(p.y - g.y), dz0 = Math.abs(p.z - g.z);
        const alignX = g.x - clamp(0.22 + 1.2 * dz0 + 0.2 * dy0, 0.22, 0.46);
        const dy = Math.abs(p.y - g.y);
        const dz = Math.abs(p.z - g.z);
        const tCross = Math.max(0.04, Math.min(0.25, (g.x - p.x) / Math.max(0.6, v.x)));
        const predY = Math.abs((p.y + v.y * tCross) - g.y);
        const predZ = Math.abs((p.z + v.z * tCross) - g.z);

        if (phase === 'align') {
          tx = alignX;
          ty = g.y;
          tz = g.z;
          kpZ = 15.0; kdZ = 7.6;
          kpY = 9.0; kdY = 5.4;

          const xReady = p.x > g.x - 0.42 && p.x < g.x - 0.08 && Math.abs(v.x) < 0.7;
          const yzReady = (obs.t > 4.2 + obs.gateIndex * 3.2 ? (dz < 0.018 && dy < 0.070 && Math.abs(v.z) < 0.15 && Math.abs(v.y) < 0.32 && gustMag < 1.35) : (dz < 0.015 && predZ < 0.017 && dy < 0.065 && Math.abs(v.z) < 0.12 && Math.abs(v.y) < 0.28 && gustMag < 1.15));
          if (xReady && yzReady && stepCount > 35) centeredCount++;
          else centeredCount = Math.max(0, centeredCount - 1);

          if (centeredCount >= 5) {
            phase = 'lunge';
          }
        }

        if (phase === 'lunge') {
          const vEff = Math.max(0, v.x + Math.max(0, 9.81 * att.pitch) * 0.08);
          const stopDist = v.x > 0 ? (v.x * 0.08 + (vEff * vEff) / 8.0) : 0;
          const canStop = p.x + stopDist < g.x - 0.04 && p.x < g.x - 0.08;
          const unsafe = (dz > 0.025 || predZ > 0.027 || dy > 0.105 || predY > 0.115 || (gustMag > 1.75 && p.x < g.x - 0.13));
          if (unsafe && canStop) {
            phase = 'align';
            centeredCount = 0;
            tx = g.x - 0.26;
            ty = g.y;
            tz = g.z;
            vxDes = -0.3;
            kpX = 8.0; kdX = 6.0;
          } else {
            tx = g.x + 0.26;
            ty = g.y;
            tz = g.z;
            vxDes = 1.45;
            kpX = 6.0; kdX = 4.5;
            kpZ = 23.0; kdZ = 9.6;
            kpY = 11.5; kdY = 6.2;
          }
        }
      } else {
        const pad = obs.pad.centre;
        if (targetGate !== 3) {
          targetGate = 3;
          phase = 'pad_approach';
        }
        const distPad = Math.hypot(p.x - pad.x, p.z - pad.z);
        const speedHoriz = Math.hypot(v.x, v.z);

        tx = pad.x;
        tz = pad.z;
        kpX = 7.2; kdX = 5.2;
        kpZ = 7.2; kdZ = 5.2;

        if (phase === 'pad_approach') {
          ty = Math.max(0.21, Math.min(0.80, 0.16 + distPad * 0.82));
          vyDes = p.y > 0.34 ? -0.26 : 0;
          if (distPad < 0.11 && speedHoriz < 0.35 && p.y < 0.35) {
            phase = 'pad_descend';
          }
        }
        if (phase === 'pad_descend') {
          if (distPad > 0.14) {
            ty = Math.max(p.y, 0.18);
          } else {
            ty = 0.05;
            vyDes = -0.22;
            kpY = 7.0; kdY = 5.5;
          }
          if (p.y <= 0.088 && distPad <= 0.135 && speedHoriz < 0.42 && Math.abs(v.y) < 0.35) {
            phase = 'pad_pin';
          }
        }
        if (phase === 'pad_pin') {
          if (distPad > 0.142 || p.y > 0.095) {
            phase = 'pad_descend';
            ty = 0.05;
          } else {
            pinning = true;
            ty = 0.0;
          }
        }
      }

      const windCompX = windFast.x + 0.30 * gustX;
      const windCompZ = windFast.z + 0.30 * gustZ;

      let ax = kpX * (tx - p.x) + kdX * (vxDes - v.x) + kDrag * v.x - windCompX;
      let az = kpZ * (tz - p.z) + kdZ * (vzDes - v.z) + kDrag * v.z - windCompZ;

      if (obs.gateIndex > 0) {
        const pg = obs.gateIndex === 1 ? { x: -1.15, y: 1.05, z: -0.85 } : (obs.gateIndex === 2 ? { x: 0.15, y: 1.40, z: 0.25 } : { x: 1.35, y: 1.05, z: 1.05 });
        if (p.x < pg.x + 0.12) {
          ax = 7.5 * (pg.x + 0.30 - p.x) + 4.5 * (1.35 - v.x) + kDrag * v.x - windCompX;
          az = 18.0 * (pg.z - p.z) - 8.5 * v.z + kDrag * v.z - windCompZ;
          ty = pg.y;
        } else if (p.x < pg.x + 0.25 && v.x < 0.35) {
          ax = Math.max(ax, 8.5 * (pg.x + 0.27 - p.x) - 6.5 * v.x - windCompX);
        }
      }
      if (obs.gateIndex < 3 && phase === "align" && (Math.abs(p.z - obs.nextGate.centre.z) > 0.024 || Math.abs(p.y - obs.nextGate.centre.y) > 0.09)) {
        const vxMax = clamp(1.8 * (obs.nextGate.centre.x - 0.20 - p.x), -0.7, 1.35);
        if (v.x > vxMax) ax = Math.min(ax, -8.5 * (v.x - vxMax) - windCompX);
      }

      if (p.x > 2.20 && v.x > 1.8 * (2.76 - p.x)) ax = Math.min(ax, -8.5 * (v.x - 1.8 * (2.76 - p.x)) - windCompX);
      if (p.x < -2.20 && v.x < -1.8 * (2.76 + p.x)) ax = Math.max(ax, -8.5 * (v.x + 1.8 * (2.76 + p.x)) - windCompX);
      if (p.z > 2.20 && v.z > 1.8 * (2.76 - p.z)) az = Math.min(az, -8.5 * (v.z - 1.8 * (2.76 - p.z)) - windCompZ);
      if (p.z < -2.20 && v.z < -1.8 * (2.76 + p.z)) az = Math.max(az, -8.5 * (v.z + 1.8 * (2.76 + p.z)) - windCompZ);

      ax = clamp(ax, -6.2, 6.2);
      az = clamp(az, -7.5, 7.5);

      let ay = kpY * (ty - p.y) + kdY * (vyDes - v.y) + kDrag * v.y;
      if (p.y > 2.65 && v.y > 0) ay = Math.min(ay, -6.0 * v.y);
      ay = clamp(ay, -5.5, 6.5);

      const totalAy = Math.max(2.5, 9.81 + ay);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const accBodyX = cy * ax - sy * az;
      const accBodyZ = sy * ax + cy * az;

      const maxTilt = (obs.gateIndex === 3 && p.y <= 0.10) ? 0.27 : 0.54;
      let desPitch = clamp(Math.atan2(accBodyX, totalAy), -maxTilt, maxTilt);
      let desRoll = clamp(-Math.atan2(accBodyZ * Math.cos(desPitch), totalAy), -maxTilt, maxTilt);

      let thrust;
      if (pinning) {
        thrust = 0.04;
        desRoll = clamp(desRoll * 0.25, -0.08, 0.08);
        desPitch = clamp(desPitch * 0.25, -0.08, 0.08);
      } else {
        const tiltCos = Math.max(0.55, Math.cos(att.roll) * Math.cos(att.pitch));
        thrust = clamp(totalAy / (thrustScale * tiltCos), 0.08, 0.95);
      }

      const wFactor = clamp(1 - (Math.abs(torqueBias.roll) + Math.abs(torqueBias.pitch)) * 0.5 / 184, 0.91, 1.0);
      const dRollDes = clamp((desRoll - prevTargetRoll) / dt, -4.0, 4.0);
      const dPitchDes = clamp((desPitch - prevTargetPitch) / dt, -4.0, 4.0);
      prevTargetRoll = desRoll;
      prevTargetPitch = desPitch;

      const kpAtt = 72, kdAtt = 14.5;
      const alphaRollDes = kpAtt * (desRoll - att.roll) + kdAtt * (0.35 * dRollDes - rates.roll);
      const alphaPitchDes = kpAtt * (desPitch - att.pitch) + kdAtt * (0.35 * dPitchDes - rates.pitch);

      const effThrustForBias = pinning ? 0.04 : clamp(0.6 * thrustLag + 0.4 * thrust, 0, 1);
      const uSimRoll = (alphaRollDes - torqueBias.roll * effThrustForBias - 0.5 * torqueRes.roll) / wFactor;
      const uSimPitch = (alphaPitchDes - torqueBias.pitch * effThrustForBias - 0.5 * torqueRes.pitch) / wFactor;

      const cmdRoll = clamp(att.roll + (uSimRoll + 13 * rates.roll - 22 * simInt.roll) / 65, -0.6, 0.6);
      const cmdPitch = clamp(att.pitch + (uSimPitch + 13 * rates.pitch - 22 * simInt.pitch) / 65, -0.6, 0.6);
      const cmdYawRate = clamp(-1.8 * att.yaw + 0.3 * rates.yaw, -2, 2);

      prevPos = { x: p.x, y: p.y, z: p.z };
      prevVel = { x: v.x, y: v.y, z: v.z };
      prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };
      prevRates = { roll: rates.roll, pitch: rates.pitch, yaw: rates.yaw };
      lastAction = { thrust, roll: cmdRoll, pitch: cmdPitch, yawRate: cmdYawRate };

      return lastAction;
    },
  };
}
