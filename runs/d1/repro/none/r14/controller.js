export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  const GATES = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];

  let initialized = false;
  let prevVel = { x: 0, y: 0, z: 0 };
  let prevDir = { x: 0, y: 1, z: 0 };
  let prevThrust = 0.45;
  let uFilt = 0;

  let kThrust = 21.0;
  let massEst = 0.72;
  let ayF = 9.81;
  let uyF = 9.81 / 21.0;

  let dxEst = 0, dzEst = 0;
  let dxSlow = 0, dzSlow = 0;
  let intX = 0, intY = 0, intZ = 0, intYaw = 0;

  let activeGate = 0;
  let phase = 'align';
  let settle = 0;
  let padSettle = 0;

  const shapedAcc = (err, vel, ffV, kp, kd, vMax, aBrake, intVal, ki) => {
    const aLin = kp * err + kd * (ffV - vel) + ki * intVal;
    if (Math.abs(err) < 0.22 && ffV === 0) return aLin;
    if (ffV !== 0) return aLin;
    const s = Math.sign(err);
    const mag = Math.abs(err);
    const vDes = s * Math.min(vMax, (kp / kd) * mag, Math.sqrt(2 * aBrake * mag));
    return kd * (vDes - vel) + ki * intVal;
  };

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;
      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const dir = {
        x: cy * bodyX + sy * bodyZ,
        y: Math.cos(att.pitch) * Math.cos(att.roll),
        z: -sy * bodyX + cy * bodyZ,
      };

      if (!initialized) {
        initialized = true;
        prevVel = { x: v.x, y: v.y, z: v.z };
        prevDir = dir;
      } else {
        const dtSub = 0.005, lagEst = 0.038;
        const alphaLag = 1 - Math.exp(-dtSub / lagEst);
        const u1 = uFilt + (prevThrust - uFilt) * alphaLag;
        const u2 = u1 + (prevThrust - u1) * alphaLag;
        const uAvg = 0.5 * (u1 + u2);
        uFilt = u2;

        const avgDirX = 0.5 * (prevDir.x + dir.x);
        const avgDirY = 0.5 * (prevDir.y + dir.y);
        const avgDirZ = 0.5 * (prevDir.z + dir.z);

        const vxAvg = 0.5 * (prevVel.x + v.x);
        const vyAvg = 0.5 * (prevVel.y + v.y);
        const vzAvg = 0.5 * (prevVel.z + v.z);

        const axStep = (v.x - prevVel.x) * 100;
        const ayStep = (v.y - prevVel.y) * 100;
        const azStep = (v.z - prevVel.z) * 100;

        if (p.y > 0.095 && prevThrust > 0.12 && prevThrust < 0.88 && avgDirY > 0.75) {
          const ayNeed = clamp(ayStep + 9.81 + (0.42 / massEst) * vyAvg, 2.0, 20.0);
          const uyStep = uAvg * avgDirY;
          if (uyStep > 0.15) {
            ayF += 0.12 * (ayNeed - ayF);
            uyF += 0.12 * (uyStep - uyF);
            kThrust = clamp(ayF / uyF, 17.2, 26.8);
            massEst = clamp(15.3 / kThrust, 0.60, 0.85);
          }
        }

        if (p.y > 0.085) {
          const axMod = kThrust * uAvg * avgDirX - (0.42 / massEst) * vxAvg;
          const azMod = kThrust * uAvg * avgDirZ - (0.42 / massEst) * vzAvg;
          const dxRaw = clamp(axStep - axMod, -6.5, 6.5);
          const dzRaw = clamp(azStep - azMod, -6.5, 6.5);
          dxEst += 0.28 * (dxRaw - dxEst);
          dzEst += 0.28 * (dzRaw - dzEst);
          dxSlow += 0.03 * (dxRaw - dxSlow);
          dzSlow += 0.03 * (dzRaw - dzSlow);
        }

        prevVel = { x: v.x, y: v.y, z: v.z };
        prevDir = dir;
      }

      if (activeGate < obs.gateIndex) {
        const crossedGate = GATES[activeGate];
        if (p.x >= crossedGate.x + 0.20) {
          activeGate = obs.gateIndex;
          phase = activeGate < 3 ? 'align' : 'pad_approach';
          settle = 0;
          padSettle = 0;
        } else {
          phase = 'cross';
        }
      }

      let target = { x: p.x, y: 1.0, z: p.z };
      let ffVx = 0, ffVy = 0, ffVz = 0;
      let kpX = 7.5, kdX = 4.8, kpZ = 8.5, kdZ = 5.0, kpY = 8.0, kdY = 4.8;
      let vMaxX = 1.05, aBrakeX = 1.6, vMaxZ = 1.25, aBrakeZ = 2.0, vMaxY = 1.15, aBrakeY = 2.0;

      if (activeGate < 3) {
        const gate = GATES[activeGate];
        const dxGate = gate.x - p.x;
        const dyGate = gate.y - p.y;
        const dzGate = gate.z - p.z;
        const yzErr = Math.hypot(dyGate, dzGate);
        const gustJumpZ = Math.abs(dzEst - dzSlow);

        if (phase === 'align') {
          const alignDist = yzErr > 0.14 ? clamp(0.28 + 0.22 * yzErr, 0.28, 0.48) : 0.25;
          target = { x: gate.x - alignDist, y: gate.y, z: gate.z };
          kpZ = 10.0; kdZ = 5.4;
          kpY = 9.0; kdY = 5.0;
          kpX = 8.0; kdX = 5.0;
          vMaxX = 0.95; aBrakeX = 1.5;

          const errAlignX = Math.abs(target.x - p.x);
          const errY = Math.abs(dyGate);
          const errZ = Math.abs(dzGate);

          if (errAlignX < 0.11 && p.x < gate.x - 0.12 && errY < 0.065 && errZ < 0.036 && Math.abs(v.y) < 0.16 && Math.abs(v.z) < 0.12 && Math.abs(v.x) < 0.22 && gustJumpZ < 0.75) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= 7) {
            phase = 'cross';
          }
        }

        if (phase === 'cross') {
          const tCross = clamp(dxGate / Math.max(0.40, v.x), 0.02, 0.35);
          const zPred = p.z + v.z * tCross;
          const yPred = p.y + v.y * tCross;
          const zErrPred = Math.abs(gate.z - zPred);
          const yErrPred = Math.abs(gate.y - yPred);

          const stopDist = 0.08 * Math.max(0, v.x) + (v.x > 0 ? (v.x * v.x) / 5.5 : 0);

          if (obs.gateIndex === activeGate && dxGate > Math.max(0.11, stopDist + 0.03) && (zErrPred > 0.065 || Math.abs(dzGate) > 0.065 || yErrPred > 0.14 || Math.abs(dyGate) > 0.14)) {
            phase = 'align';
            settle = 0;
            target = { x: gate.x - 0.26, y: gate.y, z: gate.z };
            kpX = 9.5; kdX = 5.4;
          } else {
            target = { x: gate.x + 0.42, y: gate.y, z: gate.z };
            ffVx = 0.90;
            kpZ = 12.5; kdZ = 6.0;
            kpY = 10.0; kdY = 5.2;
            kpX = 8.0; kdX = 4.5;
          }
        }
      } else {
        const pad = obs.pad.centre;
        const horizErr = Math.hypot(pad.x - p.x, pad.z - p.z);
        const horizSpeed = Math.hypot(v.x, v.z);

        if (phase === 'pad_approach') {
          const descendRef = horizErr < 0.25 ? 0.24 : 0.70;
          target = { x: pad.x, y: descendRef, z: pad.z };
          kpX = 8.5; kdX = 5.0;
          kpZ = 8.5; kdZ = 5.0;
          if (horizErr < 0.065 && horizSpeed < 0.18 && p.y < 0.32) {
            padSettle++;
          } else {
            padSettle = Math.max(0, padSettle - 1);
          }
          if (padSettle >= 6) {
            phase = 'descend';
          }
        }

        if (phase === 'descend') {
          if (horizErr > 0.095 && p.y > 0.095) {
            phase = 'pad_approach';
            padSettle = 0;
            target = { x: pad.x, y: 0.22, z: pad.z };
          } else {
            target = { x: pad.x, y: 0.055, z: pad.z };
            ffVy = p.y > 0.11 ? -0.20 : -0.05;
            kpX = 8.5; kdX = 5.0;
            kpZ = 8.5; kdZ = 5.0;
            kpY = 7.5; kdY = 5.0;
          }
        }
      }

      const ex = target.x - p.x;
      const ey = target.y - p.y;
      const ez = target.z - p.z;

      if (Math.abs(ex) < 0.30) intX = clamp(intX + ex * 0.01, -0.4, 0.4);
      else intX *= 0.98;
      if (Math.abs(ez) < 0.30) intZ = clamp(intZ + ez * 0.01, -0.4, 0.4);
      else intZ *= 0.98;
      if (p.y > 0.085) intY = clamp(intY + ey * 0.01, -0.6, 0.6);

      let axCmd = shapedAcc(ex, v.x, ffVx, kpX, kdX, vMaxX, aBrakeX, intX, 0.8);
      if (activeGate < 3 && phase === 'align') {
        const dxHard = GATES[activeGate].x - 0.16 - p.x;
        if (v.x > 2.2 * dxHard) {
          axCmd = Math.min(axCmd, 5.5 * (2.2 * dxHard - v.x));
        }
      }
      axCmd = clamp(axCmd, -4.2, 4.2);

      const azCmd = clamp(shapedAcc(ez, v.z, ffVz, kpZ, kdZ, vMaxZ, aBrakeZ, intZ, 0.8), -4.2, 4.2);
      const ayCmd = clamp(shapedAcc(ey, v.y, ffVy, kpY, kdY, vMaxY, aBrakeY, intY, 1.8), -4.0, 5.0);

      const dragCoeff = 0.42 / massEst;
      const Tx = axCmd + dragCoeff * v.x - dxEst;
      const Tz = azCmd + dragCoeff * v.z - dzEst;
      const Ty = Math.max(4.0, 9.81 + ayCmd + dragCoeff * v.y);

      const TbodyX = cy * Tx - sy * Tz;
      const TbodyZ = sy * Tx + cy * Tz;

      let maxTilt = 0.50;
      if (obs.gateIndex === 3 && phase === 'descend') {
        maxTilt = p.y <= 0.088 ? 0.14 : 0.30;
      }

      const pitchDes = clamp(Math.atan2(TbodyX, Ty), -maxTilt, maxTilt);
      const rollDes = clamp(Math.atan2(-TbodyZ, Math.hypot(TbodyX, Ty)), -maxTilt, maxTilt);

      const pitchCmd = clamp(pitchDes + 0.18 * (pitchDes - att.pitch), -maxTilt, maxTilt);
      const rollCmd = clamp(rollDes + 0.18 * (rollDes - att.roll), -maxTilt, maxTilt);

      const tiltCos = Math.max(0.65, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = clamp(Ty / (kThrust * tiltCos), 0.05, 0.95);

      if (obs.gateIndex === 3 && phase === 'descend' && p.y <= 0.088 && Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z) <= 0.12 && Math.hypot(v.x, v.y, v.z) < 0.40) {
        thrust = clamp((9.81 - 2.5) / kThrust, 0.12, 0.34);
      }

      intYaw = clamp(intYaw + att.yaw * 0.01, -0.4, 0.4);
      const yawRate = clamp(-4.0 * att.yaw - 0.25 * rates.yaw - 2.5 * intYaw, -2.0, 2.0);

      prevThrust = thrust;
      return { thrust, roll: rollCmd, pitch: pitchCmd, yawRate };
    },
  };
}
