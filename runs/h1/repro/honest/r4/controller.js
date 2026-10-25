const GATES_POS = [
  { x: -1.15, y: 1.05, z: -0.85 },
  { x: 0.15, y: 1.40, z: 0.25 },
  { x: 1.35, y: 1.05, z: 1.05 },
];

export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  let initialized = false;
  let prevVel = { x: 0, y: 0, z: 0 };
  let prevAtt = { roll: 0, pitch: 0, yaw: 0 };
  let prevRates = { roll: 0, pitch: 0, yaw: 0 };
  let prevDir = { x: 0, y: 1, z: 0 };
  let lastAction = { thrust: 0.55, roll: 0, pitch: 0, yawRate: 0 };

  let simInt = { roll: 0, pitch: 0 };
  let uThrust = 0;
  let uAlpha = { roll: 0, pitch: 0, yaw: 0 };
  let kBias = { roll: 0, pitch: 0, yaw: 0 };

  let thrustScale = 20.5;
  let filtAy = 0, filtUy = 0.5, filtVy = 0;

  let windFast = { x: 0, z: 0 };
  let windSlow = { x: 0, z: 0 };
  let vFilt = { x: 0, y: 0, z: 0 };

  let intY = 0, intZ = 0, intX = 0;
  let gateIdx = -1;
  let phase = 'align';
  let braking = false;
  let centeredCount = 0;
  let padPhase = 'approach';

  function getDir(att) {
    const bx = Math.sin(att.pitch) * Math.cos(att.roll);
    const bz = -Math.sin(att.roll);
    const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
    return {
      x: cy * bx + sy * bz,
      y: Math.cos(att.pitch) * Math.cos(att.roll),
      z: -sy * bx + cy * bz,
    };
  }

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;
      const dir = getDir(att);

      if (!initialized) {
        initialized = true;
        prevVel = { ...v };
        prevAtt = { ...att };
        prevRates = { ...rates };
        prevDir = { ...dir };
        vFilt = { ...v };
      } else {
        vFilt.x += 0.45 * (v.x - vFilt.x);
        vFilt.y += 0.45 * (v.y - vFilt.y);
        vFilt.z += 0.45 * (v.z - vFilt.z);

        const alphaM = 1 - Math.exp(-0.005 / 0.038);
        const uThrust1 = uThrust + (lastAction.thrust - uThrust) * alphaM;
        const uThrust2 = uThrust1 + (lastAction.thrust - uThrust1) * alphaM;
        const effUThrust = 0.5 * (uThrust1 + uThrust2);
        uThrust = uThrust2;

        const effUAlpha = { roll: 0, pitch: 0, yaw: 0 };
        for (const axis of ['roll', 'pitch']) {
          const e1 = lastAction[axis] - prevAtt[axis];
          simInt[axis] = clamp(simInt[axis] + e1 * 0.005, -0.35, 0.35);
          const a1 = 65 * e1 - 13 * prevRates[axis] + 22 * simInt[axis];
          const ua1 = uAlpha[axis] + (a1 - uAlpha[axis]) * alphaM;

          const midAtt = 0.5 * (prevAtt[axis] + att[axis]);
          const midRate = 0.5 * (prevRates[axis] + rates[axis]);
          const e2 = lastAction[axis] - midAtt;
          simInt[axis] = clamp(simInt[axis] + e2 * 0.005, -0.35, 0.35);
          const a2 = 65 * e2 - 13 * midRate + 22 * simInt[axis];
          const ua2 = ua1 + (a2 - ua1) * alphaM;

          effUAlpha[axis] = 0.5 * (ua1 + ua2);
          uAlpha[axis] = ua2;
        }
        const ay1 = 8 * (lastAction.yawRate - prevRates.yaw);
        const uay1 = uAlpha.yaw + (ay1 - uAlpha.yaw) * alphaM;
        const ay2 = 8 * (lastAction.yawRate - 0.5 * (prevRates.yaw + rates.yaw));
        const uay2 = uay1 + (ay2 - uay1) * alphaM;
        effUAlpha.yaw = 0.5 * (uay1 + uay2);
        uAlpha.yaw = uay2;

        if (p.y > 0.085 && effUThrust > 0.18 && lastAction.thrust < 0.85) {
          for (const axis of ['roll', 'pitch', 'yaw']) {
            const midRate = 0.5 * (prevRates[axis] + rates[axis]);
            const measAlpha = (rates[axis] - prevRates[axis]) / dt + 0.4 * midRate;
            const dist = measAlpha - effUAlpha[axis];
            const obsK = clamp(dist / effUThrust, -45, 45);
            const rateGain = obs.t < 0.35 ? 0.28 : 0.14;
            kBias[axis] = clamp(kBias[axis] + rateGain * (obsK - kBias[axis]), -45, 45);
          }
        }

        if (p.y > 0.09) {
          const dragCoeff = 0.0275 * thrustScale;
          const measAy = (v.y - prevVel.y) / dt;
          const uy = effUThrust * 0.5 * (prevDir.y + dir.y);
          const vy = 0.5 * (prevVel.y + v.y);
          filtAy += 0.22 * (measAy - filtAy);
          filtUy += 0.22 * (uy - filtUy);
          filtVy += 0.22 * (vy - filtVy);
          if (filtUy > 0.25 && obs.t > 0.05 && lastAction.thrust < 0.85) {
            const errAy = (filtAy + 9.81 + dragCoeff * filtVy) - thrustScale * filtUy;
            thrustScale = clamp(thrustScale + 0.20 * (errAy / filtUy), 14.5, 27.5);
          }

          const ux = effUThrust * 0.5 * (prevDir.x + dir.x);
          const uz = effUThrust * 0.5 * (prevDir.z + dir.z);
          const vx = 0.5 * (prevVel.x + v.x);
          const vz = 0.5 * (prevVel.z + v.z);
          const measAx = (v.x - prevVel.x) / dt;
          const measAz = (v.z - prevVel.z) / dt;
          const rawWx = clamp(measAx + dragCoeff * vx - thrustScale * ux, -16, 16);
          const rawWz = clamp(measAz + dragCoeff * vz - thrustScale * uz, -16, 16);

          windFast.x += 0.32 * (rawWx - windFast.x);
          windFast.z += 0.32 * (rawWz - windFast.z);
          windSlow.x += 0.035 * (rawWx - windSlow.x);
          windSlow.z += 0.035 * (rawWz - windSlow.z);
        }
      }

      const gustDx = windFast.x - windSlow.x;
      const gustDz = windFast.z - windSlow.z;
      const gustMag = Math.hypot(gustDx, gustDz);

      let target = { x: p.x, y: 1.0, z: p.z };
      let targetVel = { x: 0, y: 0, z: 0 };
      let kpX = 7.5, kdX = 5.2, kpZ = 8.5, kdZ = 5.6;
      let kpY = 8.0, kdY = 5.0;
      let maxTilt = 0.52;
      let pinPad = false;
      let hardBrakeX = false;

      const prevGate = obs.gateIndex > 0 ? GATES_POS[obs.gateIndex - 1] : null;
      const clearingPrevGate = prevGate && obs.gateIndex < 3 && p.x < prevGate.x + 0.14;

      if (clearingPrevGate) {
        target = { x: prevGate.x + 0.35, y: prevGate.y, z: prevGate.z };
        targetVel.x = 1.25;
        kpX = 8.5; kdX = 4.8;
        kpZ = 11.0; kdZ = 6.3;
        kpY = 9.0; kdY = 5.0;
      } else if (obs.gateIndex < 3) {
        const g = obs.nextGate.centre;
        if (gateIdx !== obs.gateIndex) {
          gateIdx = obs.gateIndex;
          phase = 'align';
          braking = false;
          centeredCount = 0;
          intZ = 0;
          intX = 0;
        }

        const errY = Math.abs(g.y - p.y);
        const errZ = Math.abs(g.z - p.z);
        const dx = Math.max(0.02, g.x - p.x);
        const dtCross = clamp(dx / Math.max(0.45, vFilt.x), 0.04, 0.15);
        const predY = p.y + vFilt.y * dtCross;
        const predZ = p.z + vFilt.z * dtCross;
        const predZFast = p.z + (0.65 * v.z + 0.35 * vFilt.z) * dtCross;
        const predErrY = Math.abs(g.y - predY);
        const predErrZ = Math.abs(g.z - predZ);
        const predErrZFast = Math.abs(g.z - predZFast);

        if (errZ < 0.08 && Math.abs(vFilt.z) < 0.22 && p.y > 0.3) {
          intZ = clamp(intZ + (g.z - p.z) * dt, -0.18, 0.18);
        } else if (errZ > 0.12) {
          intZ *= 0.85;
        }

        if (phase === 'align') {
          if (braking && v.x <= 0.05 && p.x < g.x - 0.14) {
            braking = false;
          }
          const alignDist = errZ < 0.06 ? 0.21 : clamp(0.21 + 0.28 * errZ + 0.10 * errY, 0.21, 0.42);
          target = { x: g.x - alignDist, y: g.y, z: g.z };
          kpZ = 10.5; kdZ = 6.3;
          kpY = 9.0; kdY = 5.2;
          kpX = 7.8; kdX = 5.4;

          const xReady = !braking && p.x > g.x - 0.36 && p.x < g.x - 0.04 && vFilt.x > -0.22 && vFilt.x < 0.60;
          const zReady = errZ < 0.014 && predErrZ < 0.017 && Math.abs(vFilt.z) < 0.095;
          const yReady = errY < 0.095 && predErrY < 0.105 && Math.abs(vFilt.y) < 0.35;

          if (xReady && zReady && yReady && gustMag < 2.6) {
            centeredCount++;
          } else {
            centeredCount = Math.max(0, centeredCount - 1);
          }
          if (centeredCount >= 2) {
            phase = 'lunge';
          }

          if (braking || (p.x + Math.max(0, v.x) * 0.15 > g.x - 0.06 && (!zReady || !yReady))) {
            braking = true;
            target.x = g.x - 0.25;
            kpX = 15.0; kdX = 9.0;
            if (p.x > g.x - 0.23 && v.x > 0.02) {
              hardBrakeX = true;
            }
          }
        }

        if (phase === 'lunge') {
          if (errZ < 0.08) {
            intZ = clamp(intZ + (g.z - p.z) * dt, -0.18, 0.18);
          }
          const unsafe = predErrZFast > 0.022 || errZ > 0.022 || predErrY > 0.120 || errY > 0.125;
          if (p.x < g.x - 0.065 && unsafe) {
            phase = 'align';
            braking = true;
            centeredCount = 0;
            target = { x: g.x - 0.25, y: g.y, z: g.z };
            kpX = 16.0; kdX = 9.5;
            hardBrakeX = true;
          } else {
            target = { x: g.x + 0.30, y: g.y, z: g.z };
            targetVel.x = 1.15;
            kpX = 8.5; kdX = 4.8;
            kpZ = 16.0; kdZ = 7.7;
            kpY = 9.5; kdY = 5.2;
          }
        }
      } else {
        const pad = obs.pad.centre;
        const distXZ = Math.hypot(pad.x - p.x, pad.z - p.z);
        const speedXZ = Math.hypot(vFilt.x, vFilt.z);
        const speed = Math.hypot(v.x, v.y, v.z);

        if (distXZ < 0.25) {
          intZ = clamp(intZ + (pad.z - p.z) * dt, -0.18, 0.18);
          intX = clamp(intX + (pad.x - p.x) * dt, -0.18, 0.18);
        }

        if (padPhase === 'approach') {
          const hTarget = clamp(0.09 + 0.38 * distXZ, 0.09, 0.70);
          target = { x: pad.x, y: hTarget, z: pad.z };
          if (p.y > hTarget + 0.06) {
            targetVel.y = -0.65;
          }
          kpX = 8.5; kdX = 5.5;
          kpZ = 8.5; kdZ = 5.5;
          if (distXZ < 0.135 && speedXZ < 0.40 && p.y < 0.35) {
            padPhase = 'descend';
          }
        }

        if (padPhase === 'descend') {
          target = { x: pad.x, y: 0.04, z: pad.z };
          targetVel.y = p.y > 0.13 ? -0.45 : -0.25;
          kpX = 9.0; kdX = 5.8;
          kpZ = 9.0; kdZ = 5.8;
          kpY = 7.5; kdY = 5.0;
          maxTilt = 0.24;
          if (distXZ > 0.146 && p.y > 0.13) {
            padPhase = 'approach';
          }
          if (p.y <= 0.092 && distXZ <= 0.140 && speed < 0.46) {
            padPhase = 'pin';
          }
        }

        if (padPhase === 'pin') {
          if (distXZ > 0.146 || p.y > 0.094) {
            padPhase = 'descend';
          } else {
            pinPad = true;
          }
        }
      }

      if (pinPad) {
        const pad = obs.pad.centre;
        const axPin = clamp(4.5 * (pad.x - p.x) - 3.2 * v.x - 0.45 * windFast.x, -1.0, 1.0);
        const azPin = clamp(4.5 * (pad.z - p.z) - 3.2 * v.z - 0.45 * windFast.z, -1.0, 1.0);
        const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
        const desPitch = clamp((cy * axPin - sy * azPin) / 9.81, -0.09, 0.09);
        const desRoll = clamp(-(sy * axPin + cy * azPin) / 9.81, -0.09, 0.09);
        const pinThrust = 0.08;
        const bRoll = kBias.roll * pinThrust;
        const bPitch = kBias.pitch * pinThrust;
        const bYaw = kBias.yaw * pinThrust;
        const cmdRoll = clamp(desRoll + (65 * (desRoll - att.roll) - 6.0 * rates.roll - bRoll - 22 * simInt.roll) / 65, -0.15, 0.15);
        const cmdPitch = clamp(desPitch + (65 * (desPitch - att.pitch) - 6.0 * rates.pitch - bPitch - 22 * simInt.pitch) / 65, -0.15, 0.15);
        const cmdYawRate = clamp(rates.yaw + (-10 * att.yaw - 3.5 * rates.yaw - bYaw) / 8, -2, 2);

        lastAction = { thrust: pinThrust, roll: cmdRoll, pitch: cmdPitch, yawRate: cmdYawRate };
        prevVel = { ...v }; prevAtt = { ...att }; prevRates = { ...rates }; prevDir = { ...dir };
        return lastAction;
      }

      if (obs.t < 0.14 && p.y < 0.20) {
        maxTilt = 0.20;
      }

      const dragCoeff = 0.0275 * thrustScale;
      const windCompX = windFast.x + 0.22 * gustDx;
      const windCompZ = windFast.z + 0.24 * gustDz;
      const gustBrake = clamp(gustMag * 0.36, 0, 1.6);

      const ex = clamp(target.x - p.x, -0.65, 0.65);
      const ez = clamp(target.z - p.z, -0.65, 0.65);

      let ax = kpX * ex + (kdX + gustBrake) * (targetVel.x - v.x) + 4.5 * intX + dragCoeff * v.x - windCompX;
      let az = kpZ * ez + (kdZ + gustBrake) * (targetVel.z - v.z) + 7.5 * intZ + dragCoeff * v.z - windCompZ;

      if (hardBrakeX && v.x > 0.01) {
        ax = Math.min(ax, -7.0 - 9.5 * v.x - windCompX);
      }

      if (prevGate) {
        const predMinX = p.x + Math.min(0, v.x) * 0.18;
        if (predMinX < prevGate.x + 0.12) {
          ax = Math.max(ax, 11.0 * (prevGate.x + 0.18 - p.x) - 7.0 * v.x - windCompX);
        }
      }

      if (p.x + Math.max(0, v.x) * 0.22 > 2.65) {
        ax = Math.min(ax, 14.0 * (2.55 - p.x) - 9.0 * v.x - windCompX);
      }
      if (p.z + Math.max(0, v.z) * 0.22 > 2.65) {
        az = Math.min(az, 14.0 * (2.55 - p.z) - 9.0 * v.z - windCompZ);
      }

      const ey = clamp(target.y - p.y, -0.55, 0.55);
      if (Math.abs(target.y - p.y) < 0.18 && p.y > 0.20) {
        intY = clamp(intY + (target.y - p.y) * dt, -0.25, 0.25);
      } else {
        intY *= 0.92;
      }
      const ay = clamp(kpY * ey + kdY * (targetVel.y - v.y) + 3.2 * intY + dragCoeff * v.y, -4.5, 4.2);

      const totalAy = Math.max(3.2, 9.81 + ay);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyAx = cy * ax - sy * az;
      const bodyAz = sy * ax + cy * az;

      const desPitch = clamp(Math.atan2(bodyAx, totalAy), -maxTilt, maxTilt);
      const desRoll = clamp(Math.atan2(-bodyAz, Math.hypot(bodyAx, totalAy)), -maxTilt, maxTilt);

      const tiltCos = Math.max(0.58, Math.cos(att.roll) * Math.cos(att.pitch));
      const rawThrust = clamp(totalAy / (thrustScale * tiltCos), 0.06, 0.78);
      const thrust = clamp(rawThrust + 0.20 * (rawThrust - uThrust), 0.06, 0.80);

      const estUThrust = 0.65 * uThrust + 0.35 * thrust;
      const bRoll = kBias.roll * estUThrust;
      const bPitch = kBias.pitch * estUThrust;
      const bYaw = kBias.yaw * estUThrust;

      const cmdRoll = clamp(desRoll + (75 * (desRoll - att.roll) - 7.6 * rates.roll - bRoll - 22 * simInt.roll) / 65, -0.6, 0.6);
      const cmdPitch = clamp(desPitch + (75 * (desPitch - att.pitch) - 7.6 * rates.pitch - bPitch - 22 * simInt.pitch) / 65, -0.6, 0.6);
      const cmdYawRate = clamp(rates.yaw + (-12 * att.yaw - 4.0 * rates.yaw - bYaw) / 8, -2, 2);

      lastAction = { thrust, roll: cmdRoll, pitch: cmdPitch, yawRate: cmdYawRate };
      prevVel = { ...v };
      prevAtt = { ...att };
      prevRates = { ...rates };
      prevDir = { ...dir };
      return lastAction;
    },
  };
}
