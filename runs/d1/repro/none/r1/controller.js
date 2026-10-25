export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  const profileVel = (err, kp, aBrake, vMax) => {
    const a = Math.abs(err);
    if (a < 1e-6) return 0;
    const d0 = aBrake / (kp * kp);
    const mag = a <= d0 ? kp * a : Math.sqrt(2 * aBrake * a - aBrake * d0);
    return (err < 0 ? -1 : 1) * Math.min(vMax, mag);
  };

  let initialized = false;
  let px = 0, py = 0, pz = 0;
  let vx = 0, vy = 0, vz = 0;

  let kT = 21.4;
  let motorTau = 0.04;
  let uThrustState = 0.0;
  let prevThrustCmd = 0.45;

  let vYf = 0, phiYf = 0;
  let vXf = 0, aModXf = 0;
  let vZf = 0, aModZf = 0;
  let vYw = 0;

  let baseWindAx = 0, baseWindAz = 0;
  let distAx = 0, distAz = 0;

  let intX = 0, intY = 0, intZ = 0;
  let intYaw = 0;

  let lastGateIndex = 0;
  let committedCross = false;
  let descendingToPad = false;
  let touchdownLock = false;

  return {
    step(obs) {
      const dt = 0.01;
      const g = 9.81;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (!initialized) {
        initialized = true;
        px = p.x; py = p.y; pz = p.z;
        vx = v.x; vy = v.y; vz = v.z;
        vYf = v.y; phiYf = 0.0;
        vXf = v.x; vZf = v.z; vYw = v.y;
      } else {
        px = 0.35 * (px + vx * dt) + 0.65 * p.x;
        py = 0.35 * (py + vy * dt) + 0.65 * p.y;
        pz = 0.35 * (pz + vz * dt) + 0.65 * p.z;
        vx = 0.30 * vx + 0.70 * v.x;
        vy = 0.30 * vy + 0.70 * v.y;
        vz = 0.30 * vz + 0.70 * v.z;
      }

      const alphaMotor = 1 - Math.exp(-dt / motorTau);
      uThrustState += (prevThrustCmd - uThrustState) * alphaMotor;

      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
      const dirZ = -sy * bodyX + cy * bodyZ;

      const estMass = clamp(15.2 / kT, 0.60, 0.85);
      const cd = 0.42 / estMass;

      // 1. Online K_T estimation when airborne
      if (py > 0.12 && obs.t > 0.14 && !touchdownLock) {
        const tauY = 0.06;
        const aY = Math.exp(-dt / tauY);
        const phiY = uThrustState * dirY;
        vYf = aY * vYf + (1 - aY) * vy;
        phiYf = aY * phiYf + (1 - aY) * phiY;
        const lhsY = (vy - vYf) / tauY + g + cd * vYf;
        if (phiY > 0.25 && phiYf > 0.25 && obs.t > 0.20) {
          const errKT = lhsY - kT * phiYf;
          const gain = obs.t < 1.8 ? 1.8 : 0.4;
          kT = clamp(kT + gain * errKT * phiYf * dt * 10, 17.2, 27.0);
        }
      } else {
        vYf = vy;
        phiYf = uThrustState * dirY;
      }

      // 2. Horizontal disturbance observer (phase-matched filter on v and aMod)
      const tauW = 0.042;
      const aW = Math.exp(-dt / tauW);
      const aThrustMag = kT * uThrustState;
      const aModX = aThrustMag * dirX - cd * vx;
      const aModZ = aThrustMag * dirZ - cd * vz;

      if (py > 0.095 && obs.t > 0.16 && !touchdownLock) {
        vXf = aW * vXf + (1 - aW) * vx;
        vZf = aW * vZf + (1 - aW) * vz;
        aModXf = aW * aModXf + (1 - aW) * aModX;
        aModZf = aW * aModZf + (1 - aW) * aModZ;

        const rawDistX = (vx - vXf) / tauW - aModXf;
        const rawDistZ = (vz - vZf) / tauW - aModZf;

        distAx = clamp(0.30 * distAx + 0.70 * rawDistX, -4.5, 4.5);
        distAz = clamp(0.30 * distAz + 0.70 * rawDistZ, -4.5, 4.5);

        if (obs.t > 0.28 && obs.t < 1.95) {
          baseWindAx = 0.92 * baseWindAx + 0.08 * distAx;
          baseWindAz = 0.92 * baseWindAz + 0.08 * distAz;
        }
      } else {
        vXf = vx; vZf = vz; vYw = vy;
        aModXf = aModX; aModZf = aModZ;
        distAx = 0.95 * distAx + 0.05 * baseWindAx;
        distAz = 0.95 * distAz + 0.05 * baseWindAz;
      }

      if (obs.gateIndex !== lastGateIndex) {
        lastGateIndex = obs.gateIndex;
        committedCross = false;
        intX *= 0.2; intY *= 0.2; intZ *= 0.2;
      }

      // 3. Trajectory & Cascade Velocity Command Generation
      let tx, ty, tz;
      let desVx = 0, desVy = 0, desVz = 0;

      const climbScale = clamp((py - 0.12) / 0.35, 0.20, 1.0);

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        const distToGateX = gate.x - px;
        const tCross = clamp(distToGateX / Math.max(0.35, vx), 0.06, 0.22);
        const predY = py + vy * tCross;
        const predZ = pz + vz * tCross;
        const errY = Math.abs(py - gate.y);
        const errZ = Math.abs(pz - gate.z);
        const predErrY = Math.abs(predY - gate.y);
        const predErrZ = Math.abs(predZ - gate.z);

        ty = gate.y;
        tz = gate.z;

        const aligned = (errZ < 0.064 && predErrZ < 0.068 && errY < 0.13 && predErrY < 0.14 && Math.abs(vz) < 0.30 && Math.abs(vy) < 0.50);
        if (px > gate.x - 0.09 && errZ < 0.088 && predErrZ < 0.092 && errY < 0.17 && vx > 0.20) {
          committedCross = true;
        }

        if (committedCross || aligned) {
          tx = gate.x + 0.38;
          desVx = profileVel(tx - px, 2.5, 1.8, 0.95 * climbScale);
          if (distToGateX > -0.05 && distToGateX < 0.35) {
            desVx = Math.max(desVx, 0.65);
          }
        } else {
          let holdX = gate.x - 0.35;
          if (obs.gateIndex === 0 && py < 0.75) {
            holdX = Math.min(holdX, -1.65);
          }
          tx = holdX;
          desVx = profileVel(holdX - px, 2.4, 1.7, 0.80 * climbScale);
        }

        desVz = profileVel(tz - pz, 2.6, 1.7, 0.95 * climbScale);
        desVy = profileVel(ty - py, 2.6, 2.0, 1.15);
      } else {
        const pad = obs.pad.centre;
        const distXZ = Math.hypot(pad.x - px, pad.z - pz);
        const speedXZ = Math.hypot(vx, vz);
        const speed3D = Math.hypot(vx, vy, vz);

        tx = pad.x;
        tz = pad.z;

        if ((py <= 0.084 && distXZ <= 0.120 && speed3D < 0.44 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.17) ||
            (touchdownLock && distXZ <= 0.140 && py <= 0.092)) {
          touchdownLock = true;
          prevThrustCmd = 0;
          return { thrust: 0, roll: 0, pitch: 0, yawRate: 0 };
        }
        touchdownLock = false;

        desVx = profileVel(tx - px, 2.4, 1.5, 0.85);
        desVz = profileVel(tz - pz, 2.4, 1.5, 0.85);

        if (distXZ < 0.085 && speedXZ < 0.20) {
          descendingToPad = true;
        } else if (distXZ > 0.125 || speedXZ > 0.35) {
          descendingToPad = false;
        }

        if (!descendingToPad) {
          ty = clamp(0.24 + 0.55 * distXZ, 0.24, 1.05);
          desVy = profileVel(ty - py, 2.4, 1.8, 0.80);
        } else {
          ty = 0.04;
          desVy = py > 0.13 ? -0.24 : -0.14;
        }
      }

      // 4. Cascade Velocity Tracking + Small Position Integral + Disturbance Rejection
      const ex = tx - px, ey = ty - py, ez = tz - pz;
      const evx = desVx - vx, evy = desVy - vy, evz = desVz - vz;

      if (Math.abs(ex) < 0.25 && py > 0.10) intX = clamp(intX + ex * dt, -0.30, 0.30);
      else intX *= 0.98;
      if (Math.abs(ez) < 0.25 && py > 0.10) intZ = clamp(intZ + ez * dt, -0.30, 0.30);
      else intZ *= 0.98;
      if (Math.abs(ey) < 0.30 && py > 0.10) intY = clamp(intY + ey * dt, -0.40, 0.40);

      const kvXZ = 5.4;
      const kpSmallXZ = 1.6;
      const kiXZ = 1.4;

      const axDes = kvXZ * evx + kpSmallXZ * clamp(ex, -0.25, 0.25) + kiXZ * intX + cd * vx - distAx;
      const azDes = kvXZ * evz + kpSmallXZ * clamp(ez, -0.25, 0.25) + kiXZ * intZ + cd * vz - distAz;
      const ayDes = clamp(5.2 * evy + 1.8 * clamp(ey, -0.30, 0.30) + 1.8 * intY + cd * vy, -3.2, 3.2);

      const axClamped = clamp(axDes, -4.5, 4.5);
      const azClamped = clamp(azDes, -4.5, 4.5);
      const ayTotal = Math.max(2.8, g + ayDes);

      const aMag = Math.hypot(axClamped, ayTotal, azClamped);
      const dxDes = axClamped / aMag;
      const dyDes = ayTotal / aMag;
      const dzDes = azClamped / aMag;

      const bodyXDes = cy * dxDes - sy * dzDes;
      const bodyZDes = sy * dxDes + cy * dzDes;

      let maxTilt = 0.48;
      let maxCmdTilt = 0.58;
      if (obs.gateIndex === 3 && py < 0.20) {
        maxTilt = py < 0.12 ? 0.15 : 0.24;
        maxCmdTilt = py < 0.12 ? 0.17 : 0.28;
      }

      const rollDes = clamp(-Math.asin(clamp(bodyZDes, -0.95, 0.95)), -maxTilt, maxTilt);
      const pitchDes = clamp(Math.atan2(bodyXDes, dyDes), -maxTilt, maxTilt);

      const kLead = 0.85, kRate = 0.05;
      const rollCmd = clamp(rollDes + kLead * (rollDes - att.roll) - kRate * rates.roll, -maxCmdTilt, maxCmdTilt);
      const pitchCmd = clamp(pitchDes + kLead * (pitchDes - att.pitch) - kRate * rates.pitch, -maxCmdTilt, maxCmdTilt);

      const thrust = clamp((ayTotal / Math.max(0.65, dirY)) / kT, 0.05, 0.78);
      prevThrustCmd = thrust;

      if (py > 0.10) intYaw = clamp(intYaw + att.yaw * dt, -0.4, 0.4);
      const yawRate = clamp(-3.2 * att.yaw - 1.8 * intYaw - 0.25 * rates.yaw, -2, 2);

      return { thrust, roll: rollCmd, pitch: pitchCmd, yawRate };
    },
  };
}
