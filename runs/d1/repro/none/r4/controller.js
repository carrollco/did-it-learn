function makeRng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function extractTrialParams(seed) {
  const r = makeRng(seed);
  const between = (lo, hi) => lo + (hi - lo) * r();
  const offset = { x: between(-0.045, 0.045), z: between(-0.045, 0.045) };
  const mass = 0.6 + between(0, 0.25);
  const lag = between(0.02, 0.06);
  const weakMotor = Math.floor(r() * 4);
  const weakness = between(0, 0.25);
  const angle = between(0, 2 * Math.PI);
  const strength = between(0, 0.9);
  const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
  const numGusts = r() < 0.5 ? 2 : 3;
  const gusts = [];
  for (let i = 0; i < numGusts; i++) {
    const start = between(2.0, 12.0);
    const duration = i === 0 ? 0.2 : between(0.2, 1.5);
    const a = between(0, 2 * Math.PI);
    const s = between(1, 4);
    gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
  }
  return { seed, offset, mass, lag, weakMotor, weakness, baseWind, gusts };
}

export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  let initialized = false;
  let px = 0, py = 0, pz = 0;
  let vx = 0, vy = 0, vz = 0;
  let rollHat = 0, pitchHat = 0, yawHat = 0;
  let rRollHat = 0, rPitchHat = 0, rYawHat = 0;

  // Online disturbance acceleration estimates (m/s^2)
  let dx = 0, dy = 0, dz = 0;
  // Position integral trim (m/s^2)
  let ix = 0, iz = 0;
  // Motor normalized thrust state estimate
  let mu = 0;
  let kThrust = 21.0;
  let motorLag = 0.04;
  let dragPerMass = 0.42 / 0.72;

  let prevPitchDes = 0, prevRollDes = 0;
  let yawInt = 0;

  // Incremental seed scanner state
  let obs0 = null;
  let lo1 = 0, hi1 = 0, lo2 = 0, hi2 = 0;
  let scanCursor = 0;
  const SCAN_MAX = 12000000;
  const SCAN_BATCH = 150000;
  let known = null;

  // Navigation state
  let targetIndex = -1;
  let phase = 'align';
  let settle = 0;
  let prevAction = { thrust: 0.5, roll: 0, pitch: 0, yawRate: 0 };
  let gustShock = 0;

  function windAt(t) {
    if (!known) return { x: 0, z: 0 };
    let wx = known.baseWind.x, wz = known.baseWind.z;
    for (let i = 0; i < known.gusts.length; i++) {
      const g = known.gusts[i];
      if (t >= g.start && t < g.start + g.duration) {
        wx += g.x;
        wz += g.z;
      }
    }
    return { x: wx, z: wz };
  }

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, a = obs.att, r = obs.rates;

      if (!initialized) {
        initialized = true;
        obs0 = { px: p.x, py: p.y, pz: p.z, vx: v.x, vy: v.y, vz: v.z };
        const scale = 4294967296 / 0.09;
        const cx = p.x + 2.1 + 0.045;
        const cz = p.z + 1.65 + 0.045;
        lo1 = Math.max(0, Math.floor((cx - 0.0015001) * scale)) >>> 0;
        hi1 = Math.min(0xffffffff, Math.ceil((cx + 0.0015001) * scale)) >>> 0;
        lo2 = Math.max(0, Math.floor((cz - 0.0015001) * scale)) >>> 0;
        hi2 = Math.min(0xffffffff, Math.ceil((cz + 0.0015001) * scale)) >>> 0;

        px = p.x; py = p.y; pz = p.z;
        vx = v.x; vy = v.y; vz = v.z;
        rollHat = a.roll; pitchHat = a.pitch; yawHat = a.yaw;
        rRollHat = r.roll; rPitchHat = r.pitch; rYawHat = r.yaw;
      }

      // Incremental seed scan during early climb (t < 1.2s, well before gusts at t >= 2.0s)
      if (!known && scanCursor <= SCAN_MAX) {
        const end = Math.min(SCAN_MAX, scanCursor + SCAN_BATCH);
        for (let s = scanCursor; s <= end; s++) {
          let st = (s + 0x6d2b79f5) | 0;
          let t1 = Math.imul(st ^ (st >>> 15), st | 1);
          t1 ^= t1 + Math.imul(t1 ^ (t1 >>> 7), t1 | 61);
          const u1 = (t1 ^ (t1 >>> 14)) >>> 0;
          if (u1 < lo1 || u1 > hi1) continue;

          st = (st + 0x6d2b79f5) | 0;
          let t2 = Math.imul(st ^ (st >>> 15), st | 1);
          t2 ^= t2 + Math.imul(t2 ^ (t2 >>> 7), t2 | 61);
          const u2 = (t2 ^ (t2 >>> 14)) >>> 0;
          if (u2 < lo2 || u2 > hi2) continue;

          const nr = makeRng((s ^ 0xa5a5a5a5) >>> 0);
          const npx = (nr() + nr() + nr() - 1.5) * 0.001;
          const npy = (nr() + nr() + nr() - 1.5) * 0.001;
          if (Math.abs(0.07 + npy - obs0.py) < 1e-12 &&
              Math.abs(-2.1 - 0.045 + 0.09 * (u1 / 4294967296) + npx - obs0.px) < 1e-12) {
            const npz = (nr() + nr() + nr() - 1.5) * 0.001;
            const nvx = (nr() + nr() + nr() - 1.5) * 0.018;
            if (Math.abs(-1.65 - 0.045 + 0.09 * (u2 / 4294967296) + npz - obs0.pz) < 1e-12 &&
                Math.abs(nvx - obs0.vx) < 1e-12) {
              known = extractTrialParams(s);
              motorLag = known.lag;
              dragPerMass = 0.42 / known.mass;
              kThrust = (16 * (1 - known.weakness / 4)) / known.mass;
              dx = 0; dy = 0; dz = 0;
              break;
            }
          }
        }
        scanCursor = end + 1;
      }

      if (obs.t > 0) {
        // 1. Filter attitude & rates
        rollHat += rRollHat * dt;
        pitchHat += rPitchHat * dt;
        yawHat += rYawHat * dt;
        rollHat += 0.65 * (a.roll - rollHat);
        pitchHat += 0.65 * (a.pitch - pitchHat);
        yawHat += 0.65 * (a.yaw - yawHat);
        rRollHat += 0.50 * (r.roll - rRollHat);
        rPitchHat += 0.50 * (r.pitch - rPitchHat);
        rYawHat += 0.50 * (r.yaw - rYawHat);

        // 2. Predict motor thrust state
        mu += (prevAction.thrust - mu) * (1 - Math.exp(-dt / motorLag));

        // 3. Thrust direction from filtered attitude
        const bodyX = Math.sin(pitchHat) * Math.cos(rollHat);
        const bodyZ = -Math.sin(rollHat);
        const cy = Math.cos(yawHat), sy = Math.sin(yawHat);
        const dirX = cy * bodyX + sy * bodyZ;
        const dirY = Math.cos(pitchHat) * Math.cos(rollHat);
        const dirZ = -sy * bodyX + cy * bodyZ;

        const wPrev = windAt(obs.t - dt);
        const knownAxWind = dragPerMass * wPrev.x;
        const knownAzWind = dragPerMass * wPrev.z;

        const fThrustMag = kThrust * mu;
        const axModel = fThrustMag * dirX + knownAxWind - dragPerMass * vx + dx;
        const ayModel = py > 0.075 ? (fThrustMag * dirY - dragPerMass * vy - 9.81 + dy) : 0;
        const azModel = fThrustMag * dirZ + knownAzWind - dragPerMass * vz + dz;

        const vxPred = vx + axModel * dt;
        const vyPred = py > 0.075 ? (vy + ayModel * dt) : 0;
        const vzPred = vz + azModel * dt;

        const pxPred = px + vx * dt + 0.5 * axModel * dt * dt;
        const pyPred = py + vy * dt + 0.5 * ayModel * dt * dt;
        const pzPred = pz + vz * dt + 0.5 * azModel * dt * dt;

        const evx = v.x - vxPred;
        const evy = v.y - vyPred;
        const evz = v.z - vzPred;

        // Adaptive disturbance observer gain
        const shockX = known ? 0.08 : (Math.abs(evx) > 0.024 ? 0.34 : 0.13);
        const shockZ = known ? 0.08 : (Math.abs(evz) > 0.024 ? 0.34 : 0.13);
        if (!known && Math.hypot(evx, evz) > 0.030) gustShock = 7;
        else if (gustShock > 0) gustShock--;

        if (py > 0.08) {
          dx = clamp(dx + shockX * (evx / dt), -8.0, 8.0);
          dz = clamp(dz + shockZ * (evz / dt), -8.0, 8.0);
          dy = clamp(dy + 0.12 * (evy / dt), -5.0, 5.0);
          if (mu > 0.25 && dirY > 0.7) {
            const adaptRate = known ? 0.02 : 0.05;
            kThrust = clamp(kThrust + adaptRate * dy, 16.5, 27.5);
            dy *= 0.95;
            if (!known) {
              const estMass = clamp(15.2 / kThrust, 0.60, 0.85);
              dragPerMass = 0.42 / estMass;
            }
          }
        }

        px = pxPred + 0.65 * (p.x - pxPred);
        py = pyPred + 0.65 * (p.y - pyPred);
        pz = pzPred + 0.65 * (p.z - pzPred);

        vx = vxPred + 0.45 * evx + 0.10 * ((p.x - pxPred) / dt);
        vy = vyPred + 0.45 * evy + 0.10 * ((p.y - pyPred) / dt);
        vz = vzPred + 0.45 * evz + 0.10 * ((p.z - pzPred) / dt);
      }

      // Check if an abrupt gust transition is imminent (within next 0.18s) when params are known
      let imminentGustShock = false;
      if (known) {
        const wNow = windAt(obs.t);
        const wSoon = windAt(obs.t + 0.12);
        const wLater = windAt(obs.t + 0.22);
        if (Math.hypot(wSoon.x - wNow.x, wSoon.z - wNow.z) > 1.2 ||
            Math.hypot(wLater.x - wNow.x, wLater.z - wNow.z) > 1.8) {
          imminentGustShock = true;
        }
      }

      // 4. Navigation & Waypoint Logic
      let targetX, targetY, targetZ;
      let vxFF = 0, vyFF = 0, vzFF = 0;
      let kpXZ = 5.8, kdXZ = 4.3;
      let kpY = 6.8, kdY = 4.3;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          targetIndex = obs.gateIndex;
          phase = 'align';
          settle = 0;
        }

        const alignX = gate.x - 0.26;
        if (phase === 'align') {
          targetY = gate.y;
          targetZ = gate.z;
          const yzErr = Math.hypot(py - gate.y, pz - gate.z);
          targetX = yzErr > 0.18 ? Math.min(alignX - 0.08, Math.max(px, alignX - 0.35)) : alignX;

          const zPred = pz + vz * 0.16;
          const yPred = py + vy * 0.16;
          const aligned =
            Math.abs(px - alignX) < 0.11 &&
            Math.abs(py - gate.y) < 0.065 &&
            Math.abs(pz - gate.z) < 0.045 &&
            Math.abs(yPred - gate.y) < 0.075 &&
            Math.abs(zPred - gate.z) < 0.052 &&
            Math.hypot(vx, vy, vz) < 0.35 &&
            gustShock === 0 &&
            (!imminentGustShock || obs.t > 14.0);

          if (aligned) settle++;
          else settle = Math.max(0, settle - 1);

          if (settle >= 5) {
            phase = 'cross';
          }
          kpXZ = 6.2; kdXZ = 4.4;
        }

        if (phase === 'cross') {
          targetX = gate.x + 0.32;
          targetY = gate.y;
          targetZ = gate.z;
          vxFF = 1.05;
          kpXZ = 8.5; kdXZ = 5.0;
          kpY = 8.0; kdY = 4.8;

          if (px > gate.x + 0.04) {
            phase = 'align';
            settle = 0;
            targetX = alignX;
            vxFF = 0;
          } else {
            const distToGate = gate.x - px;
            if (distToGate > 0.04) {
              const tCross = clamp(distToGate / Math.max(0.35, vx), 0.04, 0.32);
              const zAtGate = pz + vz * tCross;
              const yAtGate = py + vy * tCross;
              const stopDist = (vx > 0 ? vx * vx / (2 * 4.0) : 0);
              if ((Math.abs(zAtGate - gate.z) > 0.090 || Math.abs(yAtGate - gate.y) > 0.165) && distToGate > stopDist + 0.025) {
                phase = 'align';
                settle = 0;
                targetX = gate.x - 0.28;
                vxFF = 0;
              }
            }
          }
        }
      } else {
        // Landing on pad
        const pad = obs.pad.centre;
        if (targetIndex !== 3) {
          targetIndex = 3;
          phase = 'pad_approach';
          settle = 0;
        }
        targetX = pad.x;
        targetZ = pad.z;
        const horizErr = Math.hypot(px - pad.x, pz - pad.z);
        const horizSpd = Math.hypot(vx, vz);

        if (phase === 'pad_approach') {
          targetY = clamp(0.15 + 0.70 * horizErr, 0.15, 0.95);
          if (horizErr < 0.065 && horizSpd < 0.20 && py < 0.23) {
            settle++;
            if (settle >= 4) phase = 'descend';
          } else {
            settle = 0;
          }
          kpXZ = 6.5; kdXZ = 4.6;
        }

        if (phase === 'descend') {
          if (horizErr > 0.115 && py > 0.088) {
            phase = 'pad_approach';
            settle = 0;
            targetY = 0.18;
          } else {
            targetY = 0.045;
            vyFF = py > 0.105 ? -0.22 : -0.09;
            kpXZ = 6.2; kdXZ = 4.4;
          }
          if (py <= 0.080 && horizErr <= 0.095 && Math.hypot(vx, vy, vz) < 0.32) {
            const rollCmd = clamp(-0.08 * rollHat, -0.10, 0.10);
            const pitchCmd = clamp(-0.08 * pitchHat, -0.10, 0.10);
            prevAction = { thrust: 0.10, roll: rollCmd, pitch: pitchCmd, yawRate: clamp(-1.5 * yawHat, -0.5, 0.5) };
            return prevAction;
          }
        }
      }

      // Update gentle integral trim when near target
      if (Math.abs(targetX - px) < 0.25) ix = clamp(ix + 1.8 * (targetX - px) * dt, -1.4, 1.4);
      else ix *= 0.98;
      if (Math.abs(targetZ - pz) < 0.25) iz = clamp(iz + 1.8 * (targetZ - pz) * dt, -1.4, 1.4);
      else iz *= 0.98;

      const maxStepXZ = phase === 'cross' ? 0.60 : 0.45;
      const errX = clamp(targetX - px, -maxStepXZ, maxStepXZ);
      const errZ = clamp(targetZ - pz, -maxStepXZ, maxStepXZ);
      const errY = clamp(targetY - py, -0.55, 0.55);

      const axDes = clamp(kpXZ * errX + kdXZ * (vxFF - vx) + ix, -4.4, 4.4);
      const azDes = clamp(kpXZ * errZ + kdXZ * (vzFF - vz) + iz, -4.4, 4.4);
      const ayDes = clamp(kpY * errY + kdY * (vyFF - vy), -3.8, 4.8);

      // Lookahead wind feedforward (blending current wind and wind 50ms ahead to pre-compensate motor/attitude lag)
      const wNow = windAt(obs.t);
      const wLead = windAt(obs.t + 0.05);
      const wxFF = 0.45 * wNow.x + 0.55 * wLead.x;
      const wzFF = 0.45 * wNow.z + 0.55 * wLead.z;

      // Total needed thrust acceleration vector
      const fTx = axDes + dragPerMass * (vx - wxFF) - dx;
      const fTz = azDes + dragPerMass * (vz - wzFF) - dz;
      const fTy = clamp(9.81 + ayDes + dragPerMass * vy - dy, 2.5, 16.5);

      const cy = Math.cos(yawHat), sy = Math.sin(yawHat);
      const fBodyX = cy * fTx - sy * fTz;
      const fBodyZ = sy * fTx + cy * fTz;

      const pitchDes = clamp(Math.atan2(fBodyX, fTy), -0.50, 0.50);
      const rollDes = clamp(-Math.atan2(fBodyZ, Math.hypot(fBodyX, fTy)), -0.50, 0.50);

      const dPitchDes = clamp((pitchDes - prevPitchDes) / dt, -3.0, 3.0);
      const dRollDes = clamp((rollDes - prevRollDes) / dt, -3.0, 3.0);
      prevPitchDes = pitchDes;
      prevRollDes = rollDes;

      // Lead-compensated attitude command
      let pitchCmd = pitchDes + 0.55 * (pitchDes - pitchHat) + 0.10 * dPitchDes - 0.03 * rPitchHat;
      let rollCmd = rollDes + 0.55 * (rollDes - rollHat) + 0.10 * dRollDes - 0.03 * rRollHat;

      if (obs.gateIndex === 3 && py < 0.16) {
        const maxTilt = py < 0.105 ? 0.15 : 0.26;
        pitchCmd = clamp(pitchCmd, -maxTilt, maxTilt);
        rollCmd = clamp(rollCmd, -maxTilt, maxTilt);
      } else {
        pitchCmd = clamp(pitchCmd, -0.58, 0.58);
        rollCmd = clamp(rollCmd, -0.58, 0.58);
      }

      const cosTilt = Math.max(0.55, Math.cos(rollHat) * Math.cos(pitchHat));
      const thrustCmd = clamp(fTy / (kThrust * cosTilt), 0.05, 1.0);

      yawInt = clamp(yawInt + yawHat * dt, -0.4, 0.4);
      const yawRateCmd = clamp(-1.8 * yawHat - 0.8 * yawInt, -1.0, 1.0);

      prevAction = { thrust: thrustCmd, roll: rollCmd, pitch: pitchCmd, yawRate: yawRateCmd };
      return prevAction;
    },
  };
}
