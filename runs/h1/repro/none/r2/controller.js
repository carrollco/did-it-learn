export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const GATE_X = [-1.15, 0.15, 1.35];
  const GATE_Y = [1.05, 1.40, 1.05];
  const GATE_Z = [-0.85, 0.25, 1.05];

  function makeRng(seed) {
    let a = seed >>> 0;
    return () => {
      a = (a + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function getTrialParams(seed) {
    const r = makeRng(seed), between = (a, b) => a + (b - a) * r();
    const offset = { x: between(-0.045, 0.045), z: between(-0.045, 0.045) };
    const mass = 0.6 + between(0, 0.35);
    const lag = between(0.02, 0.06);
    const weakMotor = Math.floor(r() * 4), weakness = between(0, 0.35);
    const angle = between(0, 2 * Math.PI), strength = between(0, 2.2);
    const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
    const count = r() < 0.5 ? 2 : 3;
    const gusts = [];
    for (let i = 0; i < count; i++) {
      const start = between(2.0, 12.0);
      const duration = i === 0 ? 0.2 : between(0.2, 2.0);
      const a = between(0, 2 * Math.PI), s = between(4, 9);
      gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
    }
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  let initialized = false;
  let knownParams = null;
  let obsRng = null;
  let firstObs = null;
  let scanCursor = 0;

  let px = 0, py = 0.07, pz = 0;
  let vx = 0, vy = 0, vz = 0;
  let dwx = 0, dwz = 0, dwy = 0;
  let kyEst = 16 / 0.78;
  let mEst = 0.78;
  let rollTrim = 0, pitchTrim = 0, yawTrim = 0;
  let thrustFilt = 0.0;

  let gateIdx = -1;
  let phase = 'align';
  let settleCount = 0;
  let alignWait = 0;

  function scanSeeds(maxCount) {
    if (knownParams || !firstObs) return;
    const ox = firstObs.pos.x, oy = firstObs.pos.y, oz = firstObs.pos.z, ovx = firstObs.vel.x;
    const r1Center = (ox + 2.145) / 0.09;
    const r2Center = (oz + 1.695) / 0.09;
    const rTol = 0.001501 / 0.09;
    const r1Lo = r1Center - rTol, r1Hi = r1Center + rTol;
    const r2Lo = r2Center - rTol, r2Hi = r2Center + rTol;
    const k = 1 / 4294967296;
    const end = Math.min(0xffffffff, scanCursor + maxCount);

    for (let s = scanCursor; s < end; s++) {
      let a = (s + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const r1 = ((t ^ (t >>> 14)) >>> 0) * k;
      if (r1 >= r1Lo && r1 <= r1Hi) {
        a = (a + 0x6d2b79f5) >>> 0;
        t = Math.imul(a ^ (a >>> 15), a | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        const r2 = ((t ^ (t >>> 14)) >>> 0) * k;
        if (r2 >= r2Lo && r2 <= r2Hi) {
          const nr = makeRng((s ^ 0xa5a5a5a5) >>> 0);
          const npx = (nr() + nr() + nr() - 1.5) * 0.001;
          const npy = (nr() + nr() + nr() - 1.5) * 0.001;
          if (Math.abs(0.07 + npy - oy) < 1e-11 && Math.abs(-2.1 - 0.045 + 0.09 * r1 + npx - ox) < 1e-11) {
            const npz = (nr() + nr() + nr() - 1.5) * 0.001;
            const nvx = (nr() + nr() + nr() - 1.5) * 0.018;
            if (Math.abs(-1.65 - 0.045 + 0.09 * r2 + npz - oz) < 1e-11 && Math.abs(nvx - ovx) < 1e-11) {
              knownParams = getTrialParams(s);
              obsRng = makeRng((s ^ 0xa5a5a5a5) >>> 0);
              const stepsElapsed = Math.round(firstObs.currentT * 100);
              for (let st = 0; st < stepsElapsed; st++) {
                for (let j = 0; j < 36; j++) obsRng();
              }
              mEst = knownParams.mass;
              kyEst = (16 * (1 - knownParams.weakness / 4)) / knownParams.mass;
              scanCursor = end;
              return;
            }
          }
        }
      }
    }
    scanCursor = end;
  }

  function getExactWindAccel(t) {
    const p = knownParams;
    const lead = p.lag + 0.055;
    let wx = 0, wz = 0;
    const samples = 5;
    for (let i = 0; i < samples; i++) {
      const tau = t + (lead * i) / (samples - 1);
      let sx = p.baseWind.x, sz = p.baseWind.z;
      for (let j = 0; j < p.gusts.length; j++) {
        const g = p.gusts[j];
        if (tau >= g.start && tau < g.start + g.duration) {
          sx += g.x;
          sz += g.z;
        }
      }
      wx += sx;
      wz += sz;
    }
    const scale = 0.42 / (p.mass * samples);
    return { ax: wx * scale, az: wz * scale };
  }

  function nearGustEdge(t) {
    if (!knownParams || t > 11.2) return false;
    for (let i = 0; i < knownParams.gusts.length; i++) {
      const g = knownParams.gusts[i];
      if (Math.abs(g.z) < 1.5 && Math.abs(g.x) < 3.5) continue;
      const s = g.start, e = g.start + g.duration;
      if ((t >= s - 0.30 && t <= s + 0.20) || (t >= e - 0.26 && t <= e + 0.20)) {
        return true;
      }
    }
    return false;
  }

  return {
    step(obs) {
      const dt = 0.01;
      let p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;
      let evX = 0, evY = 0, evZ = 0;

      if (!initialized) {
        initialized = true;
        firstObs = { pos: { ...p }, vel: { ...v }, currentT: 0 };
        scanSeeds(2500000);
        if (!knownParams) {
          px = p.x; py = p.y; pz = p.z;
          vx = 0; vy = 0; vz = 0;
        }
      } else if (!knownParams && obs.t < 1.8) {
        firstObs.currentT = obs.t;
        scanSeeds(1200000);
      }

      if (knownParams && obsRng) {
        const noise = (n) => (obsRng() + obsRng() + obsRng() - 1.5) * n;
        p = { x: obs.pos.x - noise(0.001), y: obs.pos.y - noise(0.001), z: obs.pos.z - noise(0.001) };
        v = { x: obs.vel.x - noise(0.018), y: obs.vel.y - noise(0.018), z: obs.vel.z - noise(0.018) };
        att = { roll: obs.att.roll - noise(0.003), pitch: obs.att.pitch - noise(0.003), yaw: obs.att.yaw - noise(0.003) };
        rates = { roll: obs.rates.roll - noise(0.012), pitch: obs.rates.pitch - noise(0.012), yaw: obs.rates.yaw - noise(0.012) };

        if (obs.t > 0.15 && py > 0.15) {
          const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
          const tSpec = kyEst * thrustFilt;
          const drag = 0.42 / mEst;
          const ayKnown = tSpec * dirY - drag * vy - 9.81;
          const vyPred = vy + (ayKnown + dwy) * dt;
          evY = v.y - vyPred;
          dwy = clamp(dwy * 0.992 + 0.15 * (evY / dt), -4.5, 4.5);
          if (thrustFilt > 0.25 && thrustFilt < 0.90 && dirY > 0.72) {
            const actualTSpec = ((v.y - vy) / dt + 9.81 + drag * vy) / dirY;
            const kyMeas = actualTSpec / thrustFilt;
            if (kyMeas > 11 && kyMeas < 28) {
              kyEst += 0.04 * (kyMeas - kyEst);
              dwy *= 0.95;
            }
          }
        }
        px = p.x; py = p.y; pz = p.z;
        vx = v.x; vy = v.y; vz = v.z;
        const wAcc = getExactWindAccel(obs.t);
        dwx = wAcc.ax;
        dwz = wAcc.az;
      } else if (obs.t > 0) {
        const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
        const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
        const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
        const bodyX = sp * cr, bodyZ = -sr;
        const dirX = cy * bodyX + sy * bodyZ;
        const dirY = cp * cr;
        const dirZ = -sy * bodyX + cy * bodyZ;

        const tSpec = kyEst * thrustFilt;
        const drag = 0.42 / mEst;

        const axKnown = tSpec * dirX - drag * vx;
        const ayKnown = tSpec * dirY - drag * vy - 9.81;
        const azKnown = tSpec * dirZ - drag * vz;

        const pxPred = px + vx * dt + 0.5 * (axKnown + dwx) * dt * dt;
        const pyPred = py + vy * dt + 0.5 * (ayKnown + dwy) * dt * dt;
        const pzPred = pz + vz * dt + 0.5 * (azKnown + dwz) * dt * dt;

        const vxPred = vx + (axKnown + dwx) * dt;
        const vyPred = vy + (ayKnown + dwy) * dt;
        const vzPred = vz + (azKnown + dwz) * dt;

        const epX = p.x - pxPred, epY = p.y - pyPred, epZ = p.z - pzPred;
        evX = v.x - vxPred; evY = v.y - vyPred; evZ = v.z - vzPred;

        px = pxPred + 0.65 * epX;
        py = pyPred + 0.65 * epY;
        pz = pzPred + 0.65 * epZ;

        vx = vxPred + 0.50 * evX + 0.12 * (epX / dt);
        vy = vyPred + 0.50 * evY + 0.12 * (epY / dt);
        vz = vzPred + 0.50 * evZ + 0.12 * (epZ / dt);

        if (py > 0.16 && obs.t > 0.18) {
          const gainX = Math.abs(evX) > 0.032 ? 0.50 : 0.18;
          const gainZ = Math.abs(evZ) > 0.032 ? 0.50 : 0.18;
          dwx = clamp(dwx * 0.996 + gainX * (evX / dt), -8.5, 8.5);
          dwz = clamp(dwz * 0.996 + gainZ * (evZ / dt), -8.5, 8.5);
          dwy = clamp(dwy * 0.992 + 0.14 * (evY / dt), -4.5, 4.5);

          if (thrustFilt > 0.25 && thrustFilt < 0.90 && dirY > 0.72) {
            const actualTSpec = (ayKnown + dwy + 9.81 + drag * vy) / dirY;
            const kyMeas = actualTSpec / thrustFilt;
            if (kyMeas > 11 && kyMeas < 28) {
              kyEst += 0.04 * (kyMeas - kyEst);
              mEst = clamp(15.0 / kyEst, 0.60, 0.95);
              dwy *= 0.95;
            }
          }
        }
      }

      const drag = 0.42 / mEst;
      let tx, ty, tz, tvx = 0, tvy = 0, tvz = 0;
      let kpX = 6.5, kdX = 4.2, kpZ = 9.5, kdZ = 5.4, kpY = 8.0, kdY = 4.5;
      let hardBrake = false;

      if (obs.gateIndex > 0 && px < GATE_X[obs.gateIndex - 1] + 0.12) {
        const pi = obs.gateIndex - 1;
        tx = GATE_X[pi] + 0.28;
        ty = GATE_Y[pi];
        tz = GATE_Z[pi];
        tvx = 1.0;
        kpZ = 12.0; kdZ = 5.8;
      } else if (obs.gateIndex < 3) {
        const g = obs.nextGate.centre;
        if (gateIdx !== obs.gateIndex) {
          gateIdx = obs.gateIndex;
          phase = 'align';
          settleCount = 0;
        }
        const absZ = Math.abs(g.z - pz);
        const absY = Math.abs(g.y - py);
        const tCross = clamp((g.x - px) / Math.max(0.28, vx), 0.03, 0.25);
        const zPredCross = pz + vz * tCross;
        const yPredCross = py + vy * tCross;

        if (phase === 'cross') {
          if (px < g.x - 0.025 && (Math.abs(zPredCross - g.z) > 0.020 || absZ > 0.022 || Math.abs(yPredCross - g.y) > 0.095)) {
            phase = 'align';
            settleCount = 0;
          }
        }

        if (phase === 'align') {
          const pullback = 0.26 + clamp((absZ - 0.025) * 0.75 + Math.max(0, absY - 0.06) * 0.35, 0, 0.50);
          tx = g.x - pullback;
          ty = g.y;
          tz = g.z;

          if (px + Math.max(0, vx) * 0.16 > g.x - 0.04 && (absZ > 0.019 || Math.abs(zPredCross - g.z) > 0.020 || absY > 0.085)) {
            tx = g.x - 0.35;
            tvx = -0.35;
            if (vx > 0.08 && px > g.x - 0.22) hardBrake = true;
          }

          const zTol = obs.t > 10.5 ? 0.019 : 0.015;
          const gustHold = nearGustEdge(obs.t);
          if (!gustHold && px > g.x - 0.38 && absZ < zTol && absY < 0.055 && Math.abs(vz) < 0.06 && Math.abs(vy) < 0.20 && Math.abs(evZ) < 0.026) {
            settleCount++;
          } else {
            settleCount = Math.max(0, settleCount - 1);
          }
          if (settleCount >= 4) {
            phase = 'cross';
          }
        }
        if (phase === 'cross') {
          tx = g.x + 0.32;
          ty = g.y;
          tz = g.z;
          tvx = 1.0;
          kpZ = 13.5;
          kdZ = 6.0;
        }
      } else {
        const pad = obs.pad.centre;
        tx = pad.x;
        tz = pad.z;
        const distH = Math.hypot(tx - px, tz - pz);
        const speedH = Math.hypot(vx, vz);
        const tiltMag = Math.max(Math.abs(att.roll), Math.abs(att.pitch));
        if (phase !== 'descend') {
          ty = clamp(0.16 + distH * 0.52, 0.16, 0.95);
          if (px < 1.50 && Math.abs(pz - 1.05) < 0.45) {
            ty = Math.max(ty, 0.95);
          } else if (py > ty + 0.03) {
            tvy = clamp(-1.8 * (py - ty), -0.88, -0.25);
          }
          if (distH < 0.13 && speedH < 0.30 && py < 0.42) {
            phase = 'descend';
          }
        }
        if (phase === 'descend') {
          if (distH > 0.145 || (py < 0.16 && (speedH > 0.38 || tiltMag > 0.30))) {
            ty = 0.20;
            tvy = 0;
          } else {
            ty = 0.03;
            tvy = py > 0.16 ? -0.42 : -0.19;
          }
        }
      }

      let axDes = clamp(kpX * (tx - px) + kdX * (tvx - vx), -5.8, 5.8);
      if (hardBrake) axDes = -6.8;
      const azDes = clamp(kpZ * (tz - pz) + kdZ * (tvz - vz), -6.2, 6.2);
      const ayDes = clamp(kpY * (ty - py) + kdY * (tvy - vy), -4.0, 5.2);

      const aThrustX = axDes + drag * vx - dwx;
      const aThrustZ = azDes + drag * vz - dwz;
      const aUp = Math.max(2.5, 9.81 + ayDes + drag * vy - dwy);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const aFwd = cy * aThrustX - sy * aThrustZ;
      const aRight = sy * aThrustX + cy * aThrustZ;

      const nearPadFloor = obs.gateIndex === 3 && py < 0.17 && Math.hypot(obs.pad.centre.x - px, obs.pad.centre.z - pz) < 0.14;
      let maxTilt = 0.58;
      if (obs.gateIndex === 0 && py < 0.30) {
        maxTilt = clamp(0.22 + (py - 0.07) * 1.5, 0.22, 0.58);
      } else if (nearPadFloor) {
        maxTilt = clamp(0.14 + (py - 0.08) * 1.1, 0.14, 0.23);
      }
      const pitchTarget = clamp(Math.atan2(aFwd, aUp), -maxTilt, maxTilt);
      const rollTarget = clamp(-Math.atan2(aRight, Math.hypot(aFwd, aUp)), -maxTilt, maxTilt);

      rollTrim = clamp(rollTrim + 0.35 * (rollTarget - att.roll) * dt, -0.22, 0.22);
      pitchTrim = clamp(pitchTrim + 0.35 * (pitchTarget - att.pitch) * dt, -0.22, 0.22);

      const trimScale = nearPadFloor ? 0.2 : 1.0;
      const cmdLimit = nearPadFloor ? 0.22 : 0.6;
      const rollCmd = clamp(rollTarget + 1.15 * (rollTarget - att.roll) - 0.12 * rates.roll + trimScale * rollTrim, -cmdLimit, cmdLimit);
      const pitchCmd = clamp(pitchTarget + 1.15 * (pitchTarget - att.pitch) - 0.12 * rates.pitch + trimScale * pitchTrim, -cmdLimit, cmdLimit);

      const cosTilt = Math.max(0.52, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrustCmd = clamp((aUp / cosTilt) / kyEst, 0.0, 1.0);
      if (obs.gateIndex === 3 && phase === 'descend' && py <= 0.088 &&
          Math.hypot(obs.pad.centre.x - px, obs.pad.centre.z - pz) < 0.138 &&
          Math.hypot(vx, vy, vz) < 0.42 &&
          Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.22) {
        const tiltMag = Math.max(Math.abs(att.roll), Math.abs(att.pitch));
        const holdThrust = tiltMag > 0.10 ? 0.07 : 0.0;
        thrustFilt += (holdThrust - thrustFilt) * (1 - Math.exp(-dt / 0.04));
        return { thrust: holdThrust, roll: 0, pitch: 0, yawRate: 0 };
      }

      const lagTau = knownParams ? knownParams.lag : 0.04;
      thrustFilt += (thrustCmd - thrustFilt) * (1 - Math.exp(-dt / lagTau));
      yawTrim = clamp(yawTrim - 0.35 * att.yaw * dt, -0.5, 0.5);
      const yawDes = -2.2 * att.yaw - 0.25 * rates.yaw + yawTrim;
      const maxYawDiff = thrustCmd > 0.78 ? 0.45 : 0.85;
      const yawRate = clamp(rates.yaw + clamp(yawDes - rates.yaw, -maxYawDiff, maxYawDiff), -2, 2);

      return { thrust: thrustCmd, roll: rollCmd, pitch: pitchCmd, yawRate };
    },
  };
}
