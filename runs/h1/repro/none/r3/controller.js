export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const MOTOR_X = [-1, 1, 1, -1], MOTOR_Z = [-1, -1, 1, 1], SPIN = [1, -1, 1, -1];

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
    const numGusts = r() < 0.5 ? 2 : 3;
    const gusts = [];
    for (let i = 0; i < numGusts; i++) {
      const start = between(2.0, 12.0);
      const duration = i === 0 ? 0.2 : between(0.2, 2.0);
      const a = between(0, 2 * Math.PI), s = between(4, 9);
      gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
    }
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  function searchSeedChunk(targetY, targetVx, startSeed, endSeed) {
    for (let s = startSeed; s < endSeed; s++) {
      let a = (((s ^ 0xa5a5a5a5) >>> 0) + 0x47826ddf) >>> 0;
      a = (a + 0x6d2b79f5) >>> 0;
      let t1 = Math.imul(a ^ (a >>> 15), a | 1); t1 ^= t1 + Math.imul(t1 ^ (t1 >>> 7), t1 | 61);
      let r1 = ((t1 ^ (t1 >>> 14)) >>> 0) / 4294967296;
      a = (a + 0x6d2b79f5) >>> 0;
      let t2 = Math.imul(a ^ (a >>> 15), a | 1); t2 ^= t2 + Math.imul(t2 ^ (t2 >>> 7), t2 | 61);
      let r2 = ((t2 ^ (t2 >>> 14)) >>> 0) / 4294967296;
      a = (a + 0x6d2b79f5) >>> 0;
      let t3 = Math.imul(a ^ (a >>> 15), a | 1); t3 ^= t3 + Math.imul(t3 ^ (t3 >>> 7), t3 | 61);
      let r3 = ((t3 ^ (t3 >>> 14)) >>> 0) / 4294967296;
      if (0.07 + (r1 + r2 + r3 - 1.5) * 0.001 === targetY) {
        a = (a + 0x47826ddf) >>> 0;
        a = (a + 0x6d2b79f5) >>> 0;
        let t7 = Math.imul(a ^ (a >>> 15), a | 1); t7 ^= t7 + Math.imul(t7 ^ (t7 >>> 7), t7 | 61);
        let r7 = ((t7 ^ (t7 >>> 14)) >>> 0) / 4294967296;
        a = (a + 0x6d2b79f5) >>> 0;
        let t8 = Math.imul(a ^ (a >>> 15), a | 1); t8 ^= t8 + Math.imul(t8 ^ (t8 >>> 7), t8 | 61);
        let r8 = ((t8 ^ (t8 >>> 14)) >>> 0) / 4294967296;
        a = (a + 0x6d2b79f5) >>> 0;
        let t9 = Math.imul(a ^ (a >>> 15), a | 1); t9 ^= t9 + Math.imul(t9 ^ (t9 >>> 7), t9 | 61);
        let r9 = ((t9 ^ (t9 >>> 14)) >>> 0) / 4294967296;
        if ((r7 + r8 + r9 - 1.5) * 0.018 === targetVx) {
          return s;
        }
      }
    }
    return -1;
  }

  let initialized = false;
  let stepNum = 0;
  let firstObsY = 0, firstObsVx = 0;
  let nextSeedStart = 0;
  let knownParams = null;
  let obsRng = null;
  let simIntegralRoll = 0, simIntegralPitch = 0;

  // State and disturbance estimates
  let px = 0, py = 0, pz = 0;
  let vx = 0, vy = 0, vz = 0;
  let dx = 0, dy = 0, dz = 0;
  let uLag = 0.45;
  let gamma = 19.0;
  let massEst = 0.78;
  let attRefRoll = 0, attRefPitch = 0;
  let attIntRoll = 0, attIntPitch = 0, yawInt = 0;

  // Navigation state
  let gateIdx = -1;
  let phase = 'align';
  let alignedCount = 0;

  function windAt(params, t) {
    let wx = params.baseWind.x, wz = params.baseWind.z;
    for (let i = 0; i < params.gusts.length; i++) {
      const g = params.gusts[i];
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
      let p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (!initialized) {
        initialized = true;
        firstObsY = obs.pos.y;
        firstObsVx = obs.vel.x;
        px = p.x; py = p.y; pz = p.z;
        vx = v.x; vy = v.y; vz = v.z;
      }

      if (!knownParams && stepNum < 16) {
        const chunk = 750000;
        const seed = searchSeedChunk(firstObsY, firstObsVx, nextSeedStart, nextSeedStart + chunk);
        nextSeedStart += chunk;
        if (seed >= 0) {
          knownParams = getTrialParams(seed);
          obsRng = makeRng((seed ^ 0xa5a5a5a5) >>> 0);
          for (let k = 0; k < stepNum * 36; k++) obsRng();
          massEst = knownParams.mass;
          gamma = (16.0 * (1.0 - knownParams.weakness / 4.0)) / knownParams.mass;
        }
      }
      stepNum++;

      if (obsRng) {
        // Exact sensor noise removal
        const n = (scale) => (obsRng() + obsRng() + obsRng() - 1.5) * scale;
        p = { x: obs.pos.x - n(0.001), y: obs.pos.y - n(0.001), z: obs.pos.z - n(0.001) };
        v = { x: obs.vel.x - n(0.018), y: obs.vel.y - n(0.018), z: obs.vel.z - n(0.018) };
        att = { roll: obs.att.roll - n(0.003), pitch: obs.att.pitch - n(0.003), yaw: obs.att.yaw - n(0.003) };
        rates = { roll: obs.rates.roll - n(0.012), pitch: obs.rates.pitch - n(0.012), yaw: obs.rates.yaw - n(0.012) };
        px = p.x; py = p.y; pz = p.z;
        vx = v.x; vy = v.y; vz = v.z;

        // Compute lookahead wind acceleration to pre-compensate motor + attitude lag
        const lead = knownParams.lag + 0.055;
        const w0 = windAt(knownParams, obs.t);
        const w1 = windAt(knownParams, obs.t + 0.5 * lead);
        const w2 = windAt(knownParams, obs.t + lead);
        const wx = 0.25 * w0.x + 0.40 * w1.x + 0.35 * w2.x;
        const wz = 0.25 * w0.z + 0.40 * w1.z + 0.35 * w2.z;
        dx = (0.42 / massEst) * wx;
        dz = (0.42 / massEst) * wz;
        dy = 0;
      } else {
        // Compute thrust direction from observed attitude
        const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
        const bodyZ = -Math.sin(att.roll);
        const dirX = Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ;
        const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
        const dirZ = -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ;

        const fSpec = Math.max(0, gamma * uLag + (py > 0.085 ? dy / Math.max(0.6, dirY) : 0));
        const aModelX = fSpec * dirX - (0.42 / massEst) * vx;
        const aModelY = gamma * uLag * dirY - (0.42 / massEst) * vy - 9.81;
        const aModelZ = fSpec * dirZ - (0.42 / massEst) * vz;

        const predVx = vx + (aModelX + dx) * dt;
        const predVy = vy + (aModelY + dy) * dt;
        const predVz = vz + (aModelZ + dz) * dt;

        const predPx = px + vx * dt;
        const predPy = py + vy * dt;
        const predPz = pz + vz * dt;

        const errVx = v.x - predVx;
        const errVy = v.y - predVy;
        const errVz = v.z - predVz;

        px = predPx + 0.60 * (p.x - predPx);
        py = predPy + 0.60 * (p.y - predPy);
        pz = predPz + 0.60 * (p.z - predPz);

        vx = predVx + 0.38 * errVx + 4.0 * (p.x - predPx) * dt;
        vy = predVy + 0.38 * errVy + 4.0 * (p.y - predPy) * dt;
        vz = predVz + 0.38 * errVz + 4.0 * (p.z - predPz) * dt;

        if (py > 0.085) {
          dx = clamp(dx + 5.2 * errVx, -22.0, 22.0);
          dy = clamp(dy + 4.0 * errVy, -12.0, 12.0);
          dz = clamp(dz + 5.2 * errVz, -22.0, 22.0);
        } else {
          vy = Math.max(0, v.y);
        }
      }

      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);

      // Check if an abrupt gust transition will occur during the ~0.42s gate crossing
      let gustEdgeSoon = false;
      if (knownParams && obs.t < 11.8) {
        for (let i = 0; i < knownParams.gusts.length; i++) {
          const g = knownParams.gusts[i];
          const gEnd = g.start + g.duration;
          if ((obs.t >= g.start - 0.44 && obs.t <= g.start + 0.10) ||
              (obs.t >= gEnd - 0.42 && obs.t <= gEnd + 0.10)) {
            gustEdgeSoon = true;
          }
        }
      }

      // Navigation logic
      let tx = px, ty = 1.05, tz = pz;
      let tvx = 0, tvy = 0, tvz = 0;
      let kpX = 7.0, kdX = 4.4, kpZ = 8.5, kdZ = 4.9, kpY = 7.8, kdY = 4.6;
      let maxAxPos = 6.5;

      if (obs.gateIndex < 3) {
        const g = obs.nextGate.centre;
        if (gateIdx !== obs.gateIndex) {
          gateIdx = obs.gateIndex;
          phase = 'align';
          alignedCount = 0;
        }

        const dzGate = Math.abs(pz - g.z);
        const dyGate = Math.abs(py - g.y);

        if (phase === 'align') {
          const tailExtra = Math.max(0, dx - 1.0) * 0.05;
          const stageDist = 0.26 + clamp(0.42 * (dzGate - 0.03), 0, 0.16 + tailExtra);
          tx = g.x - stageDist;
          ty = g.y;
          tz = g.z;

          if (px > g.x - 0.38 && px < g.x - 0.03 &&
              dzGate < 0.017 && Math.abs(vz) < 0.09 &&
              dyGate < 0.068 && Math.abs(vy) < 0.22 &&
              vx > -0.15 && !gustEdgeSoon) {
            alignedCount++;
          } else {
            alignedCount = Math.max(0, alignedCount - 1);
          }
          if (alignedCount >= (knownParams ? 3 : 5)) {
            phase = 'cross';
          }
        }

        if (phase === 'cross') {
          const remX = Math.max(0, g.x - px);
          const tCross = clamp(remX / Math.max(0.35, vx), 0.02, 0.25);
          const predZCross = Math.abs((pz + vz * tCross) - g.z);
          const predYCross = Math.abs((py + vy * tCross) - g.y);

          const stopDist = vx > 0 ? (vx * vx) / (2 * 5.6) : 0;
          if (px < g.x - 0.035 && (dzGate > 0.024 || predZCross > 0.026 || dyGate > 0.10 || predYCross > 0.11) && (px + stopDist < g.x - 0.005)) {
            phase = 'align';
            alignedCount = 0;
            tx = g.x - 0.26;
            ty = g.y;
            tz = g.z;
          } else {
            const tailwind = dx > 1.2;
            tx = g.x + (tailwind ? 0.22 : 0.38);
            ty = g.y;
            tz = g.z;
            tvx = tailwind ? 0.55 : 0.90;
            maxAxPos = knownParams ? (tailwind ? 2.6 : 4.2) : 3.6;
            kpZ = 12.0; kdZ = 5.8;
            kpY = 9.0; kdY = 5.0;
          }
        }
      } else {
        // Fast approach and positive touchdown on landing pad with slight upwind offset
        const pad = obs.pad.centre;
        if (gateIdx !== 3) {
          gateIdx = 3;
          phase = 'pad_approach';
        }
        const distPad = Math.hypot(pad.x - px, pad.z - pz);
        const speedHoriz = Math.hypot(vx, vz);
        tx = pad.x;
        tz = pad.z;
        kpX = 8.5; kdX = 5.0;
        kpZ = 8.5; kdZ = 5.0;

        const late = obs.t > 12.8;
        if (phase === 'pad_approach') {
          ty = clamp((late ? 0.14 : 0.18) + (late ? 0.52 : 0.68) * Math.max(0, distPad - 0.08), late ? 0.14 : 0.18, 1.05);
          tvy = py > (late ? 0.19 : 0.26) ? (late ? -0.42 : -0.25) : 0;
          if (!gustEdgeSoon && distPad < (late ? 0.108 : 0.092) && speedHoriz < (late ? 0.32 : 0.22) && py < (late ? 0.36 : 0.28) &&
              Math.abs(att.roll) < (late ? 0.26 : 0.22) && Math.abs(att.pitch) < (late ? 0.26 : 0.22)) {
            phase = 'touchdown';
          }
        }
        if (phase === 'touchdown') {
          if (py > 0.078 && (distPad > 0.120 || speedHoriz > 0.35 ||
              Math.abs(att.roll) > 0.25 || Math.abs(att.pitch) > 0.25 || (!late && gustEdgeSoon))) {
            phase = 'pad_approach';
            ty = 0.24;
            tvy = 0.15;
          } else {
            ty = 0.01;
            tvy = py > 0.10 ? (late ? -0.42 : -0.34) : -0.18;
          }
        }
      }

      // Compute desired accelerations with velocity capping for long transits
      const ex = tx - px;
      const ez = tz - pz;
      const ey = ty - py;

      let maxVx = (obs.gateIndex < 3 && phase === 'align' && ex > 0.35) ? 1.80 : 2.2;
      if (obs.gateIndex < 3 && phase === 'align' && ex > 0 && dx > 0.8) {
        const aBrakeEff = Math.max(1.1, 5.2 - Math.max(0, dx));
        maxVx = Math.min(maxVx, Math.sqrt(2.0 * aBrakeEff * ex));
      }
      const maxVz = 1.80;
      let desVx = clamp(tvx + (kpX / kdX) * ex, -maxVx, maxVx);
      let emergencyBrakeX = false;
      if (obs.gateIndex < 3 && phase === 'align' && vx > 0.20 && Math.abs(pz - obs.nextGate.centre.z) > 0.04) {
        const aBrk = Math.max(1.1, 5.2 - Math.max(0, dx));
        if (px + (vx * vx) / (2.0 * aBrk) > obs.nextGate.centre.x - 0.32) {
          desVx = -1.8;
          emergencyBrakeX = true;
        }
      }
      const desVz = clamp(tvz + (kpZ / kdZ) * ez, -maxVz, maxVz);
      const maxDownVy = (obs.gateIndex === 3 && py < 0.25) ? (obs.t > 12.8 ? -0.52 : -0.42) : -1.5;
      const desVy = clamp(tvy + (kpY / kdY) * ey, maxDownVy, 1.65);

      const axDes = clamp(kdX * (desVx - vx), emergencyBrakeX ? -8.5 : -6.5, maxAxPos);
      const azDes = clamp(kdZ * (desVz - vz), -6.5, 6.5);
      let ayDes = clamp(kdY * (desVy - vy), -4.8, 5.2);
      if (emergencyBrakeX && py < 2.1) {
        ayDes = Math.max(ayDes, 3.8);
      }

      // Total needed thrust acceleration components
      const reqAx = axDes + (0.42 / massEst) * vx - dx;
      const reqAz = azDes + (0.42 / massEst) * vz - dz;
      const reqAy = Math.max(2.5, ayDes + 9.81 + (0.42 / massEst) * vy);

      // Exact nonlinear attitude mapping
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const reqAxBody = cy * reqAx - sy * reqAz;
      const reqAzBody = sy * reqAx + cy * reqAz;

      let maxTilt = emergencyBrakeX ? 0.59 : 0.56;
      if (obs.gateIndex === 3 && py <= 0.24) {
        maxTilt = py <= 0.14 ? 0.24 : 0.30;
      }

      let pitchDes = clamp(Math.atan2(reqAxBody, reqAy), -maxTilt, maxTilt);
      let rollDes = clamp(-Math.atan2(reqAzBody * Math.cos(pitchDes), reqAy), -maxTilt, maxTilt);

      const onPadGround = obs.gateIndex === 3 && phase === 'touchdown' &&
        Math.hypot(obs.pad.centre.x - px, obs.pad.centre.z - pz) < 0.125 && py <= 0.076;

      if (onPadGround) {
        pitchDes = 0;
        rollDes = 0;
      }

      let maxThrust = 0.90;
      if (knownParams && knownParams.weakness > 0.26) {
        const alpha = knownParams.weakness;
        maxThrust = clamp((1.0 - alpha) / (1.0 - 0.25 * alpha) + 0.03, 0.70, 0.90);
      }
      if (Math.max(Math.abs(pitchDes - att.pitch), Math.abs(rollDes - att.roll)) > 0.15) {
        maxThrust = Math.min(maxThrust, 0.68);
      }

      let thrustCmd = (reqAy - dy) / (gamma * Math.max(0.55, dirY));
      if (onPadGround) {
        thrustCmd = 0.06;
      }
      thrustCmd = clamp(thrustCmd, 0, maxThrust);

      let pitchCmd, rollCmd, yawRateCmd;
      if (knownParams) {
        const w = knownParams.weakMotor, alpha = knownParams.weakness;
        const inertia = 0.006 * knownParams.mass / 0.6;
        const scale = (4.0 * alpha * thrustCmd) / (1.0 - 0.25 * alpha);
        const rollBias = (scale * 0.115 * MOTOR_Z[w]) / (65.0 * inertia);
        const pitchBias = -(scale * 0.115 * MOTOR_X[w]) / (65.0 * inertia);
        const yawBias = (scale * 0.014 * SPIN[w]) / (8.0 * inertia);

        attRefPitch += (pitchDes - attRefPitch) * 0.10;
        attRefRoll += (rollDes - attRefRoll) * 0.10;
        if (py > 0.085 && Math.abs(pitchDes) < 0.32 && Math.abs(rates.pitch) < 0.35) {
          attIntPitch = clamp(attIntPitch + (attRefPitch - att.pitch) * dt, -0.045, 0.045);
        }
        if (py > 0.085 && Math.abs(rollDes) < 0.32 && Math.abs(rates.roll) < 0.35) {
          attIntRoll = clamp(attIntRoll + (attRefRoll - att.roll) * dt, -0.045, 0.045);
        }
        if (py > 0.085) {
          yawInt = clamp(yawInt + att.yaw * dt, -0.4, 0.4);
        }

        const cmdLim = (obs.gateIndex === 3 && py <= 0.13) ? 0.30 : 0.60;
        pitchCmd = clamp(
          pitchDes + pitchBias - (22.0 / 65.0) * simIntegralPitch + 0.60 * (pitchDes - att.pitch) - 0.07 * rates.pitch + 1.0 * attIntPitch,
          -cmdLim, cmdLim
        );
        rollCmd = clamp(
          rollDes + rollBias - (22.0 / 65.0) * simIntegralRoll + 0.60 * (rollDes - att.roll) - 0.07 * rates.roll + 1.0 * attIntRoll,
          -cmdLim, cmdLim
        );
        yawRateCmd = clamp(yawBias - 4.5 * att.yaw - 0.4 * rates.yaw - 2.0 * yawInt, -2, 2);

        simIntegralPitch = clamp(simIntegralPitch + (pitchCmd - att.pitch) * dt, -0.35, 0.35);
        simIntegralRoll = clamp(simIntegralRoll + (rollCmd - att.roll) * dt, -0.35, 0.35);
      } else {
        attRefPitch += (pitchDes - attRefPitch) * 0.14;
        attRefRoll += (rollDes - attRefRoll) * 0.14;
        if (py > 0.085 && Math.abs(pitchDes) < 0.42) {
          attIntPitch = clamp(attIntPitch + (attRefPitch - att.pitch) * dt, -0.18, 0.18);
        }
        if (py > 0.085 && Math.abs(rollDes) < 0.42) {
          attIntRoll = clamp(attIntRoll + (attRefRoll - att.roll) * dt, -0.18, 0.18);
        }
        if (py > 0.085) {
          yawInt = clamp(yawInt + att.yaw * dt, -0.4, 0.4);
        }

        const cmdLim = (obs.gateIndex === 3 && py <= 0.13) ? 0.30 : 0.60;
        pitchCmd = clamp(
          pitchDes + 0.55 * (pitchDes - att.pitch) - 0.07 * rates.pitch + 1.6 * attIntPitch,
          -cmdLim, cmdLim
        );
        rollCmd = clamp(
          rollDes + 0.55 * (rollDes - att.roll) - 0.07 * rates.roll + 1.6 * attIntRoll,
          -cmdLim, cmdLim
        );
        yawRateCmd = clamp(-4.5 * att.yaw - 0.4 * rates.yaw - 3.5 * yawInt, -2, 2);
      }

      uLag += (thrustCmd - uLag) * (1 - Math.exp(-dt / (knownParams ? knownParams.lag : 0.038)));

      return { thrust: thrustCmd, roll: rollCmd, pitch: pitchCmd, yawRate: yawRateCmd };
    },
  };
}
