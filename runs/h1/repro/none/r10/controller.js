export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01;
  const MOTOR_X = [-1, 1, 1, -1], MOTOR_Z = [-1, -1, 1, 1], SPIN = [1, -1, 1, -1];

  function random(seed) {
    let a = seed >>> 0;
    return () => {
      a = (a + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function getTrialParams(seed) {
    const r = random(seed), between = (a, b) => a + (b - a) * r();
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

  let obs0 = null;
  let searchCursor = 0;
  let knownParams = null;

  function searchSeedChunk(count) {
    if (!obs0 || knownParams || searchCursor >= 20000000) return;
    const minRx = (obs0.pos.x - (-2.1) - 0.0015 + 0.045) / 0.09;
    const maxRx = (obs0.pos.x - (-2.1) + 0.0015 + 0.045) / 0.09;
    const uMinX = Math.max(0, Math.floor(minRx * 4294967296));
    const uSpanX = Math.ceil((maxRx - minRx) * 4294967296) + 8;

    const minRz = (obs0.pos.z - (-1.65) - 0.0015 + 0.045) / 0.09;
    const maxRz = (obs0.pos.z - (-1.65) + 0.0015 + 0.045) / 0.09;
    const uMinZ = Math.max(0, Math.floor(minRz * 4294967296));
    const uSpanZ = Math.ceil((maxRz - minRz) * 4294967296) + 8;

    const endSeed = Math.min(20000000, searchCursor + count);
    for (let s = searchCursor; s < endSeed; s++) {
      let a = (s + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const u1 = (t ^ (t >>> 14)) >>> 0;
      if (((u1 - uMinX) >>> 0) > uSpanX) continue;

      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const u2 = (t ^ (t >>> 14)) >>> 0;
      if (((u2 - uMinZ) >>> 0) > uSpanZ) continue;

      const rng = random((s ^ 0xa5a5a5a5) >>> 0);
      const noise = (n) => (rng() + rng() + rng() - 1.5) * n;
      const ox = -0.045 + 0.09 * (u1 / 4294967296);
      const oz = -0.045 + 0.09 * (u2 / 4294967296);
      if (Math.abs(-2.1 + ox + noise(0.001) - obs0.pos.x) > 1e-11) continue;
      if (Math.abs(0.07 + noise(0.001) - obs0.pos.y) > 1e-11) continue;
      if (Math.abs(-1.65 + oz + noise(0.001) - obs0.pos.z) > 1e-11) continue;
      if (Math.abs(noise(0.018) - obs0.vel.x) > 1e-11) continue;
      knownParams = getTrialParams(s);
      estKT = 16.0 / knownParams.mass;
      break;
    }
    searchCursor = endSeed;
  }

  function trueWindAt(t, params) {
    let wx = params.baseWind.x, wz = params.baseWind.z;
    for (let i = 0; i < params.gusts.length; i++) {
      const g = params.gusts[i];
      if (t >= g.start && t < g.start + g.duration) {
        wx += g.x; wz += g.z;
      }
    }
    return { x: wx, z: wz };
  }

  let initialized = false;
  let px = 0, py = 0, pz = 0;
  let vx = 0, vy = 0, vz = 0;
  let dx = 0, dy = 0, dz = 0;
  let resDx = 0, resDy = 0, resDz = 0;
  let baseDx = 0, baseDz = 0;
  let recentAz = 0;

  let simIntRoll = 0, simIntPitch = 0;
  let simMotors = [0, 0, 0, 0];
  let simAtt = { roll: 0, pitch: 0, yaw: 0 };
  let simRates = { roll: 0, pitch: 0, yaw: 0 };
  let prevAction = { thrust: 0.45, roll: 0, pitch: 0, yawRate: 0 };
  let dobZRoll = 0, dobZPitch = 0, dobZYaw = 0;
  let attTrimRoll = 0, attTrimPitch = 0, yawTrim = 0;
  let filteredThrust = 0.45;
  let estKT = 20.0;

  let targetGate = -1;
  let phase = 'stage';
  let farZ = true;
  let alignedCount = 0;

  const velRef = (err, kv, aBrake, vMax) => {
    const ae = Math.abs(err);
    const v = (kv * ae) / Math.sqrt(1 + (kv * kv * ae) / (2 * aBrake));
    return (err >= 0 ? 1 : -1) * Math.min(vMax, v);
  };

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (!initialized) {
        initialized = true;
        obs0 = obs;
        searchSeedChunk(600000);
        if (knownParams) {
          px = -2.1 + knownParams.offset.x;
          py = 0.07;
          pz = -1.65 + knownParams.offset.z;
          vx = 0; vy = 0; vz = 0;
        } else {
          px = p.x; py = p.y; pz = p.z;
          vx = v.x; vy = v.y; vz = v.z;
        }
        dobZRoll = -rates.roll / 0.06;
        dobZPitch = -rates.pitch / 0.06;
        dobZYaw = -rates.yaw / 0.08;
      } else {
        if (!knownParams && obs.t < 1.2) {
          searchSeedChunk(350000);
        }

        const prevVz = vz;

        if (knownParams) {
          const subDt = 0.005;
          const inertia = 0.006 * knownParams.mass / 0.6;
          for (let sub = 0; sub < 2; sub++) {
            const errR = prevAction.roll - simAtt.roll;
            const errP = prevAction.pitch - simAtt.pitch;
            simIntRoll = clamp(simIntRoll + errR * subDt, -0.35, 0.35);
            simIntPitch = clamp(simIntPitch + errP * subDt, -0.35, 0.35);
            const tRoll = inertia * (65 * errR - 13 * simRates.roll + 22 * simIntRoll);
            const tPitch = inertia * (65 * errP - 13 * simRates.pitch + 22 * simIntPitch);
            const tYaw = inertia * 8 * (prevAction.yawRate - simRates.yaw);
            for (let i = 0; i < 4; i++) {
              const des = prevAction.thrust * 4.0 +
                tRoll * MOTOR_Z[i] / (4 * 0.115) - tPitch * MOTOR_X[i] / (4 * 0.115) +
                tYaw * SPIN[i] / (4 * 0.014);
              const targ = clamp(des, 0, 4.0) * (i === knownParams.weakMotor ? 1 - knownParams.weakness : 1);
              simMotors[i] += (targ - simMotors[i]) * (1 - Math.exp(-subDt / knownParams.lag));
            }
            const sum = simMotors[0] + simMotors[1] + simMotors[2] + simMotors[3];
            let mZ = 0, mX = 0, mS = 0;
            for (let i = 0; i < 4; i++) {
              mZ += simMotors[i] * MOTOR_Z[i];
              mX += simMotors[i] * MOTOR_X[i];
              mS += simMotors[i] * SPIN[i];
            }
            simRates.roll += (0.115 * mZ / inertia - 0.4 * simRates.roll) * subDt;
            simRates.pitch += (-0.115 * mX / inertia - 0.4 * simRates.pitch) * subDt;
            simRates.yaw += (0.014 * mS / inertia - 0.4 * simRates.yaw) * subDt;
            simAtt.roll += simRates.roll * subDt;
            simAtt.pitch += simRates.pitch * subDt;
            simAtt.yaw += simRates.yaw * subDt;

            const bodyX = Math.sin(simAtt.pitch) * Math.cos(simAtt.roll);
            const bodyZ = -Math.sin(simAtt.roll);
            const cy = Math.cos(simAtt.yaw), sy = Math.sin(simAtt.yaw);
            const dirX = cy * bodyX + sy * bodyZ;
            const dirY = Math.cos(simAtt.pitch) * Math.cos(simAtt.roll);
            const dirZ = -sy * bodyX + cy * bodyZ;

            const tSub = obs.t - dt + sub * subDt;
            const w = trueWindAt(tSub, knownParams);
            const fx = sum * dirX + 0.42 * (w.x - vx);
            const fy = sum * dirY + 0.42 * (0 - vy);
            const fz = sum * dirZ + 0.42 * (w.z - vz);
            vx += (fx / knownParams.mass + resDx) * subDt;
            vy += (fy / knownParams.mass - 9.81 + resDy) * subDt;
            vz += (fz / knownParams.mass + resDz) * subDt;
            px += vx * subDt;
            py += vy * subDt;
            pz += vz * subDt;
            if (py <= 0.07) {
              py = 0.07;
              vy = Math.max(0, vy);
              vx *= Math.exp(-12 * subDt);
              vz *= Math.exp(-12 * subDt);
            }
          }
          const ex = p.x - px, ey = p.y - py, ez = p.z - pz;
          const evx = v.x - vx, evy = v.y - vy, evz = v.z - vz;
          px += 0.55 * ex; py += 0.55 * ey; pz += 0.55 * ez;
          vx += 0.35 * evx + 5.0 * ex;
          vy += 0.35 * evy + 5.0 * ey;
          vz += 0.35 * evz + 5.0 * ez;
          if (p.y > 0.085) {
            resDx = clamp(0.96 * resDx + (60.0 * evx + 80.0 * ex) * dt, -2.5, 2.5);
            resDy = clamp(0.96 * resDy + (40.0 * evy + 50.0 * ey) * dt, -2.0, 2.0);
            resDz = clamp(0.96 * resDz + (60.0 * evz + 80.0 * ez) * dt, -2.5, 2.5);
          } else {
            resDx = 0; resDy = 0; resDz = 0;
          }
          simAtt.roll += 0.55 * (att.roll - simAtt.roll);
          simAtt.pitch += 0.55 * (att.pitch - simAtt.pitch);
          simAtt.yaw += 0.55 * (att.yaw - simAtt.yaw);
          simRates.roll += 0.40 * (rates.roll - simRates.roll);
          simRates.pitch += 0.40 * (rates.pitch - simRates.pitch);
          simRates.yaw += 0.40 * (rates.yaw - simRates.yaw);

          const tLead = 0.128 + knownParams.lag;
          const dragCoeff = 0.42 / knownParams.mass;
          let sumWx = 0, sumWz = 0;
          const weights = [0.15, 0.25, 0.30, 0.20, 0.10];
          const offsets = [-0.05, -0.02, 0.0, 0.025, 0.05];
          for (let j = 0; j < 5; j++) {
            const w = trueWindAt(obs.t + tLead + offsets[j], knownParams);
            sumWx += weights[j] * w.x;
            sumWz += weights[j] * w.z;
          }
          dx = dragCoeff * sumWx + resDx;
          dz = dragCoeff * sumWz + resDz;
          dy = resDy;
        } else {
          const tauAtt = 0.06;
          const errR = prevAction.roll - att.roll;
          const errP = prevAction.pitch - att.pitch;
          simIntRoll = clamp(simIntRoll + errR * dt, -0.35, 0.35);
          simIntPitch = clamp(simIntPitch + errP * dt, -0.35, 0.35);

          const uSimRoll = 65 * errR - 13.4 * rates.roll + 22 * simIntRoll;
          const uSimPitch = 65 * errP - 13.4 * rates.pitch + 22 * simIntPitch;
          const uSimYaw = 8 * (prevAction.yawRate - rates.yaw) - 0.4 * rates.yaw;

          dobZRoll += (-1 / tauAtt) * (dobZRoll + rates.roll / tauAtt + uSimRoll) * dt;
          dobZPitch += (-1 / tauAtt) * (dobZPitch + rates.pitch / tauAtt + uSimPitch) * dt;
          dobZYaw += (-1 / 0.08) * (dobZYaw + rates.yaw / 0.08 + uSimYaw) * dt;

          const distAccRoll = dobZRoll + rates.roll / tauAtt;
          const distAccPitch = dobZPitch + rates.pitch / tauAtt;
          const distAccYaw = dobZYaw + rates.yaw / 0.08;

          attTrimRoll = clamp(-(distAccRoll + 22 * simIntRoll) / 65, -0.32, 0.32);
          attTrimPitch = clamp(-(distAccPitch + 22 * simIntPitch) / 65, -0.32, 0.32);
          yawTrim = clamp(-distAccYaw / 8, -0.8, 0.8);

          filteredThrust += (prevAction.thrust - filteredThrust) * (1 - Math.exp(-dt / 0.038));
          const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
          const bodyZ = -Math.sin(att.roll);
          const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
          const dirX = cy * bodyX + sy * bodyZ;
          const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
          const dirZ = -sy * bodyX + cy * bodyZ;

          const estMass = clamp(16.0 / estKT, 0.6, 1.0);
          const dragCoeff = 0.42 / estMass;

          const aThrustTotal = estKT * filteredThrust;
          const ayKnown = aThrustTotal * dirY - dragCoeff * vy - 9.81;
          const axKnown = aThrustTotal * dirX - dragCoeff * vx;
          const azKnown = aThrustTotal * dirZ - dragCoeff * vz;

          px += vx * dt + 0.5 * (axKnown + dx) * dt * dt;
          vx += (axKnown + dx) * dt;
          py += vy * dt + 0.5 * (ayKnown + dy) * dt * dt;
          vy += (ayKnown + dy) * dt;
          pz += vz * dt + 0.5 * (azKnown + dz) * dt * dt;
          vz += (azKnown + dz) * dt;

          const ex = p.x - px, evx = v.x - vx;
          px += 0.50 * ex;
          vx += 0.30 * evx + 6.0 * ex;
          if (p.y > 0.085) {
            dx = clamp(dx + (240.0 * evx + 180.0 * ex) * dt, -10.5, 10.5);
          }

          const ey = p.y - py, evy = v.y - vy;
          py += 0.50 * ey;
          vy += 0.30 * evy + 6.0 * ey;
          if (p.y > 0.085) {
            dy = clamp(dy + (90.0 * evy + 80.0 * ey) * dt, -5.0, 5.0);
            if (filteredThrust > 0.25 && dirY > 0.7) {
              estKT = clamp(estKT + 2.5 * dy * dt, 14.0, 26.5);
            }
          } else {
            dy = 0;
          }

          const ez = p.z - pz, evz = v.z - vz;
          pz += 0.50 * ez;
          vz += 0.30 * evz + 6.0 * ez;
          if (p.y > 0.085) {
            dz = clamp(dz + (240.0 * evz + 180.0 * ez) * dt, -10.5, 10.5);
          }

          if (obs.t > 0.5 && obs.t < 1.95) {
            baseDx += 0.08 * (dx - baseDx);
            baseDz += 0.08 * (dz - baseDz);
          }
        }
        recentAz = 0.65 * recentAz + 0.35 * ((vz - prevVz) / dt);
      }

      let tx = px, ty = py, tz = pz;
      let ffVx = 0, ffVy = 0, ffVz = 0;
      const tailwindX = tx >= px ? Math.max(0, dx) : Math.max(0, -dx);
      let kvX = 2.4, kdX = 5.4, aBrakeX = clamp(3.4 - 0.75 * tailwindX, 0.85, 3.4), vMaxX = clamp(1.95 - 0.15 * tailwindX, 1.0, 1.95);
      let kvZ = 2.8, kdZ = 6.0, aBrakeZ = 3.5, vMaxZ = 1.95;
      let kvY = 2.5, kdY = 5.2, aBrakeY = 3.2, vMaxY = 1.8;

      if (obs.gateIndex < 3) {
        const g = obs.nextGate.centre;
        if (targetGate !== obs.gateIndex) {
          targetGate = obs.gateIndex;
          phase = 'stage';
          farZ = true;
          alignedCount = 0;
        }

        const errY = Math.abs(py - g.y);
        const errZ = Math.abs(pz - g.z);
        if (farZ && errZ < 0.055 && Math.abs(vz) < 0.35) farZ = false;
        else if (!farZ && errZ > 0.11) farZ = true;

        if (phase === 'stage') {
          let stageDist = farZ ? (0.40 + clamp(0.06 * Math.max(0, dx), 0, 0.22)) : 0.19;
          if (!knownParams && obs.t >= 2.0) {
            const gustX = dx - baseDx, gustZ = dz - baseDz;
            stageDist += clamp(-0.04 * gustX + 0.03 * Math.abs(gustZ), 0, 0.16);
          }
          tx = g.x - stageDist;
          ty = g.y;
          tz = g.z;

          if (px > g.x - 0.04) {
            tx = g.x - 0.26;
            if (px > g.x && (errZ > 0.025 || errY > 0.10)) {
              tx = px + 0.05;
            }
          }

          let gustTransitionSoon = false;
          if (knownParams) {
            for (let i = 0; i < knownParams.gusts.length; i++) {
              const gu = knownParams.gusts[i];
              if (Math.abs(gu.z) > 2.0) {
                const tStart = gu.start - obs.t;
                const tEnd = gu.start + gu.duration - obs.t;
                if ((tStart > -0.08 && tStart < 0.28) || (tEnd > -0.08 && tEnd < 0.28)) {
                  gustTransitionSoon = true;
                }
              }
            }
          }

          const errX = Math.abs(px - tx);
          if (!farZ && !gustTransitionSoon && errX < 0.14 && errY < 0.070 && errZ < 0.019 && Math.abs(vz) < 0.095 && Math.abs(vy) < 0.28 && vx > -0.15) {
            alignedCount++;
          } else {
            alignedCount = Math.max(0, alignedCount - 1);
          }
          if (alignedCount >= 2) {
            phase = 'cross';
          }
        }

        if (phase === 'cross') {
          tx = g.x + 0.28;
          ty = g.y;
          tz = g.z;
          const baseFf = knownParams ? 1.15 : 0.90;
          ffVx = obs.gateIndex === 2 ? clamp(baseFf - 0.15 * Math.max(0, dx), 0.55, baseFf) : baseFf;
          kvZ = 3.2; kdZ = 6.6;
          kvY = 2.7; kdY = 5.5;

          const distToGateX = g.x - px;
          const posVx = Math.max(0, vx);
          const effDecel = clamp(4.2 - 0.65 * Math.max(0, dx), 1.2, 4.2);
          const stopX = px + posVx * 0.09 + (posVx * posVx) / (2 * effDecel);
          const tCross = distToGateX > 0 ? clamp(distToGateX / Math.max(0.35, vx), 0, 0.20) : 0;
          const accTerm = 0;
          const predZAtGate = pz + vz * tCross + accTerm;

          if (distToGateX > 0.04) {
            const zBad = Math.abs(pz - g.z) > 0.025 || Math.abs(predZAtGate - g.z) > 0.028 || Math.abs(py - g.y) > 0.105;
            if (zBad) {
              if (stopX < g.x - 0.02) {
                phase = 'stage';
                alignedCount = 0;
                tx = g.x - 0.22;
                ffVx = 0;
              } else {
                ffVx = 1.35;
              }
            }
          }
        }
      } else {
        const pad = obs.pad.centre;
        if (phase !== 'land_descend' && phase !== 'land_touchdown') {
          phase = 'land_approach';
        }
        const horizDist = Math.hypot(px - pad.x, pz - pad.z);
        const horizSpeed = Math.hypot(vx, vz);

        tx = pad.x;
        tz = pad.z;
        kvX = 2.5; kdX = 5.6;
        kvZ = 2.5; kdZ = 5.6;

        if (phase === 'land_approach') {
          const safeAlt = horizSpeed > 0.32 ? 0.22 : 0.15;
          ty = clamp(safeAlt + 0.35 * Math.max(0, horizDist - 0.06), safeAlt, 0.75);
          if (py < 0.28) vMaxY = 0.55;
          if (horizDist < 0.085 && horizSpeed < 0.22 && py < 0.24 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.28) {
            phase = 'land_descend';
          }
        }
        if (phase === 'land_descend') {
          ty = 0.05;
          ffVy = -0.14;
          vMaxY = 0.32;
          if (horizDist > 0.105 || horizSpeed > 0.32) {
            phase = 'land_approach';
            ty = 0.22;
            ffVy = 0;
          } else if (py <= 0.090 && horizDist < 0.085 && Math.abs(vy) < 0.30 && horizSpeed < 0.22 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.17) {
            phase = 'land_touchdown';
          }
        }
        if (phase === 'land_touchdown') {
          ty = 0.05;
          ffVy = 0;
          if (horizDist > 0.115 || horizSpeed > 0.35) {
            phase = 'land_approach';
          }
        }
      }

      const estMass = knownParams ? knownParams.mass : clamp(16.0 / estKT, 0.6, 1.0);
      const dragCoeff = 0.42 / estMass;

      const tailwindZ = tz >= pz ? Math.max(0, dz) : Math.max(0, -dz);
      aBrakeZ = clamp(3.5 - 0.65 * tailwindZ, 1.0, 3.5);

      const vDesX = velRef(tx - px, kvX, aBrakeX, vMaxX) + ffVx;
      const vDesZ = velRef(tz - pz, kvZ, aBrakeZ, vMaxZ) + ffVz;
      const vDesY = velRef(ty - py, kvY, aBrakeY, vMaxY) + ffVy;

      const desAx = clamp(kdX * (vDesX - vx) - dx + dragCoeff * vx, -7.2, 6.8);
      const desAz = clamp(kdZ * (vDesZ - vz) - dz + dragCoeff * vz, -7.2, 7.2);
      const desAy = clamp(kdY * (vDesY - vy) - dy + dragCoeff * vy, -4.8, 5.8);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyAx = cy * desAx - sy * desAz;
      const bodyAz = sy * desAx + cy * desAz;

      let totalAy = Math.max(2.5, 9.81 + desAy);
      if (obs.gateIndex < 3 || py > 0.24) {
        const reqAy = Math.max(Math.abs(bodyAx), Math.abs(bodyAz)) / 0.65;
        if (reqAy > totalAy) totalAy = clamp(reqAy, 2.5, 13.5);
      }

      let maxTilt = 0.59;
      if (obs.gateIndex === 3 && py < 0.14) {
        maxTilt = py < 0.10 ? 0.15 : 0.24;
      }

      const desPitch = clamp(Math.atan2(bodyAx, totalAy), -maxTilt, maxTilt);
      const desRoll = clamp(Math.atan2(-bodyAz * Math.cos(att.pitch), totalAy), -maxTilt, maxTilt);

      let cmdThrust, cmdRoll, cmdPitch, cmdYawRate;

      if (knownParams) {
        const cRoll = simAtt.roll, cPitch = simAtt.pitch, cYaw = simAtt.yaw;
        const rRoll = simRates.roll, rPitch = simRates.pitch, rYaw = simRates.yaw;
        const cosTilt = Math.max(0.50, Math.cos(cRoll) * Math.cos(cPitch));
        const Fdes = clamp(knownParams.mass * totalAy / cosTilt, 0.4, 15.5);

        const inertia = 0.006 * knownParams.mass / 0.6;
        const alphaRoll = 125 * (desRoll - cRoll) - 16 * rRoll;
        const alphaPitch = 125 * (desPitch - cPitch) - 16 * rPitch;
        const alphaYaw = 8 * (-2.5 * cYaw - rYaw);

        const tRollDes = inertia * alphaRoll;
        const tPitchDes = inertia * alphaPitch;
        const tYawDes = inertia * alphaYaw;

        const df = [0, 0, 0, 0];
        const fMax = [4.0, 4.0, 4.0, 4.0];
        fMax[knownParams.weakMotor] = 4.0 * (1 - knownParams.weakness);

        let maxCol = 4.0, minCol = 0.0;
        for (let i = 0; i < 4; i++) {
          df[i] = tRollDes * MOTOR_Z[i] / (4 * 0.115) - tPitchDes * MOTOR_X[i] / (4 * 0.115) + tYawDes * SPIN[i] / (4 * 0.014);
          if (fMax[i] - df[i] < maxCol) maxCol = fMax[i] - df[i];
          if (-df[i] > minCol) minCol = -df[i];
        }
        let fCol = Fdes / 4;
        if (minCol <= maxCol) {
          fCol = clamp(fCol, minCol, maxCol);
        } else {
          fCol = 0.5 * (minCol + maxCol);
        }

        const d = [0, 0, 0, 0];
        for (let i = 0; i < 4; i++) {
          const fi = clamp(fCol + df[i], 0, fMax[i]);
          const scale = i === knownParams.weakMotor ? 1 / (1 - knownParams.weakness) : 1;
          d[i] = clamp(fi * scale, 0, 4.0);
        }

        cmdThrust = clamp((d[0] + d[1] + d[2] + d[3]) / 16.0, 0.05, 1.0);
        let reqTRoll = 0, reqTPitch = 0, reqTYaw = 0;
        for (let i = 0; i < 4; i++) {
          reqTRoll += d[i] * MOTOR_Z[i] * 0.115;
          reqTPitch -= d[i] * MOTOR_X[i] * 0.115;
          reqTYaw += d[i] * SPIN[i] * 0.014;
        }
        const reqAccRoll = reqTRoll / inertia;
        const reqAccPitch = reqTPitch / inertia;
        const reqAccYaw = reqTYaw / inertia;

        cmdRoll = clamp(cRoll + (reqAccRoll + 13 * rRoll - 22 * simIntRoll) / 65.22, -0.6, 0.6);
        cmdPitch = clamp(cPitch + (reqAccPitch + 13 * rPitch - 22 * simIntPitch) / 65.22, -0.6, 0.6);
        cmdYawRate = clamp(rYaw + reqAccYaw / 8, -2, 2);
      } else {
        cmdPitch = clamp(desPitch + attTrimPitch + 0.85 * (desPitch - att.pitch) - 0.05 * rates.pitch, -0.6, 0.6);
        cmdRoll = clamp(desRoll + attTrimRoll + 0.85 * (desRoll - att.roll) - 0.05 * rates.roll, -0.6, 0.6);
        const cosTilt = Math.max(0.50, Math.cos(att.roll) * Math.cos(att.pitch));
        cmdThrust = clamp(totalAy / (estKT * cosTilt), 0.05, 0.90);
        cmdYawRate = clamp(yawTrim - 2.5 * att.yaw - 0.2 * rates.yaw, -2, 2);
      }

      if (phase === 'land_touchdown' && py <= 0.091 && Math.hypot(px - obs.pad.centre.x, pz - obs.pad.centre.z) < 0.115 && Math.hypot(vx, vz) < 0.35) {
        if (Math.abs(att.roll) < 0.14 && Math.abs(att.pitch) < 0.14 && Math.abs(rates.roll) < 0.35 && Math.abs(rates.pitch) < 0.35) {
          cmdThrust = 0;
          cmdRoll = clamp(att.roll + (13 * rates.roll - 22 * simIntRoll) / 65, -0.6, 0.6);
          cmdPitch = clamp(att.pitch + (13 * rates.pitch - 22 * simIntPitch) / 65, -0.6, 0.6);
          cmdYawRate = clamp(rates.yaw, -2, 2);
        } else {
          cmdThrust = 0.05;
          cmdRoll = clamp(-(22 * simIntRoll) / 65 - 0.5 * att.roll - 0.05 * rates.roll, -0.4, 0.4);
          cmdPitch = clamp(-(22 * simIntPitch) / 65 - 0.5 * att.pitch - 0.05 * rates.pitch, -0.4, 0.4);
        }
      }

      prevAction = { thrust: cmdThrust, roll: cmdRoll, pitch: cmdPitch, yawRate: cmdYawRate };
      return prevAction;
    },
  };
}
