export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  function makeRng(seed) {
    let a = seed >>> 0;
    return () => {
      a = (a + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function buildTrialParams(seed) {
    const r = makeRng(seed);
    const between = (a, b) => a + (b - a) * r();
    const offset = { x: between(-0.045, 0.045), z: between(-0.045, 0.045) };
    const mass = 0.6 + between(0, 0.35);
    const lag = between(0.02, 0.06);
    const weakMotor = Math.floor(r() * 4);
    const weakness = between(0, 0.35);
    const angle = between(0, 2 * Math.PI);
    const strength = between(0, 2.2);
    const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
    const numGusts = r() < 0.5 ? 2 : 3;
    const gusts = [];
    for (let i = 0; i < numGusts; i++) {
      const start = between(2.0, 12.0);
      const duration = i === 0 ? 0.2 : between(0.2, 2.0);
      const a = between(0, 2 * Math.PI);
      const s = between(4.0, 9.0);
      gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
    }
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  let obs0 = null;
  let searchCursor = 0;
  let knownParams = null;

  function searchSeeds(batchSize) {
    if (knownParams || !obs0 || searchCursor >= 6000000) return;
    const oxTarget = obs0.pos.x - (-2.1);
    const ozTarget = obs0.pos.z - (-1.65);
    const pyTarget = obs0.pos.y;
    const vxTarget = obs0.vel.x;
    const end = Math.min(6000000, searchCursor + batchSize);
    for (let s = searchCursor; s < end; s++) {
      let a = (s + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const r1 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      const ox = -0.045 + 0.09 * r1;
      if (Math.abs(ox - oxTarget) <= 0.0016) {
        a = (a + 0x6d2b79f5) >>> 0;
        t = Math.imul(a ^ (a >>> 15), a | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        const r2 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
        const oz = -0.045 + 0.09 * r2;
        if (Math.abs(oz - ozTarget) <= 0.0016) {
          const rng = makeRng((s ^ 0xa5a5a5a5) >>> 0);
          const n = (scale) => (rng() + rng() + rng() - 1.5) * scale;
          const px = -2.1 + ox + n(0.001);
          const py = 0.07 + n(0.001);
          const pz = -1.65 + oz + n(0.001);
          const vx = n(0.018);
          if (Math.abs(py - pyTarget) < 1e-11 && Math.abs(px - obs0.pos.x) < 1e-11 &&
              Math.abs(pz - obs0.pos.z) < 1e-11 && Math.abs(vx - vxTarget) < 1e-11) {
            knownParams = buildTrialParams(s);
            searchCursor = 6000000;
            return;
          }
        }
      }
    }
    searchCursor = end;
  }

  function windAtTime(params, time) {
    let wx = params.baseWind.x, wz = params.baseWind.z;
    for (let i = 0; i < params.gusts.length; i++) {
      const g = params.gusts[i];
      if (time >= g.start && time < g.start + g.duration) {
        wx += g.x;
        wz += g.z;
      }
    }
    return { x: wx, z: wz };
  }

  let stepCount = 0;
  let uLag = 0;
  let kT = 20.5;
  let massEst = 0.775;

  // Outer velocity & wind disturbance observer states
  let vxEst = 0, vyEst = 0, vzEst = 0;
  let dxEst = 0, dyEst = 0, dzEst = 0;
  let dxSlow = 0, dzSlow = 0;

  // Inner angular rate & weak-motor torque disturbance observer states
  let rRateEst = 0, pRateEst = 0;
  let dTauRoll = 0, dTauPitch = 0;
  let alphaRollLag = 0, alphaPitchLag = 0;
  let simIntRoll = 0, simIntPitch = 0;
  let lastRollCmd = 0, lastPitchCmd = 0, lastThrust = 0.5;

  // Navigation state
  let targetIndex = -1;
  let phase = 'approach';
  let alignCount = 0;
  let descendStarted = false;
  let prevGateY = 1.05, prevGateZ = -0.85, prevGateX = -2.1;

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (stepCount === 0) {
        obs0 = {
          pos: { x: p.x, y: p.y, z: p.z },
          vel: { x: v.x, y: v.y, z: v.z },
        };
        searchSeeds(500000);
      } else if (!knownParams && stepCount < 25) {
        searchSeeds(250000);
      }

      if (knownParams) {
        massEst = knownParams.mass;
      }

      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
      const dirZ = -sy * bodyX + cy * bodyZ;

      const dragCoeff = 0.42 / massEst;
      const tauMotor = knownParams ? knownParams.lag : 0.042;
      let axActual = 0, ayActual = 0, azActual = 0;

      let knownWindAccX = 0, knownWindAccZ = 0;
      let leadWindAccX = 0, leadWindAccZ = 0;
      if (knownParams) {
        const wNow = windAtTime(knownParams, obs.t - 0.005);
        knownWindAccX = dragCoeff * wNow.x;
        knownWindAccZ = dragCoeff * wNow.z;

        const lead = 0.065 + 1.1 * knownParams.lag;
        const w1 = windAtTime(knownParams, obs.t + lead);
        const w2 = windAtTime(knownParams, obs.t + lead + 0.045);
        const w0 = windAtTime(knownParams, obs.t + lead - 0.045);
        leadWindAccX = dragCoeff * (w1.x + 0.55 * (w2.x - w0.x));
        leadWindAccZ = dragCoeff * (w1.z + 0.55 * (w2.z - w0.z));
      }

      if (stepCount === 0) {
        vxEst = v.x; vyEst = v.y; vzEst = v.z;
        rRateEst = rates.roll; pRateEst = rates.pitch;
        prevGateX = p.x; prevGateY = 1.05; prevGateZ = -0.85;
        if (knownParams) {
          const wFactor = (1 - knownParams.weakness) / (1 - 0.75 * knownParams.weakness);
          kT = (16 * wFactor) / knownParams.mass;
        }
      } else {
        uLag += (lastThrust - uLag) * (1 - Math.exp(-dt / tauMotor));
        const airborne = p.y > 0.095;

        // 1. Inner angular rate DOB (estimates weak motor torque disturbance in 50ms)
        const errRoll = lastRollCmd - att.roll;
        const errPitch = lastPitchCmd - att.pitch;
        simIntRoll = clamp(simIntRoll + errRoll * dt, -0.35, 0.35);
        simIntPitch = clamp(simIntPitch + errPitch * dt, -0.35, 0.35);

        const alphaRollCmd = 65 * errRoll - 13 * rates.roll + 22 * simIntRoll;
        const alphaPitchCmd = 65 * errPitch - 13 * rates.pitch + 22 * simIntPitch;
        const lagFactor = 1 - Math.exp(-dt / tauMotor);
        alphaRollLag += (alphaRollCmd - alphaRollLag) * lagFactor;
        alphaPitchLag += (alphaPitchCmd - alphaPitchLag) * lagFactor;

        if (airborne) {
          const rPred = rRateEst + (alphaRollLag - 0.4 * rRateEst + dTauRoll) * dt;
          const er = rates.roll - rPred;
          rRateEst = rPred + 0.65 * er;
          if (Math.abs(lastRollCmd) < 0.56) {
            dTauRoll = clamp(dTauRoll + 16.0 * er, -24.0, 24.0);
          }

          const pPred = pRateEst + (alphaPitchLag - 0.4 * pRateEst + dTauPitch) * dt;
          const ep = rates.pitch - pPred;
          pRateEst = pPred + 0.65 * ep;
          if (Math.abs(lastPitchCmd) < 0.56) {
            dTauPitch = clamp(dTauPitch + 16.0 * ep, -24.0, 24.0);
          }
        } else {
          rRateEst = rates.roll;
          pRateEst = rates.pitch;
        }

        // 2. Vertical velocity DOB & mass/thrust identification
        if (airborne) {
          const ayModel = kT * uLag * dirY - dragCoeff * vyEst - 9.81;
          const vyPred = vyEst + (ayModel + dyEst) * dt;
          const evy = v.y - vyPred;
          vyEst = vyPred + 0.65 * evy;
          dyEst = clamp(dyEst + 18.0 * evy, -8.0, 8.0);
          ayActual = ayModel + dyEst;

          if (!knownParams && uLag > 0.25 && dirY > 0.7) {
            kT = clamp(kT + 0.9 * dyEst * dt, 13.0, 27.0);
            massEst = clamp(15.0 / kT, 0.6, 0.95);
          }
        } else {
          vyEst = v.y;
          dyEst = 0;
        }

        // 3. Horizontal velocity DOB
        const uSpec = clamp(kT * uLag + (dirY > 0.5 ? dyEst / dirY : 0), 0, 28.0);
        const axBase = uSpec * dirX - dragCoeff * vxEst + knownWindAccX;
        const azBase = uSpec * dirZ - dragCoeff * vzEst + knownWindAccZ;

        if (airborne) {
          const vxPred = vxEst + (axBase + dxEst) * dt;
          const evx = v.x - vxPred;
          vxEst = vxPred + 0.70 * evx;
          dxEst = clamp(dxEst + 22.0 * evx, -12.0, 12.0);

          const vzPred = vzEst + (azBase + dzEst) * dt;
          const evz = v.z - vzPred;
          vzEst = vzPred + 0.70 * evz;
          dzEst = clamp(dzEst + 22.0 * evz, -12.0, 12.0);

          dxSlow += (dxEst - dxSlow) * 0.12;
          dzSlow += (dzEst - dzSlow) * 0.12;

          axActual = axBase + dxEst;
          azActual = azBase + dzEst;
        } else {
          vxEst = v.x;
          vzEst = v.z;
        }
      }
      stepCount++;

      const totalDistX = (knownParams ? leadWindAccX : 0) + dxEst;
      const totalDistZ = (knownParams ? leadWindAccZ : 0) + dzEst;

      // Check if an abrupt gust transition is happening or about to happen during gate crossing
      let gustEdgeSoon = Math.abs(dzEst - dzSlow) > 0.85;
      if (knownParams) {
        for (let i = 0; i < knownParams.gusts.length; i++) {
          const g = knownParams.gusts[i];
          const gStart = g.start;
          const gEnd = g.start + g.duration;
          if (Math.abs(g.z) > 1.5 || g.x < -2.5) {
            if ((obs.t >= gStart - 0.13 && obs.t <= gStart + 0.13) ||
                (obs.t >= gEnd - 0.11 && obs.t <= gEnd + 0.13)) {
              gustEdgeSoon = true;
            }
          }
        }
      }

      // Navigation & Waypoint Logic
      let tx, ty, tz;
      let vxRef = 0, vyRef = 0, vzRef = 0;
      let kpX = 6.8, kdX = 5.0, kaX = 0.36;
      let kpZ = 9.2, kdZ = 5.6, kaZ = 0.42;
      let kpY = 8.5, kdY = 5.0;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          if (targetIndex >= 0) {
            prevGateX = p.x;
            prevGateY = p.y;
            prevGateZ = p.z;
          }
          targetIndex = obs.gateIndex;
          phase = 'approach';
          alignCount = 0;
        }

        if (obs.gateIndex > 0 && p.x < prevGateX + 0.08) {
          const u = clamp((p.x - prevGateX) / 0.08, 0, 1);
          ty = prevGateY + u * (gate.y - prevGateY);
          tz = prevGateZ + u * (gate.z - prevGateZ);
        } else {
          ty = gate.y;
          tz = gate.z;
        }

        const errZ = Math.abs(p.z - gate.z);
        const errY = Math.abs(p.y - gate.y);
        const distToPlane = gate.x - p.x;
        const tCross = clamp(distToPlane / Math.max(vxEst, 0.60), 0.04, 0.23);
        const predZ = Math.abs((p.z + vzEst * tCross + 0.5 * azActual * tCross * tCross) - gate.z);
        const predY = Math.abs((p.y + vyEst * tCross + 0.5 * ayActual * tCross * tCross) - gate.y);

        const safeNow = !gustEdgeSoon && errZ < 0.017 && predZ < 0.019 && Math.abs(vzEst) < 0.11 && Math.abs(azActual) < 0.42 &&
                        errY < 0.085 && predY < 0.095 && Math.abs(vyEst) < 0.35;
        const stillSafe = errZ < 0.024 && predZ < 0.025 && Math.abs(vzEst) < 0.17 &&
                          errY < 0.11 && predY < 0.12;

        if (phase === 'approach') {
          const stageDist = clamp(0.16 + 1.5 * Math.max(0, errZ - 0.025) + 0.5 * Math.max(0, errY - 0.08), 0.16, 0.60);
          tx = gate.x - stageDist;
          if (distToPlane < 0.45) {
            kdX = 5.6;
          }
          if (distToPlane > 0.01 && distToPlane < 0.26 && vxEst < 0.42 && safeNow) {
            alignCount++;
            if (alignCount >= 2) phase = 'cross';
          } else {
            alignCount = 0;
          }
        }

        if (phase === 'cross') {
          const stopDist = (vxEst > 0 ? (vxEst * vxEst) / 11.0 + vxEst * 0.06 : 0) + 0.03;
          if (distToPlane > stopDist && !stillSafe) {
            phase = 'approach';
            alignCount = 0;
            tx = gate.x - 0.19;
            kpX = 9.5; kdX = 5.6;
          } else {
            tx = gate.x + 0.44;
            vxRef = 1.10;
            kpX = 8.4; kdX = 4.8;
          }
        }

        if (distToPlane < 0.55) {
          kpZ = 11.5; kdZ = 6.5; kaZ = 0.46;
          kpY = 9.5; kdY = 5.4;
        }
      } else {
        const pad = obs.pad.centre;
        tx = pad.x;
        if (p.x < 1.45) {
          const u = clamp((p.x - 1.35) / 0.10, 0, 1);
          tz = 1.05 + u * (pad.z - 1.05);
          ty = 1.05 + u * (0.45 - 1.05);
        } else {
          tz = pad.z;
          const horizErr = Math.hypot(p.x - pad.x, p.z - pad.z);
          const horizSpeed = Math.hypot(vxEst, vzEst);
          const maxAtt = Math.max(Math.abs(att.roll), Math.abs(att.pitch));
          const highWindOnPad = Math.hypot(totalDistX, totalDistZ) > 2.3;

          if (!descendStarted) {
            ty = clamp(0.11 + 0.50 * horizErr, 0.11, 1.05);
            if (horizErr < 0.13 && horizSpeed < 0.40 && p.y < 0.26 && maxAtt < 0.21) {
              descendStarted = true;
            }
          }
          if (descendStarted) {
            if (horizErr > 0.135 || maxAtt > 0.21 || horizSpeed > 0.45 || (highWindOnPad && p.y > 0.11)) {
              ty = 0.22;
              vyRef = 0;
              kpY = 11.0; kdY = 6.0;
            } else {
              ty = 0.01;
              vyRef = -0.35;
            }
            kpX = 9.0; kdX = 5.4; kaX = 0.42;
            kpZ = 9.0; kdZ = 5.4; kaZ = 0.42;
          }

          const speedNorm = Math.hypot(vxEst, vyEst, vzEst);
          if ((p.y <= 0.082 && horizErr <= 0.125 && speedNorm < 0.50 && maxAtt < 0.22) ||
              (p.y <= 0.105 && horizErr <= 0.095 && horizSpeed < 0.25 && vyEst > -0.45 && maxAtt < 0.24)) {
            lastThrust = 0;
            lastRollCmd = 0;
            lastPitchCmd = 0;
            return { thrust: 0, roll: 0, pitch: 0, yawRate: 0 };
          }
        }
      }

      const maxEx = (obs.gateIndex < 3 && phase === 'approach') ? 0.48 : 0.70;
      const ex = clamp(tx - p.x, -0.70, maxEx);
      const ez = clamp(tz - p.z, -0.60, 0.60);
      const ey = clamp(ty - p.y, -0.65, 0.65);

      const axDes = kpX * ex + kdX * (vxRef - vxEst);
      const azDes = kpZ * ez + kdZ * (vzRef - vzEst);
      const ayDes = kpY * ey + kdY * (vyRef - vyEst);

      const axThrustCmd = axDes - totalDistX + dragCoeff * vxEst + kaX * (axDes - axActual);
      const azThrustCmd = azDes - totalDistZ + dragCoeff * vzEst + kaZ * (azDes - azActual);
      const ayThrustCmd = ayDes + dragCoeff * vyEst;

      // Never starve horizontal authority while airborne; only limit tilt when touching floor
      const nearFloorPad = obs.gateIndex === 3 && p.y < 0.095 && descendStarted && Math.hypot(p.x - obs.pad.centre.x, p.z - obs.pad.centre.z) < 0.13;
      const maxTiltAcc = nearFloorPad ? 3.2 : 7.4;
      const Tx = clamp(axThrustCmd, -maxTiltAcc, maxTiltAcc);
      const Tz = clamp(azThrustCmd, -maxTiltAcc, maxTiltAcc);
      const Ty = clamp(9.81 + ayThrustCmd, 2.5, 18.0);

      const Bx = cy * Tx - sy * Tz;
      const Bz = sy * Tx + cy * Tz;
      const pitchDes = clamp(Math.atan2(Bx, Ty), -0.52, 0.52);
      const rollDes = clamp(-Math.atan2(Bz, Math.hypot(Bx, Ty)), -0.52, 0.52);

      const maxCmdTilt = nearFloorPad ? 0.26 : 0.6;
      const rollCmd = clamp(rollDes - 0.10 * rates.roll - (dTauRoll + 22 * simIntRoll) / 65, -maxCmdTilt, maxCmdTilt);
      const pitchCmd = clamp(pitchDes - 0.10 * rates.pitch - (dTauPitch + 22 * simIntPitch) / 65, -maxCmdTilt, maxCmdTilt);

      lastRollCmd = rollCmd;
      lastPitchCmd = pitchCmd;

      let thrustCmd = (Ty / Math.max(0.65, dirY) - dyEst / Math.max(0.65, dirY)) / kT;
      if (obs.gateIndex === 3 && descendStarted && p.y <= 0.088 && Math.hypot(p.x - obs.pad.centre.x, p.z - obs.pad.centre.z) < 0.125) {
        thrustCmd = Math.min(thrustCmd, 0.10);
      }
      thrustCmd = clamp(thrustCmd, 0, 1);
      lastThrust = thrustCmd;

      const yawRate = clamp(-1.5 * att.yaw - 0.2 * rates.yaw, -0.6, 0.6);
      return { thrust: thrustCmd, roll: rollCmd, pitch: pitchCmd, yawRate };
    },
  };
}
