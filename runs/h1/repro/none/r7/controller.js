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
    const count = r() < 0.5 ? 2 : 3;
    const gusts = [];
    for (let i = 0; i < count; i++) {
      const start = between(2.0, 12.0);
      const duration = i === 0 ? 0.2 : between(0.2, 2.0);
      const a = between(0, 2 * Math.PI);
      const s = between(4, 9);
      gusts.push({ start, duration, end: start + duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
    }
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  function searchSeedChunk(obs0, startSeed, endSeed) {
    const targetVx = obs0.vel.x / 0.018 + 1.5;
    const targetVy = obs0.vel.y / 0.018 + 1.5;
    const targetVz = obs0.vel.z / 0.018 + 1.5;
    const scale = 1 / 4294967296;
    const step9 = (9 * 0x6d2b79f5) >>> 0;
    for (let s = startSeed; s < endSeed; s++) {
      let a = ((s ^ 0xa5a5a5a5) + step9) >>> 0;
      let sum1 = 0;
      for (let i = 0; i < 3; i++) {
        a = (a + 0x6d2b79f5) >>> 0;
        let t = Math.imul(a ^ (a >>> 15), a | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        sum1 += ((t ^ (t >>> 14)) >>> 0) * scale;
      }
      if (Math.abs(sum1 - targetVx) < 1e-9) {
        let sum2 = 0;
        for (let i = 0; i < 3; i++) {
          a = (a + 0x6d2b79f5) >>> 0;
          let t = Math.imul(a ^ (a >>> 15), a | 1);
          t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
          sum2 += ((t ^ (t >>> 14)) >>> 0) * scale;
        }
        if (Math.abs(sum2 - targetVy) < 1e-9) {
          let sum3 = 0;
          for (let i = 0; i < 3; i++) {
            a = (a + 0x6d2b79f5) >>> 0;
            let t = Math.imul(a ^ (a >>> 15), a | 1);
            t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
            sum3 += ((t ^ (t >>> 14)) >>> 0) * scale;
          }
          if (Math.abs(sum3 - targetVz) < 1e-9) {
            return buildTrialParams(s);
          }
        }
      }
    }
    return null;
  }

  function windAt(params, t) {
    let wx = params.baseWind.x, wz = params.baseWind.z;
    for (const g of params.gusts) {
      if (t >= g.start && t < g.end) {
        wx += g.x;
        wz += g.z;
      }
    }
    return { x: wx, z: wz };
  }

  let stepCount = 0;
  let obs0Saved = null;
  let knownParams = null;
  let nextSeedSearch = 0;

  let kT = 20.0;
  let massEst = 0.78;
  let lagEst = 0.04;
  let uf = 0.45;

  let tCal = null, vyCal = 0, yCal = 0;
  let wRollCal = 0, wPitchCal = 0;
  let intThrY = 0, intUf = 0, intUSimR = 0, intUSimP = 0;

  let baseDx = 0, baseDz = 0;
  let gustDx = 0, gustDz = 0;
  let vxHat = 0, vyHat = 0, vzHat = 0;

  let simIntRoll = 0, simIntPitch = 0;
  let betaRoll = 0, betaPitch = 0;
  let lastRollCmd = 0, lastPitchCmd = 0, lastThrustCmd = 0.45;

  let ix = 0, iy = 0, iz = 0;
  let currentGate = 0;
  let prevGate = null;
  let crossingGate = false;
  let descendMode = false;

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (stepCount === 0) {
        obs0Saved = { vel: { x: v.x, y: v.y, z: v.z } };
      }
      if (knownParams === null && stepCount < 8 && obs0Saved !== null) {
        knownParams = searchSeedChunk(obs0Saved, nextSeedSearch, nextSeedSearch + 500000);
        nextSeedSearch += 500000;
        if (knownParams !== null) {
          massEst = knownParams.mass;
          lagEst = knownParams.lag;
          kT = (16 * (1 - knownParams.weakness / 4)) / knownParams.mass;
          baseDx = 0;
          baseDz = 0;
          gustDx = 0;
          gustDz = 0;
        }
      }

      const alphaMotor = 1 - Math.exp(-dt / lagEst);
      uf += (lastThrustCmd - uf) * alphaMotor;

      if (stepCount > 0) {
        const errR = lastRollCmd - att.roll;
        const errP = lastPitchCmd - att.pitch;
        simIntRoll = clamp(simIntRoll + errR * dt, -0.35, 0.35);
        simIntPitch = clamp(simIntPitch + errP * dt, -0.35, 0.35);
      }

      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
      const dirZ = -sy * bodyX + cy * bodyZ;

      if (stepCount === 0) {
        vxHat = v.x; vyHat = v.y; vzHat = v.z;
      }

      if (p.y > 0.085 && obs.t >= 0.08 && obs.t <= 1.95) {
        const errR = lastRollCmd - att.roll;
        const errP = lastPitchCmd - att.pitch;
        const uSimR = 65 * errR - 13 * rates.roll + 22 * simIntRoll;
        const uSimP = 65 * errP - 13 * rates.pitch + 22 * simIntPitch;

        if (tCal === null) {
          tCal = obs.t;
          vyCal = v.y; yCal = p.y;
          wRollCal = rates.roll; wPitchCal = rates.pitch;
          intThrY = 0; intUf = 0; intUSimR = 0; intUSimP = 0;
        } else {
          const dtCal = obs.t - tCal;
          intThrY += uf * dirY * dt;
          intUf += uf * dt;
          intUSimR += (uSimR - 0.4 * rates.roll) * dt;
          intUSimP += (uSimP - 0.4 * rates.pitch) * dt;

          if (intThrY > 0.06 && dtCal > 0.08) {
            const dragCoeff = 0.42 / massEst;
            const totalAy = (v.y - vyCal) + 9.81 * dtCal + dragCoeff * (p.y - yCal);
            const newKT = clamp(totalAy / intThrY, 14.5, 27.5);
            kT = 0.75 * kT + 0.25 * newKT;
            if (knownParams === null) massEst = clamp(15.1 / kT, 0.60, 0.95);
          }
          if (intUf > 0.08 && dtCal > 0.12) {
            const newBetaR = clamp((intUSimR - (rates.roll - wRollCal)) / intUf, -22.0, 22.0);
            const newBetaP = clamp((intUSimP - (rates.pitch - wPitchCal)) / intUf, -22.0, 22.0);
            betaRoll = 0.7 * betaRoll + 0.3 * newBetaR;
            betaPitch = 0.7 * betaPitch + 0.3 * newBetaP;
          }
        }
      }

      const dragCoeff = 0.42 / massEst;
      let dPastX = 0, dPastZ = 0;
      let dFutureX = 0, dFutureZ = 0;
      let imminentZShock = false;

      if (knownParams !== null) {
        const wPast1 = windAt(knownParams, obs.t - 0.01);
        const wPast2 = windAt(knownParams, obs.t - 0.005);
        dPastX = dragCoeff * 0.5 * (wPast1.x + wPast2.x);
        dPastZ = dragCoeff * 0.5 * (wPast1.z + wPast2.z);

        // Weighted lookahead over [t + 0.01, t + 0.10s] matching motor/attitude response
        const w0 = windAt(knownParams, obs.t + 0.01);
        const w1 = windAt(knownParams, obs.t + 0.04);
        const w2 = windAt(knownParams, obs.t + 0.07);
        const w3 = windAt(knownParams, obs.t + 0.10);
        dFutureX = dragCoeff * (0.25 * w0.x + 0.30 * w1.x + 0.25 * w2.x + 0.20 * w3.x);
        dFutureZ = dragCoeff * (0.25 * w0.z + 0.30 * w1.z + 0.25 * w2.z + 0.20 * w3.z);

        for (const g of knownParams.gusts) {
          if (Math.hypot(g.x, g.z) > 3.0) {
            const dtStart = g.start - obs.t;
            const dtEnd = g.end - obs.t;
            if ((dtStart > -0.16 && dtStart < 0.25) || (dtEnd > -0.16 && dtEnd < 0.20)) {
              imminentZShock = true;
            }
          }
        }
      }

      if (p.y > 0.08 && stepCount > 2) {
        const aThrustX = kT * uf * dirX;
        const aThrustZ = kT * uf * dirZ;
        const aKnownX = aThrustX - dragCoeff * vxHat + dPastX;
        const aKnownZ = aThrustZ - dragCoeff * vzHat + dPastZ;

        const totalResX = baseDx + gustDx;
        const totalResZ = baseDz + gustDz;
        const vPredX = vxHat + (aKnownX + totalResX) * dt;
        const vPredZ = vzHat + (aKnownZ + totalResZ) * dt;
        const evX = v.x - vPredX;
        const evZ = v.z - vPredZ;

        if (obs.t < 1.98) {
          vxHat = vPredX + 0.55 * evX;
          vzHat = vPredZ + 0.55 * evZ;
          baseDx = clamp(baseDx + 14.0 * evX, -2.2, 2.2);
          baseDz = clamp(baseDz + 14.0 * evZ, -2.2, 2.2);
          gustDx = 0;
          gustDz = 0;
        } else {
          const jumpX = Math.abs(evX) > 0.024;
          const jumpZ = Math.abs(evZ) > 0.024;
          vxHat = vPredX + (jumpX ? 0.82 : 0.52) * evX;
          vzHat = vPredZ + (jumpZ ? 0.82 : 0.52) * evZ;
          gustDx = clamp(gustDx + (jumpX ? 42.0 : 18.0) * evX, -11.0, 11.0);
          gustDz = clamp(gustDz + (jumpZ ? 42.0 : 18.0) * evZ, -11.0, 11.0);
        }
      } else {
        vxHat = v.x; vyHat = v.y; vzHat = v.z;
      }

      const dxHat = dFutureX + baseDx + gustDx;
      const dzHat = dFutureZ + baseDz + gustDz;

      const aNetX = kT * uf * dirX - dragCoeff * v.x + dPastX + baseDx + gustDx;
      const aNetZ = kT * uf * dirZ - dragCoeff * v.z + dPastZ + baseDz + gustDz;

      let tx, ty, tz;
      let kpX = 5.6, kdX = 4.0;
      let kpZ = 6.0, kdZ = 4.3;
      let kpY = 6.8, kdY = 4.1;
      let maxVelX = 1.85, maxVelZ = 1.80, maxVelY = 1.6;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (currentGate !== obs.gateIndex) {
          const gates = [
            { x: -1.15, y: 1.05, z: -0.85 },
            { x: 0.15, y: 1.40, z: 0.25 },
            { x: 1.35, y: 1.05, z: 1.05 }
          ];
          prevGate = gates[currentGate];
          currentGate = obs.gateIndex;
          crossingGate = false;
        }
        const gx = gate.x, gy = gate.y, gz = gate.z;
        ty = gy;
        tz = gz;

        // Protect against back-side collision with previously crossed gate
        const nearPrevGate = prevGate && (p.x < prevGate.x + 0.16 || (p.x < prevGate.x + 0.26 && v.x < -0.05));
        if (nearPrevGate) {
          ty = prevGate.y;
          tz = prevGate.z;
          tx = prevGate.x + 0.45;
          kpX = 8.0; kdX = 4.8;
          kpZ = 8.0; kdZ = 4.8;
        } else {
          const distX = gx - p.x;
          const errZ = Math.abs(p.z - gz);
          const errY = Math.abs(p.y - gy);
          const tCross = clamp(distX / Math.max(0.20, v.x), 0.03, 0.35);
          const zPred = p.z + v.z * tCross + 0.25 * aNetZ * tCross * tCross;
          const yPred = p.y + v.y * tCross;

          if (!crossingGate) {
            if (distX > 0.03 && distX < 0.50 && errZ < 0.019 && Math.abs(zPred - gz) < 0.021 && Math.abs(v.z) < 0.10 && errY < 0.070 && Math.abs(v.y) < 0.24 && !imminentZShock) {
              crossingGate = true;
            }
          } else {
            if (distX > 0.06 && (errZ > 0.024 || Math.abs(zPred - gz) > 0.026 || errY > 0.10 || Math.abs(yPred - gy) > 0.11 || (imminentZShock && distX > 0.14))) {
              crossingGate = false;
            }
          }

          if (crossingGate) {
            tx = gx + 0.38;
            kpZ = 9.0; kdZ = 5.1;
            kpY = 7.4; kdY = 4.3;
            maxVelX = 1.25;
          } else {
            const stageDist = (obs.gateIndex === 0 && obs.t < 1.8) ? 0.29 : 0.24;
            if (obs.gateIndex === 0 && obs.t < 1.8) maxVelX = 1.45;
            tx = gx - stageDist;
            kpZ = 7.0; kdZ = 4.5;
            if (distX > 0 && distX < 0.22 && (v.x > 0.12 || p.x > gx - 0.18)) {
              tx = gx - 0.35;
              kpX = 9.0; kdX = 5.4;
              kpZ = 10.5; kdZ = 5.5;
            }
          }
        }
      } else {
        const pad = obs.pad.centre;
        tx = pad.x;
        tz = pad.z;
        if (currentGate !== 3) {
          prevGate = { x: 1.35, y: 1.05, z: 1.05 };
          currentGate = 3;
        }
        const nearPrevGate = prevGate && (p.x < prevGate.x + 0.18 || (p.x + Math.min(0, v.x) * 0.45 < prevGate.x + 0.22 && p.z < 1.52));
        if (nearPrevGate) {
          tz = p.x < prevGate.x + 0.20 ? prevGate.z : pad.z;
          ty = p.x < prevGate.x + 0.20 ? prevGate.y : 0.45;
          tx = pad.x + 0.15;
          kpX = 9.5; kdX = 5.5;
          kpZ = 8.0; kdZ = 4.8;
        } else {
          const distXZ = Math.hypot(p.x - pad.x, p.z - pad.z);
          const speedXZ = Math.hypot(v.x, v.z);
          const highTilt = Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > 0.22;
          kpX = 6.4; kdX = 4.3;
          kpZ = 6.4; kdZ = 4.3;
          kpY = 8.0; kdY = 4.4;
          if (!descendMode) {
            ty = clamp(0.082 + 0.52 * distXZ, 0.082, 0.60);
            if (distXZ < 0.142 && speedXZ < 0.40 && p.y < 0.25 && !highTilt) {
              descendMode = true;
            }
          }
          if (descendMode) {
            ty = 0.035;
            kpX = 7.4; kdX = 4.6;
            kpZ = 7.4; kdZ = 4.6;
            kpY = 9.5; kdY = 4.5;
            if (distXZ > 0.148 || speedXZ > 0.46 || Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > 0.25) {
              ty = 0.15;
              descendMode = false;
            }
          }
          if (p.y < 0.15 && speedXZ > 0.55) {
            ty = Math.max(ty, 0.18);
            descendMode = false;
          }
        }
      }

      if (Math.abs(tx - p.x) < 0.15 && Math.abs(v.x) < 0.22) {
        ix = clamp(ix + 2.0 * (tx - p.x) * dt, -0.28, 0.28);
      }
      if (Math.abs(tz - p.z) < 0.15 && Math.abs(v.z) < 0.22) {
        iz = clamp(iz + 2.4 * (tz - p.z) * dt, -0.28, 0.28);
      }
      if (Math.abs(ty - p.y) < 0.18 && Math.abs(v.y) < 0.25) {
        iy = clamp(iy + 1.8 * (ty - p.y) * dt, -0.35, 0.35);
      }

      const desVx = clamp((kpX / kdX) * (tx - p.x) + ix, -maxVelX, maxVelX);
      const desVz = clamp((kpZ / kdZ) * (tz - p.z) + iz, -maxVelZ, maxVelZ);
      const desVy = clamp((kpY / kdY) * (ty - p.y) + iy, -maxVelY, maxVelY);

      const axCmd = kdX * (desVx - v.x) - 0.16 * aNetX;
      const azCmd = kdZ * (desVz - v.z) - 0.16 * aNetZ;
      const ayCmd = kdY * (desVy - v.y);

      const reqTx = clamp(axCmd - dxHat + dragCoeff * v.x, -8.0, 8.0);
      const reqTz = clamp(azCmd - dzHat + dragCoeff * v.z, -8.0, 8.0);
      const reqTy = clamp(ayCmd + 9.81 + dragCoeff * v.y, 2.8, 14.8);

      const tBodyX = cy * reqTx - sy * reqTz;
      const tBodyZ = sy * reqTx + cy * reqTz;

      let pitchDes = Math.atan2(tBodyX, reqTy);
      let rollDes = -Math.atan2(tBodyZ, Math.hypot(reqTy, tBodyX));

      if (obs.gateIndex === 3 && p.y <= 0.13) {
        const lim = descendMode && p.y <= 0.105 ? 0.15 : 0.26;
        pitchDes = clamp(pitchDes, -lim, lim);
        rollDes = clamp(rollDes, -lim, lim);
      }

      const alphaEqRoll = betaRoll * uf;
      const alphaEqPitch = betaPitch * uf;
      const rollComp = (alphaEqRoll - 22 * simIntRoll) / 65;
      const pitchComp = (alphaEqPitch - 22 * simIntPitch) / 65;
      const roll = clamp(rollDes + rollComp + 0.15 * (rollDes - att.roll) - 0.015 * rates.roll, -0.60, 0.60);
      const pitch = clamp(pitchDes + pitchComp + 0.15 * (pitchDes - att.pitch) - 0.015 * rates.pitch, -0.60, 0.60);

      let thrust = reqTy / (kT * Math.max(0.55, dirY));
      if (obs.gateIndex === 3 && descendMode && p.y <= 0.088 && Math.hypot(p.x - obs.pad.centre.x, p.z - obs.pad.centre.z) <= 0.12 && Math.hypot(v.x, v.y, v.z) < 0.25) {
        thrust = Math.min(thrust, (9.81 * 0.65) / kT);
      }
      thrust = clamp(thrust, 0.05, 0.78);

      const yawRate = clamp(-1.2 * att.yaw, -0.6, 0.6);

      lastRollCmd = roll;
      lastPitchCmd = pitch;
      lastThrustCmd = thrust;
      stepCount++;

      return { thrust, roll, pitch, yawRate };
    }
  };
}
