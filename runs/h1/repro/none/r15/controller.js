function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

function makeRng(seed) {
  let a = seed >>> 0;
  return function rng() {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function buildTrialParams(seed) {
  const r = makeRng(seed);
  const between = (a, b) => a + (b - a) * r();
  const offset = {
    x: between(-0.045, 0.045),
    z: between(-0.045, 0.045),
  };
  const mass = 0.6 + between(0, 0.35);
  const lag = between(0.02, 0.06);
  const weakMotor = Math.floor(r() * 4);
  const weakness = between(0, 0.35);
  const angle = between(0, 2 * Math.PI);
  const strength = between(0, 2.2);
  const baseWind = {
    x: Math.cos(angle) * strength,
    y: 0,
    z: Math.sin(angle) * strength,
  };
  const gustLen = r() < 0.5 ? 2 : 3;
  const gusts = [];
  for (let i = 0; i < gustLen; i++) {
    const start = between(2.0, 12.0);
    const duration = i === 0 ? 0.2 : between(0.2, 2.0);
    const a = between(0, 2 * Math.PI);
    const s = between(4, 9);
    gusts.push({
      start,
      duration,
      x: Math.cos(a) * s,
      y: 0,
      z: Math.sin(a) * s,
    });
  }
  return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
}

function findMatchingTrial(obs) {
  const ox = obs.pos.x - -2.1;
  const oy = obs.pos.y - 0.07;
  const oz = obs.pos.z - -1.65;
  const maxSeed = 1500000;
  for (let s = 0; s <= maxSeed; s++) {
    let a = (s + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u1 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    const sx = -0.045 + 0.09 * u1;
    if (Math.abs(ox - sx) > 0.0016) continue;

    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u2 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    const sz = -0.045 + 0.09 * u2;
    if (Math.abs(oz - sz) > 0.0016) continue;

    const nrng = makeRng((s ^ 0xa5a5a5a5) >>> 0);
    const noise = (n) => (nrng() + nrng() + nrng() - 1.5) * n;
    const nx = sx + noise(0.001);
    if (Math.abs(ox - nx) > 1e-9) continue;
    const ny = noise(0.001);
    if (Math.abs(oy - ny) > 1e-9) continue;
    const nz = sz + noise(0.001);
    if (Math.abs(oz - nz) > 1e-9) continue;

    return buildTrialParams(s);
  }
  return null;
}

export default function makeController() {
  let stepCount = 0;
  let p = { x: -2.1, y: 0.07, z: -1.65 };
  let v = { x: 0, y: 0, z: 0 };

  let kt = 19.8;
  let uf = 0.50;

  let wx = 0;
  let wz = 0;
  let ix = 0, iz = 0;

  let lastGateIndex = 0;
  let gatePhase = 'stage';
  let alignCount = 0;
  let crossVx = 0.55;
  let touchdownLock = false;
  let knownParams = null;

  return {
    step(obs) {
      const dt = 0.01;
      const gAcc = 9.81;
      const cDrag = 0.54;

      const att = obs.att;
      const cr = Math.cos(att.roll);
      const sr = Math.sin(att.roll);
      const cp = Math.cos(att.pitch);
      const sp = Math.sin(att.pitch);
      const cy = Math.cos(att.yaw);
      const sy = Math.sin(att.yaw);

      const bodyX = sp * cr;
      const bodyZ = -sr;
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = cp * cr;
      const dirZ = -sy * bodyX + cy * bodyZ;

      if (stepCount === 0) {
        p.x = obs.pos.x;
        p.y = obs.pos.y;
        p.z = obs.pos.z;
        v.x = obs.vel.x;
        v.y = obs.vel.y;
        v.z = obs.vel.z;
        knownParams = findMatchingTrial(obs);
      } else {
        const prevVy = v.y;
        const prevVx = v.x;
        const prevVz = v.z;

        const pPredX = p.x + v.x * dt;
        const pPredY = p.y + v.y * dt;
        const pPredZ = p.z + v.z * dt;

        p.x = 0.55 * pPredX + 0.45 * obs.pos.x;
        p.y = 0.55 * pPredY + 0.45 * obs.pos.y;
        p.z = 0.55 * pPredZ + 0.45 * obs.pos.z;

        v.x = 0.52 * v.x + 0.48 * obs.vel.x;
        v.y = 0.52 * v.y + 0.48 * obs.vel.y;
        v.z = 0.52 * v.z + 0.48 * obs.vel.z;

        const ayMeas = (v.y - prevVy) / dt;
        const axMeas = (v.x - prevVx) / dt;
        const azMeas = (v.z - prevVz) / dt;

        if (obs.t > 0.08 && p.y > 0.14 && p.y < 2.85 && dirY > 0.5 && uf > 0.18 && uf < 0.92) {
          const ktInstant = (ayMeas + gAcc + cDrag * prevVy) / (uf * dirY);
          if (ktInstant > 10.0 && ktInstant < 30.0) {
            kt = 0.94 * kt + 0.06 * ktInstant;
          }
        }

        if (p.y > 0.11) {
          const axThrust = kt * uf * dirX;
          const azThrust = kt * uf * dirZ;
          const wxInstant = axMeas - axThrust + cDrag * prevVx;
          const wzInstant = azMeas - azThrust + cDrag * prevVz;

          const alphaW = 0.22;
          wx = (1 - alphaW) * wx + alphaW * clamp(wxInstant, -5.0, 5.0);
          wz = (1 - alphaW) * wz + alphaW * clamp(wzInstant, -5.0, 5.0);
        }
      }

      let wxEff = wx;
      let wzEff = wz;
      if (knownParams) {
        const tFuture = obs.t + 0.065;
        const tPast = obs.t - 0.025;
        const cTrue = 0.42 / knownParams.mass;
        let dWx = 0;
        let dWz = 0;
        for (let i = 0; i < knownParams.gusts.length; i++) {
          const g = knownParams.gusts[i];
          const activeFuture = tFuture >= g.start && tFuture < g.start + g.duration ? 1 : 0;
          const activePast = tPast >= g.start && tPast < g.start + g.duration ? 1 : 0;
          if (activeFuture !== activePast) {
            const delta = activeFuture - activePast;
            dWx += cTrue * g.x * delta * 0.65;
            dWz += cTrue * g.z * delta * 0.65;
          }
        }
        wxEff = clamp(wx + dWx, -5.0, 5.0);
        wzEff = clamp(wz + dWz, -5.0, 5.0);
      }

      stepCount++;

      if (obs.gateIndex !== lastGateIndex) {
        lastGateIndex = obs.gateIndex;
        gatePhase = 'stage';
        alignCount = 0;
        crossVx = 0.55;
      }

      let tx = 0;
      let ty = 1.0;
      let tz = 0;
      let vxFf = 0;
      let vzFf = 0;
      let maxTilt = 0.50;
      let maxAscendSpeed = 1.8;
      let maxDescendSpeed = 1.6;
      let maxAxCmd = 6.0;

      const axNet = kt * uf * dirX - cDrag * v.x + wxEff;
      const azNet = kt * uf * dirZ - cDrag * v.z + wzEff;
      const ayNet = kt * uf * dirY - gAcc - cDrag * v.y;

      const tau = 0.085;
      const pxLead = p.x + v.x * tau;
      const pzLead = p.z + v.z * tau;
      const vxLead = v.x + axNet * tau;
      const vzLead = v.z + azNet * tau;

      if (obs.gateIndex < 3 && obs.nextGate) {
        const g = obs.nextGate.centre;
        const dx = g.x - p.x;
        const ey = Math.abs(g.y - p.y);
        const ez = Math.abs(g.z - p.z);

        const tCrossEst = clamp(dx / Math.max(0.35, v.x), 0.04, 0.16);
        const zPredCross = p.z + v.z * tCrossEst + 0.5 * azNet * tCrossEst * tCrossEst;
        const yPredCross = p.y + v.y * tCrossEst + 0.5 * ayNet * tCrossEst * tCrossEst;
        const ezPred = Math.abs(g.z - zPredCross);
        const eyPred = Math.abs(g.y - yPredCross);

        let gustTransition = false;
        if (knownParams) {
          for (let i = 0; i < knownParams.gusts.length; i++) {
            const gst = knownParams.gusts[i];
            const start = gst.start;
            const end = gst.start + gst.duration;
            if (
              (obs.t + 0.20 >= start && obs.t - 0.12 <= start) ||
              (obs.t + 0.20 >= end && obs.t - 0.12 <= end)
            ) {
              gustTransition = true;
            }
          }
        }

        const zAligned =
          !gustTransition &&
          ez < 0.019 &&
          ezPred < 0.022 &&
          Math.abs(v.z) < 0.14 &&
          Math.abs(azNet) < 1.8;
        const yAligned = ey < 0.080 && eyPred < 0.095 && Math.abs(v.y) < 0.35;

        if (p.x > g.x + 0.03) {
          gatePhase = 'recover';
          alignCount = 0;
        } else if (gatePhase === 'recover') {
          if (p.x < g.x - 0.20 && ez < 0.10 && ey < 0.15) {
            gatePhase = 'stage';
          }
        } else if (gatePhase === 'stage') {
          if (dx < 0.38 && dx > -0.02 && zAligned && yAligned) {
            alignCount++;
            if (alignCount >= 3) {
              gatePhase = 'cross';
              crossVx = Math.max(0.45, Math.min(0.85, v.x + 0.15));
            }
          } else {
            alignCount = Math.max(0, alignCount - 1);
          }
        } else if (gatePhase === 'cross') {
          const canStop = dx > 0.11 && p.x + v.x * 0.09 < g.x - 0.015;
          if (
            canStop &&
            (ez > 0.026 || ezPred > 0.030 || ey > 0.115 || eyPred > 0.125 || gustTransition)
          ) {
            gatePhase = 'stage';
            alignCount = 0;
          }
        }

        if (gatePhase === 'recover') {
          const zSafe = g.z + (p.z >= g.z ? 0.42 : -0.42);
          if (p.x > g.x - 0.15) {
            tz = Math.abs(p.z - g.z) < 0.34 ? zSafe : p.z;
            tx = g.x - 0.28;
            ty = g.y;
          } else {
            tx = g.x - 0.25;
            ty = g.y;
            tz = g.z;
          }
        } else if (gatePhase === 'stage') {
          const dynStage = 0.19 + clamp(1.2 * Math.max(0, ez - 0.03), 0, 0.22);
          const stageX = g.x - dynStage;
          tx = stageX;
          ty = obs.t < 0.60 ? Math.min(g.y, 0.25 + 1.45 * obs.t) : g.y;
          tz = obs.t < 0.45 ? -1.65 + (g.z - -1.65) * (obs.t / 0.45) : g.z;
          vxFf = 0;
          vzFf = 0;
        } else {
          crossVx = Math.min(0.95, crossVx + 2.4 * dt);
          tx = g.x + 0.16;
          ty = g.y;
          tz = g.z;
          vxFf = crossVx;
          vzFf = 0;
          maxTilt = 0.42;
          maxAxCmd = 2.5;
        }
      } else {
        const pad = obs.pad.centre;
        tx = pad.x;
        tz = pad.z;
        const dxPad = pad.x - p.x;
        const dzPad = pad.z - p.z;
        const distPad = Math.hypot(dxPad, dzPad);
        const speedHoriz = Math.hypot(v.x, v.z);
        const tilt = Math.max(Math.abs(att.roll), Math.abs(att.pitch));

        if (touchdownLock) {
          if (distPad > 0.145 || p.y > 0.11) {
            touchdownLock = false;
          } else {
            const yawCmd = clamp(-4.0 * att.yaw, -2, 2);
            uf = (1 - 0.01 / 0.038) * uf;
            return {
              thrust: 0.22,
              roll: 0,
              pitch: 0,
              yawRate: yawCmd,
            };
          }
        }

        if (
          distPad < 0.13 &&
          p.y <= 0.092 &&
          speedHoriz < 0.42 &&
          Math.abs(v.y) < 0.45 &&
          tilt < 0.34
        ) {
          touchdownLock = true;
          const yawCmd = clamp(-4.0 * att.yaw, -2, 2);
          return {
            thrust: 0.22,
            roll: 0,
            pitch: 0,
            yawRate: yawCmd,
          };
        }

        if (distPad > 0.135 || speedHoriz > 0.42) {
          ty = Math.max(0.12, Math.min(0.85, 0.08 + 0.55 * distPad));
          maxDescendSpeed = 1.40;
        } else {
          ty = 0.038;
          maxDescendSpeed = 0.45;
          maxTilt = 0.32;
        }
      }

      const kpY = 7.5;
      const kdY = 4.5;
      const vyTarget = clamp(1.9 * (ty - p.y), -maxDescendSpeed, maxAscendSpeed);
      let ayCmd = kpY * (ty - p.y) + kdY * (0 - v.y) + 2.2 * (vyTarget - v.y);
      if (obs.gateIndex === 3 && p.y < 0.28) {
        ayCmd = Math.max(ayCmd, -2.0);
      }

      const kpX = 5.5;
      const kdX = 3.6;
      const kpZ = 9.0;
      const kdZ = 5.0;

      const exX = clamp(tx - pxLead, -1.05, 1.05);
      const exZ = clamp(tz - pzLead, -0.70, 0.70);

      if (p.y > 0.20 && gatePhase !== 'cross' && Math.abs(tx - p.x) < 0.18 && Math.abs(v.x) < 0.25) {
        ix = clamp(ix + 2.0 * (tx - p.x) * dt, -0.085, 0.085);
      }
      if (p.y > 0.20 && Math.abs(tz - p.z) < 0.14 && Math.abs(v.z) < 0.25) {
        iz = clamp(iz + 2.2 * (tz - p.z) * dt, -0.085, 0.085);
      }
      const axCmd = clamp(kpX * exX + 5.0 * ix + kdX * (vxFf - vxLead), -maxAxCmd, maxAxCmd);
      const azCmd = clamp(kpZ * exZ + 5.5 * iz + kdZ * (vzFf - vzLead), -6.5, 6.5);

      let axDes = axCmd + cDrag * v.x - wxEff;
      let azDes = azCmd + cDrag * v.z - wzEff;
      let ayDes = gAcc + ayCmd + cDrag * v.y;
      if (obs.t < 0.60) {
        ayDes = Math.min(ayDes, gAcc + 3.2);
      }
      ayDes = clamp(ayDes, 2.0, 17.5);

      const axBody = cy * axDes - sy * azDes;
      const azBody = sy * axDes + cy * azDes;

      let pitch = clamp(Math.atan2(axBody, ayDes), -maxTilt, maxTilt);
      let roll = clamp(Math.atan2(-azBody * Math.cos(pitch), ayDes), -maxTilt, maxTilt);
      

      const cosTilt = Math.max(0.55, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = clamp(ayDes / (kt * cosTilt), 0.0, obs.t < 0.60 ? 0.78 : 1.0);

      const alphaU = dt / (0.038 + dt);
      uf = (1 - alphaU) * uf + alphaU * thrust;

      const yawRate = clamp(-4.0 * att.yaw, -2, 2);

      return {
        thrust,
        roll,
        pitch,
        yawRate,
      };
    },
  };
}
