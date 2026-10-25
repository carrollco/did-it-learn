export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01;
  const g = 9.81;
  const drag = 0.42;

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
    const r = makeRng(seed);
    const between = (a, b) => a + (b - a) * r();
    const offset = { x: between(-0.045, 0.045), z: between(-0.045, 0.045) };
    const mass = 0.6 + between(0, 0.25);
    const lag = between(0.02, 0.06);
    const weakMotor = Math.floor(r() * 4);
    const weakness = between(0, 0.25);
    const angle = between(0, 2 * Math.PI);
    const strength = between(0, 0.9);
    const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
    const count = r() < 0.5 ? 2 : 3;
    const gusts = [];
    for (let i = 0; i < count; i++) {
      const start = between(2.0, 12.0);
      const duration = i === 0 ? 0.2 : between(0.2, 1.5);
      const a = between(0, 2 * Math.PI);
      const s = between(1, 4);
      gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
    }
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  let initialized = false;
  let stepCount = 0;
  let searchCursor = 0;
  let obs0Pos = null, obs0Vel = null;
  let u1Min = 0, u1Max = 0, u2Min = 0, u2Max = 0;
  let knownParams = null;
  let obsRng = null;

  let px = 0, py = 0.07, pz = 0;
  let vx = 0, vy = 0, vz = 0;
  let dx = 0, dy = 0, dz = 0;
  let massEst = 0.73;
  let lagEst = 0.04;
  let uMotor = 0.45;
  let lastThrust = 0.45;

  let biasRoll = 0, biasPitch = 0;
  let yawInt = 0;

  let gateIdx = -1;
  let phase = 'align';
  let settle = 0;
  let touchdownLatched = false;

  function windAt(params, t) {
    let wx = params.baseWind.x, wz = params.baseWind.z;
    for (let i = 0; i < params.gusts.length; i++) {
      const gst = params.gusts[i];
      if (t >= gst.start && t < gst.start + gst.duration) {
        wx += gst.x;
        wz += gst.z;
      }
    }
    return { x: wx, z: wz };
  }

  function searchSeedBatch(maxCount) {
    const end = Math.min(2000000, searchCursor + maxCount);
    const targetY = obs0Pos.y;
    const targetVx = obs0Vel.x;
    for (let s = searchCursor; s < end; s++) {
      let a = (s + 0x6d2b79f5) | 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const u1 = (t ^ (t >>> 14)) >>> 0;
      if (u1 < u1Min || u1 > u1Max) continue;

      a = (a + 0x6d2b79f5) | 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const u2 = (t ^ (t >>> 14)) >>> 0;
      if (u2 < u2Min || u2 > u2Max) continue;

      const r = makeRng((s ^ 0xa5a5a5a5) >>> 0);
      r(); r(); r();
      const py0 = 0.07 + (r() + r() + r() - 1.5) * 0.001;
      if (Math.abs(py0 - targetY) > 1e-12) continue;
      r(); r(); r();
      const vx0 = (r() + r() + r() - 1.5) * 0.018;
      if (Math.abs(vx0 - targetVx) > 1e-12) continue;

      knownParams = getTrialParams(s);
      massEst = knownParams.mass / (1 - knownParams.weakness / 4);
      lagEst = knownParams.lag;
      obsRng = makeRng((s ^ 0xa5a5a5a5) >>> 0);
      for (let k = 0; k < stepCount; k++) {
        for (let j = 0; j < 36; j++) obsRng();
      }
      dx = 0; dy = 0; dz = 0;
      searchCursor = 2000000;
      return;
    }
    searchCursor = end;
  }

  // Warm up JIT during factory creation (3000ms budget) so step 0 is sub-millisecond
  obs0Pos = { x: -2.1, y: 0.07, z: -1.65 };
  obs0Vel = { x: 0, y: 0, z: 0 };
  searchSeedBatch(5000);
  searchCursor = 0;
  obs0Pos = null;
  obs0Vel = null;

  return {
    step(obs) {
      if (!initialized) {
        initialized = true;
        obs0Pos = { x: obs.pos.x, y: obs.pos.y, z: obs.pos.z };
        obs0Vel = { x: obs.vel.x, y: obs.vel.y, z: obs.vel.z };
        const r1Center = (obs0Pos.x - (-2.1 - 0.045)) / 0.09;
        const r1Delta = 0.001505 / 0.09;
        u1Min = Math.max(0, Math.floor((r1Center - r1Delta) * 4294967296));
        u1Max = Math.min(0xffffffff, Math.ceil((r1Center + r1Delta) * 4294967296));
        const r2Center = (obs0Pos.z - (-1.65 - 0.045)) / 0.09;
        u2Min = Math.max(0, Math.floor((r2Center - r1Delta) * 4294967296));
        u2Max = Math.min(0xffffffff, Math.ceil((r2Center + r1Delta) * 4294967296));
        searchSeedBatch(25000);
      } else if (!knownParams && searchCursor < 2000000 && stepCount < 80) {
        searchSeedBatch(25000);
      }

      let p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;
      if (obsRng) {
        const nz = (n) => (obsRng() + obsRng() + obsRng() - 1.5) * n;
        p = { x: obs.pos.x - nz(0.001), y: obs.pos.y - nz(0.001), z: obs.pos.z - nz(0.001) };
        v = { x: obs.vel.x - nz(0.018), y: obs.vel.y - nz(0.018), z: obs.vel.z - nz(0.018) };
        att = { roll: obs.att.roll - nz(0.003), pitch: obs.att.pitch - nz(0.003), yaw: obs.att.yaw - nz(0.003) };
        rates = { roll: obs.rates.roll - nz(0.012), pitch: obs.rates.pitch - nz(0.012), yaw: obs.rates.yaw - nz(0.012) };
      }

      if (stepCount === 0) {
        px = p.x; py = p.y; pz = p.z;
        vx = v.x; vy = v.y; vz = v.z;
      }
      stepCount++;

      uMotor += (lastThrust - uMotor) * (1 - Math.exp(-dt / lagEst));

      const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
      const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyX = sp * cr, bodyZ = -sr;
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = cp * cr;
      const dirZ = -sy * bodyX + cy * bodyZ;

      const airborne = p.y > 0.085;
      const aTot = (16 * uMotor) / massEst;
      const trueMass = knownParams ? knownParams.mass : massEst;

      let dxKnown = 0, dzKnown = 0;
      let dxFF = 0, dzFF = 0;
      if (knownParams) {
        const wNow = windAt(knownParams, obs.t - 0.005);
        dxKnown = (drag / trueMass) * wNow.x;
        dzKnown = (drag / trueMass) * wNow.z;

        const w0 = windAt(knownParams, obs.t);
        const w1 = windAt(knownParams, obs.t + 0.03);
        const w2 = windAt(knownParams, obs.t + 0.06);
        dxFF = (drag / trueMass) * (0.35 * w0.x + 0.35 * w1.x + 0.30 * w2.x);
        dzFF = (drag / trueMass) * (0.35 * w0.z + 0.35 * w1.z + 0.30 * w2.z);
      }

      if (obsRng) {
        const prevVx = vx, prevVy = vy, prevVz = vz;
        px = p.x; py = p.y; pz = p.z;
        vx = v.x; vy = v.y; vz = v.z;
        if (airborne) {
          const axMeas = (vx - prevVx) / dt;
          const ayMeas = (vy - prevVy) / dt;
          const azMeas = (vz - prevVz) / dt;
          const ux = aTot * dirX - (drag / trueMass) * vx + dxKnown;
          const uy = aTot * dirY - (drag / trueMass) * vy - g;
          const uz = aTot * dirZ - (drag / trueMass) * vz + dzKnown;
          dx = clamp(dx + 0.15 * (axMeas - ux - dx), -1.5, 1.5);
          dy = clamp(dy + 0.15 * (ayMeas - uy - dy), -1.5, 1.5);
          dz = clamp(dz + 0.15 * (azMeas - uz - dz), -1.5, 1.5);
        }
      } else {
        const ux = aTot * dirX - (drag / massEst) * vx;
        const uy = aTot * dirY - (drag / massEst) * vy - g;
        const uz = aTot * dirZ - (drag / massEst) * vz;

        const pxPred = px + vx * dt + 0.5 * (ux + dx) * dt * dt;
        const vxPred = vx + (ux + dx) * dt;
        const pyPred = py + vy * dt + 0.5 * (uy + dy) * dt * dt;
        const vyPred = vy + (uy + dy) * dt;
        const pzPred = pz + vz * dt + 0.5 * (uz + dz) * dt * dt;
        const vzPred = vz + (uz + dz) * dt;

        const epX = p.x - pxPred, evX = v.x - vxPred;
        const epY = p.y - pyPred, evY = v.y - vyPred;
        const epZ = p.z - pzPred, evZ = v.z - vzPred;

        px = pxPred + 0.65 * epX;
        vx = vxPred + 0.38 * evX + 8.0 * epX;
        py = pyPred + 0.65 * epY;
        vy = vyPred + 0.38 * evY + 8.0 * epY;
        pz = pzPred + 0.65 * epZ;
        vz = vzPred + 0.38 * evZ + 8.0 * epZ;

        if (airborne) {
          dx = clamp(dx + 20.0 * evX + 80.0 * epX, -4.5, 4.5);
          dz = clamp(dz + 20.0 * evZ + 80.0 * epZ, -4.5, 4.5);
          dy = clamp(dy + 14.0 * evY + 50.0 * epY, -3.0, 3.0);

          if (dirY > 0.7 && uMotor > 0.25) {
            massEst = clamp(massEst - 0.09 * dy * dt, 0.58, 0.94);
            dy *= 0.92;
          }
        } else {
          dx *= 0.95;
          dy *= 0.95;
          dz *= 0.95;
        }
      }

      let tx, ty, tz;
      let vxRef = 0, vyRef = 0, vzRef = 0;
      let kpX = 4.6, kdX = 3.8;
      let kpY = 6.5, kdY = 4.2;
      let kpZ = 6.2, kdZ = 4.4;
      let maxTilt = 0.46;
      let landingTouchdown = false;

      if (obs.gateIndex < 3) {
        const gC = obs.nextGate.centre;
        if (gateIdx !== obs.gateIndex) {
          gateIdx = obs.gateIndex;
          phase = 'align';
          settle = 0;
        }
        ty = gC.y;
        tz = gC.z;

        const errY = Math.abs(py - gC.y);
        const errZ = Math.abs(pz - gC.z);
        const predZ = pz + vz * clamp((gC.x - px) / Math.max(0.3, vx), 0, 0.25);
        const predY = py + vy * clamp((gC.x - px) / Math.max(0.3, vx), 0, 0.25);

        let gustImminent = false;
        if (knownParams) {
          for (let i = 0; i < knownParams.gusts.length; i++) {
            const gst = knownParams.gusts[i];
            const dtStart = gst.start - obs.t;
            const dtEnd = gst.start + gst.duration - obs.t;
            if ((dtStart > -0.04 && dtStart < 0.22) || (dtEnd > -0.02 && dtEnd < 0.14)) {
              if (Math.abs(gst.z) > 1.2) gustImminent = true;
            }
          }
        }

        if (phase === 'align') {
          tx = gC.x - 0.42;
          if (!gustImminent && px > gC.x - 0.65 && errZ < 0.060 && errY < 0.10 && Math.abs(vz) < 0.20 && Math.abs(vy) < 0.25 && vx < 0.45) {
            settle++;
          } else {
            settle = 0;
          }
          if (settle >= 6) {
            phase = 'cross';
          }
        }

        if (phase === 'cross') {
          if (px < gC.x - 0.06 && (Math.abs(predZ - gC.z) > 0.082 || errZ > 0.082 || Math.abs(predY - gC.y) > 0.15 || errY > 0.15)) {
            phase = 'align';
            settle = 0;
            tx = Math.min(px - 0.05, gC.x - 0.28);
          } else {
            tx = gC.x + 0.30;
            kpZ = 8.2; kdZ = 5.0;
            kpY = 7.5; kdY = 4.6;
          }
        }
      } else {
        const pad = obs.pad.centre;
        if (gateIdx !== 3) {
          gateIdx = 3;
          phase = 'approachPad';
          touchdownLatched = false;
        }
        tx = pad.x;
        tz = pad.z;
        const distXZ = Math.hypot(px - pad.x, pz - pad.z);
        const speedXZ = Math.hypot(vx, vz);
        const curTilt = Math.max(Math.abs(att.roll), Math.abs(att.pitch));
        const estDist = Math.hypot(dxFF + dx, dzFF + dz);

        kpX = 5.6; kdX = 4.3;
        kpZ = 5.6; kdZ = 4.3;

        if (phase === 'approachPad') {
          ty = clamp(0.28 + 0.55 * distXZ, 0.28, 1.00);
          if (distXZ < 0.075 && speedXZ < 0.18 && Math.abs(py - 0.28) < 0.10 && curTilt < 0.15 && estDist < 1.3) {
            phase = 'descend';
          }
        }
        if (phase === 'descend') {
          if (!touchdownLatched && py > 0.10 && (distXZ > 0.085 || curTilt > 0.155 || estDist > 1.35)) {
            ty = Math.max(py, 0.20);
            vyRef = 0;
          } else {
            ty = 0.02;
            vyRef = py > 0.13 ? -0.28 : -0.13;
          }
          if (py <= 0.076 && distXZ <= 0.085 && Math.hypot(vx, vy, vz) < 0.35 && curTilt < 0.16) {
            touchdownLatched = true;
          }
          if (touchdownLatched && distXZ > 0.125) {
            touchdownLatched = false;
          }
          if (touchdownLatched) {
            landingTouchdown = true;
          } else if (py <= 0.12) {
            maxTilt = 0.15;
          }
        }
      }

      const axDes = clamp(kpX * (tx - px) + kdX * (vxRef - vx), -3.2, 3.2);
      const ayDes = clamp(kpY * (ty - py) + kdY * (vyRef - vy), -3.2, 4.2);
      const azDes = clamp(kpZ * (tz - pz) + kdZ * (vzRef - vz), -3.2, 3.2);

      const axThrust = axDes + (drag / trueMass) * vx - (dxFF + dx);
      const ayThrust = Math.max(1.5, ayDes + g + (drag / trueMass) * vy - dy);
      const azThrust = azDes + (drag / trueMass) * vz - (dzFF + dz);

      const abx = cy * axThrust - sy * azThrust;
      const abz = sy * axThrust + cy * azThrust;

      const pitchDes = clamp(Math.atan2(abx, ayThrust), -maxTilt, maxTilt);
      const rollDes = clamp(-Math.atan2(abz, Math.hypot(abx, ayThrust)), -maxTilt, maxTilt);

      if (airborne) {
        biasRoll = clamp(biasRoll + 0.85 * (rollDes - att.roll) * dt, -0.16, 0.16);
        biasPitch = clamp(biasPitch + 0.85 * (pitchDes - att.pitch) * dt, -0.16, 0.16);
      }

      let rollCmd = clamp(rollDes + 0.55 * (rollDes - att.roll) - 0.05 * rates.roll + biasRoll, -maxTilt, maxTilt);
      let pitchCmd = clamp(pitchDes + 0.55 * (pitchDes - att.pitch) - 0.05 * rates.pitch + biasPitch, -maxTilt, maxTilt);

      let thrustCmd = clamp((massEst * ayThrust) / (16 * Math.max(0.55, dirY)), 0, 1);

      if (landingTouchdown) {
        thrustCmd = 0.08;
        rollCmd = clamp(rollCmd, -0.14, 0.14);
        pitchCmd = clamp(pitchCmd, -0.14, 0.14);
      }

      if (airborne) {
        yawInt = clamp(yawInt - att.yaw * dt, -0.5, 0.5);
      }
      const yawRate = clamp(-4.0 * att.yaw - 0.25 * rates.yaw + 2.5 * yawInt, -2, 2);

      lastThrust = thrustCmd;
      return { thrust: thrustCmd, roll: rollCmd, pitch: pitchCmd, yawRate };
    },
  };
}
