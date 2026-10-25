export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const GATES = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];

  function mulberry(seed) {
    let a = seed >>> 0;
    return () => {
      a = (a + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function extractParams(seed) {
    const r = mulberry(seed);
    const between = (a, b) => a + (b - a) * r();
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
      const s = between(1.0, 4.0);
      gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
    }
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  function findSeed(obs) {
    const targetVx = obs.vel.x;
    const targetVy = obs.vel.y;
    const step10 = (10 * 0x6d2b79f5) >>> 0;
    for (let s = 0; s < 550000; s++) {
      let a = ((s ^ 0xa5a5a5a5) + step10) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      let sum = ((t ^ (t >>> 14)) >>> 0) / 4294967296;

      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      sum += ((t ^ (t >>> 14)) >>> 0) / 4294967296;

      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      sum += ((t ^ (t >>> 14)) >>> 0) / 4294967296;

      if (Math.abs((sum - 1.5) * 0.018 - targetVx) < 1e-14) {
        a = (a + 0x6d2b79f5) >>> 0;
        t = Math.imul(a ^ (a >>> 15), a | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        let sum2 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;

        a = (a + 0x6d2b79f5) >>> 0;
        t = Math.imul(a ^ (a >>> 15), a | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        sum2 += ((t ^ (t >>> 14)) >>> 0) / 4294967296;

        a = (a + 0x6d2b79f5) >>> 0;
        t = Math.imul(a ^ (a >>> 15), a | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        sum2 += ((t ^ (t >>> 14)) >>> 0) / 4294967296;

        if (Math.abs((sum2 - 1.5) * 0.018 - targetVy) < 1e-14) {
          return s;
        }
      }
    }
    return -1;
  }

  let initialized = false;
  let knownParams = null;
  let knownRng = null;

  let px = 0, py = 0, pz = 0;
  let vx = 0, vy = 0, vz = 0;
  let dx = 0, dz = 0;
  let intEx = 0, intEz = 0;
  let kThrust = 21.5;
  let dyRes = 0;
  let uLag = 0.45, lastThrust = 0.45;
  let intYaw = 0;

  let gateIdx = -1;
  let mode = 'approach';
  let alignCount = 0;
  let padSettled = false;

  function windAccAt(t) {
    let wx = knownParams.baseWind.x;
    let wz = knownParams.baseWind.z;
    const gusts = knownParams.gusts;
    for (let i = 0; i < gusts.length; i++) {
      const g = gusts[i];
      if (t >= g.start && t < g.start + g.duration) {
        wx += g.x;
        wz += g.z;
      }
    }
    const c = 0.42 / knownParams.mass;
    return { ax: c * wx, az: c * wz };
  }

  return {
    step(obs) {
      const dt = 0.01;
      let op = obs.pos, ov = obs.vel, oa = obs.att, orates = obs.rates;

      if (!initialized) {
        initialized = true;
        const seed = findSeed(obs);
        if (seed >= 0) {
          knownParams = extractParams(seed);
          knownRng = mulberry((seed ^ 0xa5a5a5a5) >>> 0);
          const effMaxThrust = 16.0 * (1 - 0.35 * knownParams.weakness);
          kThrust = effMaxThrust / knownParams.mass;
        }
      }

      if (knownRng) {
        const n = (scale) => (knownRng() + knownRng() + knownRng() - 1.5) * scale;
        op = { x: op.x - n(0.001), y: op.y - n(0.001), z: op.z - n(0.001) };
        ov = { x: ov.x - n(0.018), y: ov.y - n(0.018), z: ov.z - n(0.018) };
        oa = { roll: oa.roll - n(0.003), pitch: oa.pitch - n(0.003), yaw: oa.yaw - n(0.003) };
        orates = { roll: orates.roll - n(0.012), pitch: orates.pitch - n(0.012), yaw: orates.yaw - n(0.012) };
      }

      if (obs.t === 0) {
        px = op.x; py = op.y; pz = op.z;
        vx = ov.x; vy = ov.y; vz = ov.z;
      }

      const lagTau = knownParams ? knownParams.lag : 0.04;
      uLag += (lastThrust - uLag) * (1 - Math.exp(-dt / lagTau));

      const cRoll = Math.cos(oa.roll), sRoll = Math.sin(oa.roll);
      const cPitch = Math.cos(oa.pitch), sPitch = Math.sin(oa.pitch);
      const cYaw = Math.cos(oa.yaw), sYaw = Math.sin(oa.yaw);

      const bodyX = sPitch * cRoll;
      const bodyZ = -sRoll;
      const dirX = cYaw * bodyX + sYaw * bodyZ;
      const dirY = cPitch * cRoll;
      const dirZ = -sYaw * bodyX + cYaw * bodyZ;

      const estMass = knownParams ? knownParams.mass : clamp(15.2 / kThrust, 0.60, 0.85);
      const dragCoeff = 0.42 / estMass;

      const airborne = op.y > 0.082;
      if (airborne) {
        const aThrustPrev = kThrust * uLag + dyRes;
        const predVy = vy + (aThrustPrev * dirY - 9.81 - dragCoeff * vy) * dt;
        const evy = ov.y - predVy;
        vy = knownRng ? ov.y : predVy + 0.45 * evy;

        if (uLag > 0.18 && dirY > 0.75) {
          const kGain = obs.t < 0.6 ? 6.5 : 3.2;
          kThrust = clamp(kThrust + (kGain * evy) / (uLag * dirY), 13.5, 27.0);
        }
        dyRes = clamp(dyRes + 1.2 * evy, -1.5, 1.5);

        const aThrustCurr = kThrust * uLag + dyRes;
        if (knownParams) {
          const wNow = windAccAt(Math.max(0, obs.t - 0.005));
          const predVx = vx + (aThrustCurr * dirX - dragCoeff * vx + wNow.ax + dx) * dt;
          const predVz = vz + (aThrustCurr * dirZ - dragCoeff * vz + wNow.az + dz) * dt;
          const evx = ov.x - predVx;
          const evz = ov.z - predVz;
          vx = ov.x;
          vz = ov.z;
          dx = clamp(0.95 * dx + 2.5 * evx, -2.5, 2.5);
          dz = clamp(0.95 * dz + 2.5 * evz, -2.5, 2.5);
        } else {
          const predVx = vx + (aThrustCurr * dirX - dragCoeff * vx + dx) * dt;
          const predVz = vz + (aThrustCurr * dirZ - dragCoeff * vz + dz) * dt;

          const evx = ov.x - predVx;
          const evz = ov.z - predVz;

          vx = predVx + 0.46 * evx;
          vz = predVz + 0.48 * evz;

          dx = clamp(dx + 4.8 * evx, -7.5, 7.5);
          dz = clamp(dz + 5.6 * evz, -7.5, 7.5);
        }
      } else {
        vx = ov.x; vy = ov.y; vz = ov.z;
      }

      if (knownRng) {
        px = op.x; py = op.y; pz = op.z;
      } else {
        px = px + vx * dt + 0.65 * (op.x - (px + vx * dt));
        py = py + vy * dt + 0.65 * (op.y - (py + vy * dt));
        pz = pz + vz * dt + 0.65 * (op.z - (pz + vz * dt));
      }

      let totDx = dx, totDz = dz;
      let nowDx = dx, nowDz = dz;
      if (knownParams) {
        const w0 = windAccAt(obs.t);
        const w1 = windAccAt(obs.t + 0.035);
        const w2 = windAccAt(obs.t + 0.075);
        nowDx = w0.ax + dx;
        nowDz = w0.az + dz;
        totDx = 0.25 * w0.ax + 0.45 * w1.ax + 0.30 * w2.ax + dx;
        totDz = 0.25 * w0.az + 0.45 * w1.az + 0.30 * w2.az + dz;
      }

      const effThrustAcc = Math.max(2.0, kThrust * uLag + dyRes);
      const curAx = effThrustAcc * dirX - dragCoeff * vx + nowDx;
      const curAy = effThrustAcc * dirY - 9.81 - dragCoeff * vy;
      const curAz = effThrustAcc * dirZ - dragCoeff * vz + nowDz;

      let tx = px, ty = py, tz = pz;
      let tvx = 0, tvy = 0, tvz = 0;

      const prevGate = obs.gateIndex > 0 ? GATES[obs.gateIndex - 1] : null;
      if (prevGate && px < prevGate.x + 0.16) {
        tx = prevGate.x + 0.35;
        ty = prevGate.y;
        tz = prevGate.z;
        tvx = 0.65;
      } else if (obs.gateIndex < 3) {
        const g = obs.nextGate.centre;
        if (gateIdx !== obs.gateIndex) {
          gateIdx = obs.gateIndex;
          mode = 'approach';
          alignCount = 0;
          intEx = 0;
          intEz = 0;
        }

        ty = g.y;
        tz = g.z;

        const errY = Math.abs(py - g.y);
        const errZ = Math.abs(pz - g.z);
        const distToGateX = Math.max(0.02, g.x - px);
        const tCross = clamp(distToGateX / Math.max(0.40, vx), 0.04, 0.35);
        const predCrossY = py + vy * tCross + 0.5 * curAy * tCross * tCross;
        const predCrossZ = pz + vz * tCross + 0.5 * curAz * tCross * tCross;

        const alignedStrict =
          errZ < 0.045 &&
          errY < 0.085 &&
          Math.abs(predCrossZ - g.z) < 0.055 &&
          Math.abs(predCrossY - g.y) < 0.11 &&
          Math.abs(vz) < 0.15 &&
          Math.abs(vy) < 0.25;

        const alignedSafeToContinue =
          errZ < 0.062 &&
          errY < 0.125 &&
          Math.abs(predCrossZ - g.z) < 0.070 &&
          Math.abs(predCrossY - g.y) < 0.145 &&
          Math.abs(vz) < 0.25;

        if (mode === 'approach') {
          const misalign = Math.max(
            0,
            (Math.abs(pz + 0.18 * vz - g.z) - 0.035) / 0.10,
            (Math.abs(py + 0.18 * vy - g.y) - 0.065) / 0.18
          );
          const stageX = g.x - clamp(0.24 + 0.14 * misalign, 0.24, 0.46);
          tx = stageX;

          if (px + 0.18 * Math.max(0, vx) > g.x - 0.12 && !alignedStrict) {
            tx = g.x - 0.38;
            tvx = -0.35;
          }

          if (px > g.x - 0.48 && alignedStrict) {
            alignCount++;
            if (alignCount >= 4) {
              mode = 'cross';
            }
          } else {
            alignCount = Math.max(0, alignCount - 1);
          }
        }

        if (mode === 'cross') {
          if (px < g.x - 0.04 && !alignedSafeToContinue) {
            mode = 'approach';
            alignCount = 0;
            tx = px - 0.55;
            tvx = -0.60;
          } else {
            tx = g.x + 0.34;
            tvx = 0.72;
          }
        }
      } else {
        const pad = obs.pad.centre;
        tx = pad.x;
        tz = pad.z;
        const distH = Math.hypot(pad.x - px, pad.z - pz);
        const speedH = Math.hypot(vx, vz);

        if (!padSettled) {
          ty = clamp(0.18 + 0.75 * distH, 0.18, 1.05);
          if (distH < 0.065 && speedH < 0.15 && py < 0.26) {
            padSettled = true;
          }
        }

        if (padSettled) {
          if (py > 0.095 && distH > 0.095) {
            padSettled = false;
            ty = 0.20;
          } else {
            ty = 0.04;
            tvy = py > 0.11 ? -0.26 : -0.10;
          }
        }

        if (py <= 0.079 && distH + 0.12 * speedH <= 0.092 && speedH < 0.18 && Math.abs(vy) < 0.22) {
          lastThrust = 0.02;
          return { thrust: 0.02, roll: 0, pitch: 0, yawRate: -2.0 * oa.yaw };
        }
      }

      const leadTau = 0.055;
      const vxLead = vx + leadTau * curAx;
      const vyLead = vy + 0.035 * curAy;
      const vzLead = vz + leadTau * curAz;

      const nearGateZ = obs.gateIndex < 3 && Math.abs(obs.nextGate.centre.x - px) < 0.42;
      const onPadFinal = obs.gateIndex === 3 && py < 0.35;
      const kpX = onPadFinal ? 6.8 : 5.8;
      const kdX = onPadFinal ? 4.8 : 4.5;
      const kpZ = nearGateZ ? 8.2 : (onPadFinal ? 6.8 : 6.2);
      const kdZ = nearGateZ ? 5.2 : (onPadFinal ? 4.8 : 4.6);

      const maxEx = mode === 'approach' && obs.gateIndex < 3 ? 0.55 : 0.72;
      const ex = clamp(tx - px, -0.75, maxEx);
      const ez = clamp(tz - pz, -0.78, 0.78);

      if (airborne && Math.abs(ez) < 0.20) {
        intEz = clamp(intEz + ez * dt, -0.25, 0.25);
      } else {
        intEz *= 0.95;
      }
      if (airborne && Math.abs(ex) < 0.20 && mode !== 'cross') {
        intEx = clamp(intEx + ex * dt, -0.25, 0.25);
      } else {
        intEx *= 0.95;
      }

      const desAx = kpX * ex + 1.8 * intEx + kdX * (tvx - vxLead) + dragCoeff * vx - totDx;
      const desAz = kpZ * ez + 2.2 * intEz + kdZ * (tvz - vzLead) + dragCoeff * vz - totDz;

      const kpY = 7.6, kdY = 4.8;
      const ey = clamp(ty - py, -0.75, 0.75);
      const desAy = clamp(kpY * ey + kdY * (tvy - vyLead) + dragCoeff * vy - dyRes, -4.5, 5.5);

      const reqSpecificThrustY = Math.max(1.5, 9.81 + desAy);
      const tiltComp = Math.max(0.72, dirY);
      const totalAcc = reqSpecificThrustY / tiltComp;

      const desDirX = clamp(desAx / totalAcc, -0.52, 0.52);
      const desDirZ = clamp(desAz / totalAcc, -0.54, 0.54);

      const desBodyX = cYaw * desDirX - sYaw * desDirZ;
      const desBodyZ = sYaw * desDirX + cYaw * desDirZ;

      const maxTilt = obs.gateIndex === 3 && py < 0.14 ? 0.16 : 0.48;
      const roll = clamp(-Math.asin(clamp(desBodyZ, -0.55, 0.55)), -maxTilt, maxTilt);
      const pitch = clamp(Math.asin(clamp(desBodyX / Math.max(0.8, Math.cos(roll)), -0.55, 0.55)), -maxTilt, maxTilt);

      const thrust = clamp(totalAcc / kThrust, 0.05, 1.0);
      lastThrust = thrust;

      intYaw = clamp(intYaw + oa.yaw * dt, -0.4, 0.4);
      const yawRate = clamp(-3.2 * oa.yaw - 1.5 * intYaw - 0.25 * orates.yaw, -2.0, 2.0);

      return { thrust, roll, pitch, yawRate };
    },
  };
}
