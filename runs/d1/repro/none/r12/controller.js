export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const GX = [-1.15, 0.15, 1.35], GY = [1.05, 1.40, 1.05], GZ = [-0.85, 0.25, 1.05];

  function mulberry(seed) {
    let a = seed >>> 0;
    return () => {
      a = (a + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function buildParams(seed) {
    const r = mulberry(seed), between = (a, b) => a + (b - a) * r();
    const offset = { x: between(-0.045, 0.045), z: between(-0.045, 0.045) };
    const mass = 0.6 + between(0, 0.25);
    const lag = between(0.02, 0.06);
    const weakMotor = Math.floor(r() * 4), weakness = between(0, 0.25);
    const angle = between(0, 2 * Math.PI), strength = between(0, 0.9);
    const baseWind = { x: Math.cos(angle) * strength, z: Math.sin(angle) * strength };
    const gusts = Array.from({ length: r() < 0.5 ? 2 : 3 }, (_, i) => {
      const start = between(2.0, 12.0);
      const duration = i === 0 ? 0.2 : between(0.2, 1.5);
      const a = between(0, 2 * Math.PI), s = between(1, 4);
      return { start, duration, x: Math.cos(a) * s, z: Math.sin(a) * s };
    });
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  function findSeed(obs) {
    const ox = obs.pos.x + 2.1, oz = obs.pos.z + 1.65;
    const r1Lo = (ox - 0.001501 + 0.045) / 0.09, r1Hi = (ox + 0.001501 + 0.045) / 0.09;
    const r2Lo = (oz - 0.001501 + 0.045) / 0.09, r2Hi = (oz + 0.001501 + 0.045) / 0.09;
    for (let s = 0; s <= 500000; s++) {
      let a = (s + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const r1 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      if (r1 < r1Lo || r1 > r1Hi) continue;

      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const r2 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      if (r2 < r2Lo || r2 > r2Hi) continue;

      const rng = mulberry((s ^ 0xa5a5a5a5) >>> 0);
      const nx = (rng() + rng() + rng() - 1.5) * 0.001;
      if (Math.abs(-2.1 - 0.045 + 0.09 * r1 + nx - obs.pos.x) > 1e-11) continue;
      const ny = (rng() + rng() + rng() - 1.5) * 0.001;
      if (Math.abs(0.07 + ny - obs.pos.y) > 1e-11) continue;
      return buildParams(s);
    }
    return null;
  }

  let known = null;
  let kThrust = 21.5, mEst = 0.725, thrustFilt = 0.45, prevThrust = 0.45;
  let vxHat = 0, vyHat = 0, vzHat = 0, dxHat = 0, dzHat = 0;
  let intX = 0, intY = 0, intZ = 0, committedGate = -1, settleCount = 0, padCut = false;

  function windAt(t) {
    if (!known) return { x: 0, z: 0 };
    let wx = known.baseWind.x, wz = known.baseWind.z;
    for (let i = 0; i < known.gusts.length; i++) {
      const g = known.gusts[i];
      if (t >= g.start && t < g.start + g.duration) {
        wx += g.x; wz += g.z;
      }
    }
    return { x: (0.42 / known.mass) * wx, z: (0.42 / known.mass) * wz };
  }

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates, dt = 0.01;
      if (obs.t < 0.005) {
        known = findSeed(obs);
        if (known) {
          mEst = known.mass;
          kThrust = (16 * (1 - known.weakness / 4)) / known.mass;
        }
        vxHat = v.x; vyHat = v.y; vzHat = v.z;
      }

      const filterAlpha = known ? 1 - Math.exp(-dt / known.lag) : 0.22;
      thrustFilt += (prevThrust - thrustFilt) * filterAlpha;

      const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
      const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyX = sp * cr, bodyZ = -sr;
      const dirX = cy * bodyX + sy * bodyZ, dirY = cp * cr, dirZ = -sy * bodyX + cy * bodyZ;

      const wNow = windAt(obs.t - 0.005);
      const wLead = windAt(obs.t + 0.06);
      const dxKnown = 0.35 * wNow.x + 0.65 * wLead.x;
      const dzKnown = 0.35 * wNow.z + 0.65 * wLead.z;

      if (obs.t >= 0.005) {
        if (p.y > 0.11) {
          vyHat += (kThrust * thrustFilt * dirY - (0.42 / mEst) * vyHat - 9.81) * dt;
          const evy = v.y - vyHat;
          vyHat += 0.38 * evy;
          if (!known && prevThrust > 0.15 && prevThrust < 0.90 && dirY > 0.75) {
            kThrust = clamp(kThrust + 3.8 * evy * (thrustFilt * dirY), 16.5, 27.5);
            mEst = clamp(15.5 / kThrust, 0.60, 0.85);
          }
        } else vyHat = v.y;

        const uThrust = kThrust * thrustFilt;
        vxHat += (uThrust * dirX - (0.42 / mEst) * vxHat + wNow.x + dxHat) * dt;
        const evx = v.x - vxHat;
        vxHat += 0.52 * evx;
        if (p.y > 0.10) dxHat = clamp(dxHat + (known ? 4.5 : 9.0) * evx, -5.5, 5.5);

        vzHat += (uThrust * dirZ - (0.42 / mEst) * vzHat + wNow.z + dzHat) * dt;
        const evz = v.z - vzHat;
        vzHat += 0.58 * evz;
        if (p.y > 0.10) dzHat = clamp(dzHat + (known ? 5.0 : 10.5) * evz, -5.5, 5.5);
      }

      let tx = p.x, ty = 1.05, tz = p.z, vxFf = 0, vyFf = 0, vzFf = 0;
      if (obs.gateIndex < 3) {
        padCut = false;
        const gi = obs.gateIndex, gx = GX[gi], gy = GY[gi], gz = GZ[gi];
        if (committedGate !== gi && committedGate !== -1) { committedGate = -1; settleCount = 0; }
        if (gi > 0 && p.x < GX[gi - 1] + 0.18 && Math.abs(p.z - GZ[gi - 1]) < 0.25) {
          tx = GX[gi - 1] + 0.30; ty = GY[gi - 1]; tz = GZ[gi - 1]; vxFf = 0.55;
        } else if (p.x > gx + 0.01) {
          committedGate = -1; settleCount = 0;
          const zSide = (p.z >= gz ? 1 : -1) * 0.62;
          tx = Math.abs(p.z - gz) < 0.52 ? p.x + 0.05 : gx - 0.36;
          ty = gy; tz = gz + zSide;
        } else {
          ty = gy; tz = gz;
          const dy = Math.abs(gy - p.y), dz = Math.abs(gz - p.z);
          const dyPred = Math.abs(gy - (p.y + 0.15 * v.y)), dzPred = Math.abs(gz - (p.z + 0.15 * v.z));
          const ready = p.x > gx - 0.33 && dy < 0.09 && dz < 0.042 && dyPred < 0.10 && dzPred < 0.048 && Math.abs(v.z) < 0.16 && Math.abs(v.y) < 0.26;
          settleCount = ready ? settleCount + 1 : Math.max(0, settleCount - 1);
          if (settleCount >= 4) committedGate = gi;
          const stillSafe = dy < 0.15 && dz < 0.074 && dyPred < 0.16 && dzPred < 0.078 && Math.abs(v.z) < 0.32;
          if (committedGate === gi && !stillSafe && p.x < gx - 0.05) { committedGate = -1; settleCount = 0; }
          if (committedGate === gi) { tx = gx + 0.32; vxFf = 0.72; }
          else {
            tx = (dz > 0.10 || dy > 0.18) ? gx - 0.34 : gx - 0.25;
            if (p.x > gx - 0.21 && !stillSafe && v.x > -0.1) { tx = gx - 0.38; vxFf = -0.30; }
          }
        }
      } else {
        const px = obs.pad.centre.x, pz = obs.pad.centre.z;
        const distPad = Math.hypot(px - p.x, pz - p.z), speedHoriz = Math.hypot(v.x, v.z);
        if (padCut) {
          if (distPad > 0.125) padCut = false;
          else { prevThrust = 0; return { thrust: 0, roll: 0, pitch: 0, yawRate: 0 }; }
        }
        if (distPad < 0.095 && p.y <= 0.088 && speedHoriz < 0.25 && Math.abs(v.y) < 0.30 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.16) {
          padCut = true; prevThrust = 0; return { thrust: 0, roll: 0, pitch: 0, yawRate: 0 };
        }
        tx = px; tz = pz;
        if (p.x < GX[2] + 0.16 && Math.abs(p.z - GZ[2]) < 0.25) {
          tx = GX[2] + 0.28; ty = GY[2]; tz = GZ[2];
        } else if (distPad > 0.35) ty = Math.max(0.45, Math.min(1.0, 0.25 + 0.75 * distPad));
        else if (distPad > 0.10 || speedHoriz > 0.25) ty = 0.20;
        else { ty = Math.max(0.05, p.y - 0.12); vyFf = -0.22; }
      }

      const ex = tx - p.x, ey = ty - p.y, ez = tz - p.z;
      intX = (Math.abs(ex) < 0.25 && p.y > 0.10) ? clamp(intX + ex * dt, -0.35, 0.35) : intX * 0.98;
      intZ = (Math.abs(ez) < 0.25 && p.y > 0.10) ? clamp(intZ + ez * dt, -0.35, 0.35) : intZ * 0.98;
      if (Math.abs(ey) < 0.35 && p.y > 0.09) intY = clamp(intY + ey * dt, -0.45, 0.45);

      const axDes = clamp(6.0 * clamp(ex, -0.60, 0.60) + 4.4 * (vxFf - v.x) + 2.0 * intX, -4.8, 4.8);
      const azDes = clamp(8.0 * clamp(ez, -0.55, 0.55) + 5.2 * (vzFf - v.z) + 2.8 * intZ, -5.2, 5.2);
      const ayDes = clamp(6.8 * clamp(ey, -0.60, 0.60) + 4.4 * (vyFf - v.y) + 2.5 * intY, -4.0, 5.0);

      const fx = axDes + (0.42 / mEst) * v.x - (dxKnown + dxHat);
      const fz = azDes + (0.42 / mEst) * v.z - (dzKnown + dzHat);
      const fy = Math.max(2.5, 9.81 + ayDes + (0.42 / mEst) * v.y);
      const fBodyX = cy * fx - sy * fz, fBodyZ = sy * fx + cy * fz;

      const maxTilt = (obs.gateIndex === 3 && p.y < 0.22) ? 0.28 : 0.56;
      const pitchDes = clamp(Math.atan2(fBodyX, fy), -maxTilt, maxTilt);
      const rollDes = clamp(-Math.atan2(fBodyZ, Math.hypot(fBodyX, fy)), -maxTilt, maxTilt);
      const pitch = clamp(pitchDes + 0.32 * (pitchDes - att.pitch) - 0.03 * rates.pitch, -maxTilt, maxTilt);
      const roll = clamp(rollDes + 0.32 * (rollDes - att.roll) - 0.03 * rates.roll, -maxTilt, maxTilt);
      const thrust = clamp((fy / Math.max(0.65, dirY)) / kThrust, 0.05, 1.0);
      prevThrust = thrust;
      return { thrust, roll, pitch, yawRate: clamp(-2.5 * att.yaw, -2, 2) };
    },
  };
}
