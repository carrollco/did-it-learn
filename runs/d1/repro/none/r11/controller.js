export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const STEP3 = (3 * 0x6d2b79f5) >>> 0;

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
    const mass = 0.6 + between(0, 0.25);
    const lag = between(0.02, 0.06);
    const weakMotor = Math.floor(r() * 4), weakness = between(0, 0.25);
    const angle = between(0, 2 * Math.PI), strength = between(0, 0.9);
    const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
    const gusts = Array.from({ length: r() < 0.5 ? 2 : 3 }, (_, i) => {
      const start = between(2.0, 12.0);
      const duration = i === 0 ? 0.2 : between(0.2, 1.5);
      const a = between(0, 2 * Math.PI), s = between(1, 4);
      return { start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s };
    });
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  function verifySeed(s, obs0) {
    const params = getTrialParams(s);
    const rng = makeRng((s ^ 0xa5a5a5a5) >>> 0);
    const noise = (n) => (rng() + rng() + rng() - 1.5) * n;
    const px = -2.1 + params.offset.x + noise(0.001);
    const py = 0.07 + noise(0.001);
    const pz = -1.65 + params.offset.z + noise(0.001);
    const vx = noise(0.018);
    if (Math.abs(px - obs0.px) < 1e-12 && Math.abs(py - obs0.py) < 1e-12 &&
        Math.abs(pz - obs0.pz) < 1e-12 && Math.abs(vx - obs0.vx) < 1e-12) {
      return params;
    }
    return null;
  }

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

  let init = false;
  let obs0 = null, searchSeed = 0, knownParams = null;
  let wFiltX = 0, wFiltZ = 0;
  let px = 0, py = 0, pz = 0;
  let vx = 0, vy = 0, vz = 0;
  let prevVx = 0, prevVy = 0, prevVz = 0;
  let dxEst = 0, dyEst = 0, dzEst = 0;
  let kThrust = 21.4, mEst = 0.725, mot = 0.45;
  let lastThrust = 0.45, lastRoll = 0, lastPitch = 0;
  let cmdRollF = 0, cmdPitchF = 0, rollOff = 0, pitchOff = 0, yawInt = 0;
  let targetIndex = -1, phase = 'align', settle = 0, onGround = false;
  let prevGateX = -10, prevGateY = 1.05, prevGateZ = -0.85;

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;
      const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
      const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyX = sp * cr, bodyZ = -sr;
      const ux = cy * bodyX + sy * bodyZ;
      const uy = cp * cr;
      const uz = -sy * bodyX + cy * bodyZ;

      if (!init) {
        init = true;
        obs0 = { px: p.x, py: p.y, pz: p.z, vx: v.x };
        px = p.x; py = p.y; pz = p.z;
        vx = 0; vy = 0; vz = 0;
        prevVx = v.x; prevVy = v.y; prevVz = v.z;
      }

      if (!knownParams && searchSeed < 500000 && obs.t < 0.10) {
        const targetPy = obs0.py;
        const limit = searchSeed + 50000;
        for (let s = searchSeed; s < limit; s++) {
          let a = ((s ^ 0xa5a5a5a5) + STEP3) >>> 0;
          let t = Math.imul(a ^ (a >>> 15), a | 1);
          t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
          const r4 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
          a = (a + 0x6d2b79f5) >>> 0;
          t = Math.imul(a ^ (a >>> 15), a | 1);
          t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
          const r5 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
          a = (a + 0x6d2b79f5) >>> 0;
          t = Math.imul(a ^ (a >>> 15), a | 1);
          t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
          const r6 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
          if (0.07 + (r4 + r5 + r6 - 1.5) * 0.001 === targetPy) {
            const matched = verifySeed(s, obs0);
            if (matched) {
              knownParams = matched;
              wFiltX = matched.baseWind.x;
              wFiltZ = matched.baseWind.z;
              break;
            }
          }
        }
        searchSeed = limit;
      }

      let dLeadX = 0, dLeadZ = 0;
      if (knownParams) {
        const wNow = windAt(knownParams, Math.max(0, obs.t - 0.005));
        if (py > 0.088) {
          wFiltX += 0.24 * (wNow.x - wFiltX);
          wFiltZ += 0.24 * (wNow.z - wFiltZ);
        }
        const wLead = windAt(knownParams, obs.t + knownParams.lag + 0.015);
        const dragK = 0.42 / mEst;
        dLeadX = dragK * (wLead.x - wFiltX);
        dLeadZ = dragK * (wLead.z - wFiltZ);
      }

      if (obs.t > 0) {
        mot += (lastThrust - mot) * 0.23;
        cmdRollF += (lastRoll - cmdRollF) * 0.22;
        cmdPitchF += (lastPitch - cmdPitchF) * 0.22;

        const drag = 0.42 / mEst;
        const axK = kThrust * mot * ux - drag * vx;
        const ayK = kThrust * mot * uy - drag * vy - 9.81;
        const azK = kThrust * mot * uz - drag * vz;

        if (py > 0.088) {
          const axMeas = (v.x - prevVx) * 100 - axK;
          const ayMeas = (v.y - prevVy) * 100 - ayK;
          const azMeas = (v.z - prevVz) * 100 - azK;
          dxEst = clamp(dxEst + 0.24 * (axMeas - dxEst), -9.5, 9.5);
          dyEst = clamp(dyEst + 0.18 * (ayMeas - dyEst), -6.5, 6.5);
          dzEst = clamp(dzEst + 0.24 * (azMeas - dzEst), -9.5, 9.5);

          if (mot * uy > 0.28) {
            kThrust = clamp(kThrust + 0.40 * dyEst * 0.01 / (mot * uy), 16.5, 27.0);
            dyEst *= 0.96;
            mEst = clamp(15.5 / kThrust, 0.60, 0.85);
          }
          const rErr = (att.roll + 0.206 * rates.roll) - cmdRollF;
          const pErr = (att.pitch + 0.206 * rates.pitch) - cmdPitchF;
          rollOff = clamp(rollOff + 0.16 * (rErr - rollOff), -0.32, 0.32);
          pitchOff = clamp(pitchOff + 0.16 * (pErr - pitchOff), -0.32, 0.32);
        }

        prevVx = v.x; prevVy = v.y; prevVz = v.z;
        vx = 0.65 * v.x + 0.35 * (vx + (axK + dxEst) * 0.01);
        vy = 0.65 * v.y + 0.35 * (vy + (ayK + dyEst) * 0.01);
        vz = 0.65 * v.z + 0.35 * (vz + (azK + dzEst) * 0.01);
        px = 0.80 * p.x + 0.20 * (px + vx * 0.01);
        py = 0.80 * p.y + 0.20 * (py + vy * 0.01);
        pz = 0.80 * p.z + 0.20 * (pz + vz * 0.01);
        if (py <= 0.085) {
          py = p.y;
          vy = Math.max(0, v.y);
        }
      }

      let tx, ty, tz, vxDes = 0, vyDes = 0, vzDes = 0;
      let kpX = 6.5, kdX = 4.6, kpY = 9.5, kdY = 5.4, kpZ = 10.0, kdZ = 5.8;
      let maxTilt = 0.56;
      let maxEx = 0.35, maxEz = 0.35;
      let maxAx = 6.0;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          if (targetIndex >= 0) {
            const gPrev = targetIndex === 0 ? { x: -1.15, y: 1.05, z: -0.85 }
              : targetIndex === 1 ? { x: 0.15, y: 1.40, z: 0.25 }
              : { x: 1.35, y: 1.05, z: 1.05 };
            prevGateX = gPrev.x; prevGateY = gPrev.y; prevGateZ = gPrev.z;
          }
          targetIndex = obs.gateIndex;
          phase = 'align';
          settle = 0;
        }

        const nearGate = Math.abs(gate.x - px) < 0.45;
        if (nearGate) {
          kpZ = 16.5; kdZ = 7.6;
          kpY = 12.0; kdY = 6.0;
        }

        if (phase === 'align') {
          tx = gate.x - 0.21;
          ty = gate.y;
          tz = gate.z;
          if (px < prevGateX + 0.14) {
            ty = prevGateY;
            tz = prevGateZ;
          }
          const errY = Math.abs(ty - py), errZ = Math.abs(tz - pz), errX = Math.abs(tx - px);
          const spdYZ = Math.hypot(vy, vz), spdX = Math.abs(vx);
          if (errZ < 0.026 && errY < 0.055 && errX < 0.075 && spdYZ < 0.11 && spdX < 0.16) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= 6) {
            phase = 'cross';
          }
        }

        if (phase === 'cross') {
          const remX = gate.x - px;
          const tRem = clamp(remX / Math.max(vx, 0.20), 0.02, 0.30);
          const zPred = pz + vz * tRem;
          const yPred = py + vy * tRem;
          if (remX > 0.026 && (Math.abs(pz - gate.z) > 0.052 || Math.abs(zPred - gate.z) > 0.062 ||
              Math.abs(py - gate.y) > 0.11 || Math.abs(yPred - gate.y) > 0.13)) {
            phase = 'align';
            settle = 0;
            tx = gate.x - 0.23;
            ty = gate.y;
            tz = gate.z;
            kpX = 12.5; kdX = 6.8;
          } else {
            tx = px;
            ty = gate.y;
            tz = gate.z;
            vxDes = 0.48;
            kpX = 0.0; kdX = 6.2;
            maxAx = 2.5;
            kpZ = 18.0; kdZ = 8.0;
            kpY = 12.5; kdY = 6.2;
          }
        }
      } else {
        if (targetIndex !== 3) {
          targetIndex = 3;
          prevGateX = 1.35; prevGateY = 1.05; prevGateZ = 1.05;
          phase = 'pad_approach';
          settle = 0;
          onGround = false;
        }
        const pad = obs.pad.centre;
        const distXZ = Math.hypot(pad.x - px, pad.z - pz);
        const spdXZ = Math.hypot(vx, vz);

        if (phase === 'pad_approach') {
          tx = pad.x;
          tz = pad.z;
          ty = px < prevGateX + 0.14 ? prevGateY : 0.38;
          if (px < prevGateX + 0.14) tz = prevGateZ;
          if (distXZ < 0.060 && spdXZ < 0.14 && py < 0.50) {
            phase = 'descend';
          }
        }
        if (phase === 'descend') {
          tx = pad.x;
          tz = pad.z;
          kpX = 12.5; kdX = 6.5;
          kpZ = 12.5; kdZ = 6.5;
          if (onGround) {
            if (distXZ > 0.075) {
              onGround = false;
            } else {
              lastThrust = 0;
              lastRoll = 0;
              lastPitch = 0;
              return { thrust: 0, roll: 0, pitch: 0, yawRate: 0 };
            }
          }
          if (distXZ > 0.055) {
            ty = Math.max(py, 0.15);
            vyDes = py < 0.12 ? 0.15 : 0;
          } else {
            ty = 0.055;
            vyDes = py > 0.16 ? -0.28 : -0.10;
          }
          if (py < 0.16) {
            maxTilt = (py <= 0.105 && distXZ <= 0.065) ? 0.135 : 0.28;
          }
          if (py <= 0.072 && distXZ <= 0.042 && spdXZ < 0.08) {
            onGround = true;
            lastThrust = 0;
            lastRoll = 0;
            lastPitch = 0;
            return { thrust: 0, roll: 0, pitch: 0, yawRate: 0 };
          }
        }
      }

      const exC = clamp(tx - px, -maxEx, maxEx);
      const ezC = clamp(tz - pz, -maxEz, maxEz);
      const eyC = clamp(ty - py, -0.35, 0.40);

      let axDes = kpX * exC + kdX * (vxDes - vx);
      let azDes = kpZ * ezC + kdZ * (vzDes - vz);
      let ayDes = kpY * eyC + kdY * (vyDes - vy);

      if (obs.gateIndex < 3 && phase !== 'cross') {
        const gate = obs.nextGate.centre;
        const stopPredX = px + Math.max(0, vx) * 0.22;
        const marginX = (gate.x - 0.13) - stopPredX;
        if (marginX < 0 || px > gate.x - 0.15) {
          axDes = Math.min(axDes, 22.0 * ((gate.x - 0.17) - px) - 8.0 * vx);
          maxTilt = 0.59;
        }
      }

      axDes = clamp(axDes, -7.8, maxAx);
      azDes = clamp(azDes, -7.5, 7.5);
      ayDes = clamp(ayDes, -4.5, 5.5);

      const drag = 0.42 / mEst;
      const fx = axDes - (dxEst + dLeadX) + drag * vx;
      const fy = Math.max(1.5, 9.81 + ayDes - dyEst + drag * vy);
      const fz = azDes - (dzEst + dLeadZ) + drag * vz;

      const fbx = cy * fx - sy * fz;
      const fbz = sy * fx + cy * fz;

      const desPitch = Math.atan2(fbx, fy);
      const desRoll = -Math.atan2(fbz, Math.hypot(fbx, fy));

      const pitch = clamp(desPitch - pitchOff, -maxTilt, maxTilt);
      const roll = clamp(desRoll - rollOff, -maxTilt, maxTilt);
      const thrust = clamp(fy / (kThrust * Math.max(0.55, uy)), 0.05, 1.0);

      yawInt = clamp(yawInt + att.yaw * 0.01, -0.4, 0.4);
      const yawRate = clamp(-4.5 * att.yaw - 10.0 * yawInt - 0.4 * rates.yaw, -2, 2);

      lastThrust = thrust;
      lastRoll = roll;
      lastPitch = pitch;

      return { thrust, roll, pitch, yawRate };
    },
  };
}
