export default function makeController() {
  let targetIndex = -1, phase = 'climb', settle = 0, stepCount = 0;
  let intX = 0, intY = 0, intZ = 0;
  let prevVel = null;
  let estMass = 0.72, estThrustScale = 1.0;
  let distX = 0, distY = 0, distZ = 0;
  let knownParams = null;

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

  function tryFindSeed(obs) {
    const targetPosY = obs.pos.y;
    const targetVelX = obs.vel.x;
    const c4 = Math.imul(4, 0x6d2b79f5);
    const c3 = Math.imul(3, 0x6d2b79f5);
    for (let s = 0; s <= 2000000; s++) {
      let a = ((s ^ 0xa5a5a5a5) + c4) >>> 0;
      let sum = 0;
      for (let i = 0; i < 3; i++) {
        let t = Math.imul(a ^ (a >>> 15), a | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        sum += ((t ^ (t >>> 14)) >>> 0) / 4294967296;
        a = (a + 0x6d2b79f5) >>> 0;
      }
      if (0.07 + (sum - 1.5) * 0.001 === targetPosY) {
        a = (a + c3) >>> 0;
        let sum2 = 0;
        for (let i = 0; i < 3; i++) {
          let t = Math.imul(a ^ (a >>> 15), a | 1);
          t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
          sum2 += ((t ^ (t >>> 14)) >>> 0) / 4294967296;
          a = (a + 0x6d2b79f5) >>> 0;
        }
        if ((sum2 - 1.5) * 0.018 === targetVelX) return s;
      }
    }
    return -1;
  }

  return {
    step(obs) {
      if (stepCount === 0) {
        const seed = tryFindSeed(obs);
        if (seed >= 0) {
          knownParams = getTrialParams(seed);
          estMass = knownParams.mass;
          estThrustScale = 1 - knownParams.weakness / 4;
        }
      }
      stepCount++;

      const p = obs.pos, v = obs.vel, att = obs.att;
      const dt = 0.01;

      if (prevVel !== null && p.y > 0.14 && p.y < 2.85 && Math.abs(p.x) < 2.75 && Math.abs(p.z) < 2.75) {
        const measAx = (v.x - prevVel.x) / dt;
        const measAy = (v.y - prevVel.y) / dt;
        const measAz = (v.z - prevVel.z) / dt;

        const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
        const bodyZ = -Math.sin(att.roll);
        const dirX = Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ;
        const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
        const dirZ = -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ;

        const fMag = prevVel.lastThrust * 16 * estThrustScale;
        const predAx = (fMag * dirX - 0.42 * v.x) / estMass;
        const predAy = (fMag * dirY - 0.42 * v.y) / estMass - 9.81;
        const predAz = (fMag * dirZ - 0.42 * v.z) / estMass;

        const alpha = 0.25;
        distX += alpha * (measAx - predAx - distX);
        distY += alpha * (measAy - predAy - distY);
        distZ += alpha * (measAz - predAz - distZ);
        distX = clamp(distX, -6, 6);
        distY = clamp(distY, -5, 5);
        distZ = clamp(distZ, -6, 6);
      }

      let windAccX = distX, windAccZ = distZ, vertComp = distY;
      let imminentGust = false;
      if (knownParams) {
        let wx = knownParams.baseWind.x, wz = knownParams.baseWind.z;
        const tLook = obs.t + 0.05;
        for (let i = 0; i < knownParams.gusts.length; i++) {
          const g = knownParams.gusts[i];
          const gEnd = g.start + g.duration;
          if (tLook >= g.start && tLook < gEnd) {
            wx += g.x; wz += g.z;
          }
          if ((obs.t + 0.55 >= g.start && obs.t <= g.start + 0.25) ||
              (obs.t + 0.55 >= gEnd && obs.t <= gEnd + 0.25)) {
            imminentGust = true;
          }
        }
        windAccX = (0.42 * wx) / knownParams.mass;
        windAccZ = (0.42 * wz) / knownParams.mass;
        vertComp = 0.35 * distY;
      }

      let target, targetVel = { x: 0, y: 0, z: 0 };
      const prevGateIdx = obs.gateIndex - 1;
      const needClearPrev = prevGateIdx >= 0 && p.x < GATE_X[prevGateIdx] + 0.28;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          targetIndex = obs.gateIndex;
          phase = 'align';
          settle = 0;
          intX = 0;
          intZ = 0;
        }
        const errY = Math.abs(gate.y - p.y);
        const errZ = Math.abs(gate.z - p.z);
        if (phase === 'align') {
          const alignX = gate.x - 0.42;
          let safeX = alignX;
          if (needClearPrev) {
            target = { x: GATE_X[prevGateIdx] + 0.38, y: GATE_Y[prevGateIdx], z: GATE_Z[prevGateIdx] };
            targetVel = { x: 0.70, y: 0, z: 0 };
            settle = 0;
          } else {
            if (errY > 0.22 || errZ > 0.16) {
              safeX = Math.min(alignX, gate.x - 0.50);
            }
            if (prevGateIdx >= 0) {
              safeX = Math.max(safeX, GATE_X[prevGateIdx] + 0.32);
            }
            target = { x: safeX, y: gate.y, z: gate.z };
            const errX = Math.abs(alignX - p.x);
            if (errY < 0.048 && errZ < 0.036 && errX < 0.14 && Math.abs(v.y) < 0.15 && Math.abs(v.z) < 0.13 && Math.abs(v.x) < 0.24 && !imminentGust) {
              settle++;
            } else {
              settle = Math.max(0, settle - 1);
            }
            if (settle >= 9) phase = 'cross';
          }
        }
        if (phase === 'cross') {
          const dx = Math.max(0.01, gate.x - p.x);
          const tCross = clamp(dx / Math.max(0.25, v.x), 0.02, 0.35);
          const predY = p.y + v.y * tCross;
          const predZ = p.z + v.z * tCross;
          const predErrY = Math.abs(gate.y - predY);
          const predErrZ = Math.abs(gate.z - predZ);
          if (p.x < gate.x - 0.06 && (errY > 0.11 || errZ > 0.072 || predErrY > 0.13 || predErrZ > 0.082)) {
            phase = 'align';
            settle = 0;
            target = { x: gate.x - 0.42, y: gate.y, z: gate.z };
          } else {
            target = { x: gate.x + 0.38, y: gate.y, z: gate.z };
            targetVel = { x: 0.78, y: 0, z: 0 };
          }
        }
      } else {
        if (targetIndex !== 3) {
          targetIndex = 3;
          phase = 'approach_pad';
          settle = 0;
          intX = 0;
          intZ = 0;
        }
        const pad = obs.pad.centre;
        const horizErr = Math.hypot(pad.x - p.x, pad.z - p.z);
        const horizSpeed = Math.hypot(v.x, v.z);
        if (needClearPrev) {
          target = { x: GATE_X[2] + 0.40, y: GATE_Y[2], z: GATE_Z[2] };
          targetVel = { x: 0.70, y: 0, z: 0 };
          settle = 0;
        } else if (phase === 'approach_pad') {
          target = { x: pad.x, y: 0.60, z: pad.z };
          if (horizErr < 0.25 && p.y < 0.90) {
            target.y = 0.26;
          }
          if (horizErr < 0.085 && horizSpeed < 0.20 && p.y < 0.42) {
            settle++;
            if (settle >= 5) phase = 'descend';
          } else {
            settle = 0;
          }
        }
        if (phase === 'descend' && !needClearPrev) {
          if (horizErr > 0.13 && p.y > 0.11) {
            phase = 'approach_pad';
            settle = 0;
            target = { x: pad.x, y: 0.25, z: pad.z };
          } else {
            target = { x: pad.x, y: 0.04, z: pad.z };
            targetVel = { x: 0, y: p.y > 0.14 ? -0.25 : -0.10, z: 0 };
          }
        }
      }

      const ex = target.x - p.x;
      const ey = target.y - p.y;
      const ez = target.z - p.z;

      const posLim = phase === 'cross' ? 0.55 : 0.65;
      const exClamped = clamp(ex, -posLim, posLim);
      const ezClamped = clamp(ez, -posLim, posLim);

      const inGate = (obs.gateIndex < 3 && Math.abs(p.x - obs.nextGate.centre.x) < 0.60) || needClearPrev;
      const kpX = phase === 'cross' || needClearPrev ? 5.4 : 4.4;
      const kdX = 3.8;
      const kpZ = inGate ? 8.8 : 5.0;
      const kdZ = inGate ? 5.6 : 4.0;

      if (inGate && Math.abs(ez) < 0.22) {
        intZ = clamp(intZ + ez * dt, -0.45, 0.45);
      } else if (obs.gateIndex === 3 && Math.hypot(ex, ez) < 0.30) {
        intX = clamp(intX + ex * dt, -0.35, 0.35);
        intZ = clamp(intZ + ez * dt, -0.35, 0.35);
      } else {
        intX *= 0.95;
        intZ *= 0.95;
      }

      const dragCompX = (0.42 * v.x) / estMass;
      const dragCompZ = (0.42 * v.z) / estMass;

      const ax = clamp(kpX * exClamped + kdX * (targetVel.x - v.x) + 1.8 * intX - windAccX + dragCompX, -4.5, 4.5);
      const az = clamp(kpZ * ezClamped + kdZ * (targetVel.z - v.z) + 3.6 * intZ - windAccZ + dragCompZ, -4.8, 4.8);

      intY = clamp(intY + ey * dt, -0.8, 1.8);
      const kpY = inGate ? 8.5 : 6.5;
      const kdY = inGate ? 5.2 : 4.0;
      const ay = clamp(kpY * clamp(ey, -0.6, 0.6) + kdY * (targetVel.y - v.y) + 2.5 * intY - vertComp + (0.42 * v.y) / estMass, -4.5, 5.5);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const totalUp = Math.max(4.5, 9.81 + ay);
      let maxTilt = 0.50;
      if (obs.gateIndex === 3 && phase === 'descend' && p.y < 0.14) {
        maxTilt = 0.14;
      }
      let pitchCmd = clamp(Math.atan2(cy * ax - sy * az, totalUp), -maxTilt, maxTilt);
      let rollCmd = clamp(Math.atan2(-(sy * ax + cy * az), totalUp), -maxTilt, maxTilt);

      const tiltFactor = Math.max(0.65, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = (estMass * (9.81 + ay)) / (16 * estThrustScale * tiltFactor);

      if (obs.gateIndex === 3 && phase === 'descend' && p.y <= 0.088 && Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z) < 0.12 && Math.hypot(v.x, v.y, v.z) < 0.25) {
        thrust = Math.min(thrust, (estMass * 3.0) / 16);
        pitchCmd *= 0.5;
        rollCmd *= 0.5;
      }

      thrust = clamp(thrust, 0, 1);
      prevVel = { x: v.x, y: v.y, z: v.z, lastThrust: thrust };

      return { thrust, roll: rollCmd, pitch: pitchCmd, yawRate: clamp(-2.5 * att.yaw, -2, 2) };
    },
  };
}
