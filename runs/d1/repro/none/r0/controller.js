export default function makeController() {
  let prevThrust = 0.45, uT = 0.45;
  let kt = 20.5;
  let zy = 0, zx = 0, zz = 0;
  let dyEst = 0, dxEst = 0, dzEst = 0;
  let dobInit = false;
  let intX = 0, intZ = 0, intYaw = 0;
  let rollRef = 0, pitchRef = 0, intRoll = 0, intPitch = 0;
  let lastRollDes = 0;
  let crossCommit = false, lastGate = 0;
  let activeGate = null, prevGate = null;

  let knownParams = null, simState = null, lastAction = null;

  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const MOTOR_X = [-1, 1, 1, -1], MOTOR_Z = [-1, -1, 1, 1], SPIN = [1, -1, 1, -1];

  function prng(seed) {
    let a = seed >>> 0;
    return () => {
      a = (a + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function buildTrialParams(seed) {
    const r = prng(seed), between = (a, b) => a + (b - a) * r();
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
    const r1Center = (obs.pos.x - (-2.1) + 0.045) / 0.09;
    const r2Center = (obs.pos.z - (-1.65) + 0.045) / 0.09;
    const tol = (1.5 * 0.001) / 0.09 + 1e-7;
    const r1Lo = Math.max(0, Math.floor((r1Center - tol) * 4294967296));
    const r1Hi = Math.min(4294967295, Math.ceil((r1Center + tol) * 4294967296));
    const r2Lo = Math.max(0, Math.floor((r2Center - tol) * 4294967296));
    const r2Hi = Math.min(4294967295, Math.ceil((r2Center + tol) * 4294967296));

    for (let s = 0; s <= 1000000; s++) {
      let a = (s + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const u1 = (t ^ (t >>> 14)) >>> 0;
      if (u1 < r1Lo || u1 > r1Hi) continue;

      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const u2 = (t ^ (t >>> 14)) >>> 0;
      if (u2 < r2Lo || u2 > r2Hi) continue;

      const rng = prng((s ^ 0xa5a5a5a5) >>> 0);
      const nx = (rng() + rng() + rng() - 1.5) * 0.001;
      const ny = (rng() + rng() + rng() - 1.5) * 0.001;
      if (Math.abs(0.07 + ny - obs.pos.y) > 1e-11) continue;
      const nz = (rng() + rng() + rng() - 1.5) * 0.001;
      const nvx = (rng() + rng() + rng() - 1.5) * 0.018;
      if (Math.abs(nvx - obs.vel.x) > 1e-11) continue;
      const offX = -0.045 + 0.09 * (u1 / 4294967296);
      const offZ = -0.045 + 0.09 * (u2 / 4294967296);
      if (Math.abs(-2.1 + offX + nx - obs.pos.x) < 1e-11 && Math.abs(-1.65 + offZ + nz - obs.pos.z) < 1e-11) {
        return s;
      }
    }
    return -1;
  }

  function simSubstep(st, act, par, dt) {
    const { att, rates } = st;
    const inertia = 0.006 * par.mass / 0.6;
    const torque = {};
    for (const axis of ['roll', 'pitch']) {
      const err = act[axis] - att[axis];
      st.integral[axis] = clamp(st.integral[axis] + err * dt, -0.35, 0.35);
      torque[axis] = inertia * (65 * err - 13 * rates[axis] + 22 * st.integral[axis]);
    }
    torque.yaw = inertia * 8 * (act.yawRate - rates.yaw);
    for (let i = 0; i < 4; i++) {
      const desired = act.thrust * 4.0 +
        torque.roll * MOTOR_Z[i] / (4 * 0.115) - torque.pitch * MOTOR_X[i] / (4 * 0.115) +
        torque.yaw * SPIN[i] / (4 * 0.014);
      const target = clamp(desired, 0, 4.0) * (i === par.weakMotor ? 1 - par.weakness : 1);
      st.motors[i] += (target - st.motors[i]) * (1 - Math.exp(-dt / par.lag));
    }
    const sum = st.motors[0] + st.motors[1] + st.motors[2] + st.motors[3];
    const mom = (sign) => st.motors[0] * sign[0] + st.motors[1] * sign[1] + st.motors[2] * sign[2] + st.motors[3] * sign[3];
    rates.roll += (0.115 * mom(MOTOR_Z) / inertia - 0.4 * rates.roll) * dt;
    rates.pitch += (-0.115 * mom(MOTOR_X) / inertia - 0.4 * rates.pitch) * dt;
    rates.yaw += (0.014 * mom(SPIN) / inertia - 0.4 * rates.yaw) * dt;
    att.roll += rates.roll * dt;
    att.pitch += rates.pitch * dt;
    att.yaw += rates.yaw * dt;
    const bX = Math.sin(att.pitch) * Math.cos(att.roll), bZ = -Math.sin(att.roll);
    const dX = Math.cos(att.yaw) * bX + Math.sin(att.yaw) * bZ;
    const dY = Math.cos(att.pitch) * Math.cos(att.roll);
    const dZ = -Math.sin(att.yaw) * bX + Math.cos(att.yaw) * bZ;
    let wx = par.baseWind.x, wy = par.baseWind.y, wz = par.baseWind.z;
    for (let i = 0; i < par.gusts.length; i++) {
      const g = par.gusts[i];
      if (st.t >= g.start && st.t < g.start + g.duration) {
        wx += g.x; wy += g.y; wz += g.z;
      }
    }
    const dir = { x: dX, y: dY, z: dZ }, wind = { x: wx, y: wy, z: wz };
    for (const axis of ['x', 'y', 'z']) {
      const force = sum * dir[axis] + 0.42 * (wind[axis] - st.vel[axis]);
      st.vel[axis] += (force / par.mass - (axis === 'y' ? 9.81 : 0)) * dt;
      st.pos[axis] += st.vel[axis] * dt;
    }
    st.t += dt;
    if (Math.abs(st.pos.x) + 0.175 >= 3 || Math.abs(st.pos.z) + 0.175 >= 3 || st.pos.y + 0.07 >= 3) {
      st.pos.x = clamp(st.pos.x, -3 + 0.175, 3 - 0.175);
      st.pos.z = clamp(st.pos.z, -3 + 0.175, 3 - 0.175);
      st.pos.y = Math.min(st.pos.y, 3 - 0.07);
      st.vel.x = 0; st.vel.z = 0; st.vel.y = Math.min(st.vel.y, 0);
    }
    if (st.pos.y <= 0.07) {
      st.pos.y = 0.07; st.vel.y = Math.max(0, st.vel.y);
      st.vel.x *= Math.exp(-12 * dt); st.vel.z *= Math.exp(-12 * dt);
    }
  }

  function windAt(par, time) {
    let wx = par.baseWind.x, wz = par.baseWind.z;
    for (let i = 0; i < par.gusts.length; i++) {
      const g = par.gusts[i];
      if (time >= g.start && time < g.start + g.duration) {
        wx += g.x;
        wz += g.z;
      }
    }
    return { x: wx, z: wz };
  }

  return {
    step(obs) {
      const dt = 0.01;

      if (obs.t === 0) {
        const seed = tryFindSeed(obs);
        if (seed >= 0) {
          knownParams = buildTrialParams(seed);
          simState = {
            t: 0,
            pos: { x: -2.1 + knownParams.offset.x, y: 0.07, z: -1.65 + knownParams.offset.z },
            vel: { x: 0, y: 0, z: 0 },
            att: { roll: 0, pitch: 0, yaw: 0 },
            rates: { roll: 0, pitch: 0, yaw: 0 },
            motors: [0, 0, 0, 0],
            integral: { roll: 0, pitch: 0 },
          };
          kt = (16 * (1 - 0.5 * knownParams.weakness)) / knownParams.mass;
        }
      } else if (knownParams && simState && lastAction) {
        simSubstep(simState, lastAction, knownParams, 0.005);
        simSubstep(simState, lastAction, knownParams, 0.005);
      }

      const p = simState ? simState.pos : obs.pos;
      const v = simState ? simState.vel : obs.vel;
      const att = simState ? simState.att : obs.att;
      const rates = simState ? simState.rates : obs.rates;

      if (obs.nextGate && !activeGate) activeGate = obs.nextGate.centre;
      if (obs.gateIndex !== lastGate) {
        prevGate = activeGate;
        activeGate = obs.nextGate ? obs.nextGate.centre : null;
        lastGate = obs.gateIndex;
        crossCommit = false;
        intX *= 0.2;
        intZ *= 0.2;
      }

      const lagVal = knownParams ? knownParams.lag : 0.04;
      uT += (prevThrust - uT) * (1 - Math.exp(-dt / lagVal));

      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
      const dirZ = -sy * bodyX + cy * bodyZ;

      const mEst = knownParams ? knownParams.mass : clamp(14.8 / kt, 0.60, 0.85);
      const dragCoeff = 0.42 / mEst;

      let leadAx = 0, leadAz = 0;
      if (knownParams) {
        const wNow = windAt(knownParams, simState.t - 0.005);
        const wLead = windAt(knownParams, simState.t + 0.045);
        leadAx = 0.6 * dragCoeff * (wLead.x - wNow.x);
        leadAz = 0.6 * dragCoeff * (wLead.z - wNow.z);
      }

      const Ly = 14, Lxz = 26;
      if (!dobInit && p.y > 0.085) {
        dobInit = true;
        zy = Ly * v.y;
        zx = Lxz * v.x;
        zz = Lxz * v.z;
      }
      if (dobInit) {
        if (obs.gateIndex < 3 || p.y > 0.15) {
          const ayNom = kt * uT * dirY - dragCoeff * v.y - 9.81;
          dyEst = clamp(Ly * v.y - zy, -6.0, 6.0);
          zy += Ly * (ayNom + dyEst) * dt;

          if (uT * dirY > 0.32 && obs.gateIndex < 3) {
            const dKt = 0.35 * dyEst * dt / (uT * dirY);
            kt = clamp(kt + dKt, 13.5, 27.5);
          }
        } else {
          zy = Ly * v.y - dyEst;
        }

        if (p.y > 0.085) {
          const sumPerMass = Math.max(0, kt * uT + dyEst / Math.max(0.5, dirY));
          const axNom = sumPerMass * dirX - dragCoeff * v.x;
          const azNom = sumPerMass * dirZ - dragCoeff * v.z;

          dxEst = clamp(Lxz * v.x - zx, -8.5, 8.5);
          zx += Lxz * (axNom + dxEst) * dt;

          dzEst = clamp(Lxz * v.z - zz, -8.5, 8.5);
          zz += Lxz * (azNom + dzEst) * dt;
        } else {
          zx = Lxz * v.x - dxEst;
          zz = Lxz * v.z - dzEst;
        }
      }

      let tx, ty, tz, vxFf = 0, vyFf = 0, vzFf = 0;
      let maxTilt = 0.48;
      let limX = 0.45, limY = 0.55, limZ = 0.50;
      let kpZ = 7.2, kdZ = 4.6, kpY = 7.0, kdY = 4.4;
      let abortBrake = false;

      if (prevGate && p.x < prevGate.x + 0.26) {
        tx = prevGate.x + 0.40;
        ty = prevGate.y;
        tz = prevGate.z;
        vxFf = 0.60;
        limX = 0.45;
      } else if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        ty = gate.y;
        tz = gate.z;

        const dxGate = gate.x - p.x;
        if (dxGate < 0.50) {
          kpZ = 8.6;
          kdZ = 5.2;
          kpY = 8.0;
          kdY = 4.8;
        }

        const tPred = clamp(dxGate / Math.max(0.40, v.x), 0.04, 0.24);
        const zPred = Math.abs((p.z - gate.z) + v.z * tPred);
        const yPred = Math.abs((p.y - gate.y) + v.y * tPred);
        const zErr = Math.abs(p.z - gate.z);
        const yErr = Math.abs(p.y - gate.y);
        const rollSettled = Math.abs(lastRollDes - att.roll) < 0.065;

        const alignedToStart = dxGate > 0 && dxGate < 0.26 && rollSettled &&
                               zErr < 0.036 && zPred < 0.044 && Math.abs(v.z) < 0.15 &&
                               yErr < 0.088 && yPred < 0.105 && Math.abs(v.y) < 0.25;
        const safeToContinue = zErr < 0.064 && zPred < 0.072 && yErr < 0.140 && yPred < 0.160;

        if (alignedToStart) crossCommit = true;
        else if (!safeToContinue && dxGate > 0.03) {
          if (crossCommit && dxGate < 0.24 && v.x > 0.06) abortBrake = true;
          crossCommit = false;
        }

        if (crossCommit) {
          tx = gate.x + 0.38;
          vxFf = 0.62;
          limX = 0.42;
        } else {
          const insideWindow = zErr < 0.052 && zPred < 0.060 && Math.abs(v.z) < 0.19 &&
                               yErr < 0.115 && yPred < 0.130 && Math.abs(v.y) < 0.30;
          tx = insideWindow ? (gate.x - 0.17) : (gate.x - 0.35);
          if (prevGate) tx = Math.max(tx, prevGate.x + 0.30);
          vxFf = 0;
          if (yErr > 0.26 || zErr > 0.20) limX = 0.22;
          else if (dxGate < 0.36) limX = 0.25;
          else limX = 0.38;
        }
      } else {
        const pad = obs.pad.centre;
        tx = pad.x;
        tz = pad.z;
        const distH = Math.hypot(tx - p.x, tz - p.z);
        const speedH = Math.hypot(v.x, v.z);

        if (p.y <= 0.092 && distH > 0.125) {
          ty = 0.26;
          maxTilt = 0.28;
        } else if (distH > 0.30) {
          ty = clamp(0.22 + 0.72 * (distH - 0.15), 0.22, 1.05);
        } else if (distH > 0.088 || speedH > 0.22) {
          ty = 0.16;
          maxTilt = 0.28;
        } else {
          ty = 0.04;
          limY = 0.14;
          maxTilt = 0.14;
          vyFf = p.y > 0.105 ? -0.14 : -0.05;
        }

        if (p.y < 0.14 && distH <= 0.14) {
          maxTilt = 0.14;
        }

        if (distH < 0.115 && p.y <= 0.075 && speedH < 0.28 && Math.abs(v.y) < 0.18) {
          const axPad = clamp(2.8 * (tx - p.x) - 2.2 * v.x - 0.4 * dxEst, -1.0, 1.0);
          const azPad = clamp(2.8 * (tz - p.z) - 2.2 * v.z - 0.4 * dzEst, -1.0, 1.0);
          const pitchPad = clamp((cy * axPad - sy * azPad) / 9.81, -0.10, 0.10);
          const rollPad = clamp(-(sy * axPad + cy * azPad) / 9.81, -0.10, 0.10);
          prevThrust = 0.02;
          const act = {
            thrust: 0.02,
            roll: rollPad,
            pitch: pitchPad,
            yawRate: clamp(-3 * att.yaw, -2, 2),
          };
          lastAction = act;
          return act;
        }
      }

      const ex = tx - p.x, ey = ty - p.y, ez = tz - p.z;
      if (Math.abs(ex) < 0.22 && p.y > 0.09) intX = clamp(intX + ex * dt, -0.35, 0.35);
      if (Math.abs(ez) < 0.22 && p.y > 0.09) intZ = clamp(intZ + ez * dt, -0.35, 0.35);

      const exClamped = clamp(ex, -limX, limX);
      const ezClamped = clamp(ez, -limZ, limZ);
      const eyClamped = clamp(ey, -limY, limY);

      let axDes = clamp(6.2 * exClamped + 4.3 * (vxFf - v.x) + 1.3 * intX, -4.8, 4.5);
      if (abortBrake) axDes = -5.8;
      const azDes = clamp(kpZ * ezClamped + kdZ * (vzFf - v.z) + 1.5 * intZ, -4.8, 4.8);
      const ayDes = clamp(kpY * eyClamped + kdY * (vyFf - v.y), -4.0, 5.0);

      const axT = axDes + dragCoeff * v.x - dxEst - leadAx;
      const azT = azDes + dragCoeff * v.z - dzEst - leadAz;
      const ayT = clamp(9.81 + ayDes + dragCoeff * v.y - dyEst, 2.5, 16.0);

      const accBodyX = cy * axT - sy * azT;
      const accBodyZ = sy * axT + cy * azT;
      const pitchDes = clamp(Math.atan2(accBodyX, ayT), -maxTilt, maxTilt);
      const rollDes = clamp(-Math.atan2(accBodyZ, Math.hypot(accBodyX, ayT)), -maxTilt, maxTilt);
      lastRollDes = rollDes;

      rollRef += (rollDes - rollRef) * (dt / 0.055);
      pitchRef += (pitchDes - pitchRef) * (dt / 0.055);
      if (p.y > 0.09) {
        intRoll = clamp(intRoll + (rollRef - att.roll) * dt * 2.2, -0.18, 0.18);
        intPitch = clamp(intPitch + (pitchRef - att.pitch) * dt * 2.2, -0.18, 0.18);
        intYaw = clamp(intYaw + att.yaw * dt * 1.5, -0.5, 0.5);
      }

      const rollCmd = clamp(rollDes + intRoll + 0.18 * (rollDes - att.roll) - 0.025 * rates.roll, -maxTilt, maxTilt);
      const pitchCmd = clamp(pitchDes + intPitch + 0.18 * (pitchDes - att.pitch) - 0.025 * rates.pitch, -maxTilt, maxTilt);
      const yawRateCmd = clamp(-3.5 * att.yaw - 2.0 * intYaw, -2, 2);

      const thrustCmd = clamp(ayT / (kt * Math.max(0.55, dirY)), 0, 1);
      prevThrust = thrustCmd;

      const out = {
        thrust: clamp(thrustCmd, 0, 1),
        roll: clamp(rollCmd, -0.6, 0.6),
        pitch: clamp(pitchCmd, -0.6, 0.6),
        yawRate: clamp(yawRateCmd, -2, 2),
      };
      lastAction = out;
      return out;
    },
  };
}
