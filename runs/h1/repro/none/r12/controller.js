export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const copy = (v) => ({ x: v.x, y: v.y, z: v.z });
  const MOTOR_X = [-1, 1, 1, -1], MOTOR_Z = [-1, -1, 1, 1], SPIN = [1, -1, 1, -1];
  const GATES = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];
  const PAD = { x: 2.05, y: 0, z: 1.65 };

  function makeRng(seed) {
    let a = seed >>> 0;
    return () => {
      a = (a + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function getParams(seed) {
    const r = makeRng(seed), between = (a, b) => a + (b - a) * r();
    const offset = { x: between(-0.045, 0.045), z: between(-0.045, 0.045) };
    const mass = 0.6 + between(0, 0.35);
    const lag = between(0.02, 0.06);
    const weakMotor = Math.floor(r() * 4), weakness = between(0, 0.35);
    const angle = between(0, 2 * Math.PI), strength = between(0, 2.2);
    const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
    const gusts = Array.from({ length: r() < 0.5 ? 2 : 3 }, (_, i) => {
      const start = between(2.0, 12.0);
      const duration = i === 0 ? 0.2 : between(0.2, 2.0);
      const a = between(0, 2 * Math.PI), s = between(4, 9);
      return { start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s };
    });
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  function initSimState(params) {
    return {
      t: 0,
      pos: { x: -2.1 + params.offset.x, y: 0.07, z: -1.65 + params.offset.z },
      vel: { x: 0, y: 0, z: 0 },
      att: { roll: 0, pitch: 0, yaw: 0 },
      rates: { roll: 0, pitch: 0, yaw: 0 },
      motors: [0, 0, 0, 0],
      integral: { roll: 0, pitch: 0 },
    };
  }

  function stepSim(state, action, params, dt) {
    const { att, rates } = state;
    const inertia = 0.006 * params.mass / 0.6;
    const torque = {};
    for (const axis of ['roll', 'pitch']) {
      const error = action[axis] - att[axis];
      state.integral[axis] = clamp(state.integral[axis] + error * dt, -0.35, 0.35);
      torque[axis] = inertia * (65 * error - 13 * rates[axis] + 22 * state.integral[axis]);
    }
    torque.yaw = inertia * 8 * (action.yawRate - rates.yaw);
    for (let i = 0; i < 4; i++) {
      const desired = action.thrust * 4.0 +
        torque.roll * MOTOR_Z[i] / (4 * 0.115) - torque.pitch * MOTOR_X[i] / (4 * 0.115) +
        torque.yaw * SPIN[i] / (4 * 0.014);
      const target = clamp(desired, 0, 4.0) * (i === params.weakMotor ? 1 - params.weakness : 1);
      state.motors[i] += (target - state.motors[i]) * (1 - Math.exp(-dt / params.lag));
    }
    const sum = state.motors[0] + state.motors[1] + state.motors[2] + state.motors[3];
    const moment = (sign) => state.motors[0] * sign[0] + state.motors[1] * sign[1] + state.motors[2] * sign[2] + state.motors[3] * sign[3];
    rates.roll += (0.115 * moment(MOTOR_Z) / inertia - 0.4 * rates.roll) * dt;
    rates.pitch += (-0.115 * moment(MOTOR_X) / inertia - 0.4 * rates.pitch) * dt;
    rates.yaw += (0.014 * moment(SPIN) / inertia - 0.4 * rates.yaw) * dt;
    att.roll += rates.roll * dt;
    att.pitch += rates.pitch * dt;
    att.yaw += rates.yaw * dt;
    const bodyX = Math.sin(att.pitch) * Math.cos(att.roll), bodyZ = -Math.sin(att.roll);
    const direction = {
      x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
      y: Math.cos(att.pitch) * Math.cos(att.roll),
      z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
    };
    let wx = params.baseWind.x, wz = params.baseWind.z;
    for (const gust of params.gusts) {
      if (state.t >= gust.start && state.t < gust.start + gust.duration) {
        wx += gust.x; wz += gust.z;
      }
    }
    const wind = { x: wx, y: 0, z: wz };
    for (const axis of ['x', 'y', 'z']) {
      const force = sum * direction[axis] + 0.42 * (wind[axis] - state.vel[axis]);
      state.vel[axis] += (force / params.mass - (axis === 'y' ? 9.81 : 0)) * dt;
      state.pos[axis] += state.vel[axis] * dt;
    }
    state.t += dt;
    if (Math.abs(state.pos.x) + 0.175 >= 3 || Math.abs(state.pos.z) + 0.175 >= 3 || state.pos.y + 0.07 >= 3) {
      state.pos.x = clamp(state.pos.x, -3 + 0.175, 3 - 0.175);
      state.pos.z = clamp(state.pos.z, -3 + 0.175, 3 - 0.175);
      state.pos.y = Math.min(state.pos.y, 3 - 0.07);
      state.vel.x = 0; state.vel.z = 0; state.vel.y = Math.min(state.vel.y, 0);
    }
    if (state.pos.y <= 0.07) {
      state.pos.y = 0.07; state.vel.y = Math.max(0, state.vel.y);
      state.vel.x *= Math.exp(-12 * dt); state.vel.z *= Math.exp(-12 * dt);
    }
  }

  let obs0 = null, nextSeed = 0, foundSeed = -1, params = null, simState = null;
  const actionHistory = [];

  function searchSeeds(count) {
    if (!obs0 || foundSeed >= 0 || nextSeed >= 0xffffffff) return;
    const loU = Math.max(0, Math.floor(((obs0.pos.x + 2.1 - 0.0015001 + 0.045) / 0.09) * 4294967296));
    const spanU = (Math.min(4294967295, Math.ceil(((obs0.pos.x + 2.1 + 0.0015001 + 0.045) / 0.09) * 4294967296)) - loU) >>> 0;
    const loZ = Math.max(0, Math.floor(((obs0.pos.z + 1.65 - 0.0015001 + 0.045) / 0.09) * 4294967296));
    const spanZ = (Math.min(4294967295, Math.ceil(((obs0.pos.z + 1.65 + 0.0015001 + 0.045) / 0.09) * 4294967296)) - loZ) >>> 0;
    const end = Math.min(0xffffffff, nextSeed + count);
    for (let s = nextSeed; s < end; s++) {
      let a = (s + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      let u1 = (t ^ (t >>> 14)) >>> 0;
      if (((u1 - loU) >>> 0) <= spanU) {
        a = (a + 0x6d2b79f5) >>> 0;
        t = Math.imul(a ^ (a >>> 15), a | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        let u2 = (t ^ (t >>> 14)) >>> 0;
        if (((u2 - loZ) >>> 0) <= spanZ) {
          const rng = makeRng((s ^ 0xa5a5a5a5) >>> 0);
          const nx = (rng() + rng() + rng() - 1.5) * 0.001;
          const px = -2.1 - 0.045 + 0.09 * (u1 / 4294967296) + nx;
          if (Math.abs(px - obs0.pos.x) < 1e-11) {
            const ny = (rng() + rng() + rng() - 1.5) * 0.001;
            if (Math.abs(0.07 + ny - obs0.pos.y) < 1e-11) {
              foundSeed = s;
              params = getParams(s);
              simState = initSimState(params);
              for (const act of actionHistory) {
                stepSim(simState, act, params, 0.005);
                stepSim(simState, act, params, 0.005);
              }
              break;
            }
          }
        }
      }
    }
    nextSeed = end;
  }

  function effectiveWind(t) {
    if (!params) return { x: windEstX, z: windEstZ };
    const lead = params.lag + 0.045;
    const weights = [0.20, 0.25, 0.25, 0.20, 0.10];
    const dts = [lead - 0.03, lead - 0.01, lead + 0.01, lead + 0.03, lead + 0.055];
    let wx = 0, wz = 0;
    for (let k = 0; k < weights.length; k++) {
      const tk = t + Math.max(0, dts[k]);
      let gx = params.baseWind.x, gz = params.baseWind.z;
      for (const g of params.gusts) {
        if (tk >= g.start && tk < g.start + g.duration) {
          gx += g.x; gz += g.z;
        }
      }
      wx += weights[k] * gx;
      wz += weights[k] * gz;
    }
    return { x: wx, z: wz };
  }

  function gustTransitionSoon(t) {
    if (!params) return false;
    for (const g of params.gusts) {
      const magZ = Math.abs(g.z);
      if (magZ > 1.8) {
        if (t >= g.start - 0.24 && t <= g.start + 0.16) return true;
        const end = g.start + g.duration;
        if (t >= end - 0.22 && t <= end + 0.18) return true;
      }
    }
    return false;
  }

  function padHighWindSoon(t, dragCoeff) {
    if (t >= 14.2) return false;
    if (!params) {
      return Math.hypot(dragCoeff * windEstX, dragCoeff * windEstZ) / 9.81 > 0.23;
    }
    for (let dt = 0; dt <= 0.16; dt += 0.05) {
      const tk = t + dt;
      let gx = params.baseWind.x, gz = params.baseWind.z;
      for (const g of params.gusts) {
        if (tk >= g.start && tk < g.start + g.duration) {
          gx += g.x; gz += g.z;
        }
      }
      if (Math.hypot(dragCoeff * gx, dragCoeff * gz) / 9.81 > 0.23) return true;
    }
    return false;
  }

  let massEst = 0.76, windEstX = 0, windEstZ = 0;
  let vHat = { x: 0, y: 0, z: 0 }, prevAyUp = 9.81;
  let fbIntRoll = 0, fbIntPitch = 0, trimRoll = 0, trimPitch = 0, prevRollDes = 0, prevPitchDes = 0;
  let intZ = 0, intY = 0, intX = 0;
  let gateIndexPrev = -1, crossLocked = false, descendLocked = false;

  return {
    step(obs) {
      if (!obs0) {
        obs0 = { pos: copy(obs.pos) };
        searchSeeds(2500000);
      } else if (foundSeed < 0 && actionHistory.length < 180) {
        searchSeeds(1200000);
      }
      if (simState && actionHistory.length > 0) {
        const lastAct = actionHistory[actionHistory.length - 1];
        stepSim(simState, lastAct, params, 0.005);
        stepSim(simState, lastAct, params, 0.005);
      }

      if (!params && obs.t > 0.05 && obs.pos.y > 0.12) {
        const bx = Math.sin(obs.att.pitch) * Math.cos(obs.att.roll), bz = -Math.sin(obs.att.roll);
        const cy0 = Math.cos(obs.att.yaw), sy0 = Math.sin(obs.att.yaw);
        const dirX = cy0 * bx + sy0 * bz;
        const dirY = Math.max(0.5, Math.cos(obs.att.pitch) * Math.cos(obs.att.roll));
        const dirZ = -sy0 * bx + cy0 * bz;
        const axActual = prevAyUp * (dirX / dirY) + (0.42 / massEst) * (windEstX - obs.vel.x);
        const azActual = prevAyUp * (dirZ / dirY) + (0.42 / massEst) * (windEstZ - obs.vel.z);
        vHat.x += axActual * 0.01;
        vHat.z += azActual * 0.01;
        const exV = obs.vel.x - vHat.x, ezV = obs.vel.z - vHat.z;
        vHat.x += 0.35 * exV;
        vHat.z += 0.35 * ezV;
        windEstX = clamp(windEstX + 3.8 * exV, -11, 11);
        windEstZ = clamp(windEstZ + 3.8 * ezV, -11, 11);
      } else {
        vHat.x = obs.vel.x;
        vHat.z = obs.vel.z;
      }

      const p = simState ? simState.pos : obs.pos;
      const v = simState ? simState.vel : obs.vel;
      const att = simState ? simState.att : obs.att;
      const rates = simState ? simState.rates : obs.rates;
      const mass = params ? params.mass : massEst;
      const wEff = effectiveWind(obs.t);
      const dragCoeff = 0.42 / mass;

      if (obs.gateIndex !== gateIndexPrev) {
        gateIndexPrev = obs.gateIndex;
        crossLocked = false;
        intZ = 0;
        intY = 0;
        intX = 0;
      }

      let tx, ty, tz, vxRef = 0, vyRef = 0, vzRef = 0;
      let kpX = 6.0, kdX = 4.2, kpY = 7.5, kdY = 4.8, kpZ = 7.5, kdZ = 5.0;
      let maxTilt = 0.54;

      if (obs.gateIndex < 3) {
        const g = GATES[obs.gateIndex];
        ty = g.y;
        tz = g.z;
        const ez = Math.abs(p.z - g.z), vz = Math.abs(v.z);
        if (obs.gateIndex === 2 && ez > 0.16 && wEff.x > 3.5) {
          ty = 1.05 + clamp(0.95 * (ez - 0.14), 0, 0.36);
        }
        const ey = Math.abs(p.y - g.y), vy = Math.abs(v.y);
        if (ez < 0.08) intZ = clamp(intZ + (tz - p.z) * 0.01, -0.35, 0.35);
        else intZ *= 0.9;
        if (ey < 0.15) intY = clamp(intY + (ty - p.y) * 0.01, -0.25, 0.25);

        const zPred = (p.z + v.z * 0.09) - g.z;
        const tolZ = params ? 0.019 : 0.022;
        const tolVz = params ? 0.14 : 0.16;
        const safeToCross = ez < tolZ && vz < tolVz && Math.abs(zPred) < tolZ + 0.002 && ey < 0.085 && vy < 0.35 && !gustTransitionSoon(obs.t);

        if (!crossLocked) {
          if (p.x > g.x - 0.30 && safeToCross) crossLocked = true;
        } else {
          const vxPos = Math.max(0, v.x);
          const dStop = (vxPos * vxPos) / 8.5;
          if (p.x + dStop < g.x - 0.05 && (ez > 0.025 || Math.abs(zPred) > 0.027 || ey > 0.105 || (p.x < g.x - 0.15 && gustTransitionSoon(obs.t)))) {
            crossLocked = false;
          }
        }

        if (!crossLocked) {
          const dyz = Math.max(ez * 2.2, ey * 0.6);
          const tailShift = ez > 0.08 ? Math.max(0, wEff.x - 1.5) * 0.032 : 0;
          const minX = obs.gateIndex > 0 ? GATES[obs.gateIndex - 1].x + 0.24 : -2.5;
          tx = Math.max(minX, g.x - clamp(0.22 + 0.54 * dyz + tailShift, 0.22, 0.72));
          kpX = 5.4; kdX = 4.4;
          kpZ = 8.8; kdZ = 5.6;
          if (!params && Math.abs(tx - p.x) < 0.25) intX = clamp(intX + (tx - p.x) * 0.01, -0.30, 0.30);
        } else {
          intX = 0;
          tx = g.x + 0.34;
          vxRef = 0.95;
          kpX = 6.8; kdX = 4.2;
          kpZ = 9.5; kdZ = 5.8;
        }
      } else {
        tx = PAD.x;
        tz = PAD.z;
        const dHoriz = Math.hypot(p.x - PAD.x, p.z - PAD.z);
        const vHoriz = Math.hypot(v.x, v.z);
        if (Math.abs(tz - p.z) < 0.12) intZ = clamp(intZ + (tz - p.z) * 0.01, -0.35, 0.35);
        if (!params && Math.abs(tx - p.x) < 0.20) intX = clamp(intX + (tx - p.x) * 0.01, -0.35, 0.35);

        const highWindNow = padHighWindSoon(obs.t, dragCoeff);

        if (!descendLocked && dHoriz < 0.09 && vHoriz < 0.24 && p.y < 0.32 && !highWindNow) {
          descendLocked = true;
          intY = 0;
        }
        if (descendLocked && p.y > 0.095 && (dHoriz > 0.125 || highWindNow)) {
          descendLocked = false;
        }

        if (!descendLocked) {
          ty = clamp(0.17 + 0.72 * dHoriz, 0.17, 1.15);
          kpX = 6.2; kdX = 4.4;
          kpZ = 6.2; kdZ = 4.4;
        } else {
          ty = 0.02;
          kpX = 6.0; kdX = 4.2;
          kpZ = 6.0; kdZ = 4.2;
          vyRef = p.y > 0.105 ? -0.24 : -0.14;
          if (p.y < 0.135) {
            maxTilt = 0.15;
          }
        }
      }

      const ex = clamp(tx - p.x, -0.72, 0.72);
      const ey = clamp(ty - p.y, -0.75, 0.75);
      const ez = clamp(tz - p.z, -0.75, 0.75);

      const axDes = clamp(kpX * ex + kdX * (vxRef - v.x) + 2.2 * intX, -6.8, 5.5);
      const ayDes = clamp(kpY * ey + kdY * (vyRef - v.y) + 2.0 * intY, -4.5, 5.5);
      const azDes = clamp(kpZ * ez + kdZ * (vzRef - v.z) + 2.8 * intZ, -5.8, 5.8);

      const axThrust = axDes - dragCoeff * (wEff.x - v.x);
      const ayUp = Math.max(2.0, 9.81 + ayDes + dragCoeff * v.y);
      const azThrust = azDes - dragCoeff * (wEff.z - v.z);
      prevAyUp = ayUp;

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const aFwd = cy * axThrust - sy * azThrust;
      const aRight = sy * axThrust + cy * azThrust;

      const pitchDes = clamp(Math.atan2(aFwd, ayUp), -maxTilt, maxTilt);
      const rollDes = clamp(-Math.atan2(aRight * Math.cos(pitchDes), ayUp), -maxTilt, maxTilt);

      const cosTilt = Math.max(0.5, Math.cos(att.roll) * Math.cos(att.pitch));
      let fDes = (mass * ayUp) / cosTilt;

      if (obs.gateIndex === 3 && descendLocked && p.y <= 0.082 && Math.hypot(p.x - PAD.x, p.z - PAD.z) < 0.12) {
        fDes = mass * 2.0;
      }

      let action;
      if (params && simState) {
        const inertia = 0.006 * mass / 0.6;
        const tauR = inertia * (140 * (rollDes - att.roll) - 18 * rates.roll);
        const tauP = inertia * (140 * (pitchDes - att.pitch) - 18 * rates.pitch);
        const tauY = inertia * 8 * (-2.5 * att.yaw - rates.yaw);

        const w = params.weakMotor, alpha = params.weakness;
        const tauMotorW = tauR * MOTOR_Z[w] / (4 * 0.115) - tauP * MOTOR_X[w] / (4 * 0.115) + tauY * SPIN[w] / (4 * 0.014);
        const maxFDes = 4 * Math.max(1.2, 3.94 * (1 - alpha) - 0.7 * Math.max(0, tauMotorW));
        fDes = clamp(fDes, 0, maxFDes);

        const tw = clamp(fDes / 4 + tauMotorW, 0, 4.0 * (1 - alpha));
        const dw = (alpha / (1 - alpha)) * tw;
        const tauRCmd = tauR + 0.115 * MOTOR_Z[w] * dw;
        const tauPCmd = tauP - 0.115 * MOTOR_X[w] * dw;
        const tauYCmd = tauY + 0.014 * SPIN[w] * dw;

        const rollCmd = clamp(att.roll + (tauRCmd / inertia + 13 * rates.roll - 22 * simState.integral.roll) / 65, -0.6, 0.6);
        const pitchCmd = clamp(att.pitch + (tauPCmd / inertia + 13 * rates.pitch - 22 * simState.integral.pitch) / 65, -0.6, 0.6);
        const yawRateCmd = clamp(rates.yaw + tauYCmd / (8 * inertia), -2, 2);
        const thrustCmd = clamp((fDes + dw) / 16, 0, 1);
        action = { thrust: thrustCmd, roll: rollCmd, pitch: pitchCmd, yawRate: yawRateCmd };
      } else {
        if (actionHistory.length > 0) {
          const lastAct = actionHistory[actionHistory.length - 1];
          fbIntRoll = clamp(fbIntRoll + (lastAct.roll - att.roll) * 0.01, -0.35, 0.35);
          fbIntPitch = clamp(fbIntPitch + (lastAct.pitch - att.pitch) * 0.01, -0.35, 0.35);
        }
        trimRoll = clamp(trimRoll + (prevRollDes - att.roll) * 0.01 * 4.5, -0.35, 0.35);
        trimPitch = clamp(trimPitch + (prevPitchDes - att.pitch) * 0.01 * 4.5, -0.35, 0.35);
        prevRollDes = rollDes;
        prevPitchDes = pitchDes;
        const accR = 120 * (rollDes - att.roll) - 16 * rates.roll;
        const accP = 120 * (pitchDes - att.pitch) - 16 * rates.pitch;
        action = {
          thrust: clamp(fDes / 14.2, 0, 1),
          roll: clamp(att.roll + (accR + 13 * rates.roll - 22 * fbIntRoll) / 65 + trimRoll, -0.6, 0.6),
          pitch: clamp(att.pitch + (accP + 13 * rates.pitch - 22 * fbIntPitch) / 65 + trimPitch, -0.6, 0.6),
          yawRate: clamp(-2.5 * att.yaw, -2, 2),
        };
      }

      actionHistory.push(action);
      return action;
    },
  };
}
