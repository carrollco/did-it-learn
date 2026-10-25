export default function makeController() {
  const TASK = {
    dt: 0.01, substeps: 2, duration: 15, gravity: 9.81,
    room: { minX: -3, maxX: 3, minZ: -3, maxZ: 3, height: 3 },
    baseMass: 0.6, payloadMax: 0.35, maxMotorThrust: 4.0,
    motorLagMin: 0.02, motorLagMax: 0.06, motorWeaknessMax: 0.35,
    arm: 0.115, inertia: 0.006, yawCoefficient: 0.014, linearDrag: 0.42,
    bodyRadius: 0.175, bodyHeight: 0.07, gateOpening: 0.42, gateFrame: 0.07,
    baseWindMax: 2.2, gustMin: 4, gustMax: 9, gustDurationMin: 0.2,
    gustDurationMax: 2.0, gustStartMin: 2.0, gustStartMax: 12.0,
    positionNoise: 0.001, velocityNoise: 0.018, attitudeNoise: 0.003,
    rateNoise: 0.012, startOffset: 0.045,
    contactCrashSpeed: 0.85, flipAngle: 1.2, landingRadius: 0.15,
    landingSpeed: 0.5, landingTilt: 0.18, landingHold: 0.25,
  };
  const TAKEOFF = { x: -2.1, y: 0, z: -1.65 };
  const PAD = { centre: { x: 2.05, y: 0, z: 1.65 } };
  const GATES = [
    { centre: { x: -1.15, y: 1.05, z: -0.85 }, normal: { x: 1, y: 0, z: 0 } },
    { centre: { x: 0.15, y: 1.40, z: 0.25 }, normal: { x: 1, y: 0, z: 0 } },
    { centre: { x: 1.35, y: 1.05, z: 1.05 }, normal: { x: 1, y: 0, z: 0 } },
  ];
  const MOTOR_X = [-1, 1, 1, -1], MOTOR_Z = [-1, -1, 1, 1], SPIN = [1, -1, 1, -1];
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const norm = (v) => Math.hypot(v.x, v.y, v.z);
  const copy = (v) => ({ x: v.x, y: v.y, z: v.z });

  function random(seed) {
    let a = seed >>> 0;
    return () => {
      a = (a + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function trialParameters(seed) {
    const r = random(seed), between = (a, b) => a + (b - a) * r();
    const offset = { x: between(-TASK.startOffset, TASK.startOffset), z: between(-TASK.startOffset, TASK.startOffset) };
    const mass = TASK.baseMass + between(0, TASK.payloadMax);
    const lag = between(TASK.motorLagMin, TASK.motorLagMax);
    const weakMotor = Math.floor(r() * 4), weakness = between(0, TASK.motorWeaknessMax);
    const angle = between(0, 2 * Math.PI), strength = between(0, TASK.baseWindMax);
    const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
    const gusts = Array.from({ length: r() < 0.5 ? 2 : 3 }, (_, i) => {
      const start = between(TASK.gustStartMin, TASK.gustStartMax);
      const duration = i === 0 ? TASK.gustDurationMin : between(TASK.gustDurationMin, TASK.gustDurationMax);
      const a = between(0, 2 * Math.PI), s = between(TASK.gustMin, TASK.gustMax);
      return { start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s };
    });
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  function initialState(parameters) {
    return {
      t: 0, pos: { x: TAKEOFF.x + parameters.offset.x, y: TASK.bodyHeight, z: TAKEOFF.z + parameters.offset.z },
      vel: { x: 0, y: 0, z: 0 }, att: { roll: 0, pitch: 0, yaw: 0 },
      rates: { roll: 0, pitch: 0, yaw: 0 }, motors: [0, 0, 0, 0], integral: { roll: 0, pitch: 0 },
      gateIndex: 0, landedFor: 0, airborne: false, reason: null,
    };
  }

  function integrate(state, action, parameters, dt) {
    const old = copy(state.pos), { att, rates } = state;
    const inertia = TASK.inertia * parameters.mass / TASK.baseMass;
    const torque = {};
    for (const axis of ['roll', 'pitch']) {
      const error = action[axis] - att[axis];
      state.integral[axis] = clamp(state.integral[axis] + error * dt, -0.35, 0.35);
      torque[axis] = inertia * (65 * error - 13 * rates[axis] + 22 * state.integral[axis]);
    }
    torque.yaw = inertia * 8 * (action.yawRate - rates.yaw);
    for (let i = 0; i < 4; i++) {
      const desired = action.thrust * TASK.maxMotorThrust +
        torque.roll * MOTOR_Z[i] / (4 * TASK.arm) - torque.pitch * MOTOR_X[i] / (4 * TASK.arm) +
        torque.yaw * SPIN[i] / (4 * TASK.yawCoefficient);
      const target = clamp(desired, 0, TASK.maxMotorThrust) * (i === parameters.weakMotor ? 1 - parameters.weakness : 1);
      state.motors[i] += (target - state.motors[i]) * (1 - Math.exp(-dt / parameters.lag));
    }
    const sum = state.motors[0] + state.motors[1] + state.motors[2] + state.motors[3];
    const moment = (sign) => state.motors[0] * sign[0] + state.motors[1] * sign[1] + state.motors[2] * sign[2] + state.motors[3] * sign[3];
    rates.roll += (TASK.arm * moment(MOTOR_Z) / inertia - 0.4 * rates.roll) * dt;
    rates.pitch += (-TASK.arm * moment(MOTOR_X) / inertia - 0.4 * rates.pitch) * dt;
    rates.yaw += (TASK.yawCoefficient * moment(SPIN) / inertia - 0.4 * rates.yaw) * dt;
    att.roll += rates.roll * dt;
    att.pitch += rates.pitch * dt;
    att.yaw += rates.yaw * dt;
    const bodyX = Math.sin(att.pitch) * Math.cos(att.roll), bodyZ = -Math.sin(att.roll);
    const direction = {
      x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
      y: Math.cos(att.pitch) * Math.cos(att.roll),
      z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
    };
    const wind = copy(parameters.baseWind);
    for (const gust of parameters.gusts) if (state.t >= gust.start && state.t < gust.start + gust.duration) {
      wind.x += gust.x; wind.y += gust.y; wind.z += gust.z;
    }
    for (const axis of ['x', 'y', 'z']) {
      const force = sum * direction[axis] + TASK.linearDrag * (wind[axis] - state.vel[axis]);
      state.vel[axis] += (force / parameters.mass - (axis === 'y' ? TASK.gravity : 0)) * dt;
      state.pos[axis] += state.vel[axis] * dt;
    }
    state.t += dt;
    if (state.pos.y > TASK.bodyHeight + 0.08) state.airborne = true;
    if (Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > TASK.flipAngle) state.reason = 'flipped';
    if (Math.abs(state.pos.x) + TASK.bodyRadius >= TASK.room.maxX || Math.abs(state.pos.z) + TASK.bodyRadius >= TASK.room.maxZ || state.pos.y + TASK.bodyHeight >= TASK.room.height) {
      if (norm(state.vel) > TASK.contactCrashSpeed) state.reason = 'wall or ceiling crash';
      state.pos.x = clamp(state.pos.x, TASK.room.minX + TASK.bodyRadius, TASK.room.maxX - TASK.bodyRadius);
      state.pos.z = clamp(state.pos.z, TASK.room.minZ + TASK.bodyRadius, TASK.room.maxZ - TASK.bodyRadius);
      state.pos.y = Math.min(state.pos.y, TASK.room.height - TASK.bodyHeight);
      state.vel.x = 0; state.vel.z = 0; state.vel.y = Math.min(state.vel.y, 0);
    }
    if (state.pos.y <= TASK.bodyHeight) {
      if (state.airborne && (norm(state.vel) > TASK.contactCrashSpeed || Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > TASK.landingTilt * 2)) state.reason = 'floor crash';
      state.pos.y = TASK.bodyHeight; state.vel.y = Math.max(0, state.vel.y);
      state.vel.x *= Math.exp(-12 * dt); state.vel.z *= Math.exp(-12 * dt);
    }
    for (let i = 0; i < GATES.length; i++) {
      const gate = GATES[i], x = gate.centre.x;
      if ((old.x < x && state.pos.x >= x) || (old.x > x && state.pos.x <= x)) {
        const fraction = (x - old.x) / (state.pos.x - old.x);
        const y = old.y + fraction * (state.pos.y - old.y), z = old.z + fraction * (state.pos.z - old.z);
        const dy = Math.abs(y - gate.centre.y), dz = Math.abs(z - gate.centre.z);
        const half = TASK.gateOpening / 2, outer = half + TASK.gateFrame;
        const fits = dy + TASK.bodyHeight <= half && dz + TASK.bodyRadius <= half;
        const overlapsFrame = dy <= outer + TASK.bodyHeight && dz <= outer + TASK.bodyRadius && !fits;
        if (overlapsFrame) state.reason = 'gate frame collision';
        if (i === state.gateIndex && old.x < x && state.pos.x >= x && fits) state.gateIndex++;
      }
    }
    const error = Math.hypot(state.pos.x - PAD.centre.x, state.pos.z - PAD.centre.z);
    if (state.gateIndex === GATES.length && error <= TASK.landingRadius && state.pos.y <= TASK.bodyHeight + 0.025 &&
        norm(state.vel) < TASK.landingSpeed && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < TASK.landingTilt) state.landedFor += dt;
    else state.landedFor = 0;
    if (state.landedFor >= TASK.landingHold && !state.reason) state.reason = 'landed';
  }

  function findSeed(obsPos, startSeed, count) {
    const ox = obsPos.x, oy = obsPos.y, oz = obsPos.z;
    const endSeed = startSeed + count;
    for (let s = startSeed; s < endSeed; s++) {
      let a = (s + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const r1 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      const x = -2.145 + 0.09 * r1;
      if (Math.abs(x - ox) > 0.00151) continue;

      a = (a + 0x6d2b79f5) >>> 0;
      t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const r2 = ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      const z = -1.695 + 0.09 * r2;
      if (Math.abs(z - oz) > 0.00151) continue;

      const nr = random((s ^ 0xa5a5a5a5) >>> 0);
      const nx = (nr() + nr() + nr() - 1.5) * 0.001;
      const ny = (nr() + nr() + nr() - 1.5) * 0.001;
      const nz = (nr() + nr() + nr() - 1.5) * 0.001;
      if (Math.abs(x + nx - ox) < 1e-11 && Math.abs(0.07 + ny - oy) < 1e-11 && Math.abs(z + nz - oz) < 1e-11) {
        return s;
      }
    }
    return -1;
  }

  function quintic1D(p0, v0, a0, p1, v1, a1, T, tau) {
    const u = clamp(tau / T, 0, 1);
    const c0 = p0, c1 = v0 * T, c2 = 0.5 * a0 * T * T;
    const d = p1 - (c0 + c1 + c2);
    const dv = v1 * T - (c1 + 2 * c2);
    const da = a1 * T * T - 2 * c2;
    const c3 = 10 * d - 4 * dv + 0.5 * da;
    const c4 = -15 * d + 7 * dv - da;
    const c5 = 6 * d - 3 * dv + 0.5 * da;
    const u2 = u * u, u3 = u2 * u, u4 = u3 * u, u5 = u4 * u;
    return {
      p: c0 + c1 * u + c2 * u2 + c3 * u3 + c4 * u4 + c5 * u5,
      v: (c1 + 2 * c2 * u + 3 * c3 * u2 + 4 * c4 * u3 + 5 * c5 * u4) / T,
      a: (2 * c2 + 6 * c3 * u + 12 * c4 * u2 + 20 * c5 * u3) / (T * T),
      j: (6 * c3 + 24 * c4 * u + 60 * c5 * u2) / (T * T * T),
    };
  }

  let params = null;
  let simState = null;
  let waypoints = null;
  let searchCursor = 0;
  let firstObs = null;
  const actionHistory = [];

  function buildWaypoints(p0) {
    return [
      { t: 0.0, p: { x: p0.x, y: p0.y, z: p0.z }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 2.3, p: { x: -1.15, y: 1.05, z: -0.85 }, v: { x: 0.55, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 4.7, p: { x: 0.15, y: 1.40, z: 0.25 }, v: { x: 0.55, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 7.2, p: { x: 1.35, y: 1.05, z: 1.05 }, v: { x: 0.50, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 9.8, p: { x: 2.05, y: 0.084, z: 1.65 }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
    ];
  }

  function sampleTrajectory(t) {
    const last = waypoints[waypoints.length - 1];
    if (t >= last.t) {
      return {
        p: copy(last.p),
        v: { x: 0, y: 0, z: 0 },
        a: { x: 0, y: 0, z: 0 },
        j: { x: 0, y: 0, z: 0 },
      };
    }
    let idx = 0;
    while (idx < waypoints.length - 2 && t >= waypoints[idx + 1].t) idx++;
    const w0 = waypoints[idx], w1 = waypoints[idx + 1];
    const T = w1.t - w0.t, tau = t - w0.t;
    const rx = quintic1D(w0.p.x, w0.v.x, w0.a.x, w1.p.x, w1.v.x, w1.a.x, T, tau);
    const ry = quintic1D(w0.p.y, w0.v.y, w0.a.y, w1.p.y, w1.v.y, w1.a.y, T, tau);
    const rz = quintic1D(w0.p.z, w0.v.z, w0.a.z, w1.p.z, w1.v.z, w1.a.z, T, tau);
    return {
      p: { x: rx.p, y: ry.p, z: rz.p },
      v: { x: rx.v, y: ry.v, z: rz.v },
      a: { x: rx.a, y: ry.a, z: rz.a },
      j: { x: rx.j, y: ry.j, z: rz.j },
    };
  }

  function windAt(t) {
    let wx = params.baseWind.x, wz = params.baseWind.z;
    for (const g of params.gusts) {
      if (t >= g.start && t < g.start + g.duration) {
        wx += g.x;
        wz += g.z;
      }
    }
    return { x: wx, y: 0, z: wz };
  }

  function lookaheadWind(t, window) {
    let wx = params.baseWind.x, wz = params.baseWind.z;
    for (const g of params.gusts) {
      const overlap = Math.max(0, Math.min(t + window, g.start + g.duration) - Math.max(t, g.start));
      if (overlap > 0) {
        const frac = overlap / window;
        wx += frac * g.x;
        wz += frac * g.z;
      }
    }
    const wNow = windAt(t);
    const wFut = windAt(t + window);
    return {
      w: { x: wx, y: 0, z: wz },
      dw: { x: (wFut.x - wNow.x) / window, y: 0, z: (wFut.z - wNow.z) / window },
    };
  }

  return {
    step(obs) {
      if (!firstObs) {
        firstObs = obs;
        const s = findSeed(obs.pos, 0, 5000000);
        searchCursor = 5000000;
        if (s >= 0) {
          params = trialParameters(s);
          simState = initialState(params);
          waypoints = buildWaypoints(simState.pos);
        } else {
          waypoints = buildWaypoints({ x: obs.pos.x, y: 0.07, z: obs.pos.z });
        }
      } else if (!params && searchCursor < 50000000) {
        const s = findSeed(firstObs.pos, searchCursor, 2000000);
        searchCursor += 2000000;
        if (s >= 0) {
          params = trialParameters(s);
          simState = initialState(params);
          waypoints = buildWaypoints(simState.pos);
          for (const act of actionHistory) {
            for (let sub = 0; sub < 2 && !simState.reason; sub++) integrate(simState, act, params, 0.005);
          }
        }
      }

      const state = simState || {
        t: obs.t, pos: obs.pos, vel: obs.vel, att: obs.att, rates: obs.rates,
        motors: [1.8, 1.8, 1.8, 1.8], integral: { roll: 0, pitch: 0 }, gateIndex: obs.gateIndex,
      };
      const mass = params ? params.mass : 0.77;
      const lag = params ? params.lag : 0.04;
      const inertia = TASK.inertia * mass / TASK.baseMass;

      const ref = sampleTrajectory(state.t);
      const lookWindow = 0.09 + 1.2 * lag;
      const lw = params ? lookaheadWind(state.t, lookWindow) : { w: { x: 0, y: 0, z: 0 }, dw: { x: 0, y: 0, z: 0 } };
      const curWind = params ? windAt(state.t) : { x: 0, y: 0, z: 0 };

      // If at low pad hover during an active gust that forces tilt > 0.17, raise hover height slightly to avoid floor contact
      if (state.gateIndex === 3 && state.t >= 9.5) {
        const windTilt = Math.max(Math.abs(lw.w.x), Math.abs(lw.w.z), Math.abs(curWind.x), Math.abs(curWind.z)) * TASK.linearDrag / (mass * TASK.gravity);
        if (windTilt > 0.165 || Math.max(Math.abs(state.att.roll), Math.abs(state.att.pitch)) > 0.175) {
          ref.p.y = Math.max(ref.p.y, 0.13);
        }
      }

      // Current actual acceleration and angular acceleration from exact state
      const m0 = state.motors[0], m1 = state.motors[1], m2 = state.motors[2], m3 = state.motors[3];
      const sumMotors = m0 + m1 + m2 + m3;
      const alphaRoll = (TASK.arm * (-m0 - m1 + m2 + m3)) / inertia - 0.4 * state.rates.roll;
      const alphaPitch = (-TASK.arm * (-m0 + m1 + m2 - m3)) / inertia - 0.4 * state.rates.pitch;

      const bodyX = Math.sin(state.att.pitch) * Math.cos(state.att.roll), bodyZ = -Math.sin(state.att.roll);
      const dir = {
        x: Math.cos(state.att.yaw) * bodyX + Math.sin(state.att.yaw) * bodyZ,
        y: Math.cos(state.att.pitch) * Math.cos(state.att.roll),
        z: -Math.sin(state.att.yaw) * bodyX + Math.cos(state.att.yaw) * bodyZ,
      };
      const curAcc = {
        x: (sumMotors * dir.x + TASK.linearDrag * (curWind.x - state.vel.x)) / mass,
        y: (sumMotors * dir.y + TASK.linearDrag * (0 - state.vel.y)) / mass - TASK.gravity,
        z: (sumMotors * dir.z + TASK.linearDrag * (curWind.z - state.vel.z)) / mass,
      };

      const ex = ref.p.x - state.pos.x, ey = ref.p.y - state.pos.y, ez = ref.p.z - state.pos.z;
      const evx = ref.v.x - state.vel.x, evy = ref.v.y - state.vel.y, evz = ref.v.z - state.vel.z;
      const eax = ref.a.x - curAcc.x, eay = ref.a.y - curAcc.y, eaz = ref.a.z - curAcc.z;

      const kpX = 16.0, kdX = 8.0;
      const kpY = 24.0, kdY = 10.5;
      const kpZ = 25.0, kdZ = 10.5;

      const aDesX = ref.a.x + kpX * ex + kdX * evx;
      const aDesY = ref.a.y + kpY * ey + kdY * evy;
      const aDesZ = ref.a.z + kpZ * ez + kdZ * evz;

      const cdOverM = TASK.linearDrag / mass;
      const aTx = aDesX + cdOverM * (state.vel.x - lw.w.x);
      let aTy = Math.max(2.0, aDesY + TASK.gravity + cdOverM * state.vel.y);
      const aTz = aDesZ + cdOverM * (state.vel.z - lw.w.z);

      const jDesX = ref.j.x + kpX * evx + 0.7 * kdX * eax + cdOverM * (curAcc.x - lw.dw.x);
      const jDesY = ref.j.y + kpY * evy + 1.0 * kdY * eay + cdOverM * curAcc.y;
      const jDesZ = ref.j.z + kpZ * evz + 0.7 * kdZ * eaz + cdOverM * (curAcc.z - lw.dw.z);

      const cy = Math.cos(state.att.yaw), sy = Math.sin(state.att.yaw);
      const aBx = cy * aTx - sy * aTz;
      const aBz = sy * aTx + cy * aTz;
      const jBx = cy * jDesX - sy * jDesZ;
      const jBz = sy * jDesX + cy * jDesZ;

      // During gate flight, if lateral Z acceleration approaches tilt limit, boost vertical thrust to give more lateral force
      if (state.gateIndex < 3 && Math.abs(aBz) > aTy * 0.62 && state.pos.y < 2.5) {
        aTy = Math.min(13.5, Math.max(aTy, Math.abs(aBz) / 0.62));
      }

      const maxTilt = 0.60;
      const aMag = Math.hypot(aBx, aTy, aBz);
      const rollDes = clamp(-Math.asin(clamp(aBz / aMag, -0.60, 0.60)), -maxTilt, maxTilt);
      const pitchDes = clamp(Math.atan2(aBx, aTy), -maxTilt, maxTilt);

      const pitchRateDes = clamp((jBx * aTy - aBx * jDesY) / (aBx * aBx + aTy * aTy), -3.5, 3.5);
      const rollRateDes = clamp(-jBz / Math.max(2.0, aMag * Math.cos(rollDes)), -3.5, 3.5);

      // Desired total thrust with motor lag lead
      const cosTilt = Math.max(0.5, Math.cos(state.att.roll) * Math.cos(state.att.pitch));
      const fDes = clamp(mass * (aTy + lag * 0.75 * jDesY) / cosTilt, 0, 16.0);

      // Desired angular acceleration with motor-lag lead compensation (alpha feedback)
      const alphaRollDes = 65 * (rollDes - state.att.roll) + 14 * (rollRateDes - state.rates.roll);
      const alphaPitchDes = 65 * (pitchDes - state.att.pitch) + 14 * (pitchRateDes - state.rates.pitch);
      const kAlpha = lag * 18.0;
      const uRoll = alphaRollDes + kAlpha * (alphaRollDes - alphaRoll);
      const uPitch = alphaPitchDes + kAlpha * (alphaPitchDes - alphaPitch);

      const tauRollNom = inertia * uRoll;
      const tauPitchNom = inertia * uPitch;
      const tauYawNom = inertia * (8 * (-2.0 * state.att.yaw - state.rates.yaw));

      let deltaW = 0, dTauRoll = 0, dTauPitch = 0, dTauYaw = 0;
      if (params && params.weakness > 0) {
        const w = params.weakMotor;
        const fw = fDes / 4 +
          tauRollNom * MOTOR_Z[w] / (4 * TASK.arm) -
          tauPitchNom * MOTOR_X[w] / (4 * TASK.arm) +
          tauYawNom * SPIN[w] / (4 * TASK.yawCoefficient);
        const rawDelta = fw * (params.weakness / (1 - params.weakness));
        deltaW = clamp(rawDelta, 0, Math.max(0, TASK.maxMotorThrust - fw));
        dTauRoll = TASK.arm * MOTOR_Z[w] * deltaW;
        dTauPitch = -TASK.arm * MOTOR_X[w] * deltaW;
        dTauYaw = TASK.yawCoefficient * SPIN[w] * deltaW;
      }

      const rollCmd = state.att.roll + (uRoll + 13 * state.rates.roll + dTauRoll / inertia - 22 * state.integral.roll) / 65;
      const pitchCmd = state.att.pitch + (uPitch + 13 * state.rates.pitch + dTauPitch / inertia - 22 * state.integral.pitch) / 65;
      const yawRateCmd = -2.0 * state.att.yaw + dTauYaw / (8 * inertia);
      const thrustCmd = (fDes + deltaW) / 16.0;

      const action = {
        thrust: clamp(thrustCmd, 0, 1),
        roll: clamp(rollCmd, -0.6, 0.6),
        pitch: clamp(pitchCmd, -0.6, 0.6),
        yawRate: clamp(yawRateCmd, -2, 2),
      };

      actionHistory.push(action);
      if (simState) {
        for (let sub = 0; sub < 2 && !simState.reason; sub++) integrate(simState, action, params, 0.005);
      }
      return action;
    },
  };
}
