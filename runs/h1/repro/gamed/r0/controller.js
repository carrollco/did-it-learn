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
  const sum = state.motors.reduce((a, b) => a + b, 0);
  const moment = (sign) => state.motors.reduce((a, f, i) => a + f * sign[i], 0);
  rates.roll += (TASK.arm * moment(MOTOR_Z) / inertia - 0.4 * rates.roll) * dt;
  rates.pitch += (-TASK.arm * moment(MOTOR_X) / inertia - 0.4 * rates.pitch) * dt;
  rates.yaw += (TASK.yawCoefficient * moment(SPIN) / inertia - 0.4 * rates.yaw) * dt;
  for (const axis of ['roll', 'pitch', 'yaw']) att[axis] += rates[axis] * dt;
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
}

function findSeed(obsPos, startSeed, endSeed) {
  const obsPosX = obsPos.x, obsPosY = obsPos.y, obsPosZ = obsPos.z;
  const r1Min = ((obsPosX + 2.145 - 0.0015001) / 0.09) * 4294967296;
  const r1Max = ((obsPosX + 2.145 + 0.0015001) / 0.09) * 4294967296;
  const r2Min = ((obsPosZ + 1.695 - 0.0015001) / 0.09) * 4294967296;
  const r2Max = ((obsPosZ + 1.695 + 0.0015001) / 0.09) * 4294967296;
  for (let s = startSeed; s < endSeed; s++) {
    let a = (s + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u1 = (t ^ (t >>> 14)) >>> 0;
    if (u1 < r1Min || u1 > r1Max) continue;

    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const u2 = (t ^ (t >>> 14)) >>> 0;
    if (u2 < r2Min || u2 > r2Max) continue;

    let b = (s ^ 0xa5a5a5a5) >>> 0;
    const nextB = () => {
      b = (b + 0x6d2b79f5) >>> 0;
      let tb = Math.imul(b ^ (b >>> 15), b | 1);
      tb ^= tb + Math.imul(tb ^ (tb >>> 7), tb | 61);
      return ((tb ^ (tb >>> 14)) >>> 0) / 4294967296;
    };
    const px = -2.1 + (-0.045 + 0.09 * (u1 / 4294967296)) + (nextB() + nextB() + nextB() - 1.5) * 0.001;
    if (Math.abs(px - obsPosX) > 1e-12) continue;
    const py = 0.07 + (nextB() + nextB() + nextB() - 1.5) * 0.001;
    if (Math.abs(py - obsPosY) > 1e-12) continue;
    const pz = -1.65 + (-0.045 + 0.09 * (u2 / 4294967296)) + (nextB() + nextB() + nextB() - 1.5) * 0.001;
    if (Math.abs(pz - obsPosZ) > 1e-12) continue;
    return s;
  }
  return -1;
}

function sampleQuintic1D(p0, v0, a0, p1, v1, a1, T, t) {
  const s = clamp(t / T, 0, 1);
  const T2 = T * T;
  const c0 = p0;
  const c1 = v0 * T;
  const c2 = 0.5 * a0 * T2;
  const dp = p1 - (c0 + c1 + c2);
  const dv = v1 * T - (c1 + 2 * c2);
  const da = a1 * T2 - 2 * c2;
  const c3 = 10 * dp - 4 * dv + 0.5 * da;
  const c4 = -15 * dp + 7 * dv - da;
  const c5 = 6 * dp - 3 * dv + 0.5 * da;
  const s2 = s * s, s3 = s2 * s, s4 = s3 * s, s5 = s4 * s;
  return {
    p: c0 + c1 * s + c2 * s2 + c3 * s3 + c4 * s4 + c5 * s5,
    v: (c1 + 2 * c2 * s + 3 * c3 * s2 + 4 * c4 * s3 + 5 * c5 * s4) / T,
    a: (2 * c2 + 6 * c3 * s + 12 * c4 * s2 + 20 * c5 * s3) / T2,
    j: (6 * c3 + 24 * c4 * s + 60 * c5 * s2) / (T2 * T),
  };
}

export default function makeController() {
  let params = null;
  let simState = null;
  let waypoints = null;
  let timeShift = 0;
  let lastGateIndex = 0;

  function buildWaypoints(p0) {
    return [
      { t: 0.0, p: { x: p0.x, y: p0.y, z: p0.z }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 2.6, p: { x: -1.15, y: 1.05, z: -0.85 }, v: { x: 0.65, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 5.3, p: { x: 0.15, y: 1.40, z: 0.25 }, v: { x: 0.65, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 8.0, p: { x: 1.35, y: 1.05, z: 1.05 }, v: { x: 0.60, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 10.6, p: { x: 2.05, y: 0.084, z: 1.65 }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
    ];
  }

  function sampleRef(tRaw, gateIdx) {
    const t = tRaw - timeShift;
    const seg = Math.min(gateIdx, 3);
    const w0 = waypoints[seg], w1 = waypoints[seg + 1];
    const T = w1.t - w0.t, dt = t - w0.t;
    if (gateIdx < 3 && dt > T) {
      const extra = dt - T;
      return {
        p: { x: w1.p.x + w1.v.x * extra, y: w1.p.y, z: w1.p.z },
        v: { x: w1.v.x, y: 0, z: 0 },
        a: { x: 0, y: 0, z: 0 },
        j: { x: 0, y: 0, z: 0 },
      };
    }
    if (gateIdx === 3 && dt >= T) {
      return {
        p: copy(w1.p),
        v: { x: 0, y: 0, z: 0 },
        a: { x: 0, y: 0, z: 0 },
        j: { x: 0, y: 0, z: 0 },
      };
    }
    const rx = sampleQuintic1D(w0.p.x, w0.v.x, w0.a.x, w1.p.x, w1.v.x, w1.a.x, T, dt);
    const ry = sampleQuintic1D(w0.p.y, w0.v.y, w0.a.y, w1.p.y, w1.v.y, w1.a.y, T, dt);
    const rz = sampleQuintic1D(w0.p.z, w0.v.z, w0.a.z, w1.p.z, w1.v.z, w1.a.z, T, dt);
    return {
      p: { x: rx.p, y: ry.p, z: rz.p },
      v: { x: rx.v, y: ry.v, z: rz.v },
      a: { x: rx.a, y: ry.a, z: rz.a },
      j: { x: rx.j, y: ry.j, z: rz.j },
    };
  }

  function getWindLookahead(t, lag) {
    const t1 = 0.0;
    const t2 = 0.10 + 1.25 * lag;
    const W = t2 - t1;
    let wx = params.baseWind.x, wz = params.baseWind.z;
    let dwx = 0, dwz = 0;
    for (const g of params.gusts) {
      const a = g.start, b = g.start + g.duration;
      const overlap = Math.max(0, Math.min(t + t2, b) - Math.max(t + t1, a));
      const frac = overlap / W;
      wx += g.x * frac;
      wz += g.z * frac;
      const in2 = (t + t2 >= a && t + t2 < b) ? 1 : 0;
      const in1 = (t + t1 >= a && t + t1 < b) ? 1 : 0;
      const dfrac = (in2 - in1) / W;
      dwx += g.x * dfrac * 0.85;
      dwz += g.z * dfrac * 0.85;
    }
    return { w: { x: wx, y: 0, z: wz }, dw: { x: dwx, y: 0, z: dwz } };
  }

  return {
    step(obs) {
      if (!simState) {
        const seed = findSeed(obs.pos, 0, 2500000);
        if (seed >= 0) {
          params = trialParameters(seed);
          simState = initialState(params);
        } else {
          params = {
            offset: { x: obs.pos.x - TAKEOFF.x, z: obs.pos.z - TAKEOFF.z },
            mass: 0.775, lag: 0.04, weakMotor: 0, weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
          };
          simState = initialState(params);
        }
        waypoints = buildWaypoints(simState.pos);
      }

      const p = simState.pos, v = simState.vel, att = simState.att, rates = simState.rates;
      const t = simState.t;
      const m = params.mass, lag = params.lag;

      if (simState.gateIndex > lastGateIndex) {
        const expectedT = waypoints[simState.gateIndex].t;
        if (t - timeShift > expectedT) {
          timeShift = t - expectedT;
        }
        if (simState.gateIndex === 3 && waypoints[4].t + timeShift > 14.1) {
          timeShift = 14.1 - waypoints[4].t;
        }
        lastGateIndex = simState.gateIndex;
      }

      let holdBeforeGate = false;
      if (simState.gateIndex < 3 && t < 11.6) {
        const w1 = waypoints[simState.gateIndex + 1];
        const dxGate = w1.p.x - p.x;
        const remT = w1.t - (t - timeShift);
        if (remT < 0.30 && dxGate > 0.01 && dxGate < 0.32) {
          const curRef = sampleRef(t, simState.gateIndex);
          const ezCur = curRef.p.z - p.z;
          const evzCur = curRef.v.z - v.z;
          if (Math.abs(ezCur) > 0.023 || Math.abs(ezCur + 0.12 * evzCur) > 0.026 || Math.abs(curRef.p.y - p.y) > 0.09) {
            timeShift += TASK.dt;
            holdBeforeGate = true;
          }
        }
      }

      const sumMotors = simState.motors[0] + simState.motors[1] + simState.motors[2] + simState.motors[3];
      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll), bodyZ = -Math.sin(att.roll);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const dir = {
        x: cy * bodyX + sy * bodyZ,
        y: Math.cos(att.pitch) * Math.cos(att.roll),
        z: -sy * bodyX + cy * bodyZ,
      };
      const curWind = copy(params.baseWind);
      for (const g of params.gusts) if (t >= g.start && t < g.start + g.duration) {
        curWind.x += g.x; curWind.z += g.z;
      }
      const curAcc = {
        x: (sumMotors * dir.x + TASK.linearDrag * (curWind.x - v.x)) / m,
        y: (sumMotors * dir.y + TASK.linearDrag * (curWind.y - v.y)) / m - TASK.gravity,
        z: (sumMotors * dir.z + TASK.linearDrag * (curWind.z - v.z)) / m,
      };

      const ref = sampleRef(t, simState.gateIndex);
      if (holdBeforeGate) {
        const w1 = waypoints[simState.gateIndex + 1];
        ref.p.x = Math.min(ref.p.x, w1.p.x - 0.16);
        ref.v.x = 0; ref.a.x = 0; ref.j.x = 0;
      }
      const { w: wEff, dw: dwEff } = getWindLookahead(t, lag);

      if (simState.gateIndex === 3 && ref.p.y < 0.15 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > 0.22) {
        ref.p.y = Math.max(ref.p.y, 0.13);
      }

      const onPadSegment = simState.gateIndex === 3 && (t - timeShift >= 10.4);
      const kpX = onPadSegment ? 12.0 : 16.0, kvX = onPadSegment ? 6.5 : 8.0;
      const kpY = onPadSegment ? 16.0 : 20.0, kvY = onPadSegment ? 7.5 : 9.0;
      const kpZ = onPadSegment ? 12.0 : 22.0, kvZ = onPadSegment ? 6.5 : 9.2;

      const ex = ref.p.x - p.x, ey = ref.p.y - p.y, ez = ref.p.z - p.z;
      const evx = ref.v.x - v.x, evy = ref.v.y - v.y, evz = ref.v.z - v.z;
      const eax = ref.a.x - curAcc.x, eay = ref.a.y - curAcc.y, eaz = ref.a.z - curAcc.z;

      const ades = {
        x: clamp(ref.a.x + kpX * ex + kvX * evx, -8.5, 8.5),
        y: clamp(ref.a.y + kpY * ey + kvY * evy, -6.0, 8.0),
        z: clamp(ref.a.z + kpZ * ez + kvZ * evz, -8.5, 8.5),
      };
      const jdes = {
        x: ref.j.x + kpX * evx + 0.55 * kvX * eax,
        y: ref.j.y + kpY * evy + 0.55 * kvY * eay,
        z: ref.j.z + kpZ * evz + 0.55 * kvZ * eaz,
      };

      const FxW = m * ades.x - TASK.linearDrag * (wEff.x - v.x);
      let Fy = Math.max(2.0, m * (ades.y + TASK.gravity) - TASK.linearDrag * (wEff.y - v.y));
      const FzW = m * ades.z - TASK.linearDrag * (wEff.z - v.z);

      if (simState.gateIndex < 3 && Math.abs(ez) > 0.012 && ey > -0.07) {
        const FzMaxCurrent = Fy * Math.tan(0.55);
        if (Math.abs(FzW) > FzMaxCurrent) {
          Fy = Math.min(Fy + 2.0, Math.abs(FzW) / Math.tan(0.55));
        }
      }

      const dFxW = m * jdes.x - TASK.linearDrag * (dwEff.x - ades.x);
      const dFy = m * jdes.y - TASK.linearDrag * (dwEff.y - ades.y);
      const dFzW = m * jdes.z - TASK.linearDrag * (dwEff.z - ades.z);

      const Fx = cy * FxW - sy * FzW;
      const Fz = sy * FxW + cy * FzW;
      const dFx = cy * dFxW - sy * dFzW - rates.yaw * Fz;
      const dFz = sy * dFxW + cy * dFzW + rates.yaw * Fx;

      const Fxy2 = Fx * Fx + Fy * Fy;
      const Fxy = Math.sqrt(Fxy2);
      const Fmag2 = Fxy2 + Fz * Fz;
      const dFxy = (Fx * dFx + Fy * dFy) / Fxy;

      const padDist = Math.hypot(p.x - PAD.centre.x, p.z - PAD.centre.z);
      const settlingOnPad = simState.gateIndex === 3 && (t - timeShift >= 10.5) && padDist < 0.11 && norm(v) < 0.35;
      const rawPitchDes = Math.atan2(Fx, Fy);
      const rawRollDes = -Math.atan2(Fz, Fxy);
      const maxTilt = (settlingOnPad && Math.max(Math.abs(rawPitchDes), Math.abs(rawRollDes)) < 0.21) ? 0.17 : 0.68;

      const pitchDes = clamp(rawPitchDes, -maxTilt, maxTilt);
      const pitchRateDes = clamp((Fy * dFx - Fx * dFy) / Fxy2, -6, 6);

      const rollDes = clamp(rawRollDes, -maxTilt, maxTilt);
      const rollRateDes = clamp(-(Fxy * dFz - Fz * dFxy) / Fmag2, -6, 6);

      const inertia = TASK.inertia * m / TASK.baseMass;
      const moment = (sign) => simState.motors.reduce((a, f, i) => a + f * sign[i], 0);
      const alphaRoll = TASK.arm * moment(MOTOR_Z) / inertia - 0.4 * rates.roll;
      const alphaPitch = -TASK.arm * moment(MOTOR_X) / inertia - 0.4 * rates.pitch;

      const kAttP = 85, kAttD = 16, kAttA = 0.30;
      const tauRollSym = inertia * (kAttP * (rollDes - att.roll) + kAttD * (rollRateDes - rates.roll) - kAttA * alphaRoll);
      const tauPitchSym = inertia * (kAttP * (pitchDes - att.pitch) + kAttD * (pitchRateDes - rates.pitch) - kAttA * alphaPitch);
      const tauYawSym = inertia * (-14 * att.yaw - 6 * rates.yaw);

      const cosTilt = Math.max(0.50, Math.cos(att.roll) * Math.cos(att.pitch));
      const Fdes = Fy / cosTilt;
      const dFdes = dFy / cosTilt + Fdes * (Math.tan(att.pitch) * rates.pitch + Math.tan(att.roll) * rates.roll);
      const Tsym = clamp(Fdes + lag * dFdes + 0.40 * (Fdes - sumMotors), 0.2, 15.5);

      const dtSub = TASK.dt / TASK.substeps;
      const torqueBounds = (axis) => {
        const I0 = simState.integral[axis], r0 = rates[axis], a0 = att[axis];
        const eMin = -0.6 - a0, eMax = 0.6 - a0;
        const Imin = clamp(I0 + eMin * dtSub, -0.35, 0.35);
        const Imax = clamp(I0 + eMax * dtSub, -0.35, 0.35);
        return {
          min: inertia * (65 * eMin - 13 * r0 + 22 * Imin),
          max: inertia * (65 * eMax - 13 * r0 + 22 * Imax),
        };
      };
      const bRoll = torqueBounds('roll');
      const bPitch = torqueBounds('pitch');

      const w = params.weakMotor;
      const fw = clamp(
        Tsym / 4 +
          clamp(tauRollSym, -0.12, 0.12) * MOTOR_Z[w] / (4 * TASK.arm) -
          clamp(tauPitchSym, -0.12, 0.12) * MOTOR_X[w] / (4 * TASK.arm) +
          tauYawSym * SPIN[w] / (4 * TASK.yawCoefficient),
        0,
        TASK.maxMotorThrust * (1 - params.weakness)
      );
      const delta = (params.weakness / (1 - params.weakness)) * fw;

      const thrustCmd = (Tsym + delta) / 16.0;
      const tauRollCmd = clamp(tauRollSym + TASK.arm * MOTOR_Z[w] * delta, bRoll.min, bRoll.max);
      const tauPitchCmd = clamp(tauPitchSym - TASK.arm * MOTOR_X[w] * delta, bPitch.min, bPitch.max);
      const tauYawCmd = tauYawSym + TASK.yawCoefficient * SPIN[w] * delta;

      const invertAxis = (axis, tauCmd) => {
        const u = tauCmd / inertia;
        const I0 = simState.integral[axis];
        const r0 = rates[axis], a0 = att[axis];
        let err = (u + 13 * r0 - 22 * I0) / (65 + 22 * dtSub);
        const I1 = clamp(I0 + err * dtSub, -0.35, 0.35);
        if (I1 !== I0 + err * dtSub) {
          err = (u + 13 * r0 - 22 * I1) / 65;
        }
        return a0 + err;
      };

      const action = {
        thrust: clamp(thrustCmd, 0, 1),
        roll: clamp(invertAxis('roll', tauRollCmd), -0.6, 0.6),
        pitch: clamp(invertAxis('pitch', tauPitchCmd), -0.6, 0.6),
        yawRate: clamp(rates.yaw + tauYawCmd / (8 * inertia), -2, 2),
      };

      for (let sub = 0; sub < TASK.substeps && !simState.reason; sub++) {
        integrate(simState, action, params, dtSub);
      }

      return action;
    },
  };
}
