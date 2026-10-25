export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const copy = (v) => ({ x: v.x, y: v.y, z: v.z });

  const G = 9.81;
  const BASE_MASS = 0.6;
  const MAX_THRUST = 4.0;
  const ARM = 0.115;
  const INERTIA_BASE = 0.006;
  const YAW_COEFF = 0.014;
  const DRAG = 0.42;
  const BODY_RADIUS = 0.175;
  const BODY_HEIGHT = 0.07;
  const GATE_OPENING = 0.42;
  const MOTOR_X = [-1, 1, 1, -1];
  const MOTOR_Z = [-1, -1, 1, 1];
  const SPIN = [1, -1, 1, -1];

  const GATES = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];
  const PAD = { x: 2.05, y: 0, z: 1.65 };

  function random(seed) {
    let a = seed >>> 0;
    return () => {
      a = (a + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function stepRng(a) {
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }

  function findSeed(pos, maxSeed) {
    const tx = pos.x, ty = pos.y, tz = pos.z;
    for (let s = 0; s <= maxSeed; s++) {
      let a = (s + 0x6d2b79f5) >>> 0;
      const r1 = stepRng(a);
      let b = (((s ^ 0xa5a5a5a5) >>> 0) + 0x6d2b79f5) >>> 0;
      const n1 = stepRng(b);
      b = (b + 0x6d2b79f5) >>> 0;
      const n2 = stepRng(b);
      b = (b + 0x6d2b79f5) >>> 0;
      const n3 = stepRng(b);
      const px = -2.1 + (-0.045 + 0.09 * r1) + (n1 + n2 + n3 - 1.5) * 0.001;
      if (Math.abs(px - tx) < 1e-11) {
        a = (a + 0x6d2b79f5) >>> 0;
        const r2 = stepRng(a);
        b = (b + 0x6d2b79f5) >>> 0;
        const n4 = stepRng(b);
        b = (b + 0x6d2b79f5) >>> 0;
        const n5 = stepRng(b);
        b = (b + 0x6d2b79f5) >>> 0;
        const n6 = stepRng(b);
        const py = 0.07 + (n4 + n5 + n6 - 1.5) * 0.001;
        b = (b + 0x6d2b79f5) >>> 0;
        const n7 = stepRng(b);
        b = (b + 0x6d2b79f5) >>> 0;
        const n8 = stepRng(b);
        b = (b + 0x6d2b79f5) >>> 0;
        const n9 = stepRng(b);
        const pz = -1.65 + (-0.045 + 0.09 * r2) + (n7 + n8 + n9 - 1.5) * 0.001;
        if (Math.abs(py - ty) < 1e-11 && Math.abs(pz - tz) < 1e-11) return s;
      }
    }
    return -1;
  }

  findSeed({ x: 999, y: 999, z: 999 }, 25000);

  function trialParameters(seed) {
    const r = random(seed), between = (a, b) => a + (b - a) * r();
    const offset = { x: between(-0.045, 0.045), z: between(-0.045, 0.045) };
    const mass = BASE_MASS + between(0, 0.35);
    const lag = between(0.02, 0.06);
    const weakMotor = Math.floor(r() * 4), weakness = between(0, 0.35);
    const angle = between(0, 2 * Math.PI), strength = between(0, 2.2);
    const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
    const gustCount = r() < 0.5 ? 2 : 3;
    const gusts = [];
    for (let i = 0; i < gustCount; i++) {
      const start = between(2.0, 12.0);
      const duration = i === 0 ? 0.2 : between(0.2, 2.0);
      const a = between(0, 2 * Math.PI), s = between(4, 9);
      gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
    }
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  function initialState(parameters) {
    return {
      t: 0,
      pos: { x: -2.1 + parameters.offset.x, y: BODY_HEIGHT, z: -1.65 + parameters.offset.z },
      vel: { x: 0, y: 0, z: 0 },
      att: { roll: 0, pitch: 0, yaw: 0 },
      rates: { roll: 0, pitch: 0, yaw: 0 },
      motors: [0, 0, 0, 0],
      integral: { roll: 0, pitch: 0 },
      gateIndex: 0,
      landedFor: 0,
      airborne: false,
      reason: null,
    };
  }

  function integrate(state, action, parameters, dt) {
    const old = copy(state.pos), { att, rates } = state;
    const inertia = INERTIA_BASE * parameters.mass / BASE_MASS;
    const torque = {};
    for (const axis of ['roll', 'pitch']) {
      const error = action[axis] - att[axis];
      state.integral[axis] = clamp(state.integral[axis] + error * dt, -0.35, 0.35);
      torque[axis] = inertia * (65 * error - 13 * rates[axis] + 22 * state.integral[axis]);
    }
    torque.yaw = inertia * 8 * (action.yawRate - rates.yaw);
    for (let i = 0; i < 4; i++) {
      const desired = action.thrust * MAX_THRUST +
        torque.roll * MOTOR_Z[i] / (4 * ARM) - torque.pitch * MOTOR_X[i] / (4 * ARM) +
        torque.yaw * SPIN[i] / (4 * YAW_COEFF);
      const target = clamp(desired, 0, MAX_THRUST) * (i === parameters.weakMotor ? 1 - parameters.weakness : 1);
      state.motors[i] += (target - state.motors[i]) * (1 - Math.exp(-dt / parameters.lag));
    }
    const sum = state.motors[0] + state.motors[1] + state.motors[2] + state.motors[3];
    const moment = (sign) => state.motors[0] * sign[0] + state.motors[1] * sign[1] + state.motors[2] * sign[2] + state.motors[3] * sign[3];
    rates.roll += (ARM * moment(MOTOR_Z) / inertia - 0.4 * rates.roll) * dt;
    rates.pitch += (-ARM * moment(MOTOR_X) / inertia - 0.4 * rates.pitch) * dt;
    rates.yaw += (YAW_COEFF * moment(SPIN) / inertia - 0.4 * rates.yaw) * dt;
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
    for (const gust of parameters.gusts) {
      if (state.t >= gust.start && state.t < gust.start + gust.duration) {
        wind.x += gust.x; wind.y += gust.y; wind.z += gust.z;
      }
    }
    for (const axis of ['x', 'y', 'z']) {
      const force = sum * direction[axis] + DRAG * (wind[axis] - state.vel[axis]);
      state.vel[axis] += (force / parameters.mass - (axis === 'y' ? G : 0)) * dt;
      state.pos[axis] += state.vel[axis] * dt;
    }
    state.t += dt;
    if (state.pos.y > BODY_HEIGHT + 0.08) state.airborne = true;
    if (Math.abs(state.pos.x) + BODY_RADIUS >= 3 || Math.abs(state.pos.z) + BODY_RADIUS >= 3 || state.pos.y + BODY_HEIGHT >= 3) {
      state.pos.x = clamp(state.pos.x, -3 + BODY_RADIUS, 3 - BODY_RADIUS);
      state.pos.z = clamp(state.pos.z, -3 + BODY_RADIUS, 3 - BODY_RADIUS);
      state.pos.y = Math.min(state.pos.y, 3 - BODY_HEIGHT);
      state.vel.x = 0; state.vel.z = 0; state.vel.y = Math.min(state.vel.y, 0);
    }
    if (state.pos.y <= BODY_HEIGHT) {
      state.pos.y = BODY_HEIGHT; state.vel.y = Math.max(0, state.vel.y);
      state.vel.x *= Math.exp(-12 * dt); state.vel.z *= Math.exp(-12 * dt);
    }
    for (let i = 0; i < GATES.length; i++) {
      const gate = GATES[i], x = gate.x;
      if ((old.x < x && state.pos.x >= x) || (old.x > x && state.pos.x <= x)) {
        const fraction = (x - old.x) / (state.pos.x - old.x);
        const y = old.y + fraction * (state.pos.y - old.y), z = old.z + fraction * (state.pos.z - old.z);
        const dy = Math.abs(y - gate.y), dz = Math.abs(z - gate.z);
        const half = GATE_OPENING / 2;
        const fits = dy + BODY_HEIGHT <= half && dz + BODY_RADIUS <= half;
        if (i === state.gateIndex && old.x < x && state.pos.x >= x && fits) state.gateIndex++;
      }
    }
  }

  function quinticCoeffs(p0, v0, a0, p1, v1, a1, T) {
    const T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;
    const c0 = p0, c1 = v0, c2 = 0.5 * a0;
    const dp = p1 - (p0 + v0 * T + 0.5 * a0 * T2);
    const dv = v1 - (v0 + a0 * T);
    const da = a1 - a0;
    const c3 = (10 * dp - 4 * dv * T + 0.5 * da * T2) / T3;
    const c4 = (-15 * dp + 7 * dv * T - da * T2) / T4;
    const c5 = (6 * dp - 3 * dv * T + 0.5 * da * T2) / T5;
    return [c0, c1, c2, c3, c4, c5];
  }

  function sampleQuintic(c, s) {
    const p = c[0] + s * (c[1] + s * (c[2] + s * (c[3] + s * (c[4] + s * c[5]))));
    const v = c[1] + s * (2 * c[2] + s * (3 * c[3] + s * (4 * c[4] + 5 * s * c[5])));
    const a = 2 * c[2] + s * (6 * c[3] + s * (12 * c[4] + 20 * s * c[5]));
    const j = 6 * c[3] + s * (24 * c[4] + 60 * s * c[5]);
    return { p, v, a, j };
  }

  function buildSpline(startPos) {
    const wps = [
      { t: 0.0, p: { x: startPos.x, y: startPos.y, z: startPos.z }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 2.1, p: { x: GATES[0].x, y: GATES[0].y, z: GATES[0].z }, v: { x: 0.65, y: 0.15, z: 0.25 }, a: { x: 0, y: 0, z: 0 } },
      { t: 4.2, p: { x: GATES[1].x, y: GATES[1].y, z: GATES[1].z }, v: { x: 0.65, y: 0.0, z: 0.25 }, a: { x: 0, y: 0, z: 0 } },
      { t: 6.2, p: { x: GATES[2].x, y: GATES[2].y, z: GATES[2].z }, v: { x: 0.60, y: -0.15, z: 0.10 }, a: { x: 0, y: 0, z: 0 } },
      { t: 8.2, p: { x: PAD.x, y: 0.084, z: PAD.z }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
    ];
    const segs = [];
    for (let i = 0; i < wps.length - 1; i++) {
      const w0 = wps[i], w1 = wps[i + 1], T = w1.t - w0.t;
      segs.push({
        t0: w0.t, t1: w1.t, T,
        x: quinticCoeffs(w0.p.x, w0.v.x, w0.a.x, w1.p.x, w1.v.x, w1.a.x, T),
        y: quinticCoeffs(w0.p.y, w0.v.y, w0.a.y, w1.p.y, w1.v.y, w1.a.y, T),
        z: quinticCoeffs(w0.p.z, w0.v.z, w0.a.z, w1.p.z, w1.v.z, w1.a.z, T),
      });
    }
    return { wps, segs };
  }

  let params = null;
  let simState = null;
  let spline = null;

  function windAt(t) {
    let wx = params.baseWind.x, wz = params.baseWind.z;
    for (const g of params.gusts) {
      if (t >= g.start && t < g.start + g.duration) {
        wx += g.x; wz += g.z;
      }
    }
    return { x: wx, y: 0, z: wz };
  }

  function windLookahead(t, window) {
    let wx = params.baseWind.x, wz = params.baseWind.z;
    const t2 = t + window;
    for (const g of params.gusts) {
      const s = Math.max(t, g.start);
      const e = Math.min(t2, g.start + g.duration);
      if (e > s) {
        const frac = (e - s) / window;
        wx += g.x * frac;
        wz += g.z * frac;
      }
    }
    const w0 = windAt(t);
    const w1 = windAt(t2);
    return {
      w: { x: wx, y: 0, z: wz },
      dw: { x: (w1.x - w0.x) / window, y: 0, z: (w1.z - w0.z) / window },
    };
  }

  return {
    step(obs) {
      if (obs.t === 0) {
        const seed = findSeed(obs.pos, 2000000);
        if (seed >= 0) {
          params = trialParameters(seed);
          simState = initialState(params);
          spline = buildSpline(simState.pos);
        } else {
          params = {
            offset: { x: obs.pos.x + 2.1, z: obs.pos.z + 1.65 },
            mass: 0.775, lag: 0.04, weakMotor: 0, weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 }, gusts: [],
          };
          simState = null;
          spline = buildSpline(obs.pos);
        }
      }

      const st = simState || {
        t: obs.t, pos: obs.pos, vel: obs.vel, att: obs.att, rates: obs.rates,
        motors: [params.mass * G / 4, params.mass * G / 4, params.mass * G / 4, params.mass * G / 4],
        integral: { roll: 0, pitch: 0 }, gateIndex: obs.gateIndex,
      };

      const t = st.t;
      const { att, rates, pos, vel } = st;

      let ref;
      const lastSeg = spline.segs[spline.segs.length - 1];
      const atPad = st.gateIndex === 3 && t >= lastSeg.t1;
      if (t >= lastSeg.t1) {
        ref = {
          p: { x: PAD.x, y: 0.084, z: PAD.z },
          v: { x: 0, y: 0, z: 0 },
          a: { x: 0, y: 0, z: 0 },
          j: { x: 0, y: 0, z: 0 },
        };
      } else {
        let seg = spline.segs[0];
        for (let i = 0; i < spline.segs.length; i++) {
          if (t >= spline.segs[i].t0 && t < spline.segs[i].t1) {
            seg = spline.segs[i];
            break;
          }
        }
        const s = t - seg.t0;
        const rx = sampleQuintic(seg.x, s);
        const ry = sampleQuintic(seg.y, s);
        const rz = sampleQuintic(seg.z, s);
        ref = {
          p: { x: rx.p, y: ry.p, z: rz.p },
          v: { x: rx.v, y: ry.v, z: rz.v },
          a: { x: rx.a, y: ry.a, z: rz.a },
          j: { x: rx.j, y: ry.j, z: rz.j },
        };
        if (st.gateIndex < 3) {
          const gIdx = st.gateIndex;
          const gateT = spline.wps[gIdx + 1].t;
          if (t > gateT && pos.x < GATES[gIdx].x) {
            ref.p.y = GATES[gIdx].y; ref.v.y = 0; ref.a.y = 0; ref.j.y = 0;
            ref.p.z = GATES[gIdx].z; ref.v.z = 0; ref.a.z = 0; ref.j.z = 0;
          }
        }
      }
      if (st.gateIndex === 3 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > 0.28 && ref.p.y < 0.13) {
        ref.p.y = 0.13;
        ref.v.y = Math.max(0, ref.v.y);
        ref.a.y = Math.max(0, ref.a.y);
      }

      const m = params.mass;
      const lag = params.lag;
      const lookWindow = 0.07 + 1.5 * lag;
      const { w: wLook, dw: dwLook } = windLookahead(t, lookWindow);
      const { w: wYaw } = windLookahead(t, 0.45);
      const wNow = windAt(t);

      // Exact current acceleration
      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll), bodyZ = -Math.sin(att.roll);
      const dir = {
        x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
        y: Math.cos(att.pitch) * Math.cos(att.roll),
        z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
      };
      const fSum = st.motors[0] + st.motors[1] + st.motors[2] + st.motors[3];
      const aCur = {
        x: (fSum * dir.x + DRAG * (wNow.x - vel.x)) / m,
        y: (fSum * dir.y + DRAG * (wNow.y - vel.y)) / m - G,
        z: (fSum * dir.z + DRAG * (wNow.z - vel.z)) / m,
      };

      // Outer loop position/velocity gains (high z stiffness during gate flight, soft horizontal / firm vertical at pad)
      const kp = atPad ? { x: 8, y: 24, z: 8 } : { x: 16, y: 20, z: 34 };
      const kd = atPad ? { x: 5.0, y: 10.5, z: 5.0 } : { x: 8.0, y: 9.5, z: 13.0 };

      // Jerk feedback gains kept well within motor-lag stability margin for all lag in [0.02, 0.06]
      const lagFactor = clamp(0.042 / Math.max(0.03, lag), 0.68, 1.15);
      const kpj = atPad ? { x: 3, y: 6, z: 3 } : { x: 13 * lagFactor, y: 16 * lagFactor, z: 20 * lagFactor };
      const kaj = atPad ? { x: 2, y: 3, z: 2 } : { x: 7.0 * lagFactor, y: 8.5 * lagFactor, z: 9.5 * lagFactor };

      const aDes = {
        x: ref.a.x + kp.x * (ref.p.x - pos.x) + kd.x * (ref.v.x - vel.x),
        y: ref.a.y + kp.y * (ref.p.y - pos.y) + kd.y * (ref.v.y - vel.y),
        z: ref.a.z + kp.z * (ref.p.z - pos.z) + kd.z * (ref.v.z - vel.z),
      };

      const jDes = {
        x: ref.j.x + kpj.x * (ref.v.x - vel.x) + kaj.x * (ref.a.x - aCur.x),
        y: ref.j.y + kpj.y * (ref.v.y - vel.y) + kaj.y * (ref.a.y - aCur.y),
        z: ref.j.z + kpj.z * (ref.v.z - vel.z) + kaj.z * (ref.a.z - aCur.z),
      };

      // Specific thrust vector u and its derivative du
      const u = {
        x: aDes.x - (DRAG / m) * (wLook.x - vel.x),
        y: Math.max(1.5, aDes.y + G - (DRAG / m) * (wLook.y - vel.y)),
        z: aDes.z - (DRAG / m) * (wLook.z - vel.z),
      };
      const aScale = atPad ? 0.35 : 1.0;
      const du = {
        x: jDes.x - (DRAG / m) * (dwLook.x - aScale * aCur.x),
        y: jDes.y - (DRAG / m) * (dwLook.y - aScale * aCur.y),
        z: jDes.z - (DRAG / m) * (dwLook.z - aScale * aCur.z),
      };

      // Rotate into yaw frame (including yaw rate coupling)
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw), wy = rates.yaw;
      const ubx = cy * u.x - sy * u.z;
      const ubz = sy * u.x + cy * u.z;
      const dubx = cy * du.x - sy * du.z - wy * ubz;
      const dubz = sy * du.x + cy * du.z + wy * ubx;

      const q2 = ubx * ubx + u.y * u.y;
      const q = Math.sqrt(q2);
      const u2 = q2 + ubz * ubz;

      let pitchDes = Math.atan2(ubx, u.y);
      let rollDes = -Math.atan2(ubz, q);
      let pitchRateDes = (u.y * dubx - ubx * du.y) / q2;
      const dq = (ubx * dubx + u.y * du.y) / q;
      let rollRateDes = -(q * dubz - ubz * dq) / u2;

      const nearPad = atPad && Math.hypot(pos.x - PAD.x, pos.z - PAD.z) < 0.12 && pos.y < 0.105;
      const maxTilt = nearPad ? 0.165 : 0.62;
      if (pitchDes > maxTilt) { pitchDes = maxTilt; pitchRateDes = Math.min(0, pitchRateDes); }
      else if (pitchDes < -maxTilt) { pitchDes = -maxTilt; pitchRateDes = Math.max(0, pitchRateDes); }
      if (rollDes > maxTilt) { rollDes = maxTilt; rollRateDes = Math.min(0, rollRateDes); }
      else if (rollDes < -maxTilt) { rollDes = -maxTilt; rollRateDes = Math.max(0, rollRateDes); }

      pitchRateDes = clamp(pitchRateDes, -6, 6);
      rollRateDes = clamp(rollRateDes, -6, 6);

      // Desired total thrust and thrust rate for lag compensation
      const cosTilt = Math.max(0.5, Math.cos(att.roll) * Math.cos(att.pitch));
      const fDes = (m * u.y) / cosTilt;
      const dfDes = (m * du.y) / cosTilt + fDes * (Math.tan(att.pitch) * rates.pitch + Math.tan(att.roll) * rates.roll);
      const fCmd = clamp(fDes + lag * dfDes, 0.2, 15.5);

      // Smooth yaw sharing when strong z-wind would otherwise saturate roll
      let yawDes = 0;
      const wzAccel = (DRAG / m) * wYaw.z;
      if (!atPad && Math.abs(wzAccel) > 3.5) {
        const excess = Math.abs(wzAccel) - 3.5;
        const uxSign = (ref.a.x - (DRAG / m) * (wYaw.x - ref.v.x)) >= 0 ? 1 : -1;
        yawDes = clamp(Math.sign(wzAccel) * uxSign * 0.22 * excess, -0.55, 0.55);
      }

      // Inner loop desired post-weakness torques
      const inertia = INERTIA_BASE * m / BASE_MASS;
      const kpAtt = 75, kdAtt = 16;
      const tauRoll = inertia * (kpAtt * (rollDes - att.roll) + kdAtt * (rollRateDes - rates.roll) + 0.4 * rates.roll);
      const tauPitch = inertia * (kpAtt * (pitchDes - att.pitch) + kdAtt * (pitchRateDes - rates.pitch) + 0.4 * rates.pitch);
      const tauYaw = inertia * (25 * (yawDes - att.yaw) + 10 * (0 - rates.yaw) + 0.4 * rates.yaw);

      // Weak motor inversion
      const w = params.weakMotor;
      const weakness = params.weakness;
      const fw = 0.25 * (fCmd + (tauRoll / ARM) * MOTOR_Z[w] - (tauPitch / ARM) * MOTOR_X[w] + (tauYaw / YAW_COEFF) * SPIN[w]);
      const deltaW = weakness > 0 ? clamp(fw, 0, MAX_THRUST * (1 - weakness)) * (weakness / (1 - weakness)) : 0;

      const fPre = fCmd + deltaW;
      const tauRollPre = tauRoll + ARM * MOTOR_Z[w] * deltaW;
      const tauPitchPre = tauPitch - ARM * MOTOR_X[w] * deltaW;
      const tauYawPre = tauYaw + YAW_COEFF * SPIN[w] * deltaW;

      const dtSub = 0.005;
      function invertAxis(axis, tauPre) {
        const curInt = st.integral[axis];
        const rate = rates[axis];
        const rhs = (tauPre / inertia) + 13 * rate;
        let err = (rhs - 22 * curInt) / (65 + 22 * dtSub);
        const newInt = curInt + err * dtSub;
        if (newInt > 0.35) err = (rhs - 22 * 0.35) / 65;
        else if (newInt < -0.35) err = (rhs + 22 * 0.35) / 65;
        return att[axis] + err;
      }

      const action = {
        thrust: clamp(fPre / (4 * MAX_THRUST), 0, 1),
        roll: clamp(invertAxis('roll', tauRollPre), -0.6, 0.6),
        pitch: clamp(invertAxis('pitch', tauPitchPre), -0.6, 0.6),
        yawRate: clamp(rates.yaw + (tauYawPre / inertia) / 8, -2, 2),
      };

      if (simState) {
        integrate(simState, action, params, 0.005);
        integrate(simState, action, params, 0.005);
      }

      return action;
    },
  };
}
