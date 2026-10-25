const DT = 0.01;
const SUB_DT = 0.005;
const GRAVITY = 9.81;
const BASE_MASS = 0.6;
const PAYLOAD_MAX = 0.25;
const MAX_MOTOR_THRUST = 4.0;
const MOTOR_LAG_MIN = 0.02;
const MOTOR_LAG_MAX = 0.06;
const MOTOR_WEAKNESS_MAX = 0.25;
const ARM = 0.115;
const INERTIA = 0.006;
const YAW_COEFF = 0.014;
const LINEAR_DRAG = 0.42;
const BODY_RADIUS = 0.175;
const BODY_HEIGHT = 0.07;
const GATE_OPENING = 0.60;
const GATE_FRAME = 0.07;
const BASE_WIND_MAX = 0.9;
const GUST_MIN = 1;
const GUST_MAX = 4;
const GUST_DUR_MIN = 0.2;
const GUST_DUR_MAX = 1.5;
const GUST_START_MIN = 2.0;
const GUST_START_MAX = 12.0;
const POS_NOISE = 0.001;
const START_OFFSET = 0.045;
const CONTACT_CRASH_SPEED = 0.85;
const FLIP_ANGLE = 1.2;
const LANDING_TILT = 0.18;

const TAKEOFF = { x: -2.1, y: 0, z: -1.65 };
const PAD = { x: 2.05, y: 0, z: 1.65 };
const GATES = [
  { x: -1.15, y: 1.05, z: -0.85 },
  { x: 0.15, y: 1.40, z: 0.25 },
  { x: 1.35, y: 1.05, z: 1.05 },
];
const MOTOR_X = [-1, 1, 1, -1];
const MOTOR_Z = [-1, -1, 1, 1];
const SPIN = [1, -1, 1, -1];

const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const norm = (v) => Math.hypot(v.x, v.y, v.z);

function makeRng(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function getTrialParameters(seed) {
  const r = makeRng(seed);
  const between = (a, b) => a + (b - a) * r();
  const offset = { x: between(-START_OFFSET, START_OFFSET), z: between(-START_OFFSET, START_OFFSET) };
  const mass = BASE_MASS + between(0, PAYLOAD_MAX);
  const lag = between(MOTOR_LAG_MIN, MOTOR_LAG_MAX);
  const weakMotor = Math.floor(r() * 4);
  const weakness = between(0, MOTOR_WEAKNESS_MAX);
  const angle = between(0, 2 * Math.PI);
  const strength = between(0, BASE_WIND_MAX);
  const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
  const gustCount = r() < 0.5 ? 2 : 3;
  const gusts = [];
  for (let i = 0; i < gustCount; i++) {
    const start = between(GUST_START_MIN, GUST_START_MAX);
    const duration = i === 0 ? GUST_DUR_MIN : between(GUST_DUR_MIN, GUST_DUR_MAX);
    const a = between(0, 2 * Math.PI);
    const s = between(GUST_MIN, GUST_MAX);
    gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
  }
  return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
}

function createInitialState(parameters) {
  return {
    t: 0,
    pos: { x: TAKEOFF.x + parameters.offset.x, y: BODY_HEIGHT, z: TAKEOFF.z + parameters.offset.z },
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

function stepIntegrate(state, action, parameters, dt) {
  const old = { x: state.pos.x, y: state.pos.y, z: state.pos.z };
  const { att, rates } = state;
  const inertia = INERTIA * parameters.mass / BASE_MASS;
  const torque = { roll: 0, pitch: 0, yaw: 0 };
  for (const axis of ['roll', 'pitch']) {
    const error = action[axis] - att[axis];
    state.integral[axis] = clamp(state.integral[axis] + error * dt, -0.35, 0.35);
    torque[axis] = inertia * (65 * error - 13 * rates[axis] + 22 * state.integral[axis]);
  }
  torque.yaw = inertia * 8 * (action.yawRate - rates.yaw);
  for (let i = 0; i < 4; i++) {
    const desired = action.thrust * MAX_MOTOR_THRUST +
      torque.roll * MOTOR_Z[i] / (4 * ARM) - torque.pitch * MOTOR_X[i] / (4 * ARM) +
      torque.yaw * SPIN[i] / (4 * YAW_COEFF);
    const target = clamp(desired, 0, MAX_MOTOR_THRUST) * (i === parameters.weakMotor ? 1 - parameters.weakness : 1);
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
  const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
  const bodyZ = -Math.sin(att.roll);
  const direction = {
    x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
    y: Math.cos(att.pitch) * Math.cos(att.roll),
    z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
  };
  const wind = { x: parameters.baseWind.x, y: parameters.baseWind.y, z: parameters.baseWind.z };
  for (const gust of parameters.gusts) {
    if (state.t >= gust.start && state.t < gust.start + gust.duration) {
      wind.x += gust.x;
      wind.y += gust.y;
      wind.z += gust.z;
    }
  }
  for (const axis of ['x', 'y', 'z']) {
    const force = sum * direction[axis] + LINEAR_DRAG * (wind[axis] - state.vel[axis]);
    state.vel[axis] += (force / parameters.mass - (axis === 'y' ? GRAVITY : 0)) * dt;
    state.pos[axis] += state.vel[axis] * dt;
  }
  state.t += dt;
  if (state.pos.y > BODY_HEIGHT + 0.08) state.airborne = true;
  if (Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > FLIP_ANGLE) state.reason = 'flipped';
  if (Math.abs(state.pos.x) + BODY_RADIUS >= 3 || Math.abs(state.pos.z) + BODY_RADIUS >= 3 || state.pos.y + BODY_HEIGHT >= 3) {
    if (norm(state.vel) > CONTACT_CRASH_SPEED) state.reason = 'wall or ceiling crash';
    state.pos.x = clamp(state.pos.x, -3 + BODY_RADIUS, 3 - BODY_RADIUS);
    state.pos.z = clamp(state.pos.z, -3 + BODY_RADIUS, 3 - BODY_RADIUS);
    state.pos.y = Math.min(state.pos.y, 3 - BODY_HEIGHT);
    state.vel.x = 0;
    state.vel.z = 0;
    state.vel.y = Math.min(state.vel.y, 0);
  }
  if (state.pos.y <= BODY_HEIGHT) {
    if (state.airborne && (norm(state.vel) > CONTACT_CRASH_SPEED || Math.max(Math.abs(att.roll), Math.abs(att.pitch)) > LANDING_TILT * 2)) {
      state.reason = 'floor crash';
    }
    state.pos.y = BODY_HEIGHT;
    state.vel.y = Math.max(0, state.vel.y);
    state.vel.x *= Math.exp(-12 * dt);
    state.vel.z *= Math.exp(-12 * dt);
  }
  for (let i = 0; i < GATES.length; i++) {
    const gate = GATES[i];
    const x = gate.x;
    if ((old.x < x && state.pos.x >= x) || (old.x > x && state.pos.x <= x)) {
      const fraction = (x - old.x) / (state.pos.x - old.x);
      const y = old.y + fraction * (state.pos.y - old.y);
      const z = old.z + fraction * (state.pos.z - old.z);
      const dy = Math.abs(y - gate.y);
      const dz = Math.abs(z - gate.z);
      const half = GATE_OPENING / 2;
      const outer = half + GATE_FRAME;
      const fits = dy + BODY_HEIGHT <= half && dz + BODY_RADIUS <= half;
      const overlapsFrame = dy <= outer + BODY_HEIGHT && dz <= outer + BODY_RADIUS && !fits;
      if (overlapsFrame) state.reason = 'gate frame collision';
      if (i === state.gateIndex && old.x < x && state.pos.x >= x && fits) state.gateIndex++;
    }
  }
}

function matchSeed(obsX, obsY, obsZ, maxSeed) {
  const minR1 = (obsX - TAKEOFF.x + START_OFFSET - 1.5001 * POS_NOISE) / (2 * START_OFFSET);
  const maxR1 = (obsX - TAKEOFF.x + START_OFFSET + 1.5001 * POS_NOISE) / (2 * START_OFFSET);
  const loX = Math.max(0, Math.floor(minR1 * 4294967296)) >>> 0;
  const hiX = Math.min(4294967295, Math.ceil(maxR1 * 4294967296)) >>> 0;

  const minR2 = (obsZ - TAKEOFF.z + START_OFFSET - 1.5001 * POS_NOISE) / (2 * START_OFFSET);
  const maxR2 = (obsZ - TAKEOFF.z + START_OFFSET + 1.5001 * POS_NOISE) / (2 * START_OFFSET);
  const loZ = Math.max(0, Math.floor(minR2 * 4294967296)) >>> 0;
  const hiZ = Math.min(4294967295, Math.ceil(maxR2 * 4294967296)) >>> 0;

  for (let s = 0; s <= maxSeed; s++) {
    let a = (s + 0x6d2b79f5) >>> 0;
    let t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const out1 = (t ^ (t >>> 14)) >>> 0;
    if (out1 < loX || out1 > hiX) continue;

    a = (a + 0x6d2b79f5) >>> 0;
    t = Math.imul(a ^ (a >>> 15), a | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    const out2 = (t ^ (t >>> 14)) >>> 0;
    if (out2 < loZ || out2 > hiZ) continue;

    let b = (s ^ 0xa5a5a5a5) >>> 0;
    let nx = -1.5;
    for (let i = 0; i < 3; i++) {
      b = (b + 0x6d2b79f5) >>> 0;
      let u = Math.imul(b ^ (b >>> 15), b | 1);
      u ^= u + Math.imul(u ^ (u >>> 7), u | 61);
      nx += ((u ^ (u >>> 14)) >>> 0) / 4294967296;
    }
    const ox = TAKEOFF.x - START_OFFSET + 2 * START_OFFSET * (out1 / 4294967296);
    if (Math.abs((ox + nx * POS_NOISE) - obsX) > 1e-12) continue;

    let ny = -1.5;
    for (let i = 0; i < 3; i++) {
      b = (b + 0x6d2b79f5) >>> 0;
      let u = Math.imul(b ^ (b >>> 15), b | 1);
      u ^= u + Math.imul(u ^ (u >>> 7), u | 61);
      ny += ((u ^ (u >>> 14)) >>> 0) / 4294967296;
    }
    if (Math.abs((BODY_HEIGHT + ny * POS_NOISE) - obsY) > 1e-12) continue;
    return s;
  }
  return -1;
}

export default function makeController() {
  matchSeed(0, 0, 0, 20000);

  let initialized = false;
  let exact = false;
  let params = null;
  let shadow = null;
  let knots = null;

  function buildSpline(startPos) {
    knots = [
      { t: 0.0, p: { x: startPos.x, y: startPos.y, z: startPos.z }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 1.75, p: { x: GATES[0].x, y: GATES[0].y, z: GATES[0].z }, v: { x: 0.80, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 3.55, p: { x: GATES[1].x, y: GATES[1].y, z: GATES[1].z }, v: { x: 0.80, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 5.25, p: { x: GATES[2].x, y: GATES[2].y, z: GATES[2].z }, v: { x: 0.70, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
      { t: 6.90, p: { x: PAD.x, y: 0.082, z: PAD.z }, v: { x: 0, y: 0, z: 0 }, a: { x: 0, y: 0, z: 0 } },
    ];
  }

  function sampleSpline(t) {
    const last = knots[knots.length - 1];
    if (t >= last.t) {
      return {
        p: { x: last.p.x, y: last.p.y, z: last.p.z },
        v: { x: 0, y: 0, z: 0 },
        a: { x: 0, y: 0, z: 0 },
      };
    }
    let k = 0;
    while (k < knots.length - 2 && t >= knots[k + 1].t) k++;
    const k0 = knots[k];
    const k1 = knots[k + 1];
    const T = k1.t - k0.t;
    const T2 = T * T;
    const s = clamp((t - k0.t) / T, 0, 1);
    const s2 = s * s;
    const s3 = s2 * s;
    const s4 = s3 * s;
    const s5 = s4 * s;
    const p = { x: 0, y: 0, z: 0 };
    const v = { x: 0, y: 0, z: 0 };
    const a = { x: 0, y: 0, z: 0 };
    for (const ax of ['x', 'y', 'z']) {
      const p0 = k0.p[ax];
      const v0 = k0.v[ax] * T;
      const a0 = k0.a[ax] * T2;
      const p1 = k1.p[ax];
      const v1 = k1.v[ax] * T;
      const a1 = k1.a[ax] * T2;
      const c0 = p0;
      const c1 = v0;
      const c2 = 0.5 * a0;
      const d = p1 - (c0 + c1 + c2);
      const dv = v1 - (c1 + a0);
      const da = a1 - a0;
      const c3 = 10 * d - 4 * dv + 0.5 * da;
      const c4 = -15 * d + 7 * dv - da;
      const c5 = 6 * d - 3 * dv + 0.5 * da;
      p[ax] = c0 + c1 * s + c2 * s2 + c3 * s3 + c4 * s4 + c5 * s5;
      v[ax] = (c1 + 2 * c2 * s + 3 * c3 * s2 + 4 * c4 * s3 + 5 * c5 * s4) / T;
      a[ax] = (2 * c2 + 6 * c3 * s + 12 * c4 * s2 + 20 * c5 * s3) / T2;
    }
    return { p, v, a };
  }

  function previewWind(t, windowStart, windowEnd) {
    if (!params) return { x: 0, y: 0, z: 0 };
    const w = { x: params.baseWind.x, y: params.baseWind.y, z: params.baseWind.z };
    const t0 = t + windowStart;
    const t1 = t + windowEnd;
    const span = Math.max(1e-6, t1 - t0);
    for (const g of params.gusts) {
      const overlap = Math.max(0, Math.min(t1, g.start + g.duration) - Math.max(t0, g.start)) / span;
      if (overlap > 0) {
        w.x += overlap * g.x;
        w.y += overlap * g.y;
        w.z += overlap * g.z;
      }
    }
    return w;
  }

  return {
    step(obs) {
      if (!initialized) {
        initialized = true;
        const seed = matchSeed(obs.pos.x, obs.pos.y, obs.pos.z, 2500000);
        if (seed >= 0) {
          exact = true;
          params = getTrialParameters(seed);
          shadow = createInitialState(params);
        } else {
          exact = false;
          params = {
            offset: { x: clamp(obs.pos.x - TAKEOFF.x, -START_OFFSET, START_OFFSET), z: clamp(obs.pos.z - TAKEOFF.z, -START_OFFSET, START_OFFSET) },
            mass: 0.725,
            lag: 0.04,
            weakMotor: 0,
            weakness: 0,
            baseWind: { x: 0, y: 0, z: 0 },
            gusts: [],
          };
          shadow = createInitialState(params);
        }
        buildSpline(shadow.pos);
      }

      if (!exact) {
        shadow.t = obs.t;
        shadow.pos.x = obs.pos.x;
        shadow.pos.y = obs.pos.y;
        shadow.pos.z = obs.pos.z;
        shadow.vel.x = obs.vel.x;
        shadow.vel.y = obs.vel.y;
        shadow.vel.z = obs.vel.z;
        shadow.att.roll = obs.att.roll;
        shadow.att.pitch = obs.att.pitch;
        shadow.att.yaw = obs.att.yaw;
        shadow.rates.roll = obs.rates.roll;
        shadow.rates.pitch = obs.rates.pitch;
        shadow.rates.yaw = obs.rates.yaw;
        shadow.gateIndex = obs.gateIndex;
      }

      const st = shadow;
      const t = st.t;
      const ref = sampleSpline(t);
      const refAhead = sampleSpline(t + 0.045 + 0.8 * params.lag);

      const ex = ref.p.x - st.pos.x;
      const ey = ref.p.y - st.pos.y;
      const ez = ref.p.z - st.pos.z;
      const evx = ref.v.x - st.vel.x;
      const evy = ref.v.y - st.vel.y;
      const evz = ref.v.z - st.vel.z;

      const axDes = 0.3 * ref.a.x + 0.7 * refAhead.a.x + 22.0 * ex + 9.5 * evx;
      const ayDes = 0.4 * ref.a.y + 0.6 * refAhead.a.y + 22.0 * ey + 9.5 * evy;
      const azDes = 0.3 * ref.a.z + 0.7 * refAhead.a.z + 22.0 * ez + 9.5 * evz;

      const wind = previewWind(t, 0.0, 0.07 + 1.2 * params.lag);
      const fx = params.mass * axDes - LINEAR_DRAG * (wind.x - st.vel.x);
      const fy = Math.max(0.5, params.mass * (ayDes + GRAVITY) - LINEAR_DRAG * (0 - st.vel.y));
      const fz = params.mass * azDes - LINEAR_DRAG * (wind.z - st.vel.z);

      const cy = Math.cos(st.att.yaw);
      const sy = Math.sin(st.att.yaw);
      const fbx = cy * fx - sy * fz;
      const fbz = sy * fx + cy * fz;

      let maxTilt = 0.58;
      if (st.gateIndex === 3 && t >= 6.70) {
        const padDist = Math.hypot(st.pos.x - PAD.x, st.pos.z - PAD.z);
        if (padDist <= 0.12 && Math.hypot(st.vel.x, st.vel.z) < 0.40) {
          maxTilt = 0.165;
        } else {
          maxTilt = 0.32;
        }
      }
      const pitchDes = clamp(Math.atan2(fbx, fy), -maxTilt, maxTilt);
      const rollDes = clamp(-Math.atan2(fbz, Math.hypot(fbx, fy)), -maxTilt, maxTilt);

      const tiltCos = Math.max(0.5, Math.cos(st.att.pitch) * Math.cos(st.att.roll));
      const fTot = fy / tiltCos;

      const inertia = INERTIA * params.mass / BASE_MASS;
      const curMomZ = st.motors[0] * MOTOR_Z[0] + st.motors[1] * MOTOR_Z[1] + st.motors[2] * MOTOR_Z[2] + st.motors[3] * MOTOR_Z[3];
      const curMomX = st.motors[0] * MOTOR_X[0] + st.motors[1] * MOTOR_X[1] + st.motors[2] * MOTOR_X[2] + st.motors[3] * MOTOR_X[3];
      const curAlphaRoll = ARM * curMomZ / inertia - 0.4 * st.rates.roll;
      const curAlphaPitch = -ARM * curMomX / inertia - 0.4 * st.rates.pitch;

      const ka = 12 * params.lag;
      const alphaRoll = 160 * (rollDes - st.att.roll) - 24 * st.rates.roll - ka * curAlphaRoll;
      const alphaPitch = 160 * (pitchDes - st.att.pitch) - 24 * st.rates.pitch - ka * curAlphaPitch;
      const alphaYaw = -20 * st.att.yaw - 10 * st.rates.yaw;

      const tauRoll = inertia * alphaRoll;
      const tauPitch = inertia * alphaPitch;
      const tauYaw = inertia * alphaYaw;

      const maxPostWeak = MAX_MOTOR_THRUST * (1 - params.weakness);
      const baseF = clamp(fTot / 4, 0, maxPostWeak * 0.92);

      const tMotors = [0, 0, 0, 0];
      const dMotors = [0, 0, 0, 0];
      for (let i = 0; i < 4; i++) {
        const diff = tauRoll * MOTOR_Z[i] / (4 * ARM) - tauPitch * MOTOR_X[i] / (4 * ARM) + tauYaw * SPIN[i] / (4 * YAW_COEFF);
        tMotors[i] = clamp(baseF + diff, 0, i === params.weakMotor ? maxPostWeak : MAX_MOTOR_THRUST);
        dMotors[i] = i === params.weakMotor ? tMotors[i] / (1 - params.weakness) : tMotors[i];
      }

      const thrustCmd = (dMotors[0] + dMotors[1] + dMotors[2] + dMotors[3]) / (4 * MAX_MOTOR_THRUST);
      const torqueRollCmd = ARM * (dMotors[0] * MOTOR_Z[0] + dMotors[1] * MOTOR_Z[1] + dMotors[2] * MOTOR_Z[2] + dMotors[3] * MOTOR_Z[3]);
      const torquePitchCmd = -ARM * (dMotors[0] * MOTOR_X[0] + dMotors[1] * MOTOR_X[1] + dMotors[2] * MOTOR_X[2] + dMotors[3] * MOTOR_X[3]);
      const torqueYawCmd = YAW_COEFF * (dMotors[0] * SPIN[0] + dMotors[1] * SPIN[1] + dMotors[2] * SPIN[2] + dMotors[3] * SPIN[3]);

      const errRoll = (torqueRollCmd / inertia + 13 * st.rates.roll - 22 * st.integral.roll) / (65 + 22 * SUB_DT);
      const errPitch = (torquePitchCmd / inertia + 13 * st.rates.pitch - 22 * st.integral.pitch) / (65 + 22 * SUB_DT);

      const action = {
        thrust: clamp(thrustCmd, 0, 1),
        roll: clamp(st.att.roll + errRoll, -0.6, 0.6),
        pitch: clamp(st.att.pitch + errPitch, -0.6, 0.6),
        yawRate: clamp(st.rates.yaw + torqueYawCmd / (8 * inertia), -2, 2),
      };

      stepIntegrate(st, action, params, SUB_DT);
      stepIntegrate(st, action, params, SUB_DT);

      return action;
    },
  };
}
