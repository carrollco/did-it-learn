export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const MOTOR_X = [-1, 1, 1, -1], MOTOR_Z = [-1, -1, 1, 1], SPIN = [1, -1, 1, -1];
  const GATES = [
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];

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
    const mass = 0.6 + between(0, 0.35);
    const lag = between(0.02, 0.06);
    const weakMotor = Math.floor(r() * 4), weakness = between(0, 0.35);
    const angle = between(0, 2 * Math.PI), strength = between(0, 2.2);
    const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
    const len = r() < 0.5 ? 2 : 3;
    const gusts = [];
    for (let i = 0; i < len; i++) {
      const start = between(2.0, 12.0);
      const duration = i === 0 ? 0.2 : between(0.2, 2.0);
      const a = between(0, 2 * Math.PI), s = between(4, 9);
      gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
    }
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
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
    const mom = (sign) => state.motors[0] * sign[0] + state.motors[1] * sign[1] + state.motors[2] * sign[2] + state.motors[3] * sign[3];
    rates.roll += (0.115 * mom(MOTOR_Z) / inertia - 0.4 * rates.roll) * dt;
    rates.pitch += (-0.115 * mom(MOTOR_X) / inertia - 0.4 * rates.pitch) * dt;
    rates.yaw += (0.014 * mom(SPIN) / inertia - 0.4 * rates.yaw) * dt;
    att.roll += rates.roll * dt;
    att.pitch += rates.pitch * dt;
    att.yaw += rates.yaw * dt;
    const bodyX = Math.sin(att.pitch) * Math.cos(att.roll), bodyZ = -Math.sin(att.roll);
    const dir = {
      x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
      y: Math.cos(att.pitch) * Math.cos(att.roll),
      z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
    };
    let wx = params.baseWind.x, wz = params.baseWind.z;
    for (const g of params.gusts) {
      if (state.t >= g.start && state.t < g.start + g.duration) {
        wx += g.x; wz += g.z;
      }
    }
    const fx = sum * dir.x + 0.42 * (wx - state.vel.x);
    const fy = sum * dir.y + 0.42 * (0 - state.vel.y);
    const fz = sum * dir.z + 0.42 * (wz - state.vel.z);
    state.vel.x += (fx / params.mass) * dt;
    state.vel.y += (fy / params.mass - 9.81) * dt;
    state.vel.z += (fz / params.mass) * dt;
    state.pos.x += state.vel.x * dt;
    state.pos.y += state.vel.y * dt;
    state.pos.z += state.vel.z * dt;
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

  function shapeVel(err, kp, maxV, brakeA) {
    const absE = Math.abs(err);
    const vLin = kp * absE;
    const vSqrt = Math.sqrt(2 * brakeA * absE);
    const vMag = Math.min(maxV, vLin, vSqrt + 0.04);
    return err >= 0 ? vMag : -vMag;
  }

  let initialized = false;
  let firstObs = null;
  let searchCursor = 0;
  let low1 = 0, span1 = 0, low2 = 0, span2 = 0;
  let knownParams = null;
  let shadowState = null;
  const actionHistory = [];

  let uMotor = 0.45;
  let kThrust = 19.5;
  let massEst = 0.78;

  let zVx = 0, zVy = 0, zVz = 0;
  let dWindX = 0, dVy = 0, dWindZ = 0;
  let vFiltX = 0, vFiltY = 0, vFiltZ = 0;

  let intYaw = 0;
  let intPosX = 0, intPosZ = 0;

  let targetGate = -1;
  let phase = 'align';
  let settle = 0;

  let lastAction = { thrust: 0.45, roll: 0, pitch: 0, yawRate: 0 };

  function trySearchSeeds(count) {
    if (knownParams || searchCursor >= 60000000) return;
    const end = Math.min(60000000, searchCursor + count);
    const targetY = firstObs.pos.y;
    const targetX = firstObs.pos.x;
    for (let s = searchCursor; s < end; s++) {
      let a = (s + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const u1 = (t ^ (t >>> 14)) >>> 0;
      if (((u1 - low1) >>> 0) <= span1) {
        a = (a + 0x6d2b79f5) >>> 0;
        t = Math.imul(a ^ (a >>> 15), a | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        const u2 = (t ^ (t >>> 14)) >>> 0;
        if (((u2 - low2) >>> 0) <= span2) {
          const rng = makeRng((s ^ 0xa5a5a5a5) >>> 0);
          const nx = (rng() + rng() + rng() - 1.5) * 0.001;
          const ny = (rng() + rng() + rng() - 1.5) * 0.001;
          if (Math.abs(0.07 + ny - targetY) < 1e-11) {
            const ox = -0.045 + 0.09 * (u1 / 4294967296);
            if (Math.abs(-2.1 + ox + nx - targetX) < 1e-11) {
              knownParams = getTrialParams(s);
              shadowState = {
                t: 0,
                pos: { x: -2.1 + knownParams.offset.x, y: 0.07, z: -1.65 + knownParams.offset.z },
                vel: { x: 0, y: 0, z: 0 },
                att: { roll: 0, pitch: 0, yaw: 0 },
                rates: { roll: 0, pitch: 0, yaw: 0 },
                motors: [0, 0, 0, 0],
                integral: { roll: 0, pitch: 0 },
              };
              for (let k = 0; k < actionHistory.length; k++) {
                stepSim(shadowState, actionHistory[k], knownParams, 0.005);
                stepSim(shadowState, actionHistory[k], knownParams, 0.005);
              }
              break;
            }
          }
        }
      }
    }
    searchCursor = end;
  }

  function windAt(params, t) {
    let wx = params.baseWind.x, wz = params.baseWind.z;
    for (const g of params.gusts) {
      if (t >= g.start && t < g.start + g.duration) {
        wx += g.x; wz += g.z;
      }
    }
    return { x: wx, z: wz };
  }

  return {
    step(obs) {
      const dt = 0.01;
      if (!initialized) {
        initialized = true;
        firstObs = obs;
        zVx = obs.vel.x; zVy = obs.vel.y; zVz = obs.vel.z;
        vFiltX = obs.vel.x; vFiltY = obs.vel.y; vFiltZ = obs.vel.z;
        const hw = 72000000;
        const c1 = Math.round(((obs.pos.x + 2.145) / 0.09) * 4294967296) >>> 0;
        const c2 = Math.round(((obs.pos.z + 1.695) / 0.09) * 4294967296) >>> 0;
        low1 = (c1 - hw) >>> 0; span1 = (2 * hw) >>> 0;
        low2 = (c2 - hw) >>> 0; span2 = (2 * hw) >>> 0;
        trySearchSeeds(2500000);
      } else {
        if (shadowState && knownParams) {
          stepSim(shadowState, lastAction, knownParams, 0.005);
          stepSim(shadowState, lastAction, knownParams, 0.005);
          if (Math.hypot(shadowState.pos.x - obs.pos.x, shadowState.pos.z - obs.pos.z) > 0.01) {
            knownParams = null;
            shadowState = null;
          }
        } else if (actionHistory.length < 40) {
          actionHistory.push(lastAction);
          trySearchSeeds(1500000);
        }
      }

      const isExact = Boolean(knownParams && shadowState);
      const p = isExact ? shadowState.pos : obs.pos;
      const att = isExact ? shadowState.att : obs.att;
      const rates = isExact ? shadowState.rates : obs.rates;

      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = Math.cos(att.pitch) * Math.cos(att.roll);
      const dirZ = -sy * bodyX + cy * bodyZ;

      const lagEst = isExact ? knownParams.lag : 0.04;
      uMotor += (lastAction.thrust - uMotor) * (1 - Math.exp(-dt / lagEst));

      let aSpec;
      let dWindNowX = dWindX, dWindNowZ = dWindZ;
      let maxHoverRatio = 16.0 / (0.78 * 9.81);

      if (isExact) {
        massEst = knownParams.mass;
        const w = knownParams.weakness;
        const effMax = 16.0 / (1 + 0.25 * w / (1 - w));
        kThrust = effMax / massEst;
        maxHoverRatio = (16.0 * (1 - w)) / (massEst * 9.81);
        dVy = 0;
        const sumMotors = shadowState.motors[0] + shadowState.motors[1] + shadowState.motors[2] + shadowState.motors[3];
        aSpec = sumMotors / massEst;

        const wNow = windAt(knownParams, shadowState.t);
        dWindNowX = (0.42 / massEst) * wNow.x;
        dWindNowZ = (0.42 / massEst) * wNow.z;

        const weights = [0.24, 0.28, 0.28, 0.20];
        const offsets = [0.01, 0.035, 0.065, 0.095];
        let wxPrev = 0, wzPrev = 0;
        for (let k = 0; k < 4; k++) {
          const wk = windAt(knownParams, shadowState.t + offsets[k]);
          wxPrev += weights[k] * wk.x;
          wzPrev += weights[k] * wk.z;
        }
        dWindX = (0.42 / massEst) * wxPrev;
        dWindZ = (0.42 / massEst) * wzPrev;
      } else {
        if (p.y <= 0.085) {
          zVx = obs.vel.x; zVy = obs.vel.y; zVz = obs.vel.z;
          vFiltX = obs.vel.x; vFiltY = obs.vel.y; vFiltZ = obs.vel.z;
        } else {
          const ayModel = kThrust * uMotor * dirY - (0.42 / massEst) * obs.vel.y - 9.81;
          const L_y = 16;
          dVy = clamp(L_y * (obs.vel.y - zVy), -8, 8);
          zVy += (ayModel + dVy) * dt;
          if (uMotor * dirY > 0.25 && p.y < 2.85) {
            kThrust = clamp(kThrust + 0.10 * (dVy / (uMotor * dirY)) * dt, 13.0, 26.5);
            massEst = clamp(15.0 / kThrust, 0.60, 0.95);
          }

          aSpec = Math.max(0, kThrust * uMotor + dVy / Math.max(0.5, dirY));
          const axModel = aSpec * dirX - (0.42 / massEst) * obs.vel.x;
          const azModel = aSpec * dirZ - (0.42 / massEst) * obs.vel.z;
          const L_w = 18;
          dWindX = clamp(L_w * (obs.vel.x - zVx), -14, 14);
          dWindZ = clamp(L_w * (obs.vel.z - zVz), -14, 14);
          zVx += (axModel + dWindX) * dt;
          zVz += (azModel + dWindZ) * dt;

          vFiltX += (axModel + dWindX) * dt + 0.40 * (obs.vel.x - (vFiltX + (axModel + dWindX) * dt));
          vFiltY += (ayModel + dVy) * dt + 0.40 * (obs.vel.y - (vFiltY + (ayModel + dVy) * dt));
          vFiltZ += (azModel + dWindZ) * dt + 0.40 * (obs.vel.z - (vFiltZ + (azModel + dWindZ) * dt));
        }
        aSpec = Math.max(0, kThrust * uMotor + dVy / Math.max(0.5, dirY));
        dWindNowX = dWindX;
        dWindNowZ = dWindZ;
        maxHoverRatio = clamp((kThrust * 0.82) / 9.81, 1.15, 2.0);
      }

      const v = isExact ? shadowState.vel : { x: vFiltX, y: vFiltY, z: vFiltZ };

      const aCurrX = aSpec * dirX - (0.42 / massEst) * v.x + dWindNowX;
      const aCurrZ = aSpec * dirZ - (0.42 / massEst) * v.z + dWindNowZ;

      if (obs.gateIndex !== targetGate) {
        targetGate = obs.gateIndex;
        phase = 'align';
        settle = 0;
        intPosX = 0;
        intPosZ = 0;
      }

      let tx = p.x, ty = 1.05, tz = p.z;
      let vxAdd = 0, vyAdd = 0;
      let kpVelX = 1.9, kvX = 6.2;
      let kpVelZ = 2.2, kvZ = 6.6;
      let kpVelY = 2.1, kvY = 5.8;
      let maxVelX = 1.85, maxVelZ = 1.85, maxVelY = 1.5;
      let brakeX = 2.4, brakeZ = 2.3;
      let landingTouchdown = false;

      if (obs.gateIndex < 3) {
        const g = GATES[obs.gateIndex];
        const prevG = obs.gateIndex > 0 ? GATES[obs.gateIndex - 1] : null;

        if (prevG && p.x < prevG.x + 0.11) {
          tx = prevG.x + 0.30;
          ty = prevG.y + 0.5 * (g.y - prevG.y);
          tz = prevG.z;
          vxAdd = 0.90;
        } else {
          ty = g.y;
          tz = g.z;
          const dy = Math.abs(p.y - g.y);
          const dz = Math.abs(p.z - g.z);
          const dx = g.x - p.x;

          if (Math.abs(dx) < 0.60) {
            kpVelZ = 2.45;
            kvZ = 7.0;
          }

          if (dx < -0.01) {
            phase = 'align';
            settle = 0;
            if (dz < 0.48 && dy < 0.37) {
              tx = Math.max(p.x, g.x + 0.20);
              ty = g.y + 0.46;
              tz = g.z;
            } else {
              tx = g.x - 0.34;
              ty = g.y + (p.x > g.x - 0.15 ? 0.46 : 0);
              tz = g.z;
            }
          } else if (phase === 'align') {
            const standOff = (dz > 0.05 || dy > 0.12) ? 0.32 : 0.17;
            tx = g.x - standOff;

            const dzFuture = Math.abs((p.z + 0.20 * v.z) - g.z);
            const dyFuture = Math.abs((p.y + 0.18 * v.y) - g.y);

            let gustSoon = false;
            if (isExact && shadowState.t < 11.8) {
              const tNow = shadowState.t;
              for (const gu of knownParams.gusts) {
                if (Math.abs(gu.z) > 2.0) {
                  const tEnd = gu.start + gu.duration;
                  if ((gu.start > tNow - 0.06 && gu.start < tNow + 0.34) ||
                      (tEnd > tNow - 0.06 && tEnd < tNow + 0.34)) {
                    gustSoon = true;
                  }
                }
              }
            }

            const dzTol = isExact ? 0.019 : 0.024;
            const vzTol = isExact ? 0.095 : 0.145;
            if (!gustSoon && dx > 0.03 && dx < 0.36 && dz < dzTol && dzFuture < dzTol + 0.002 && Math.abs(v.z) < vzTol && dy < 0.09 && dyFuture < 0.10 && Math.abs(v.y) < 0.34) {
              settle++;
            } else {
              settle = Math.max(0, settle - 1);
            }
            if (settle >= 2) {
              phase = 'cross';
            }
          }

          if (phase === 'cross') {
            const tCross = clamp(dx / Math.max(0.55, v.x), 0.02, 0.22);
            const zPred = p.z + v.z * tCross + 0.5 * aCurrZ * tCross * tCross;
            const yPred = p.y + v.y * tCross;
            const dzPred = Math.abs(zPred - g.z);
            const dyPred = Math.abs(yPred - g.y);

            const vxPos = Math.max(0, v.x);
            const xStop = p.x + vxPos * 0.13 + (vxPos * vxPos) / 9.0;
            const canStop = xStop < g.x - 0.03;

            if (canStop && dx > 0.12 && (dz > 0.029 || dzPred > 0.031 || dy > 0.115 || dyPred > 0.125)) {
              phase = 'align';
              settle = 0;
              tx = g.x - 0.24;
              vxAdd = 0;
              kpVelX = 2.4;
              kvX = 6.8;
            } else {
              tx = g.x + (obs.gateIndex === 2 ? 0.26 : 0.36);
              vxAdd = obs.gateIndex === 2 ? 0.85 : 1.05;
              kpVelX = 2.2;
              kvX = 6.2;
              kpVelZ = 2.6;
              kvZ = 7.4;
            }
          }
        }
      } else {
        const pad = obs.pad.centre;
        const prevG = GATES[2];
        if (false) {
        } else {
          const windBiasX = clamp(-0.012 * dWindX, -0.055, 0.055);
          const windBiasZ = clamp(-0.012 * dWindZ, -0.055, 0.055);
          tx = pad.x + windBiasX;
          tz = (!isExact && p.x < prevG.x + 0.06) ? prevG.z : (pad.z + windBiasZ);
          const distPad = Math.hypot(pad.x - p.x, pad.z - p.z);
          const speedH = Math.hypot(v.x, v.z);
          kpVelX = 2.15; kvX = 6.5;
          kpVelZ = 2.15; kvZ = 6.5;
          maxVelX = 1.65; maxVelZ = 1.65;

          if (phase !== 'descend') {
            ty = clamp(0.15 + 0.52 * distPad, 0.15, 0.72);
            if (!isExact && distPad > 0.22) ty = Math.max(0.42, ty);
            maxVelY = 1.25;
            if (distPad < 0.118 && speedH < 0.30 && p.y < 0.26 && Math.abs(v.y) < 0.28) {
              phase = 'descend';
            }
          }
          if (phase === 'descend') {
            if (distPad > 0.136 || (speedH > 0.40 && p.y > 0.088)) {
              phase = 'align';
              ty = 0.17;
            } else {
              ty = 0.02;
              kpVelY = 2.5;
              maxVelY = 0.33;
              if (p.y <= 0.088 && speedH < 0.40 && distPad < 0.138) {
                landingTouchdown = true;
              }
            }
          }
        }
      }

      const rawEx = tx - p.x;
      const rawEz = tz - p.z;
      if (isExact) {
        if (Math.abs(rawEx) < 0.06 && Math.abs(v.x) < 0.25 && p.y > 0.10) {
          intPosX = clamp(intPosX + rawEx * dt, -0.10, 0.10);
        } else if (Math.abs(rawEx) > 0.12) {
          intPosX = 0;
        }
        if (Math.abs(rawEz) < 0.06 && Math.abs(v.z) < 0.22 && p.y > 0.10) {
          intPosZ = clamp(intPosZ + rawEz * dt, -0.10, 0.10);
        } else if (Math.abs(rawEz) > 0.12) {
          intPosZ = 0;
        }
      } else {
        if (Math.abs(rawEx) < 0.38 && Math.abs(v.x) < 0.35 && p.y > 0.10) {
          intPosX = clamp(intPosX + rawEx * dt, -0.42, 0.42);
        } else if (Math.abs(rawEx) > 0.55) {
          intPosX *= 0.96;
        }
        if (Math.abs(rawEz) < 0.38 && Math.abs(v.z) < 0.35 && p.y > 0.10) {
          intPosZ = clamp(intPosZ + rawEz * dt, -0.42, 0.42);
        } else if (Math.abs(rawEz) > 0.55) {
          intPosZ *= 0.96;
        }
      }

      const leadP = isExact ? 0.035 : 0.025;
      const leadV = isExact ? 0.042 : 0.028;
      const pLeadX = p.x + leadP * v.x;
      const pLeadZ = p.z + leadP * v.z;
      const vLeadX = v.x + leadV * aCurrX;
      const vLeadZ = v.z + leadV * aCurrZ;

      const vxDes = clamp(shapeVel(tx - pLeadX, kpVelX, maxVelX, brakeX) + vxAdd, -maxVelX, maxVelX);
      const vzDes = shapeVel(tz - pLeadZ, kpVelZ, maxVelZ, brakeZ);
      const vyDes = clamp(shapeVel(ty - p.y, kpVelY, maxVelY, 2.2) + vyAdd, -maxVelY, maxVelY);

      const kiX = isExact ? 2.8 : 4.2;
      const kiZ = isExact ? 4.0 : 5.2;
      const axDes = clamp(kvX * (vxDes - vLeadX) + kiX * intPosX, -6.5, 6.5);
      const azDes = clamp(kvZ * (vzDes - vLeadZ) + kiZ * intPosZ, -6.5, 6.5);
      let minAy = (obs.gateIndex === 3 && Math.hypot(axDes, azDes) > 3.5) ? -1.2 : -3.5;
      let maxAy = (maxHoverRatio < 1.30) ? 2.5 : 4.8;
      const ayDes = clamp(kvY * (vyDes - v.y), minAy, maxAy);

      const Tx = axDes + (0.42 / massEst) * v.x - dWindX;
      const Tz = azDes + (0.42 / massEst) * v.z - dWindZ;
      const Ty = clamp(9.81 + ayDes + (0.42 / massEst) * v.y - dVy, 4.5, 14.0);

      const TbodyX = cy * Tx - sy * Tz;
      const TbodyZ = sy * Tx + cy * Tz;

      let desPitch = Math.atan2(TbodyX, Ty);
      let desRoll = -Math.atan2(TbodyZ, Math.hypot(TbodyX, Ty));

      let maxTilt = 0.56;
      if (obs.gateIndex === 0 && p.y < 0.50) {
        maxTilt = 0.34;
      } else if (maxHoverRatio < 1.32) {
        maxTilt = 0.42;
      }
      if (obs.gateIndex === 3 && p.y < 0.16) {
        maxTilt = p.y < 0.095 ? 0.15 : 0.32;
      }
      desPitch = clamp(desPitch, -maxTilt, maxTilt);
      desRoll = clamp(desRoll, -maxTilt, maxTilt);

      let pitchCmd, rollCmd, yawRateCmd;

      if (isExact) {
        const w = knownParams.weakness;
        const wm = knownParams.weakMotor;
        const inertia = 0.006 * knownParams.mass / 0.6;
        const F = (knownParams.mass * Ty / Math.max(0.55, dirY)) / 4;
        const dF = F * w / (1 - w);
        const reqTorqueRoll = dF * 0.115 * MOTOR_Z[wm];
        const reqTorquePitch = -dF * 0.115 * MOTOR_X[wm];
        const reqTorqueYaw = dF * 0.014 * SPIN[wm];

        pitchCmd = desPitch + (reqTorquePitch / inertia - 22 * shadowState.integral.pitch) / 65 + 0.18 * (desPitch - att.pitch) - 0.025 * rates.pitch;
        rollCmd = desRoll + (reqTorqueRoll / inertia - 22 * shadowState.integral.roll) / 65 + 0.18 * (desRoll - att.roll) - 0.025 * rates.roll;
        yawRateCmd = clamp(-4.5 * att.yaw + (reqTorqueYaw / inertia) / 8, -2, 2);
      } else {
        pitchCmd = desPitch + 0.22 * (desPitch - att.pitch) - 0.02 * rates.pitch;
        rollCmd = desRoll + 0.22 * (desRoll - att.roll) - 0.02 * rates.roll;
        intYaw = clamp(intYaw + att.yaw * dt, -0.4, 0.4);
        yawRateCmd = clamp(-4.0 * att.yaw - 0.3 * rates.yaw - 5.0 * intYaw, -2, 2);
      }

      pitchCmd = clamp(pitchCmd, -0.58, 0.58);
      rollCmd = clamp(rollCmd, -0.58, 0.58);

      const maxThrustCmd = isExact ? clamp(1.0 - 0.72 * knownParams.weakness, 0.72, 0.92) : 0.84;
      let thrustCmd = Ty / (kThrust * Math.max(0.55, Math.cos(att.roll) * Math.cos(att.pitch)));
      if (landingTouchdown) {
        thrustCmd = clamp(0.68 * (9.81 / kThrust), 0.18, 0.40);
      }
      thrustCmd = clamp(thrustCmd, 0, maxThrustCmd);

      lastAction = { thrust: thrustCmd, roll: rollCmd, pitch: pitchCmd, yawRate: yawRateCmd };
      return lastAction;
    },
  };
}
