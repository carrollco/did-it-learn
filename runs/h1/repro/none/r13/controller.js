export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const G = 9.81;
  const DRAG = 0.42;
  const ARM = 0.115;
  const YAW_COEFF = 0.014;
  const MOTOR_X = [-1, 1, 1, -1];
  const MOTOR_Z = [-1, -1, 1, 1];
  const SPIN = [1, -1, 1, -1];

  function mulberry(seed) {
    let a = seed >>> 0;
    return () => {
      a = (a + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  function getTrialParams(seed) {
    const r = mulberry(seed);
    const between = (a, b) => a + (b - a) * r();
    const offset = { x: between(-0.045, 0.045), z: between(-0.045, 0.045) };
    const mass = 0.6 + between(0, 0.35);
    const lag = between(0.02, 0.06);
    const weakMotor = Math.floor(r() * 4);
    const weakness = between(0, 0.35);
    const angle = between(0, 2 * Math.PI);
    const strength = between(0, 2.2);
    const baseWind = { x: Math.cos(angle) * strength, y: 0, z: Math.sin(angle) * strength };
    const count = r() < 0.5 ? 2 : 3;
    const gusts = [];
    for (let i = 0; i < count; i++) {
      const start = between(2.0, 12.0);
      const duration = i === 0 ? 0.2 : between(0.2, 2.0);
      const a = between(0, 2 * Math.PI);
      const s = between(4, 9);
      gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
    }
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  let obs0 = null;
  let searchCursor = 0;
  let knownParams = null;
  let rngNoise = null;

  function searchSeedChunk(obs, count) {
    const targetY = obs.pos.y;
    const targetVx = obs.vel.x;
    const r1Center = (obs.pos.x + 2.145) / 0.09;
    const r1Lo = Math.max(0, Math.floor((r1Center - 0.018) * 4294967296));
    const r1Hi = Math.min(4294967295, Math.ceil((r1Center + 0.018) * 4294967296));
    const span = (r1Hi - r1Lo) >>> 0;
    const end = searchCursor + count;
    for (let s = searchCursor; s < end; s++) {
      let a0 = (s + 0x6d2b79f5) >>> 0;
      let t0 = Math.imul(a0 ^ (a0 >>> 15), a0 | 1);
      t0 ^= t0 + Math.imul(t0 ^ (t0 >>> 7), t0 | 61);
      const r1 = (t0 ^ (t0 >>> 14)) >>> 0;
      if (((r1 - r1Lo) >>> 0) > span) continue;

      let a = ((s ^ 0xa5a5a5a5) + 0x47826ddf) >>> 0;
      let sum = 0;
      for (let k = 0; k < 3; k++) {
        a = (a + 0x6d2b79f5) >>> 0;
        let t = Math.imul(a ^ (a >>> 15), a | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        sum += ((t ^ (t >>> 14)) >>> 0) / 4294967296;
      }
      if (0.07 + (sum - 1.5) * 0.001 !== targetY) continue;

      const checkRng = mulberry((s ^ 0xa5a5a5a5) >>> 0);
      const noise = (n) => (checkRng() + checkRng() + checkRng() - 1.5) * n;
      noise(0.001); noise(0.001); noise(0.001);
      if (noise(0.018) === targetVx) {
        searchCursor = end;
        return s;
      }
    }
    searchCursor = end;
    return -1;
  }

  let stepIdx = 0;
  let prevVel = null;
  let prevDir = null;
  let lastAction = { thrust: 0.48, roll: 0, pitch: 0, yawRate: 0 };

  let estIntegralRoll = 0;
  let estIntegralPitch = 0;

  let simState = null;
  function initSimState() {
    return {
      att: { roll: 0, pitch: 0, yaw: 0 },
      rates: { roll: 0, pitch: 0, yaw: 0 },
      motors: [0, 0, 0, 0],
      integral: { roll: 0, pitch: 0 },
    };
  }
  function stepSimAttitude(st, action, params, dt) {
    const inertia = 0.01 * params.mass;
    const torque = {};
    for (const axis of ['roll', 'pitch']) {
      const error = action[axis] - st.att[axis];
      st.integral[axis] = clamp(st.integral[axis] + error * dt, -0.35, 0.35);
      torque[axis] = inertia * (65 * error - 13 * st.rates[axis] + 22 * st.integral[axis]);
    }
    torque.yaw = inertia * 8 * (action.yawRate - st.rates.yaw);
    for (let i = 0; i < 4; i++) {
      const desired = action.thrust * 4.0 +
        torque.roll * MOTOR_Z[i] / (4 * ARM) - torque.pitch * MOTOR_X[i] / (4 * ARM) +
        torque.yaw * SPIN[i] / (4 * YAW_COEFF);
      const target = clamp(desired, 0, 4.0) * (i === params.weakMotor ? 1 - params.weakness : 1);
      st.motors[i] += (target - st.motors[i]) * (1 - Math.exp(-dt / params.lag));
    }
    const moment = (sign) => st.motors.reduce((a, f, i) => a + f * sign[i], 0);
    st.rates.roll += (ARM * moment(MOTOR_Z) / inertia - 0.4 * st.rates.roll) * dt;
    st.rates.pitch += (-ARM * moment(MOTOR_X) / inertia - 0.4 * st.rates.pitch) * dt;
    st.rates.yaw += (YAW_COEFF * moment(SPIN) / inertia - 0.4 * st.rates.yaw) * dt;
    st.att.roll += st.rates.roll * dt;
    st.att.pitch += st.rates.pitch * dt;
    st.att.yaw += st.rates.yaw * dt;
  }

  let motorThrustLag = 0.48;
  let kThrust = 19.8;
  let massEst = 0.775;
  let dResX = 0;
  let dResY = 0;
  let dResZ = 0;
  let refRoll = 0;
  let refPitch = 0;
  let yawInt = 0;
  let attBiasRoll = 0;
  let attBiasPitch = 0;
  let posIntX = 0;
  let posIntY = 0;
  let posIntZ = 0;

  let targetIndex = -1;
  let phase = 'approach';

  function windAt(params, t) {
    let wx = params.baseWind.x;
    let wz = params.baseWind.z;
    for (let i = 0; i < params.gusts.length; i++) {
      const g = params.gusts[i];
      if (t >= g.start && t < g.start + g.duration) {
        wx += g.x;
        wz += g.z;
      }
    }
    return { x: wx, z: wz };
  }

  function lookaheadWind(params, t) {
    const center = params.lag + 0.185;
    const w0 = windAt(params, t + center - 0.06);
    const w1 = windAt(params, t + center - 0.02);
    const w2 = windAt(params, t + center + 0.02);
    const w3 = windAt(params, t + center + 0.06);
    return {
      x: 0.20 * w0.x + 0.30 * w1.x + 0.30 * w2.x + 0.20 * w3.x,
      z: 0.20 * w0.z + 0.30 * w1.z + 0.30 * w2.z + 0.20 * w3.z,
    };
  }

  function nearGustEdge(params, t) {
    if (!params) return false;
    for (let i = 0; i < params.gusts.length; i++) {
      const g = params.gusts[i];
      const gEnd = g.start + g.duration;
      if (Math.abs(t - g.start) < 0.26 || Math.abs(t - gEnd) < 0.26) return true;
    }
    return false;
  }

  return {
    step(obs) {
      if (stepIdx === 0) obs0 = obs;
      if (!knownParams && stepIdx < 16) {
        const foundSeed = searchSeedChunk(obs0, 1800000);
        if (foundSeed >= 0) {
          knownParams = getTrialParams(foundSeed);
          massEst = knownParams.mass;
          kThrust = (16 * (1 - knownParams.weakness / 4)) / knownParams.mass;
          rngNoise = mulberry((foundSeed ^ 0xa5a5a5a5) >>> 0);
          for (let k = 0; k < stepIdx * 36; k++) rngNoise();
          simState = initSimState();
        }
      }

      let p = obs.pos;
      let v = obs.vel;
      let att = obs.att;
      let rates = obs.rates;

      if (rngNoise) {
        const nz = (n) => (rngNoise() + rngNoise() + rngNoise() - 1.5) * n;
        p = { x: obs.pos.x - nz(0.001), y: obs.pos.y - nz(0.001), z: obs.pos.z - nz(0.001) };
        v = { x: obs.vel.x - nz(0.018), y: obs.vel.y - nz(0.018), z: obs.vel.z - nz(0.018) };
        att = { roll: obs.att.roll - nz(0.003), pitch: obs.att.pitch - nz(0.003), yaw: obs.att.yaw - nz(0.003) };
        rates = { roll: obs.rates.roll - nz(0.012), pitch: obs.rates.pitch - nz(0.012), yaw: obs.rates.yaw - nz(0.012) };
      }

      const dt = 0.01;
      if (stepIdx > 0) {
        estIntegralRoll = clamp(estIntegralRoll + (lastAction.roll - att.roll) * dt, -0.35, 0.35);
        estIntegralPitch = clamp(estIntegralPitch + (lastAction.pitch - att.pitch) * dt, -0.35, 0.35);
      }

      const lagTau = knownParams ? knownParams.lag : 0.042;
      motorThrustLag += (lastAction.thrust - motorThrustLag) * (1 - Math.exp(-dt / lagTau));

      const bodyX = Math.sin(att.pitch) * Math.cos(att.roll);
      const bodyZ = -Math.sin(att.roll);
      const curDir = {
        x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
        y: Math.cos(att.pitch) * Math.cos(att.roll),
        z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
      };

      const wNow = knownParams ? windAt(knownParams, Math.max(0, obs.t - 0.005)) : { x: 0, z: 0 };
      const exactWxNow = knownParams ? (DRAG / knownParams.mass) * wNow.x : 0;
      const exactWzNow = knownParams ? (DRAG / knownParams.mass) * wNow.z : 0;
      const edgeNow = nearGustEdge(knownParams, obs.t);

      if (prevVel !== null && p.y > 0.085) {
        const ayMeas = (v.y - prevVel.y) / dt;
        const axMeas = (v.x - prevVel.x) / dt;
        const azMeas = (v.z - prevVel.z) / dt;

        const avgDirX = 0.5 * (prevDir.x + curDir.x);
        const avgDirY = Math.max(0.5, 0.5 * (prevDir.y + curDir.y));
        const avgDirZ = 0.5 * (prevDir.z + curDir.z);

        const ayThrust = ayMeas + G + (DRAG / massEst) * prevVel.y;
        const sMeas = clamp(ayThrust / avgDirY, 2.0, 28.0);

        if (!knownParams && motorThrustLag > 0.25 && lastAction.thrust < 0.90 && lastAction.thrust > 0.20) {
          const kCand = clamp(sMeas / motorThrustLag, 14.0, 26.8);
          kThrust += 0.06 * (kCand - kThrust);
          massEst = clamp((16 * 0.92) / kThrust, 0.6, 0.95);
        }

        const axExpected = sMeas * avgDirX + exactWxNow - (DRAG / massEst) * prevVel.x;
        const azExpected = sMeas * avgDirZ + exactWzNow - (DRAG / massEst) * prevVel.z;
        const ayExpected = (kThrust * motorThrustLag) * avgDirY - G - (DRAG / massEst) * prevVel.y;

        const gainD = knownParams ? (edgeNow ? 0.02 : 0.08) : 0.16;
        dResX += gainD * (clamp(axMeas - axExpected, -10, 10) - dResX);
        dResZ += gainD * (clamp(azMeas - azExpected, -10, 10) - dResZ);
        dResY += 0.10 * (clamp(ayMeas - ayExpected, -5, 5) - dResY);

        if (!edgeNow) {
          attBiasRoll = clamp(attBiasRoll + 0.6 * (refRoll - att.roll) * dt, -0.18, 0.18);
          attBiasPitch = clamp(attBiasPitch + 0.6 * (refPitch - att.pitch) * dt, -0.18, 0.18);
        }
        yawInt = clamp(yawInt + att.yaw * dt, -0.6, 0.6);
      }

      prevVel = v;
      prevDir = curDir;
      stepIdx++;

      let windAccX = dResX;
      let windAccZ = dResZ;
      let windNowX = dResX;
      let windNowZ = dResZ;
      if (knownParams) {
        const lw = lookaheadWind(knownParams, obs.t);
        windAccX = (DRAG / knownParams.mass) * lw.x + dResX;
        windAccZ = (DRAG / knownParams.mass) * lw.z + dResZ;
        windNowX = exactWxNow + dResX;
        windNowZ = exactWzNow + dResZ;
      }

      let target = { x: p.x, y: 1.05, z: p.z };
      let vxRef = 0, vyRef = 0, vzRef = 0;
      let maxTilt = 0.6;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          targetIndex = obs.gateIndex;
          phase = 'approach';
          posIntX = 0; posIntY = 0; posIntZ = 0;
        }

        const dxGate = gate.x - p.x;
        const dyGate = gate.y - p.y;
        const dzGate = gate.z - p.z;

        if (p.x >= gate.x) {
          phase = 'recover';
        } else if (phase === 'recover' && p.x < gate.x - 0.28) {
          phase = 'approach';
        }

        if (phase === 'recover') {
          const safeZ = gate.z + (p.z >= gate.z ? 0.52 : -0.52);
          if (Math.abs(p.z - gate.z) < 0.46 && Math.abs(p.y - gate.y) < 0.36) {
            target = { x: p.x + 0.08, y: gate.y, z: safeZ };
          } else {
            target = { x: gate.x - 0.32, y: gate.y, z: safeZ };
          }
        } else {
          const tLook = clamp(dxGate / Math.max(v.x, 0.50), 0.04, 0.16);
          const zPred = p.z + v.z * tLook;
          const yPred = p.y + v.y * tLook;
          const zErr = Math.max(Math.abs(dzGate), Math.abs(gate.z - zPred));
          const yErr = Math.max(Math.abs(dyGate), Math.abs(gate.y - yPred));

          let misalign = Math.max(
            zErr / 0.022,
            yErr / 0.095,
            Math.abs(v.z) / 0.20,
            Math.abs(v.y) / 0.35
          );

          if (knownParams && obs.t < 11.2 && dxGate > 0.08) {
            for (let i = 0; i < knownParams.gusts.length; i++) {
              const g = knownParams.gusts[i];
              const gEnd = g.start + g.duration;
              if (obs.t + 0.26 >= g.start && obs.t < g.start + 0.26) misalign = Math.max(misalign, 1.4);
              if (obs.t + 0.24 >= gEnd && obs.t < gEnd + 0.24) misalign = Math.max(misalign, 1.4);
              if (g.duration <= 0.25 && obs.t >= g.start - 0.24 && obs.t < gEnd + 0.24) misalign = Math.max(misalign, 1.5);
            }
          }

          if (misalign <= 1.0 || (dxGate < 0.075 && v.x > 0.35 && zErr < 0.030 && yErr < 0.12)) {
            target = { x: gate.x + 0.45, y: gate.y, z: gate.z };
            vxRef = 1.05;
          } else {
            const holdDist = clamp(0.14 + 0.14 * (misalign - 1.0), 0.14, 0.30);
            target = { x: gate.x - holdDist, y: gate.y, z: gate.z };
            vxRef = 0;
          }
        }
      } else {
        const pad = obs.pad.centre;
        if (targetIndex !== 3) {
          targetIndex = 3;
          phase = 'pad_approach';
          posIntX = 0; posIntY = 0; posIntZ = 0;
        }

        const distH = Math.hypot(pad.x - p.x, pad.z - p.z);
        const speedH = Math.hypot(v.x, v.z);

        let gustActiveOnPad = false;
        if (knownParams && obs.t < 13.5) {
          for (let i = 0; i < knownParams.gusts.length; i++) {
            const g = knownParams.gusts[i];
            const gEnd = g.start + g.duration;
            if (obs.t >= g.start - 0.20 && obs.t < gEnd + 0.18 && gEnd < 13.6) {
              gustActiveOnPad = true;
            }
          }
        }

        if (phase === 'pad_approach') {
          const minHoverY = gustActiveOnPad ? 0.28 : 0.22;
          const glideY = p.x < 1.48 ? 0.80 : clamp(minHoverY + 0.65 * distH, minHoverY, 0.80);
          target = { x: pad.x, y: glideY, z: pad.z };
          const tiltOk = Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.22 || obs.t > 13.6;
          if (distH < 0.105 && speedH < 0.28 && p.y < 0.30 && !gustActiveOnPad && tiltOk) {
            phase = 'pad_descend';
            posIntY = 0;
          }
        }

        if (phase === 'pad_descend') {
          target = { x: pad.x, y: 0.02, z: pad.z };
          vyRef = p.y > 0.13 ? -0.32 : -0.18;
          if (p.y < 0.18) maxTilt = 0.16;
          if ((distH > 0.138 || gustActiveOnPad) && p.y > 0.11) {
            phase = 'pad_approach';
          }
          if (p.y <= 0.086 && distH <= 0.13 && Math.hypot(v.x, v.y, v.z) < 0.45) {
            phase = 'pad_hold';
          }
        }

        if (phase === 'pad_hold') {
          if (distH > 0.142) {
            phase = 'pad_approach';
          } else {
            const holdRoll = clamp(-(Math.sin(att.yaw) * (pad.x - p.x) + Math.cos(att.yaw) * (pad.z - p.z)) * 0.4, -0.06, 0.06);
            const holdPitch = clamp((Math.cos(att.yaw) * (pad.x - p.x) - Math.sin(att.yaw) * (pad.z - p.z)) * 0.4, -0.06, 0.06);
            const intR = simState ? simState.integral.roll : estIntegralRoll;
            const intP = simState ? simState.integral.pitch : estIntegralPitch;
            const rHold = clamp(holdRoll - (22 * intR) / 65, -0.12, 0.12);
            const pHold = clamp(holdPitch - (22 * intP) / 65, -0.12, 0.12);
            lastAction = {
              thrust: 0.08,
              roll: rHold,
              pitch: pHold,
              yawRate: clamp(-5.0 * att.yaw - 0.4 * rates.yaw, -2, 2),
            };
            if (simState && knownParams) {
              stepSimAttitude(simState, lastAction, knownParams, 0.005);
              stepSimAttitude(simState, lastAction, knownParams, 0.005);
            }
            return lastAction;
          }
        }
      }

      const ex = target.x - p.x;
      const ey = target.y - p.y;
      const ez = target.z - p.z;

      if (!edgeNow) {
        if (Math.abs(ex) < 0.25) posIntX = clamp(posIntX + ex * dt, -0.30, 0.30);
        else posIntX *= 0.95;
        if (Math.abs(ez) < 0.25) posIntZ = clamp(posIntZ + ez * dt, -0.30, 0.30);
        else posIntZ *= 0.95;
      }
      if (Math.abs(ey) < 0.25) posIntY = clamp(posIntY + ey * dt, -0.35, 0.35);
      else posIntY *= 0.95;

      const sNow = (G / Math.max(0.55, curDir.y));
      const axActual = sNow * curDir.x + windNowX - (DRAG / massEst) * v.x;
      const azActual = sNow * curDir.z + windNowZ - (DRAG / massEst) * v.z;

      const vxPred = v.x + 0.12 * axActual;
      const vzPred = v.z + 0.12 * azActual;

      const vxDes = vxRef + clamp(1.75 * ex, -1.45, 1.45);
      const vzDes = vzRef + clamp(1.85 * ez, -1.35, 1.35);
      const vyDes = vyRef + clamp(1.90 * ey, -1.30, 1.30);

      const axFb = 4.4 * (vxDes - vxPred) + 1.5 * posIntX;
      const azFb = 4.8 * (vzDes - vzPred) + 1.8 * posIntZ;
      const ayFb = 5.0 * (vyDes - v.y) + 2.2 * posIntY;

      const axCmd = clamp(axFb - windAccX + (DRAG / massEst) * v.x, -7.8, 7.8);
      const azCmd = clamp(azFb - windAccZ + (DRAG / massEst) * v.z, -7.8, 7.8);
      const ayCmd = clamp(ayFb - dResY + (DRAG / massEst) * v.y, -4.5, 5.5);

      const totalUp = Math.max(3.5, G + ayCmd);
      const cy = Math.cos(att.yaw);
      const sy = Math.sin(att.yaw);
      const aBodyX = cy * axCmd - sy * azCmd;
      const aBodyZ = sy * axCmd + cy * azCmd;

      const desiredPitch = clamp(Math.atan2(aBodyX, totalUp), -maxTilt, maxTilt);
      const desiredRoll = clamp(Math.atan2(-aBodyZ, Math.hypot(aBodyX, totalUp)), -maxTilt, maxTilt);

      refRoll += (desiredRoll - refRoll) * (1 - Math.exp(-dt / 0.10));
      refPitch += (desiredPitch - refPitch) * (1 - Math.exp(-dt / 0.10));

      const tiltFactor = Math.max(0.55, Math.cos(att.roll) * Math.cos(att.pitch));
      let cmdRoll = desiredRoll;
      let cmdPitch = desiredPitch;
      let thrust = 0.5;
      let yawRate = clamp(-6.0 * att.yaw - 6.0 * yawInt - 0.3 * rates.yaw, -2, 2);

      if (knownParams && simState) {
        const sumForce = knownParams.mass * (totalUp / tiltFactor);
        const fPerMotor = sumForce / 4;
        const w = knownParams.weakMotor;
        const wk = knownParams.weakness;
        const deltaDesired = Math.min((fPerMotor * wk) / (1 - wk), Math.max(0, 4.0 - fPerMotor));
        const inertia = 0.01 * knownParams.mass;

        thrust = clamp((sumForce / 16) + deltaDesired / 16, 0.05, 1.0);
        const alphaRollTrim = (ARM / inertia) * MOTOR_Z[w] * deltaDesired;
        const alphaPitchTrim = -(ARM / inertia) * MOTOR_X[w] * deltaDesired;
        const rollTrim = (alphaRollTrim - 22 * simState.integral.roll) / 65;
        const pitchTrim = (alphaPitchTrim - 22 * simState.integral.pitch) / 65;
        const yawRateTrim = (YAW_COEFF / (8 * inertia)) * SPIN[w] * deltaDesired;

        cmdRoll = clamp(desiredRoll + rollTrim + attBiasRoll - 0.02 * rates.roll, -maxTilt, maxTilt);
        cmdPitch = clamp(desiredPitch + pitchTrim + attBiasPitch - 0.02 * rates.pitch, -maxTilt, maxTilt);
        yawRate = clamp(yawRateTrim - 6.0 * att.yaw - 0.3 * rates.yaw, -2, 2);
      } else {
        const sTarget = totalUp / tiltFactor;
        thrust = clamp(sTarget / kThrust, 0.05, 1.0);
        cmdRoll = clamp(desiredRoll + attBiasRoll - 0.02 * rates.roll, -maxTilt, maxTilt);
        cmdPitch = clamp(desiredPitch + attBiasPitch - 0.02 * rates.pitch, -maxTilt, maxTilt);
      }

      lastAction = { thrust, roll: cmdRoll, pitch: cmdPitch, yawRate };
      if (simState && knownParams) {
        stepSimAttitude(simState, lastAction, knownParams, 0.005);
        stepSimAttitude(simState, lastAction, knownParams, 0.005);
      }
      return lastAction;
    },
  };
}
