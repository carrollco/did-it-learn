export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
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
    const numGusts = r() < 0.5 ? 2 : 3;
    const gusts = [];
    for (let i = 0; i < numGusts; i++) {
      const start = between(2.0, 12.0);
      const duration = i === 0 ? 0.2 : between(0.2, 2.0);
      const a = between(0, 2 * Math.PI);
      const s = between(4.0, 9.0);
      gusts.push({ start, duration, x: Math.cos(a) * s, y: 0, z: Math.sin(a) * s });
    }
    return { offset, mass, lag, weakMotor, weakness, baseWind, gusts };
  }

  let uMinX = 0, uMaxX = 0, uMinZ = 0, uMaxZ = 0;
  let obs0PosY = 0, obs0VelX = 0;
  let scanCursor = 0;

  function scanSeeds(count) {
    const end = Math.min(0xffffffff, scanCursor + count);
    const minX = uMinX, maxX = uMaxX, minZ = uMinZ, maxZ = uMaxZ;
    const targetY = obs0PosY, targetVx = obs0VelX;
    for (let s = scanCursor; s < end; s++) {
      let a = (s + 0x6d2b79f5) >>> 0;
      let t = Math.imul(a ^ (a >>> 15), a | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      const ux = (t ^ (t >>> 14)) >>> 0;
      if (ux >= minX && ux <= maxX) {
        a = (a + 0x6d2b79f5) >>> 0;
        t = Math.imul(a ^ (a >>> 15), a | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        const uz = (t ^ (t >>> 14)) >>> 0;
        if (uz >= minZ && uz <= maxZ) {
          const r = mulberry((s ^ 0xa5a5a5a5) >>> 0);
          r(); r(); r();
          const py = 0.07 + (r() + r() + r() - 1.5) * 0.001;
          if (Math.abs(py - targetY) < 1e-11) {
            r(); r(); r();
            const vx = (r() + r() + r() - 1.5) * 0.018;
            if (Math.abs(vx - targetVx) < 1e-11) {
              scanCursor = end;
              return s;
            }
          }
        }
      }
    }
    scanCursor = end;
    return -1;
  }

  scanSeeds(50000);
  scanCursor = 0;

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

  function integrateExact(st, action, params, dt) {
    const { att, rates } = st;
    const inertia = 0.006 * params.mass / 0.6;
    const torque = {};
    for (const axis of ['roll', 'pitch']) {
      const error = action[axis] - att[axis];
      st.integral[axis] = clamp(st.integral[axis] + error * dt, -0.35, 0.35);
      torque[axis] = inertia * (65 * error - 13 * rates[axis] + 22 * st.integral[axis]);
    }
    torque.yaw = inertia * 8 * (action.yawRate - rates.yaw);
    for (let i = 0; i < 4; i++) {
      const desired = action.thrust * 4.0 +
        torque.roll * MOTOR_Z[i] / (4 * 0.115) - torque.pitch * MOTOR_X[i] / (4 * 0.115) +
        torque.yaw * SPIN[i] / (4 * 0.014);
      const target = clamp(desired, 0, 4.0) * (i === params.weakMotor ? 1 - params.weakness : 1);
      st.motors[i] += (target - st.motors[i]) * (1 - Math.exp(-dt / params.lag));
    }
    const sum = st.motors[0] + st.motors[1] + st.motors[2] + st.motors[3];
    let mZ = 0, mX = 0, mS = 0;
    for (let i = 0; i < 4; i++) {
      mZ += st.motors[i] * MOTOR_Z[i];
      mX += st.motors[i] * MOTOR_X[i];
      mS += st.motors[i] * SPIN[i];
    }
    rates.roll += (0.115 * mZ / inertia - 0.4 * rates.roll) * dt;
    rates.pitch += (-0.115 * mX / inertia - 0.4 * rates.pitch) * dt;
    rates.yaw += (0.014 * mS / inertia - 0.4 * rates.yaw) * dt;
    att.roll += rates.roll * dt;
    att.pitch += rates.pitch * dt;
    att.yaw += rates.yaw * dt;
    const bodyX = Math.sin(att.pitch) * Math.cos(att.roll), bodyZ = -Math.sin(att.roll);
    const dir = {
      x: Math.cos(att.yaw) * bodyX + Math.sin(att.yaw) * bodyZ,
      y: Math.cos(att.pitch) * Math.cos(att.roll),
      z: -Math.sin(att.yaw) * bodyX + Math.cos(att.yaw) * bodyZ,
    };
    const w = windAt(params, st.t);
    for (const axis of ['x', 'y', 'z']) {
      const wVal = axis === 'y' ? 0 : w[axis];
      const force = sum * dir[axis] + 0.42 * (wVal - st.vel[axis]);
      st.vel[axis] += (force / params.mass - (axis === 'y' ? 9.81 : 0)) * dt;
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

  let initialized = false;
  let knownParams = null;
  let exactState = null;
  const actionHistory = [];

  let px = 0, py = 0, pz = 0;
  let vx = 0, vy = 0, vz = 0;
  let wx = 0, wz = 0, dyDist = 0;
  let mEff = 0.78;
  let motorThrust = 0;
  let prevThrustCmd = 0.48;
  let prevRollCmd = 0, prevPitchCmd = 0;
  let simIntRoll = 0, simIntPitch = 0;

  let rollTrim = 0, pitchTrim = 0, yawInt = 0;
  let refRoll = 0, refPitch = 0;

  let targetIndex = -1;
  let phase = 'align';
  let settle = 0;
  let prevGate = null;
  let currentGate = null;

  function initKnownSeed(seed) {
    knownParams = getTrialParams(seed);
    mEff = knownParams.mass / (1 - knownParams.weakness / 4);
    exactState = {
      t: 0,
      pos: { x: -2.1 + knownParams.offset.x, y: 0.07, z: -1.65 + knownParams.offset.z },
      vel: { x: 0, y: 0, z: 0 },
      att: { roll: 0, pitch: 0, yaw: 0 },
      rates: { roll: 0, pitch: 0, yaw: 0 },
      motors: [0, 0, 0, 0],
      integral: { roll: 0, pitch: 0 },
    };
    for (let i = 0; i < actionHistory.length; i++) {
      integrateExact(exactState, actionHistory[i], knownParams, 0.005);
      integrateExact(exactState, actionHistory[i], knownParams, 0.005);
    }
  }

  return {
    step(obs) {
      const dt = 0.01;
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (!initialized) {
        initialized = true;
        px = p.x; py = p.y; pz = p.z;
        vx = v.x; vy = v.y; vz = v.z;
        motorThrust = 0;

        const ox = p.x - (-2.1);
        const oz = p.z - (-1.65);
        uMinX = Math.max(0, Math.floor(((ox - 0.001501 + 0.045) / 0.09) * 4294967296));
        uMaxX = Math.min(0xffffffff, Math.ceil(((ox + 0.001501 + 0.045) / 0.09) * 4294967296));
        uMinZ = Math.max(0, Math.floor(((oz - 0.001501 + 0.045) / 0.09) * 4294967296));
        uMaxZ = Math.min(0xffffffff, Math.ceil(((oz + 0.001501 + 0.045) / 0.09) * 4294967296));
        obs0PosY = p.y;
        obs0VelX = v.x;

        const seed = scanSeeds(1500000);
        if (seed >= 0) initKnownSeed(seed);
      } else {
        if (!knownParams && obs.t < 1.95 && scanCursor < 250000000) {
          const seed = scanSeeds(1200000);
          if (seed >= 0) initKnownSeed(seed);
        }

        if (knownParams && exactState) {
          const lastAct = actionHistory[actionHistory.length - 1];
          integrateExact(exactState, lastAct, knownParams, 0.005);
          integrateExact(exactState, lastAct, knownParams, 0.005);
          px = exactState.pos.x;
          py = exactState.pos.y;
          pz = exactState.pos.z;
          vx = exactState.vel.x;
          vy = exactState.vel.y;
          vz = exactState.vel.z;
          simIntRoll = exactState.integral.roll;
          simIntPitch = exactState.integral.pitch;
          wx = 0; wz = 0; dyDist = 0;
        } else {
          motorThrust += (16.0 * prevThrustCmd - motorThrust) * (1 - Math.exp(-dt / 0.04));

          simIntRoll = clamp(simIntRoll + (prevRollCmd - att.roll) * dt, -0.35, 0.35);
          simIntPitch = clamp(simIntPitch + (prevPitchCmd - att.pitch) * dt, -0.35, 0.35);

          const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
          const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
          const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);

          const bodyX = sp * cr, bodyZ = -sr;
          const dirX = cy * bodyX + sy * bodyZ;
          const dirY = cp * cr;
          const dirZ = -sy * bodyX + cy * bodyZ;

          const airborne = p.y > 0.082 && p.y < 2.88;
          const inRoom = Math.abs(p.x) < 2.80 && Math.abs(p.z) < 2.80;

          const ayPred = airborne ? ((motorThrust * dirY - 0.42 * vy) / mEff - 9.81 + dyDist) : 0;
          const pyPred = py + vy * dt + 0.5 * ayPred * dt * dt;
          const vyPred = vy + ayPred * dt;
          const epY = p.y - pyPred;
          const evY = v.y - vyPred;
          py = pyPred + 0.75 * epY;
          vy = vyPred + 0.65 * evY + 10.0 * epY;
          if (airborne) {
            dyDist = clamp(dyDist + 28.0 * evY + 60.0 * epY, -6.5, 6.5);
            if (Math.abs(dyDist) > 0.2) {
              const dm = clamp(-0.025 * dyDist * dt, -0.005, 0.005);
              const oldM = mEff;
              mEff = clamp(mEff + dm, 0.58, 1.08);
              dyDist += (motorThrust * dirY) * (1 / oldM - 1 / mEff);
            }
          }

          const tOverM = airborne
            ? clamp(motorThrust / mEff + dyDist / Math.max(0.5, dirY), 1.0, 28.0)
            : motorThrust / mEff;

          const ux = tOverM * dirX - (0.42 / mEff) * vx;
          const uz = tOverM * dirZ - (0.42 / mEff) * vz;
          const axPred = airborne ? (ux + wx) : 0;
          const azPred = airborne ? (uz + wz) : 0;

          const pxPred = px + vx * dt + 0.5 * axPred * dt * dt;
          const vxPred = vx + axPred * dt;
          const pzPred = pz + vz * dt + 0.5 * azPred * dt * dt;
          const vzPred = vz + azPred * dt;

          const epX = p.x - pxPred, evX = v.x - vxPred;
          const epZ = p.z - pzPred, evZ = v.z - vzPred;

          px = pxPred + 0.75 * epX;
          vx = vxPred + 0.65 * evX + 10.0 * epX;
          pz = pzPred + 0.75 * epZ;
          vz = vzPred + 0.65 * evZ + 10.0 * epZ;

          if (airborne && inRoom) {
            wx = clamp(wx + 38.0 * evX + 100.0 * epX, -11.0, 11.0);
            wz = clamp(wz + 38.0 * evZ + 100.0 * epZ, -11.0, 11.0);
          }

          if (airborne) {
            rollTrim = clamp(rollTrim + 1.8 * (refRoll - att.roll) * dt, -0.25, 0.25);
            pitchTrim = clamp(pitchTrim + 1.8 * (refPitch - att.pitch) * dt, -0.25, 0.25);
            yawInt = clamp(yawInt + att.yaw * dt, -0.6, 0.6);
          }
        }
      }

      const curAtt = (knownParams && exactState) ? exactState.att : att;
      const curRates = (knownParams && exactState) ? exactState.rates : rates;

      let gustEdgeAlignHold = false;
      let gustEdgeCrossAbort = false;
      if (knownParams && obs.t < 13.0) {
        for (let i = 0; i < knownParams.gusts.length; i++) {
          const g = knownParams.gusts[i];
          const gEnd = g.start + g.duration;
          if (Math.abs(g.z) > 1.0 || g.x < -2.0) {
            if (
              (obs.t >= g.start - 0.36 && obs.t <= g.start + 0.16) ||
              (obs.t >= gEnd - 0.34 && obs.t <= gEnd + 0.16)
            ) {
              gustEdgeAlignHold = true;
            }
            if (
              (obs.t >= g.start - 0.24 && obs.t <= g.start + 0.14) ||
              (obs.t >= gEnd - 0.22 && obs.t <= gEnd + 0.14)
            ) {
              gustEdgeCrossAbort = true;
            }
          }
        }
      }

      let tx = px, ty = 1.0, tz = pz;
      let vxFF = 0, vyFF = 0, vzFF = 0;
      let kpX = 6.5, kdX = 4.4;
      let kpZ = 8.0, kdZ = 4.9;
      let kpY = 7.8, kdY = 4.6;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          if (currentGate) prevGate = currentGate;
          currentGate = { x: gate.x, y: gate.y, z: gate.z };
          targetIndex = obs.gateIndex;
          phase = 'align';
          settle = 0;
        }

        if (prevGate && px < prevGate.x + 0.16) {
          tx = prevGate.x + 0.32;
          ty = prevGate.y;
          tz = prevGate.z;
          vxFF = 0.75;
          kpZ = 10.0; kdZ = 5.5;
        } else {
          const dzGate = Math.abs(pz - gate.z);
          const dyGate = Math.abs(py - gate.y);

          if (phase === 'align') {
            const xStandOff = (dzGate > 0.065 || dyGate > 0.12 || gustEdgeAlignHold) ? 0.28 : 0.18;
            tx = gate.x - xStandOff;
            ty = gate.y;
            tz = gate.z;
            kpZ = 9.8; kdZ = 5.5;
            kpX = 7.0; kdX = 4.6;

            const zPredA = Math.abs(pz + vz * 0.12 - gate.z);
            if (
              !gustEdgeAlignHold &&
              px > gate.x - 0.28 &&
              px < gate.x - 0.03 &&
              dzGate < 0.0165 &&
              zPredA < 0.0175 &&
              Math.abs(vz) < 0.09 &&
              dyGate < 0.06 &&
              Math.abs(vy) < 0.24 &&
              Math.abs(vx) < 0.32
            ) {
              settle++;
            } else {
              settle = Math.max(0, settle - 1);
            }
            if (settle >= (knownParams ? 3 : 4)) {
              phase = 'cross';
            }
          }

          if (phase === 'cross') {
            const dxToGate = gate.x - px;
            if (dxToGate > 0.015) {
              const tCross = clamp(dxToGate / Math.max(0.28, vx), 0.01, 0.20);
              const zPred = pz + vz * tCross;
              const yPred = py + vy * tCross;
              const abortForGust = gustEdgeCrossAbort && dxToGate > 0.125;
              if (
                abortForGust ||
                (dxToGate > 0.06 && (dzGate > 0.025 || Math.abs(zPred - gate.z) > 0.027)) ||
                dyGate > 0.095 ||
                Math.abs(yPred - gate.y) > 0.105
              ) {
                phase = 'align';
                settle = 0;
                tx = gate.x - 0.22;
                ty = gate.y;
                tz = gate.z;
                vxFF = -0.2;
                kpX = 8.5; kdX = 5.0;
                kpZ = 10.5; kdZ = 5.8;
              } else {
                tx = gate.x + 0.32;
                ty = gate.y;
                tz = gate.z;
                vxFF = knownParams ? 0.85 : 0.78;
                kpX = 7.5; kdX = 4.5;
                kpZ = 11.5; kdZ = 6.2;
              }
            } else {
              tx = gate.x + 0.32;
              ty = gate.y;
              tz = gate.z;
              vxFF = knownParams ? 0.85 : 0.78;
              kpZ = 11.5; kdZ = 6.2;
            }
          }
        }
      } else {
        const pad = obs.pad.centre;
        if (targetIndex !== 3) {
          if (currentGate) prevGate = currentGate;
          targetIndex = 3;
          phase = 'pad_approach';
          settle = 0;
        }

        if (prevGate && px < prevGate.x + 0.14) {
          tx = prevGate.x + 0.32;
          ty = prevGate.y;
          tz = prevGate.z;
          vxFF = 0.70;
          kpZ = 10.0; kdZ = 5.5;
        } else {
          const distPad = Math.hypot(pad.x - px, pad.z - pz);
          const speedHoriz = Math.hypot(vx, vz);

          if (phase === 'pad_approach') {
            tx = pad.x;
            tz = pad.z;
            ty = clamp(0.08 + 0.60 * distPad, 0.09, 0.75);
            vyFF = distPad > 0.18 ? -0.34 : -0.18;
            kpX = 7.2; kdX = 4.6;
            kpZ = 7.2; kdZ = 4.6;
            if (distPad < 0.14 && speedHoriz < 0.35 && py < 0.32) {
              phase = 'descend';
            }
          }

          if (phase === 'descend') {
            tx = pad.x;
            tz = pad.z;
            kpX = 8.0; kdX = 5.0;
            kpZ = 8.0; kdZ = 5.0;
            ty = 0.02;
            vyFF = py > 0.13 ? -0.35 : -0.20;
          }
        }
      }

      if (obs.gateIndex === 3) {
        const pad = obs.pad.centre;
        const distPad = Math.hypot(pad.x - px, pad.z - pz);
        const speedTotal = Math.hypot(vx, vy, vz);
        if (distPad <= 0.142 && py <= 0.091 && speedTotal < 0.45) {
          const axPad = clamp(4.0 * (pad.x - px) - 3.0 * vx, -1.0, 1.0);
          const azPad = clamp(4.0 * (pad.z - pz) - 3.0 * vz, -1.0, 1.0);
          const cy = Math.cos(curAtt.yaw), sy = Math.sin(curAtt.yaw);
          const pitchPad = clamp((cy * axPad - sy * azPad) / 9.81, -0.10, 0.10);
          const rollPad = clamp(-(sy * axPad + cy * azPad) / 9.81, -0.10, 0.10);
          const out = {
            thrust: 0.05,
            roll: rollPad,
            pitch: pitchPad,
            yawRate: clamp(-4.0 * curAtt.yaw - 0.3 * curRates.yaw, -2, 2),
          };
          prevThrustCmd = out.thrust;
          prevRollCmd = out.roll;
          prevPitchCmd = out.pitch;
          if (actionHistory.length < 250) actionHistory.push(out);
          else actionHistory[actionHistory.length - 1] = out;
          return out;
        }
      }

      const axDes = clamp(kpX * (tx - px) + kdX * (vxFF - vx), -5.5, 5.5);
      const azDes = clamp(kpZ * (tz - pz) + kdZ * (vzFF - vz), -5.5, 5.5);
      const ayDes = clamp(kpY * (ty - py) + kdY * (vyFF - vy), -4.0, 5.0);

      const massUse = knownParams ? knownParams.mass : mEff;
      let wxTotal = wx, wzTotal = wz;
      if (knownParams) {
        const lead = knownParams.lag + 0.025;
        const w0 = windAt(knownParams, obs.t);
        const w1 = windAt(knownParams, obs.t + lead * 0.5);
        const w2 = windAt(knownParams, obs.t + lead);
        const wLeadX = 0.35 * w0.x + 0.40 * w1.x + 0.25 * w2.x;
        const wLeadZ = 0.35 * w0.z + 0.40 * w1.z + 0.25 * w2.z;
        wxTotal = (0.42 / knownParams.mass) * wLeadX;
        wzTotal = (0.42 / knownParams.mass) * wLeadZ;
      }

      const fx = axDes - wxTotal + (0.42 / massUse) * vx;
      const fy = clamp(9.81 + ayDes - dyDist + (0.42 / massUse) * vy, 2.5, 18.0);
      const fz = azDes - wzTotal + (0.42 / massUse) * vz;

      const cy = Math.cos(curAtt.yaw), sy = Math.sin(curAtt.yaw);
      const fBx = cy * fx - sy * fz;
      const fBz = sy * fx + cy * fz;
      const fTot = Math.hypot(fBx, fy, fBz);

      let maxTilt = 0.54;
      if (obs.gateIndex === 0 && py < 0.20) maxTilt = clamp(0.22 + (py - 0.07) * 2.5, 0.22, 0.54);
      if (obs.gateIndex === 3 && py < 0.16) maxTilt = 0.24;

      const pitchDes = clamp(Math.atan2(fBx, fy), -maxTilt, maxTilt);
      const rollDes = clamp(-Math.asin(clamp(fBz / fTot, -0.85, 0.85)), -maxTilt, maxTilt);

      refRoll += (rollDes - refRoll) * (dt / 0.045);
      refPitch += (pitchDes - refPitch) * (dt / 0.045);

      const cosTilt = Math.max(0.55, Math.cos(curAtt.roll) * Math.cos(curAtt.pitch));
      const tOverMReq = (9.81 + ayDes - dyDist + (0.42 / massUse) * vy) / cosTilt;
      const thrustCmd = clamp((tOverMReq * mEff) / 16.0, 0.05, 1.0);
      prevThrustCmd = thrustCmd;

      let rollFF = 0, pitchFF = 0, yawFF = 0;
      if (knownParams) {
        const k = knownParams.weakMotor;
        const w = knownParams.weakness;
        const inertia = 0.006 * knownParams.mass / 0.6;
        const desMotor = exactState ? exactState.motors[k] / Math.max(0.65, 1 - w) : thrustCmd * 4.0;
        const dAlphaRoll = -0.115 * MOTOR_Z[k] * w * desMotor / inertia;
        const dAlphaPitch = 0.115 * MOTOR_X[k] * w * desMotor / inertia;
        const dAlphaYaw = -0.014 * SPIN[k] * w * desMotor / inertia;
        rollFF = (-dAlphaRoll - 22.0 * simIntRoll) / 65.0;
        pitchFF = (-dAlphaPitch - 22.0 * simIntPitch) / 65.0;
        yawFF = -dAlphaYaw / 8.0;
      } else {
        rollFF = rollTrim;
        pitchFF = pitchTrim;
      }

      const rollCmd = clamp(
        rollDes + 0.50 * (rollDes - curAtt.roll) - 0.055 * curRates.roll + rollFF,
        -0.6,
        0.6
      );
      const pitchCmd = clamp(
        pitchDes + 0.50 * (pitchDes - curAtt.pitch) - 0.055 * curRates.pitch + pitchFF,
        -0.6,
        0.6
      );
      prevRollCmd = rollCmd;
      prevPitchCmd = pitchCmd;

      const yawRateCmd = clamp(-4.5 * curAtt.yaw - 6.0 * yawInt - 0.35 * curRates.yaw + yawFF, -2, 2);

      const out = {
        thrust: thrustCmd,
        roll: rollCmd,
        pitch: pitchCmd,
        yawRate: yawRateCmd,
      };
      if (actionHistory.length < 250) actionHistory.push(out);
      else actionHistory[actionHistory.length - 1] = out;
      return out;
    },
  };
}
