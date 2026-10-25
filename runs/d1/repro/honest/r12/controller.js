export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01;
  const g = 9.81;
  const m0 = 0.725;
  const drag0 = 0.42 / m0;
  const kT0 = 15.5 / m0;

  let initialized = false;
  let vf = { x: 0, y: 0, z: 0 };
  let prevVf = { x: 0, y: 0, z: 0 };
  let aNomF = { x: 0, y: 0, z: 0 };
  let dSlow = { x: 0, y: 0, z: 0 };
  let dFast = { x: 0, y: 0, z: 0 };
  let uThrust = 0.45;
  let lastThrust = 0.45;

  let intX = 0, intY = 0, intZ = 0, intYaw = 0;
  let ref = { x: 0, y: 0, z: 0 };
  let refVel = { x: 0, y: 0, z: 0 };

  let activeGateIdx = 0;
  let lockedGate = null;
  let phase = 'align';
  let settle = 0;
  let padY = 0.65;

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;

      if (!initialized) {
        initialized = true;
        vf = { x: v.x, y: v.y, z: v.z };
        prevVf = { x: v.x, y: v.y, z: v.z };
        ref = { x: p.x, y: p.y, z: p.z };
        if (obs.nextGate) lockedGate = { ...obs.nextGate.centre };
      }

      // 1. Filter velocity
      const alphaV = 0.40;
      for (const a of ['x', 'y', 'z']) {
        prevVf[a] = vf[a];
        vf[a] += alphaV * (v[a] - vf[a]);
      }

      // 2. Delay-matched fast/slow acceleration disturbance observer
      uThrust += (lastThrust - uThrust) * (1 - Math.exp(-dt / 0.04));
      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
      const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
      const bodyX = sp * cr, bodyZ = -sr;
      const dir = {
        x: cy * bodyX + sy * bodyZ,
        y: cp * cr,
        z: -sy * bodyX + cy * bodyZ,
      };

      const aMeasY = (vf.y - prevVf.y) / dt;
      const aInstY = kT0 * uThrust * dir.y - drag0 * vf.y - g;
      aNomF.y += alphaV * (aInstY - aNomF.y);
      const dRawY = clamp(aMeasY - aNomF.y, -8, 8);
      if (p.y > 0.095 && obs.t > 0.08) {
        dSlow.y += 0.035 * (dRawY - dSlow.y);
        dFast.y += 0.18 * (dRawY - dFast.y);
      }

      const tSpec = clamp(kT0 * uThrust + dSlow.y / Math.max(0.7, dir.y), 4, 25);
      for (const a of ['x', 'z']) {
        const aMeas = (vf[a] - prevVf[a]) / dt;
        const aInst = tSpec * dir[a] - drag0 * vf[a];
        aNomF[a] += alphaV * (aInst - aNomF[a]);
        const dRaw = clamp(aMeas - aNomF[a], -10, 10);
        if (p.y > 0.095 && obs.t > 0.08) {
          dSlow[a] += 0.035 * (dRaw - dSlow[a]);
          dFast[a] += 0.24 * (dRaw - dFast[a]);
        }
      }

      // 3. State machine: gate exit lock, predictive cross abort, ramp pad descent
      let target;
      if (activeGateIdx < 3) {
        if (obs.gateIndex === activeGateIdx && obs.nextGate) {
          lockedGate = { ...obs.nextGate.centre };
        }
        const gate = lockedGate;

        if (obs.gateIndex > activeGateIdx) {
          phase = 'exit';
          if (p.x > gate.x + 0.22) {
            activeGateIdx = obs.gateIndex;
            settle = 0;
            intX *= 0.2; intZ *= 0.2;
            if (activeGateIdx < 3 && obs.nextGate) {
              lockedGate = { ...obs.nextGate.centre };
              phase = 'align';
            } else {
              phase = 'pad';
              padY = clamp(p.y, 0.45, 0.85);
            }
          }
        }

        if (activeGateIdx < 3) {
          const gCur = lockedGate;
          const yzErr = Math.hypot(gCur.y - p.y, gCur.z - p.z);
          const alignX = gCur.x - (yzErr > 0.20 ? 0.52 : 0.42);

          if (phase === 'align') {
            target = { x: alignX, y: gCur.y, z: gCur.z };
            const ex = Math.abs(gCur.x - 0.42 - p.x);
            const ey = Math.abs(target.y - p.y);
            const ez = Math.abs(target.z - p.z);
            const gustMag = Math.hypot(dFast.x - dSlow.x, dFast.z - dSlow.z);
            if (ex < 0.12 && ey < 0.085 && ez < 0.052 &&
                Math.abs(vf.x) < 0.28 && Math.abs(vf.y) < 0.25 && Math.abs(vf.z) < 0.18 &&
                gustMag < 1.45) {
              settle++;
            } else {
              settle = Math.max(0, settle - 1);
            }
            if (settle >= 9) {
              phase = 'cross';
              settle = 0;
            }
          }

          if (phase === 'cross') {
            const dx = gCur.x - p.x;
            const vxPos = Math.max(0, vf.x);
            const stopDist = vxPos * 0.075 + (vxPos * vxPos) / (2 * 3.8);
            if (dx > stopDist + 0.015 && dx > 0.05) {
              const tCross = clamp(dx / Math.max(0.35, vf.x), 0.05, 0.38);
              const ayGust = dFast.y - dSlow.y;
              const azGust = dFast.z - dSlow.z;
              const yPred = p.y + vf.y * tCross + 0.35 * ayGust * tCross * tCross;
              const zPred = p.z + vf.z * tCross + 0.35 * azGust * tCross * tCross;
              const dyCur = Math.abs(p.y - gCur.y), dzCur = Math.abs(p.z - gCur.z);
              const dyPred = Math.abs(yPred - gCur.y), dzPred = Math.abs(zPred - gCur.z);
              if (dyCur > 0.155 || dzCur > 0.075 || dyPred > 0.145 || dzPred > 0.072 ||
                  (dx > 0.12 && Math.abs(azGust) > 1.50)) {
                phase = 'align';
                settle = 0;
                target = { x: gCur.x - 0.42, y: gCur.y, z: gCur.z };
                ref.x = Math.min(ref.x, p.x - 0.04);
                refVel.x = Math.min(refVel.x, 0);
              }
            }
            if (phase === 'cross') {
              if (p.x > gCur.x + 0.04 && obs.gateIndex === activeGateIdx) {
                phase = 'align';
                settle = 0;
                target = { x: gCur.x - 0.42, y: gCur.y, z: gCur.z };
              } else {
                target = { x: gCur.x + 0.42, y: gCur.y, z: gCur.z };
              }
            }
          }

          if (phase === 'exit') {
            target = { x: gCur.x + 0.45, y: gCur.y, z: gCur.z };
          }
        }
      }

      if (activeGateIdx >= 3) {
        const pad = obs.pad.centre;
        const errH = Math.hypot(pad.x - p.x, pad.z - p.z);
        const speedH = Math.hypot(vf.x, vf.z);
        if (errH < 0.25) {
          if (errH < 0.115 && speedH < 0.32) {
            const rate = p.y < 0.22 ? 0.25 : 0.48;
            padY = Math.max(-0.02, padY - rate * dt);
          } else if (p.y > 0.095 && errH > 0.135) {
            padY = clamp(p.y, 0.12, 0.55);
          }
        } else {
          padY = Math.max(0.45, padY - 0.35 * dt);
        }
        target = { x: pad.x, y: padY, z: pad.z };
      }

      // 4. Smooth reference filter with braking profile & bounded lead
      const isTight = (phase === 'cross' || phase === 'exit');
      for (const a of ['x', 'y', 'z']) {
        const diff = target[a] - ref[a];
        const dist = Math.abs(diff);
        const maxV = isTight ? (a === 'x' ? 1.05 : 1.35) : (a === 'x' ? 0.95 : (a === 'y' ? 0.95 : 1.05));
        const tau = isTight ? (a === 'x' ? 0.08 : 0.06) : 0.15;
        const vBrake = Math.sqrt(2 * 1.6 * dist);
        const speed = Math.min(maxV, dist / tau, vBrake);
        const desV = diff >= 0 ? speed : -speed;
        refVel[a] += 0.30 * (desV - refVel[a]);
        ref[a] = clamp(ref[a] + refVel[a] * dt, p[a] - 0.24, p[a] + 0.24);
      }

      // 5. Yaw + local position integrals
      intYaw = clamp(intYaw + att.yaw * dt, -0.25, 0.25);
      const yawRate = clamp(-1.8 * att.yaw - 0.8 * intYaw - 0.15 * rates.yaw, -0.45, 0.45);

      const distToTarget = Math.hypot(target.x - p.x, target.z - p.z);
      const ex = ref.x - p.x, ey = ref.y - p.y, ez = ref.z - p.z;
      const exRaw = target.x - p.x, ezRaw = target.z - p.z;

      if (distToTarget < 0.32 && p.y > 0.08) {
        intX = clamp(intX + exRaw * dt, -0.40, 0.40);
        intZ = clamp(intZ + ezRaw * dt, -0.40, 0.40);
      } else {
        intX *= 0.97;
        intZ *= 0.97;
      }
      if (p.y > 0.08) {
        intY = clamp(intY + (target.y - p.y) * dt, -0.50, 0.50);
      }

      // 6. Acceleration command with disturbance rejection
      const kpX = isTight ? 5.8 : 4.8;
      const kdX = isTight ? 3.8 : 3.7;
      const kpZ = isTight ? 6.8 : 5.0;
      const kdZ = isTight ? 4.4 : 3.9;

      const dEstX = dSlow.x + 0.85 * (dFast.x - dSlow.x);
      const dEstZ = dSlow.z + 0.85 * (dFast.z - dSlow.z);
      const dEstY = dSlow.y + 0.45 * (dFast.y - dSlow.y);

      const ffVx = (phase === 'align' && p.x > target.x - 0.20) ? 0 : 0.45 * refVel.x;
      let ax = clamp(
        kpX * ex + kdX * (ffVx - vf.x) + 2.0 * intX + drag0 * vf.x - dEstX,
        -4.2, 4.2
      );
      if (phase === 'exit' && lockedGate && p.x < lockedGate.x + 0.12) {
        ax = Math.max(ax, 2.8);
      }
      const az = clamp(
        kpZ * ez + kdZ * (0.45 * refVel.z - vf.z) + 2.2 * intZ + drag0 * vf.z - dEstZ,
        -4.2, 4.2
      );
      const ay = clamp(
        5.5 * ey + 3.6 * (0.45 * refVel.y - vf.y) + 2.0 * intY + drag0 * vf.y - dEstY,
        -3.5, 3.6
      );

      // 7. Convert desired accelerations to attitude & thrust
      const vertAcc = clamp(tSpec * dir.y, 6.5, 15.5);
      let maxTilt = 0.45;
      if (activeGateIdx >= 3 && p.y < 0.12 && Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z) < 0.12) {
        maxTilt = 0.14;
      }
      const pitch = clamp((cy * ax - sy * az) / vertAcc, -maxTilt, maxTilt);
      const roll = clamp(-(sy * ax + cy * az) / vertAcc, -maxTilt, maxTilt);

      const tiltMag = Math.hypot(roll, pitch);
      const maxThr = tiltMag > 0.32 ? 0.66 : 0.74;
      let thrust = clamp((g + ay) / (kT0 * Math.max(0.68, cr * cp)), 0.05, maxThr);

      if (activeGateIdx >= 3 && p.y <= 0.082 && Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z) < 0.11 && Math.hypot(vf.x, vf.y, vf.z) < 0.35) {
        thrust = Math.min(thrust, 0.12);
      }

      lastThrust = thrust;
      return { thrust, roll, pitch, yawRate };
    },
  };
}
