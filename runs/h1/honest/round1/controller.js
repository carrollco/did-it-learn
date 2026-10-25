export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01;
  const g = 9.81;

  // State and disturbance estimators
  let initialized = false;
  let kT = 20.0; // Effective max specific thrust (m/s^2)
  let motorThrust = 0;
  let vyHat = 0;
  let vxHat = 0, vzHat = 0;
  let wxHat = 0, wzHat = 0;
  let wxBase = 0, wzBase = 0;

  // Inner attitude loop tracking & rotational disturbance observer
  let simIntRoll = 0, simIntPitch = 0;
  let tauFiltRoll = 0, tauFiltPitch = 0;
  let rateHatRoll = 0, rateHatPitch = 0;
  let bRoll = 0, bPitch = 0;
  let lastAction = { thrust: 0.5, roll: 0, pitch: 0, yawRate: 0 };

  // Navigation state
  let currentGate = 0;
  let gatePhase = 'approach'; // 'approach' | 'lunge'
  let padPhase = 'approach'; // 'approach' | 'descend' | 'touchdown'

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;
      const t = obs.t;

      if (!initialized) {
        initialized = true;
        vyHat = v.y;
        vxHat = v.x;
        vzHat = v.z;
        rateHatRoll = rates.roll;
        rateHatPitch = rates.pitch;
      } else {
        // 1. Update inner attitude loop replica & rotational disturbance observer
        const errRoll = lastAction.roll - att.roll;
        const errPitch = lastAction.pitch - att.pitch;
        simIntRoll = clamp(simIntRoll + errRoll * dt, -0.35, 0.35);
        simIntPitch = clamp(simIntPitch + errPitch * dt, -0.35, 0.35);

        const tauCmdRoll = 65 * errRoll - 13 * rates.roll + 22 * simIntRoll;
        const tauCmdPitch = 65 * errPitch - 13 * rates.pitch + 22 * simIntPitch;
        const alphaLag = 1 - Math.exp(-dt / 0.04);
        tauFiltRoll += (tauCmdRoll - tauFiltRoll) * alphaLag;
        tauFiltPitch += (tauCmdPitch - tauFiltPitch) * alphaLag;

        rateHatRoll += (tauFiltRoll - 0.4 * rateHatRoll + bRoll) * dt;
        rateHatPitch += (tauFiltPitch - 0.4 * rateHatPitch + bPitch) * dt;

        // Rotational dynamics in sim.mjs are unaffected by floor contact as long as motors are not saturated at 0
        const validRot = obs.gateIndex < 3 ? (p.y > 0.085) : (lastAction.thrust > 0.18);
        if (validRot) {
          const eRoll = rates.roll - rateHatRoll;
          const ePitch = rates.pitch - rateHatPitch;
          rateHatRoll += 0.65 * eRoll;
          rateHatPitch += 0.65 * ePitch;
          bRoll = clamp(bRoll + 25.0 * eRoll, -25, 25);
          bPitch = clamp(bPitch + 25.0 * ePitch, -25, 25);
        } else {
          rateHatRoll = rates.roll;
          rateHatPitch = rates.pitch;
        }

        // 2. Update motor thrust filter & translational observers
        motorThrust += (lastAction.thrust - motorThrust) * alphaLag;

        const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
        const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
        const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
        const bodyX = sp * cr, bodyZ = -sr;
        const dirX = cy * bodyX + sy * bodyZ;
        const dirY = cp * cr;
        const dirZ = -sy * bodyX + cy * bodyZ;

        const cd = 0.42 / clamp(14.5 / kT, 0.6, 0.95);
        const aTot = kT * motorThrust;

        const validTrans = obs.gateIndex < 3 ? (p.y > 0.09 && p.y < 2.88) : (p.y > 0.10 && p.y < 2.88 && lastAction.thrust > 0.18);
        if (validTrans) {
          vyHat += (aTot * dirY - g - cd * vyHat) * dt;
          const evy = v.y - vyHat;
          vyHat += 0.6 * evy;
          if (motorThrust * dirY > 0.22) {
            kT = clamp(kT + 2.2 * evy / (motorThrust * dirY), 13.5, 27.5);
          }
        } else {
          vyHat = v.y;
        }

        const nearWall = Math.abs(p.x) > 2.78 || Math.abs(p.z) > 2.78;
        if (validTrans && !nearWall) {
          vxHat += (aTot * dirX - cd * vxHat + wxHat) * dt;
          vzHat += (aTot * dirZ - cd * vzHat + wzHat) * dt;
          const evx = v.x - vxHat;
          const evz = v.z - vzHat;
          vxHat += 0.7 * evx;
          vzHat += 0.7 * evz;
          wxHat = clamp(wxHat + 30.0 * evx, -18, 18);
          wzHat = clamp(wzHat + 30.0 * evz, -18, 18);
          if (t > 0.25 && t < 1.95) {
            wxBase += 0.08 * (wxHat - wxBase);
            wzBase += 0.08 * (wzHat - wzBase);
          }
        } else {
          vxHat = v.x;
          vzHat = v.z;
        }
      }

      // 3. Navigation & Trajectory Planning
      let tx = p.x, ty = 1.0, tz = p.z;
      let tvx = 0, tvy = 0, tvz = 0;
      let kpX = 6.0, kdX = 4.2;
      let kpZ = 8.5, kdZ = 5.0;
      let kpY = 8.0, kdY = 4.8;

      if (obs.gateIndex < 3) {
        const gPos = obs.nextGate.centre;
        if (currentGate !== obs.gateIndex) {
          currentGate = obs.gateIndex;
          gatePhase = 'approach';
        }

        const dxGate = gPos.x - p.x;
        const dyGate = gPos.y - p.y;
        const dzGate = gPos.z - p.z;

        // Predict y and z when crossing x = gPos.x
        const tCross = clamp(dxGate / Math.max(0.5, v.x), 0.02, 0.28);
        const zPred = p.z + v.z * tCross;
        const yPred = p.y + v.y * tCross;

        const zAligned = Math.abs(dzGate) < 0.015 && Math.abs(zPred - gPos.z) < 0.018 && Math.abs(v.z) < 0.14;
        const yAligned = Math.abs(dyGate) < 0.055 && Math.abs(yPred - gPos.y) < 0.075 && Math.abs(v.y) < 0.30;
        const gustDeltaZ = Math.abs(wzHat - wzBase);

        if (gatePhase === 'approach') {
          ty = gPos.y;
          tz = gPos.z;
          // Approach point before gate
          const standOff = 0.24;
          tx = gPos.x - standOff;

          // Smooth velocity feedforward toward standoff if far
          if (dxGate > 0.35) {
            const distYZ = Math.hypot(dyGate, dzGate);
            if (distYZ > 0.15) {
              const speedYZ = Math.min(1.4, distYZ * 2.2);
              tvy = (dyGate / distYZ) * speedYZ * 0.5;
              tvz = (dzGate / distYZ) * speedYZ * 0.5;
            }
          }

          if (dxGate < 0.38 && dxGate > -0.02 && zAligned && yAligned && gustDeltaZ < 2.2) {
            gatePhase = 'lunge';
          }
        }

        if (gatePhase === 'lunge') {
          ty = gPos.y;
          tz = gPos.z;
          tx = gPos.x + 0.38;
          tvx = 0.95;
          kpZ = 12.0;
          kdZ = 6.0;
          kpY = 10.0;
          kdY = 5.5;

          // Abort lunge if knocked off alignment before committing (when still > 0.075m before gate)
          const zSafeNow = Math.abs(dzGate) < 0.023 && Math.abs(zPred - gPos.z) < 0.025;
          const ySafeNow = Math.abs(dyGate) < 0.095 && Math.abs(yPred - gPos.y) < 0.11;
          if (dxGate > 0.075 && (!zSafeNow || !ySafeNow)) {
            gatePhase = 'approach';
            tx = gPos.x - 0.25;
            tvx = 0;
          }
        }

        // If we somehow passed x = gPos.x without crossing (missed gate), go around safely
        if (dxGate <= 0) {
          gatePhase = 'approach';
          if (Math.abs(dzGate) < 0.022 && Math.abs(dyGate) < 0.08) {
            tx = gPos.x - 0.28;
          } else {
            tx = p.x;
            ty = gPos.y;
            tz = gPos.z;
          }
        }
      } else {
        // Gate 3 passed -> fly to landing pad (2.05, 0, 1.65)
        const pad = obs.pad.centre;
        const dxPad = pad.x - p.x;
        const dzPad = pad.z - p.z;
        const distH = Math.hypot(dxPad, dzPad);
        const speedH = Math.hypot(v.x, v.z);
        const curTilt = Math.max(Math.abs(att.roll), Math.abs(att.pitch));
        const windMag = Math.hypot(wxHat, wzHat);

        tx = pad.x;
        tz = pad.z;
        kpX = 8.0; kdX = 5.0;
        kpZ = 8.0; kdZ = 5.0;

        // Hold safe hover height y >= 0.24m when tilt or wind is high
        const safeToTouchFloor = curTilt < 0.24 && windMag < 3.2 && speedH < 0.38;

        if (padPhase === 'approach') {
          ty = (p.x > 1.42 || p.y < 0.55) ? (distH <= 0.45 ? 0.21 : 0.36) : 0.65;
          if (distH < 0.12 && p.y < 0.38 && safeToTouchFloor) {
            padPhase = 'descend';
          }
        }
        if (padPhase === 'descend') {
          ty = 0.068;
          tvy = p.y > 0.15 ? -0.34 : -0.22;
          if (distH > 0.135 || curTilt > 0.27 || Math.hypot(v.x, v.y, v.z) > 0.60) {
            padPhase = 'approach';
            ty = 0.22;
            tvy = 0;
          } else if (p.y <= 0.090 && distH <= 0.132 && speedH < 0.40 && curTilt < 0.17) {
            padPhase = 'touchdown';
          }
        }
        if (padPhase === 'touchdown') {
          if (distH > 0.142) {
            padPhase = 'approach';
            ty = 0.24;
          } else {
            // Active pinned hold on the pad: light positive thrust keeps motors linear while gravity pins to y=0.07
            const axHold = clamp(12.0 * dxPad - 5.5 * v.x - 0.7 * wxHat, -1.5, 1.5);
            const azHold = clamp(12.0 * dzPad - 5.5 * v.z - 0.7 * wzHat, -1.5, 1.5);
            const aHoldY = 5.2;
            const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
            const bX = (cy * axHold - sy * azHold) / aHoldY;
            const bZ = (sy * axHold + cy * azHold) / aHoldY;
            const rDes = clamp(-bZ, -0.11, 0.11);
            const pDes = clamp(bX, -0.11, 0.11);
            const alphaRoll = 120 * (rDes - att.roll) - 20 * rates.roll;
            const alphaPitch = 120 * (pDes - att.pitch) - 20 * rates.pitch;
            const cmdRoll = clamp(att.roll + (alphaRoll + 13 * rates.roll - 22 * simIntRoll - bRoll) / 65, -0.35, 0.35);
            const cmdPitch = clamp(att.pitch + (alphaPitch + 13 * rates.pitch - 22 * simIntPitch - bPitch) / 65, -0.35, 0.35);
            const act = { thrust: clamp(aHoldY / kT, 0.22, 0.36), roll: cmdRoll, pitch: cmdPitch, yawRate: clamp(-0.4 * att.yaw + 0.5 * rates.yaw, -0.5, 0.5) };
            lastAction = act;
            return act;
          }
        }
      }

      // 4. Compute desired world acceleration (including wind cancellation & drag compensation)
      const cd = 0.42 / clamp(14.5 / kT, 0.6, 0.95);
      const ex = tx - p.x, ey = ty - p.y, ez = tz - p.z;
      const evx = tvx - v.x, evy = tvy - v.y, evz = tvz - v.z;

      let axDes = kpX * ex + kdX * evx + cd * v.x - wxHat;
      let azDes = kpZ * ez + kdZ * evz + cd * v.z - wzHat;
      let ayDes = g + kpY * ey + kdY * evy + cd * v.y;

      // Wall safety barrier
      if (p.x < -2.55 && v.x < 0) axDes = Math.max(axDes, 4.0);
      if (p.x > 2.55 && v.x > 0) axDes = Math.min(axDes, -4.0);
      if (p.z < -2.55 && v.z < 0) azDes = Math.max(azDes, 4.0);
      if (p.z > 2.55 && v.z > 0) azDes = Math.min(azDes, -4.0);
      if (p.y > 2.65 && v.y > 0) ayDes = Math.min(ayDes, g - 3.0);

      ayDes = clamp(ayDes, 2.5, 18.0);
      const maxHorizAcc = obs.gateIndex === 3 ? 9.5 : 6.5;
      const horizMag = Math.hypot(axDes, azDes);
      if (horizMag > maxHorizAcc) {
        axDes *= maxHorizAcc / horizMag;
        azDes *= maxHorizAcc / horizMag;
      }

      // 5. Exact inverse kinematics from desired thrust vector to (thrust, rollDes, pitchDes)
      const aMag = Math.hypot(axDes, ayDes, azDes);
      const dx = axDes / aMag, dy = ayDes / aMag, dz = azDes / aMag;

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const bodyX = cy * dx - sy * dz;
      const bodyZ = sy * dx + cy * dz;

      let maxTilt = obs.gateIndex === 3 ? 0.58 : 0.52;
      if (obs.gateIndex === 3 && padPhase === 'descend' && p.y < 0.12) {
        maxTilt = 0.20;
      }
      const rollDes = clamp(-Math.asin(clamp(bodyZ, -0.9, 0.9)), -maxTilt, maxTilt);
      const pitchDes = clamp(Math.atan2(bodyX, dy), -maxTilt, maxTilt);

      // Desired thrust: project target vertical acceleration onto actual attitude direction
      const actualCos = Math.max(0.55, Math.cos(att.roll) * Math.cos(att.pitch));
      const thrust = clamp((ayDes / actualCos) / kT, 0.05, 0.98);

      // 6. High-bandwidth inner-loop attitude command with disturbance cancellation
      const alphaRoll = 125 * (rollDes - att.roll) - 20 * rates.roll;
      const alphaPitch = 125 * (pitchDes - att.pitch) - 20 * rates.pitch;

      const rollCmd = clamp(att.roll + (alphaRoll + 13 * rates.roll - 22 * simIntRoll - bRoll) / 65, -0.6, 0.6);
      const pitchCmd = clamp(att.pitch + (alphaPitch + 13 * rates.pitch - 22 * simIntPitch - bPitch) / 65, -0.6, 0.6);
      const yawRateCmd = clamp(-0.6 * att.yaw + 0.5 * rates.yaw, -0.8, 0.8);

      const action = { thrust, roll: rollCmd, pitch: pitchCmd, yawRate: yawRateCmd };
      lastAction = action;
      return action;
    },
  };
}
