export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const dt = 0.01;

  let initialized = false;
  let px = 0, py = 0, pz = 0;
  let vx = 0, vy = 0, vz = 0;
  let dx = 0, dy = 0, dz = 0;
  let Ku = 21.4;
  let uFilt = 0.45;
  let lastThrust = 0.45;
  let lastPitchDes = 0, lastRollDes = 0;
  let pitchExp = 0, rollExp = 0;
  let pitchTrim = 0, rollTrim = 0, yawInt = 0;

  let gateIdx = -1;
  let phase = 'stage';
  let stageTimer = 0;

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att, rates = obs.rates;
      if (!initialized) {
        initialized = true;
        px = p.x; py = p.y; pz = p.z;
        vx = v.x; vy = v.y; vz = v.z;
      }

      const cRoll = Math.cos(att.roll), sRoll = Math.sin(att.roll);
      const cPitch = Math.cos(att.pitch), sPitch = Math.sin(att.pitch);
      const cYaw = Math.cos(att.yaw), sYaw = Math.sin(att.yaw);
      const bodyX = sPitch * cRoll, bodyZ = -sRoll;
      const dirX = cYaw * bodyX + sYaw * bodyZ;
      const dirY = cPitch * cRoll;
      const dirZ = -sYaw * bodyX + cYaw * bodyZ;

      uFilt += (lastThrust - uFilt) * 0.22;
      const massHat = clamp(15.5 / Ku, 0.58, 0.88);
      const dragCoeff = 0.42 / massHat;

      if (p.y > 0.082) {
        const aThrustMag = Math.max(0, Ku * uFilt);
        const axKnown = aThrustMag * dirX - dragCoeff * vx;
        const ayKnown = aThrustMag * dirY - 9.81 - dragCoeff * vy;
        const azKnown = aThrustMag * dirZ - dragCoeff * vz;

        px += (vx + 0.5 * (axKnown + dx) * dt) * dt;
        vx += (axKnown + dx) * dt;
        const epx = p.x - px, evx = v.x - vx;
        px += 0.55 * epx + 0.004 * evx;
        vx += 12.0 * epx + 0.45 * evx;
        dx = clamp(dx + 35.0 * epx + 18.0 * evx, -5.0, 5.0);

        py += (vy + 0.5 * (ayKnown + dy) * dt) * dt;
        vy += (ayKnown + dy) * dt;
        const epy = p.y - py, evy = v.y - vy;
        py += 0.55 * epy + 0.004 * evy;
        vy += 12.0 * epy + 0.45 * evy;
        dy = clamp(dy + 25.0 * epy + 12.0 * evy, -4.0, 4.0);
        if (uFilt > 0.25 && uFilt < 0.78 && dirY > 0.75) {
          Ku = clamp(Ku + 0.8 * dy * dt, 16.5, 27.5);
          dy *= 0.96;
        }

        pz += (vz + 0.5 * (azKnown + dz) * dt) * dt;
        vz += (azKnown + dz) * dt;
        const epz = p.z - pz, evz = v.z - vz;
        pz += 0.55 * epz + 0.004 * evz;
        vz += 12.0 * epz + 0.45 * evz;
        dz = clamp(dz + 35.0 * epz + 18.0 * evz, -5.0, 5.0);

        pitchExp += (lastPitchDes - pitchExp) * (dt / 0.075);
        rollExp += (lastRollDes - rollExp) * (dt / 0.075);
        pitchTrim = clamp(pitchTrim + 1.8 * (pitchExp - att.pitch) * dt, -0.18, 0.18);
        rollTrim = clamp(rollTrim + 1.8 * (rollExp - att.roll) * dt, -0.18, 0.18);
      } else {
        px = p.x; py = p.y; pz = p.z;
        vx = v.x; vy = v.y; vz = v.z;
      }

      let tx = px, ty = py, tz = pz;
      let tvx = 0, tvy = 0, tvz = 0;

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (gateIdx !== obs.gateIndex) {
          gateIdx = obs.gateIndex;
          phase = 'stage';
          stageTimer = 0;
        }

        const distYZ = Math.hypot(gate.y - py, gate.z - pz);
        const velYZ = Math.hypot(vy, vz);

        if (phase === 'stage') {
          const stageOffset = distYZ > 0.25 ? 0.58 : (distYZ > 0.12 ? 0.44 : 0.34);
          tx = gate.x - stageOffset;
          ty = gate.y;
          tz = gate.z;
          const distX = Math.abs((gate.x - 0.34) - px);
          if (distYZ < 0.042 && velYZ < 0.16 && distX < 0.11 && Math.abs(vx) < 0.22) {
            stageTimer++;
          } else {
            stageTimer = Math.max(0, stageTimer - 1);
          }
          if (stageTimer >= 6) {
            phase = 'cross';
          }
        }

        if (phase === 'cross') {
          const remX = gate.x - px;
          if (remX > 0.06) {
            const tCross = clamp(remX / Math.max(0.35, vx), 0.05, 0.35);
            const predY = py + vy * tCross;
            const predZ = pz + vz * tCross;
            const canStop = px + (vx > 0 ? (vx * vx) / 7.0 : 0) < gate.x - 0.04;
            if (canStop && (Math.abs(predZ - gate.z) > 0.065 || Math.abs(predY - gate.y) > 0.13 || Math.abs(vz) > 0.28)) {
              phase = 'stage';
              stageTimer = 0;
              tx = gate.x - 0.34;
              ty = gate.y;
              tz = gate.z;
            } else {
              tx = gate.x + 0.28;
              ty = gate.y;
              tz = gate.z;
              tvx = 0.85;
            }
          } else {
            tx = gate.x + 0.28;
            ty = gate.y;
            tz = gate.z;
            tvx = 0.8;
          }
        }

        if (obs.gateIndex > 0) {
          const prevX = obs.gateIndex === 1 ? -1.15 : 0.15;
          const prevY = obs.gateIndex === 1 ? 1.05 : 1.40;
          const prevZ = obs.gateIndex === 1 ? -0.85 : 0.25;
          if (px < prevX + 0.18) {
            tx = Math.max(tx, prevX + 0.28);
            ty = prevY + clamp((px - prevX) / 0.22, 0, 1) * (ty - prevY);
            tz = prevZ + clamp((px - prevX) / 0.22, 0, 1) * (tz - prevZ);
          }
        }
        if (Math.abs(pz - gate.z) > 0.07 || Math.abs(py - gate.y) > 0.14) {
          const safeBack = distYZ > 0.25 ? 0.55 : 0.34;
          tx = Math.min(tx, gate.x - safeBack);
        }
      } else {
        const pad = obs.pad.centre;
        if (gateIdx !== 3) {
          gateIdx = 3;
          phase = 'pad_approach';
          stageTimer = 0;
        }
        tx = pad.x;
        tz = pad.z;
        if (px < 1.55) {
          tx = Math.max(tx, 1.65);
          const u = clamp((px - 1.35) / 0.22, 0, 1);
          ty = 1.05 + u * (0.65 - 1.05);
          tz = 1.05 + u * (pad.z - 1.05);
        } else {
          const distXZ = Math.hypot(pad.x - px, pad.z - pz);
          const velXZ = Math.hypot(vx, vz);
          const maxTilt = Math.max(Math.abs(att.roll), Math.abs(att.pitch));
          if (phase === 'pad_approach') {
            ty = distXZ > 0.25 ? 0.55 : 0.20;
            if (distXZ < 0.05 && velXZ < 0.15 && py < 0.26 && maxTilt < 0.24) {
              phase = 'descend';
            }
          }
          if (phase === 'descend') {
            if (distXZ > 0.062 || velXZ > 0.25 || maxTilt > 0.28) {
              ty = Math.max(0.15, py);
              tvy = 0;
            } else {
              ty = 0.05;
              tvy = py > 0.12 ? -0.28 : -0.14;
            }
          }
        }
      }

      const ex = clamp(tx - px, -0.6, 0.6);
      const ez = clamp(tz - pz, -0.6, 0.6);
      const ey = clamp(ty - py, -0.7, 0.7);

      const inGateZone = obs.gateIndex < 3 && Math.abs(px - obs.nextGate.centre.x) < 0.45;
      const kpX = 6.2, kdX = 4.5;
      const kpZ = inGateZone ? 9.5 : 6.5, kdZ = inGateZone ? 5.6 : 4.5;
      const kpY = inGateZone ? 9.0 : 7.2, kdY = 4.8;

      const axCmd = clamp(kpX * ex + kdX * (tvx - vx) - dx + dragCoeff * vx, -4.2, 4.2);
      const azCmd = clamp(kpZ * ez + kdZ * (tvz - vz) - dz + dragCoeff * vz, -4.2, 4.2);
      const ayCmd = clamp(kpY * ey + kdY * (tvy - vy) - dy + dragCoeff * vy, -4.0, 4.5);

      const nearPadFloor = obs.gateIndex === 3 && phase === 'descend' && py < 0.20;
      if (nearPadFloor && py <= 0.080 && Math.hypot(obs.pad.centre.x - px, obs.pad.centre.z - pz) <= 0.095 &&
          Math.hypot(vx, vy, vz) < 0.35 && Math.max(Math.abs(att.roll), Math.abs(att.pitch)) < 0.28) {
        const bxPad = (cYaw * axCmd - sYaw * azCmd) / 9.81;
        const bzPad = (sYaw * axCmd + cYaw * azCmd) / 9.81;
        const pDes = clamp(bxPad, -0.13, 0.13);
        const rDes = clamp(-bzPad, -0.13, 0.13);
        const pPad = clamp(pDes + pitchTrim, -0.15, 0.15);
        const rPad = clamp(rDes + rollTrim, -0.15, 0.15);
        lastThrust = 0.26;
        lastPitchDes = pDes;
        lastRollDes = rDes;
        return { thrust: 0.26, roll: rPad, pitch: pPad, yawRate: -2 * att.yaw };
      }

      const ayThrust = Math.max(4.5, 9.81 + ayCmd);
      const bx = (cYaw * axCmd - sYaw * azCmd) / ayThrust;
      const bz = (sYaw * axCmd + cYaw * azCmd) / ayThrust;
      const maxDesTilt = nearPadFloor ? 0.24 : 0.46;
      const pitchDes = clamp(Math.atan(bx), -maxDesTilt, maxDesTilt);
      const rollDes = clamp(Math.atan(-bz * Math.cos(pitchDes)), -maxDesTilt, maxDesTilt);
      lastPitchDes = pitchDes;
      lastRollDes = rollDes;

      const maxCmdTilt = nearPadFloor ? 0.27 : 0.55;
      const pitch = clamp(pitchDes + 0.55 * (pitchDes - att.pitch) - 0.035 * rates.pitch + pitchTrim, -maxCmdTilt, maxCmdTilt);
      const roll = clamp(rollDes + 0.55 * (rollDes - att.roll) - 0.035 * rates.roll + rollTrim, -maxCmdTilt, maxCmdTilt);
      const thrust = clamp(ayThrust / (Ku * Math.max(0.55, dirY)), 0.08, 0.83);
      lastThrust = thrust;

      yawInt = clamp(yawInt + att.yaw * dt, -0.4, 0.4);
      const yawRate = clamp(-3.5 * att.yaw - 1.5 * yawInt - 0.25 * rates.yaw, -2, 2);

      return { thrust, roll, pitch, yawRate };
    },
  };
}
