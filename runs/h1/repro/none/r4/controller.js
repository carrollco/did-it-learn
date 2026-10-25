function clamp(x, lo, hi) {
  return x < lo ? lo : x > hi ? hi : x;
}

function wrapAngle(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

export default function makeController() {
  let prevGateIndex = 0;
  let phase = "align";
  let settle = 0;
  let posEst = null;
  let velEst = null;
  let prevObsVel = null;
  let thrustEst = 0.45;
  let prevCmd = { x: 0, y: 0, z: 0 };
  let dxEst = 0;
  let dzEst = 0;
  let b0Est = 20.0;
  let ix = 0;
  let iy = 0;
  let iz = 0;
  let prevGate = null;
  let currentGateCache = null;

  return {
    step(obs) {
      const dt = 0.01;
      const g = 9.81;

      if (!posEst) {
        posEst = { x: obs.pos.x, y: obs.pos.y, z: obs.pos.z };
        velEst = { x: obs.vel.x, y: obs.vel.y, z: obs.vel.z };
        prevObsVel = { x: obs.vel.x, y: obs.vel.y, z: obs.vel.z };
      } else {
        posEst.x = 0.65 * obs.pos.x + 0.35 * (posEst.x + velEst.x * dt);
        posEst.y = 0.65 * obs.pos.y + 0.35 * (posEst.y + velEst.y * dt);
        posEst.z = 0.65 * obs.pos.z + 0.35 * (posEst.z + velEst.z * dt);
        velEst.x = 0.55 * obs.vel.x + 0.45 * velEst.x;
        velEst.y = 0.55 * obs.vel.y + 0.45 * velEst.y;
        velEst.z = 0.55 * obs.vel.z + 0.45 * velEst.z;
      }

      const pos = posEst;
      const v = velEst;

      const cy = Math.cos(obs.att.yaw);
      const sy = Math.sin(obs.att.yaw);
      const cr = Math.cos(obs.att.roll);
      const sr = Math.sin(obs.att.roll);
      const cp = Math.cos(obs.att.pitch);
      const sp = Math.sin(obs.att.pitch);
      const bodyX = sp * cr;
      const bodyZ = -sr;
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = cp * cr;
      const dirZ = -sy * bodyX + cy * bodyZ;

      if (obs.t > 0.015) {
        const axRaw = (obs.vel.x - prevObsVel.x) / dt;
        const ayRaw = (obs.vel.y - prevObsVel.y) / dt;
        const azRaw = (obs.vel.z - prevObsVel.z) / dt;
        const curCmdX = 0.5 * (prevCmd.x + b0Est * thrustEst * dirX - 0.55 * v.x);
        const curCmdY = 0.5 * (prevCmd.y + b0Est * thrustEst * dirY);
        const curCmdZ = 0.5 * (prevCmd.z + b0Est * thrustEst * dirZ - 0.55 * v.z);

        dxEst = clamp(dxEst + 0.10 * (axRaw - curCmdX - dxEst), -7.5, 7.5);
        dzEst = clamp(dzEst + 0.10 * (azRaw - curCmdZ - dzEst), -7.5, 7.5);

        if (obs.t > 0.05 && curCmdY > 6.0 && Math.abs(ayRaw) < 15.0) {
          const b0Meas = ((ayRaw + g + 0.55 * v.y) / curCmdY) * b0Est;
          b0Est += 0.04 * (clamp(b0Meas, 13.5, 27.5) - b0Est);
        }
      }
      prevObsVel.x = obs.vel.x;
      prevObsVel.y = obs.vel.y;
      prevObsVel.z = obs.vel.z;

      if (obs.gateIndex !== prevGateIndex) {
        prevGate = currentGateCache;
        prevGateIndex = obs.gateIndex;
        phase = obs.gateIndex < 3 ? "align" : "padApproach";
        settle = 0;
      }
      if (obs.nextGate) {
        currentGateCache = obs.nextGate.centre;
      }

      const rLead = obs.att.roll + 0.040 * obs.rates.roll;
      const pLead = obs.att.pitch + 0.040 * obs.rates.pitch;
      const crL = Math.cos(rLead);
      const srL = Math.sin(rLead);
      const cpL = Math.cos(pLead);
      const spL = Math.sin(pLead);
      const dirXL = cy * (spL * crL) + sy * (-srL);
      const dirYL = cpL * crL;
      const dirZL = -sy * (spL * crL) + cy * (-srL);

      const estHoverThrust = g / (b0Est * Math.max(0.5, dirYL));
      const axActual = b0Est * estHoverThrust * dirXL;
      const azActual = b0Est * estHoverThrust * dirZL;

      const axEq = -0.72 * dxEst + 2.5 * ix;
      const azEq = -0.72 * dzEst + 2.5 * iz;
      const azNet = azActual - azEq;

      let target;
      let velTarget = { x: 0, y: 0, z: 0 };

      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (prevGate && pos.x < prevGate.x + 0.15 && phase === "align") {
          target = { x: prevGate.x + 0.35, y: prevGate.y, z: prevGate.z };
          velTarget = { x: 0.75, y: 0, z: 0 };
        } else {
          if (pos.x > gate.x + 0.03) {
            phase = "returnBehind";
            settle = 0;
          }

          if (phase === "returnBehind") {
            const safeY = clamp(gate.y + 0.62, 0.45, 2.55);
            const zClear = Math.abs(pos.z - gate.z) > 0.52 || Math.abs(pos.y - gate.y) > 0.42;
            const targetX = zClear ? gate.x - 0.38 : Math.max(pos.x + 0.10, gate.x + 0.22);
            target = { x: targetX, y: safeY, z: gate.z };
            if (pos.x < gate.x - 0.22 && Math.abs(pos.y - safeY) < 0.25) {
              phase = "align";
              settle = 0;
            }
          } else if (phase === "align") {
            const errY = Math.abs(pos.y - gate.y);
            const errZ = Math.abs(pos.z - gate.z);
            const xStandOff = (errY > 0.16 || errZ > 0.12) ? 0.38 : (errY > 0.08 || errZ > 0.024) ? 0.28 : 0.20;
            target = { x: gate.x - xStandOff, y: gate.y, z: gate.z };

            const predErrY = Math.abs(pos.y + v.y * 0.09 - gate.y);
            const predErrZ = Math.abs(pos.z + v.z * 0.09 + 0.5 * azNet * 0.004 - gate.z);

            const ready =
              pos.x > gate.x - 0.40 &&
              pos.x < gate.x - 0.05 &&
              errY < 0.070 &&
              errZ < 0.024 &&
              predErrY < 0.065 &&
              predErrZ < 0.021 &&
              Math.abs(v.y) < 0.24 &&
              Math.abs(v.z) < 0.14 &&
              Math.abs(azNet) < 0.81;

            settle = ready ? settle + 1 : 0;
            if (settle >= 2) {
              phase = "cross";
              settle = 0;
            }
          } else {
            const dxToGate = gate.x - pos.x;
            const tCross = clamp(dxToGate / Math.max(0.35, v.x), 0.03, 0.18);
            const predY = pos.y + v.y * tCross;
            const predZ = pos.z + v.z * tCross;

            const stopDist = (v.x > 0 ? v.x * 0.06 + (v.x * v.x) / 9.0 : 0) + 0.07;
            if (
              pos.x < gate.x - Math.max(0.12, stopDist) &&
              (Math.abs(gate.y - pos.y) > 0.090 ||
                Math.abs(gate.z - pos.z) > 0.024 ||
                Math.abs(gate.y - predY) > 0.100 ||
                Math.abs(gate.z - predZ) > 0.025)
            ) {
              phase = "align";
              settle = 0;
              target = { x: gate.x - 0.28, y: gate.y, z: gate.z };
            } else {
              target = { x: gate.x + 0.34, y: gate.y, z: gate.z };
              velTarget = { x: 0.75, y: 0, z: 0 };
            }
          }
        }
      } else {
        const pad = obs.pad.centre;
        if (prevGate && pos.x < prevGate.x + 0.14) {
          target = { x: prevGate.x + 0.35, y: prevGate.y, z: prevGate.z };
          velTarget = { x: 0.75, y: 0, z: 0 };
        } else {
          const distXZ = Math.hypot(pos.x - pad.x, pos.z - pad.z);
          const predDistXZ = Math.hypot(pos.x + v.x * 0.20 - pad.x, pos.z + v.z * 0.20 - pad.z);
          const speedXZ = Math.hypot(v.x, v.z);
          if (phase !== "descend") {
            const coneY = clamp(0.09 + 0.45 * distXZ, 0.10, 0.42);
            target = { x: pad.x, y: coneY, z: pad.z };
            if (
              (distXZ < 0.145 && speedXZ < 0.46 && pos.y < 0.35) ||
              (predDistXZ < 0.11 && distXZ < 0.22 && speedXZ < 0.50 && pos.y < 0.35)
            ) {
              phase = "descend";
            }
          }
          if (phase === "descend") {
            const unsafeTouch = Math.max(Math.abs(obs.att.roll), Math.abs(obs.att.pitch)) > 0.25 || speedXZ > 0.55;
            target = { x: pad.x, y: unsafeTouch ? 0.12 : 0.0, z: pad.z };
            if (distXZ > 0.24 && pos.y > 0.13) {
              phase = "padApproach";
            }
          }
        }
      }

      const minEy = obs.gateIndex === 3 && pos.y > 0.35 ? -0.50 : -0.35;
      const ex = clamp(target.x - pos.x, -0.65, 0.65);
      const ey = clamp(target.y - pos.y, minEy, 0.55);
      const ez = clamp(target.z - pos.z, -0.65, 0.65);

      if (Math.abs(ex) < 0.25) {
        ix = clamp(ix + ex * dt, -0.4, 0.4);
      } else {
        ix *= 0.95;
      }
      if (Math.abs(ey) < 0.25) {
        iy = clamp(iy + ey * dt, -0.4, 0.4);
      } else {
        iy *= 0.95;
      }
      if (Math.abs(ez) < 0.25) {
        iz = clamp(iz + ez * dt, -0.4, 0.4);
      } else {
        iz *= 0.95;
      }

      const axCmd = 14.0 * ex + 7.5 * (velTarget.x - v.x) + 0.55 * v.x + axEq - 0.45 * (axActual - axEq);
      const azCmd = 16.5 * ez + 8.2 * (velTarget.z - v.z) + 0.55 * v.z + azEq - 0.45 * (azActual - azEq);

      let ayCmd = g + 10.5 * ey - 6.8 * v.y + 3.0 * iy + 0.55 * v.y;
      ayCmd = clamp(ayCmd, 2.0, 15.5);

      const af = axCmd * cy - azCmd * sy;
      const ar = axCmd * sy + azCmd * cy;

      const maxTilt = obs.gateIndex === 3 && pos.y < 0.16 ? 0.15 : obs.gateIndex === 3 && pos.y < 0.26 ? 0.28 : 0.55;
      let pitch = clamp(Math.atan2(af, ayCmd), -maxTilt, maxTilt);
      let roll = clamp(-Math.atan2(ar, ayCmd), -maxTilt, maxTilt);

      let thrust = clamp(ayCmd / (b0Est * Math.max(0.55, dirY)), 0, 0.78);
      if (obs.gateIndex === 3 && pos.y <= 0.092) {
        const distPad = Math.hypot(pos.x - obs.pad.centre.x, pos.z - obs.pad.centre.z);
        if (distPad < 0.145 && v.y > -0.45 && Math.hypot(v.x, v.z) < 0.45) {
          thrust = 0;
          roll = 0;
          pitch = 0;
        }
      }
      thrustEst = 0.75 * thrustEst + 0.25 * thrust;

      prevCmd.x = b0Est * thrustEst * dirX - 0.55 * v.x;
      prevCmd.y = b0Est * thrustEst * dirY;
      prevCmd.z = b0Est * thrustEst * dirZ - 0.55 * v.z;

      const yawRate = clamp(-3.0 * wrapAngle(obs.att.yaw), -2, 2);

      return { thrust, roll, pitch, yawRate };
    },
  };
}
