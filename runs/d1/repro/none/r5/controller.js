export default function makeController() {
  let targetIndex = -1, phase = 'climb', integralX = 0, integralY = 0, integralZ = 0, settle = 0;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const PREV_GATES = [
    null,
    { x: -1.15, y: 1.05, z: -0.85 },
    { x: 0.15, y: 1.40, z: 0.25 },
    { x: 1.35, y: 1.05, z: 1.05 },
  ];

  // Disturbance observer state
  let prevObs = null;
  let lastAction = { thrust: 0.45, roll: 0, pitch: 0, yawRate: 0 };
  let uLag = 0.45;
  let kT = 22.0;
  let massHat = 0.725;
  let vxHat = 0, vyHat = 0, vzHat = 0;
  let dxHat = 0, dyHat = 0, dzHat = 0;
  let lastVzErr = 0;
  let startPos = { x: -2.1, y: 0.07, z: -1.65 };

  function getDir(att) {
    const cr = Math.cos(att.roll), sr = Math.sin(att.roll);
    const cp = Math.cos(att.pitch), sp = Math.sin(att.pitch);
    const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
    const bodyX = sp * cr, bodyZ = -sr;
    return {
      x: cy * bodyX + sy * bodyZ,
      y: cp * cr,
      z: -sy * bodyX + cy * bodyZ,
    };
  }

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att;
      const dt = 0.01;

      if (!prevObs) {
        vxHat = v.x; vyHat = v.y; vzHat = v.z;
        startPos = { x: p.x, y: p.y, z: p.z };
      } else {
        uLag += (lastAction.thrust - uLag) * (1 - Math.exp(-dt / 0.04));
        const dirPrev = getDir(prevObs.att);
        const dirCurr = getDir(att);
        const dirAvg = {
          x: 0.5 * (dirPrev.x + dirCurr.x),
          y: 0.5 * (dirPrev.y + dirCurr.y),
          z: 0.5 * (dirPrev.z + dirCurr.z),
        };
        const drag = 0.42 / massHat;
        if (p.y > 0.09 && prevObs.pos.y > 0.09) {
          const ayModel = kT * uLag * dirAvg.y - 9.81 - drag * 0.5 * (prevObs.vel.y + v.y);
          vyHat += (ayModel + dyHat) * dt;
          const vyErr = v.y - vyHat;
          vyHat += 0.4 * vyErr;
          dyHat = clamp(dyHat + 6.0 * vyErr, -5.0, 5.0);

          const fSpec = Math.max(0, kT * uLag + dyHat / Math.max(0.6, dirAvg.y));
          const axModel = fSpec * dirAvg.x - drag * 0.5 * (prevObs.vel.x + v.x);
          const azModel = fSpec * dirAvg.z - drag * 0.5 * (prevObs.vel.z + v.z);

          vxHat += (axModel + dxHat) * dt;
          const vxErr = v.x - vxHat;
          vxHat += 0.45 * vxErr;
          dxHat = clamp(dxHat + 9.5 * vxErr, -6.5, 6.5);

          vzHat += (azModel + dzHat) * dt;
          const vzErr = v.z - vzHat;
          lastVzErr = vzErr;
          vzHat += 0.45 * vzErr;
          dzHat = clamp(dzHat + 9.5 * vzErr, -6.5, 6.5);
        } else {
          vxHat = v.x; vyHat = v.y; vzHat = v.z;
          lastVzErr = 0;
        }
      }
      prevObs = {
        pos: { x: p.x, y: p.y, z: p.z },
        vel: { x: v.x, y: v.y, z: v.z },
        att: { roll: att.roll, pitch: att.pitch, yaw: att.yaw },
      };

      let target;
      let brakingAbort = false;
      const prevGate = PREV_GATES[obs.gateIndex];

      // Ensure we are safely past the previous gate plane before slewing Y/Z toward the next waypoint
      if (prevGate && p.x < prevGate.x + 0.16) {
        const ezPrev = Math.abs(p.z - prevGate.z);
        const eyPrev = Math.abs(p.y - prevGate.y);
        const zPredPrev = Math.abs((p.z + 0.12 * vzHat) - prevGate.z);
        if (p.x < prevGate.x && (ezPrev > 0.058 || zPredPrev > 0.068 || eyPrev > 0.12)) {
          // Blown behind previous gate and off-center: hold in front of it until re-centered
          target = { x: prevGate.x - 0.32, y: prevGate.y, z: prevGate.z };
          brakingAbort = true;
        } else {
          target = { x: prevGate.x + 0.45, y: prevGate.y, z: prevGate.z };
        }
        settle = 0;
      } else if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          targetIndex = obs.gateIndex;
          phase = 'align';
          settle = 0;
        }

        const alignX = gate.x - 0.42;
        const ez = Math.abs(p.z - gate.z);
        const ey = Math.abs(p.y - gate.y);
        const ezPred = Math.abs((p.z + 0.14 * vzHat) - gate.z);

        if (phase === 'cross') {
          const distToGateX = gate.x - p.x;
          const timeToGate = distToGateX / Math.max(0.25, vxHat);
          const zAtGate = p.z + clamp(timeToGate, 0.05, 0.35) * vzHat;
          const yAtGate = p.y + clamp(timeToGate, 0.05, 0.35) * vyHat;

          if (
            distToGateX > 0.05 &&
            (ez > 0.060 || Math.abs(zAtGate - gate.z) > 0.070 || ey > 0.12 || Math.abs(yAtGate - gate.y) > 0.14)
          ) {
            phase = 'align';
            settle = 0;
          } else {
            target = { x: gate.x + 0.45, y: gate.y, z: gate.z };
          }
        }

        if (phase === 'align') {
          if (obs.gateIndex === 0 && (ey > 0.32 || ez > 0.32)) {
            target = { x: Math.min(startPos.x + 0.25, alignX - 0.20), y: gate.y, z: gate.z };
          } else {
            target = { x: alignX, y: gate.y, z: gate.z };
          }

          if (p.x > alignX + 0.05 || (p.x + 0.22 * v.x > alignX + 0.08 && (ez > 0.05 || ey > 0.09))) {
            brakingAbort = true;
          }

          if (
            Math.hypot(alignX - p.x, gate.y - p.y, gate.z - p.z) < 0.10 &&
            ez < 0.042 &&
            ezPred < 0.052 &&
            Math.abs(v.z) < 0.13 &&
            Math.abs(v.y) < 0.20 &&
            Math.hypot(v.x, v.y, v.z) < 0.30 &&
            Math.abs(lastVzErr) < 0.04
          ) {
            settle++;
          } else {
            settle = 0;
          }
          if (settle > 10) phase = 'cross';
        }
      } else {
        if (targetIndex !== 3) {
          targetIndex = 3;
          phase = 'pad_align';
        }
        target = { x: obs.pad.centre.x, y: 0.85, z: obs.pad.centre.z };
        const distH = Math.hypot(target.x - p.x, target.z - p.z);
        if (phase !== 'descend' && distH < 0.09 && Math.hypot(v.x, v.z) < 0.20) phase = 'descend';
        if (phase === 'descend') {
          if (distH > 0.135) phase = 'pad_align';
          else target.y = 0.025;
        }
      }

      const ex = target.x - p.x;
      const ey = target.y - p.y;
      const ez = target.z - p.z;

      if (Math.hypot(ex, ez) < 0.28 && p.y > 0.10) {
        integralX = clamp(integralX + ex * dt, -0.35, 0.35);
        integralZ = clamp(integralZ + ez * dt, -0.35, 0.35);
      }

      const drag = 0.42 / massHat;
      const kpX = brakingAbort ? 7.0 : 3.9;
      const kdX = brakingAbort ? 4.6 : 3.2;
      const minAx = brakingAbort ? -4.8 : -2.8;

      const nearGate = (obs.gateIndex < 3 && p.x > obs.nextGate.centre.x - 0.55) || (prevGate && p.x < prevGate.x + 0.16);
      const kpZ = nearGate ? 5.8 : 4.0;
      const kdZ = nearGate ? 4.1 : 3.3;

      const ax = clamp(kpX * ex - kdX * v.x + 1.8 * integralX, minAx, 2.8) + drag * v.x - dxHat;
      const az = clamp(kpZ * ez - kdZ * v.z + 1.8 * integralZ, -3.6, 3.6) + drag * v.z - dzHat;

      integralY = clamp(integralY + ey * dt, -0.5, 1.9);
      const ay = clamp(5.5 * ey - 3.5 * v.y + 2.4 * integralY, -3.5, 4.5);
      const cy = Math.cos(obs.att.yaw), sy = Math.sin(obs.att.yaw);
      const maxTilt = (obs.gateIndex === 3 && phase === 'descend' && p.y < 0.12) ? 0.15 : 0.46;
      const pitch = clamp((cy * ax - sy * az) / 9.81, -maxTilt, maxTilt);
      const roll = clamp(-(sy * ax + cy * az) / 9.81, -maxTilt, maxTilt);
      const thrust = clamp((0.72 * (9.81 + ay)) / (16 * Math.max(0.7, Math.cos(obs.att.roll) * Math.cos(obs.att.pitch))), 0, 1);
      lastAction = { thrust, roll, pitch, yawRate: -2 * obs.att.yaw };
      return lastAction;
    },
  };
}
