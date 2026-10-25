export default function makeController() {
  let prevVel = null, prevAtt = null, prevThrust = 0.5;
  let distX = 0, distZ = 0, distY = 0;
  let intX = 0, intZ = 0, intY = 0;
  let estMass = 0.78;
  let targetIndex = -1, phase = 'align', settle = 0;
  let vzFilt = 0, vyFilt = 0, vxFilt = 0;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  return {
    step(obs) {
      const p = obs.pos, v = obs.vel, att = obs.att;
      const dt = 0.01;

      vxFilt += 0.45 * (v.x - vxFilt);
      vyFilt += 0.45 * (v.y - vyFilt);
      vzFilt += 0.45 * (v.z - vzFilt);

      if (prevVel !== null && obs.t > 0.08 && p.y > 0.12) {
        const ax_meas = (v.x - prevVel.x) / dt;
        const ay_meas = (v.y - prevVel.y) / dt;
        const az_meas = (v.z - prevVel.z) / dt;

        const bodyX = Math.sin(prevAtt.pitch) * Math.cos(prevAtt.roll);
        const bodyZ = -Math.sin(prevAtt.roll);
        const dirX = Math.cos(prevAtt.yaw) * bodyX + Math.sin(prevAtt.yaw) * bodyZ;
        const dirY = Math.cos(prevAtt.pitch) * Math.cos(prevAtt.roll);
        const dirZ = -Math.sin(prevAtt.yaw) * bodyX + Math.cos(prevAtt.yaw) * bodyZ;

        const specThrust = (prevThrust * 16.0 * 0.9) / estMass;
        const ax_model = specThrust * dirX - (0.42 / estMass) * v.x;
        const ay_model = specThrust * dirY - 9.81 - (0.42 / estMass) * v.y;
        const az_model = specThrust * dirZ - (0.42 / estMass) * v.z;

        distX = clamp(distX + 0.18 * ((ax_meas - ax_model) - distX), -8.5, 8.5);
        distY = clamp(distY + 0.08 * ((ay_meas - ay_model) - distY), -4.5, 4.5);
        distZ = clamp(distZ + 0.18 * ((az_meas - az_model) - distZ), -8.5, 8.5);
      }
      prevVel = { x: v.x, y: v.y, z: v.z };
      prevAtt = { roll: att.roll, pitch: att.pitch, yaw: att.yaw };

      let target;
      let vx_ref = 0, vy_ref = 0;
      if (obs.gateIndex < 3) {
        const gate = obs.nextGate.centre;
        if (targetIndex !== obs.gateIndex) {
          targetIndex = obs.gateIndex;
          phase = 'align';
          settle = 0;
        }
        const dy = Math.abs(p.y - gate.y);
        const dz = Math.abs(p.z - gate.z);
        const vy = Math.abs(vyFilt);
        const vz = Math.abs(vzFilt);

        if (phase === 'align') {
          const approachX = gate.x - 0.35;
          target = { x: approachX, y: gate.y, z: gate.z };
          if (p.x > gate.x - 0.55 && p.x < gate.x - 0.05 && dy < 0.08 && dz < 0.019 &&
              vy < 0.35 && vz < 0.18) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= 4) phase = 'cross';
        }
        if (phase === 'cross') {
          target = { x: gate.x + 0.40, y: gate.y, z: gate.z };
          vx_ref = 0.45;
          if (p.x < gate.x - 0.12 && (dy > 0.11 || dz > 0.030)) {
            phase = 'align';
            settle = 0;
            vx_ref = 0;
            target = { x: gate.x - 0.35, y: gate.y, z: gate.z };
          }
        }
      } else {
        const pad = obs.pad.centre;
        const distHoriz = Math.hypot(pad.x - p.x, pad.z - p.z);
        const speedHoriz = Math.hypot(vxFilt, vzFilt);
        if (targetIndex !== 3) {
          targetIndex = 3;
          phase = 'approach_pad';
        }
        if (phase === 'approach_pad') {
          target = { x: pad.x, y: 0.35, z: pad.z };
          if (distHoriz < 0.12 && speedHoriz < 0.32 && p.y < 0.58) {
            phase = 'descend';
          }
        }
        if (phase === 'descend') {
          if (distHoriz > 0.145 && p.y > 0.085) {
            phase = 'approach_pad';
            target = { x: pad.x, y: 0.28, z: pad.z };
          } else {
            target = { x: pad.x, y: 0.04, z: pad.z };
            if (p.y > 0.12) {
              vy_ref = clamp(-1.8 * (p.y - 0.05), -0.45, 0.0);
            }
          }
        }
      }

      const ex = target.x - p.x;
      const ey = target.y - p.y;
      const ez = target.z - p.z;

      if (p.y > 0.12) {
        if (Math.abs(ex) < 0.30) intX = clamp(intX + ex * dt, -0.4, 0.4);
        if (Math.abs(ez) < 0.20) intZ = clamp(intZ + ez * dt, -0.4, 0.4);
        if (Math.abs(ey) < 0.30) intY = clamp(intY + ey * dt, -0.4, 0.4);
      }

      let kp_x = 6.5, kd_x = 4.5;
      let kp_z = 15.5, kd_z = Math.sqrt(15.5) * 1.85;
      let kp_y = 8.0, kd_y = 4.8;

      if (obs.gateIndex === 3 && p.y < 0.22) {
        kp_x = 5.0; kd_x = 3.8;
        kp_z = 6.5; kd_z = 4.5;
      }

      const ax_des = kp_x * ex + kd_x * (vx_ref - v.x) - distX + (0.42 / estMass) * v.x;
      const az_des = kp_z * ez - kd_z * v.z + 4.0 * intZ - distZ + (0.42 / estMass) * v.z;
      let ay_des = kp_y * ey + kd_y * (vy_ref - v.y) - distY + (0.42 / estMass) * v.y;

      if (phase === 'descend' && p.y < 0.18) {
        ay_des = clamp(ay_des, -3.0, 2.5);
      } else {
        ay_des = clamp(ay_des, -5.0, 5.5);
      }

      const total_ay = Math.max(2.5, 9.81 + ay_des);

      const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
      const a_fwd = cy * ax_des - sy * az_des;
      const a_lat = sy * ax_des + cy * az_des;
      const a_norm = Math.hypot(a_fwd, total_ay, a_lat);

      let maxTilt = 0.52;
      if (obs.t < 0.45 && p.y < 0.25) maxTilt = 0.28;
      if (obs.gateIndex === 3 && p.y < 0.18) maxTilt = 0.24;
      if (obs.gateIndex === 3 && p.y < 0.10) maxTilt = 0.16;

      const sinRoll = clamp(-a_lat / a_norm, -Math.sin(maxTilt), Math.sin(maxTilt));
      const eqRoll = Math.asin(sinRoll);
      const sinPitch = clamp(a_fwd / (a_norm * Math.cos(eqRoll)), -Math.sin(maxTilt), Math.sin(maxTilt));
      const eqPitch = Math.asin(sinPitch);

      const kL = (obs.gateIndex === 3 && p.y < 0.18) ? 0.5 : 1.2;
      let roll = eqRoll + kL * (eqRoll - att.roll) - 0.06 * obs.rates.roll;
      let pitch = eqPitch + kL * (eqPitch - att.pitch) - 0.06 * obs.rates.pitch;
      roll = clamp(roll, -maxTilt, maxTilt);
      pitch = clamp(pitch, -maxTilt, maxTilt);

      const tiltFactor = Math.max(0.6, Math.cos(att.roll) * Math.cos(att.pitch));
      let thrust = clamp((estMass * total_ay) / (16.0 * 0.9 * tiltFactor), 0, 0.78);
      if (phase === 'descend' && p.y <= 0.088 && Math.hypot(p.x - obs.pad.centre.x, p.z - obs.pad.centre.z) < 0.135 && Math.abs(v.y) < 0.35) {
        thrust = 0.08;
        roll = clamp(roll, -0.14, 0.14);
        pitch = clamp(pitch, -0.14, 0.14);
      }

      prevThrust = thrust;
      const yawRate = clamp(-3.0 * att.yaw - 0.5 * obs.rates.yaw, -2, 2);
      return { thrust, roll: clamp(roll, -0.6, 0.6), pitch: clamp(pitch, -0.6, 0.6), yawRate };
    }
  };
}
