export default function makeController() {
  const dt = 0.01;
  const g = 9.81;
  const mNom = 0.725;
  const accGain = 15.5 / mNom;
  const dragGain = 0.42 / mNom;
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

  let initialized = false;
  let velX = 0, velY = 0, velZ = 0;
  let accX = 0, accY = 0, accZ = 0;
  let fastDx = 0, fastDy = 0, fastDz = 0;
  let slowDx = 0, slowDy = 0, slowDz = 0;
  let intX = 0, intY = 0, intZ = 0;
  let yawInt = 0;
  let motorThrust = 0.45;
  let lastThrust = 0.45;

  let refX = 0, refY = 0, refZ = 0;
  let targetIndex = -1;
  let phase = 'align';
  let settle = 0;
  let gateX = 0, gateY = 0, gateZ = 0;
  let lockSteps = 0;
  let padY = 0.85;
  const actionOut = { thrust: 0.45, roll: 0, pitch: 0, yawRate: 0 };

  return {
    step(obs) {
      const p = obs.pos;
      const a = obs.att;
      const r = obs.rates;

      if (!initialized) {
        initialized = true;
        velX = obs.vel.x;
        velY = obs.vel.y;
        velZ = obs.vel.z;
        refX = p.x;
        refY = p.y;
        refZ = p.z;
        if (obs.nextGate) {
          gateX = obs.nextGate.centre.x;
          gateY = obs.nextGate.centre.y;
          gateZ = obs.nextGate.centre.z;
          targetIndex = obs.gateIndex;
        }
      }

      // 1. Filter velocity
      const prevVx = velX, prevVy = velY, prevVz = velZ;
      const alphaV = 0.35;
      velX += alphaV * (obs.vel.x - velX);
      velY += alphaV * (obs.vel.y - velY);
      velZ += alphaV * (obs.vel.z - velZ);

      const measAx = (velX - prevVx) / dt;
      const measAy = (velY - prevVy) / dt;
      const measAz = (velZ - prevVz) / dt;

      accX += 0.25 * (measAx - accX);
      accY += 0.25 * (measAy - accY);
      accZ += 0.25 * (measAz - accZ);

      // 2. Fast/slow accel disturbance observer
      motorThrust += (1 - Math.exp(-dt / 0.04)) * (lastThrust - motorThrust);
      const bodyX = Math.sin(a.pitch) * Math.cos(a.roll);
      const bodyZ = -Math.sin(a.roll);
      const cy = Math.cos(a.yaw);
      const sy = Math.sin(a.yaw);
      const dirX = cy * bodyX + sy * bodyZ;
      const dirY = Math.cos(a.pitch) * Math.cos(a.roll);
      const dirZ = -sy * bodyX + cy * bodyZ;

      const expAx = accGain * motorThrust * dirX - dragGain * velX;
      const expAy = accGain * motorThrust * dirY - dragGain * velY - g;
      const expAz = accGain * motorThrust * dirZ - dragGain * velZ;

      const airborne = p.y > 0.095 && p.y < 2.88 && Math.abs(p.x) < 2.78 && Math.abs(p.z) < 2.78;
      if (airborne) {
        const rawDx = clamp(measAx - expAx, -8, 8);
        const rawDy = clamp(measAy - expAy, -6, 6);
        const rawDz = clamp(measAz - expAz, -8, 8);

        fastDx += 0.20 * (rawDx - fastDx);
        fastDy += 0.20 * (rawDy - fastDy);
        fastDz += 0.20 * (rawDz - fastDz);

        slowDx += 0.025 * (rawDx - slowDx);
        slowDy += 0.025 * (rawDy - slowDy);
        slowDz += 0.025 * (rawDz - slowDz);
      } else {
        fastDx += 0.08 * (slowDx - fastDx);
        fastDy += 0.08 * (slowDy - fastDy);
        fastDz += 0.08 * (slowDz - fastDz);
      }

      // 5. Gate exit lock & navigation state machine
      if (targetIndex < 3 && obs.gateIndex !== targetIndex && phase !== 'exit') {
        phase = 'exit';
        lockSteps = 0;
        intX = 0;
        refX = Math.max(refX, p.x + 0.18);
      }

      if (phase === 'exit') {
        lockSteps++;
        if ((p.x >= gateX + 0.22 && velX >= -0.05) || (lockSteps > 32 && p.x >= gateX + 0.10)) {
          targetIndex = obs.gateIndex;
          intX = 0;
          intY = 0;
          intZ = 0;
          settle = 0;
          if (obs.gateIndex < 3) {
            gateX = obs.nextGate.centre.x;
            gateY = obs.nextGate.centre.y;
            gateZ = obs.nextGate.centre.z;
            phase = 'align';
          } else {
            phase = 'pad_align';
            padY = 0.65;
          }
        }
      }

      const alignDist = 0.44;
      let targetX = 0, targetY = 0, targetZ = 0;
      let committedCross = false;

      if (phase === 'exit') {
        targetX = gateX + 0.35;
        targetY = gateY;
        targetZ = gateZ;
        if (p.x < gateX + 0.16 && velX < 0.35) {
          refX = Math.max(refX, gateX + 0.42);
        }
      } else if (targetIndex < 3) {
        if (phase === 'align') {
          targetX = targetIndex === 0 && p.y < 0.65 ? Math.min(gateX - 0.62, refX + 0.05) : gateX - alignDist;
          targetY = gateY;
          targetZ = gateZ;

          if (p.x > gateX - 0.075 && velX > 0.22 && Math.abs(p.y - gateY) < 0.18 && Math.abs(p.z - gateZ) < 0.115) {
            phase = 'cross';
            committedCross = true;
            refX = gateX + 0.35;
          }

          const ex = Math.abs(gateX - alignDist - p.x);
          const ey = Math.abs(gateY - p.y);
          const ez = Math.abs(gateZ - p.z);
          const spd = Math.hypot(velX, velY, velZ);
          const predY = p.y + velY * 0.20 + 0.5 * accY * 0.14 * 0.14;
          const predZ = p.z + velZ * 0.20 + 0.5 * accZ * 0.14 * 0.14;
          const gustOk = Math.abs(fastDz - slowDz) < 1.25 && Math.abs(fastDy - slowDy) < 1.25;
          if (
            ex < 0.105 &&
            ey < 0.080 &&
            ez < 0.055 &&
            spd < 0.28 &&
            Math.abs(velZ) < 0.14 &&
            Math.abs(predY - gateY) < 0.105 &&
            Math.abs(predZ - gateZ) < 0.070 &&
            gustOk
          ) {
            settle++;
          } else {
            settle = Math.max(0, settle - 1);
          }
          if (settle >= 10) {
            phase = 'cross';
            intX = 0;
          }
        }

        if (phase === 'cross') {
          targetX = gateX + 0.32;
          targetY = gateY;
          targetZ = gateZ;
          // 6. Predictive cross abort with stopping-distance check
          const dx = gateX - p.x;
          if (dx > 0.01) {
            const vxPos = Math.max(0, velX);
            const dStop = vxPos * 0.065 + (vxPos * vxPos) / 7.5 + 0.035;

            const vxEff = Math.max(velX, 0.30);
            const tau = clamp(dx / vxEff, 0.04, 0.34);
            const tauA = Math.min(tau, 0.17);
            const ty = velY * accY < 0 ? Math.min(tau, -velY / accY) : tau;
            const tz = velZ * accZ < 0 ? Math.min(tau, -velZ / accZ) : tau;
            const tya = velY * accY < 0 ? ty : tauA;
            const tza = velZ * accZ < 0 ? tz : tauA;
            const predY = p.y + velY * ty + 0.5 * accY * tya * tya;
            const predZ = p.z + velZ * tz + 0.5 * accZ * tza * tza;
            const curY = Math.abs(p.y - gateY);
            const curZ = Math.abs(p.z - gateZ);
            const unsafe =
              curY > 0.150 ||
              curZ > 0.080 ||
              Math.abs(predY - gateY) > 0.160 ||
              Math.abs(predZ - gateZ) > 0.085;

            if (unsafe && dx > dStop) {
              phase = 'align';
              settle = 0;
              intX = 0;
              targetX = gateX - alignDist;
              refX = Math.min(refX, targetX - 0.05);
            } else if (dx <= dStop) {
              committedCross = true;
              refX = Math.max(refX, p.x + 0.25);
            }
          }
        }
      } else {
        // 7. Ramp pad descent
        const pad = obs.pad.centre;
        const hErr = Math.hypot(pad.x - p.x, pad.z - p.z);
        const hSpd = Math.hypot(velX, velZ);
        if (phase === 'pad_align') {
          if (hErr < 0.135 && hSpd < 0.26 && Math.abs(p.y - padY) < 0.18) {
            phase = 'descend';
          }
        }
        if (phase === 'descend') {
          if (hErr < 0.115 && hSpd < 0.32) {
            const rate = p.y > 0.26 ? 0.46 : 0.22;
            padY = Math.max(0.02, padY - rate * dt);
          } else if (hErr > 0.128) {
            padY = clamp(Math.max(padY, p.y) + 0.35 * dt, 0.24, 0.50);
            intY = Math.max(0, intY);
          }
        }
        targetX = pad.x;
        targetY = padY;
        targetZ = pad.z;
      }

      // 4. Smooth ref filter
      const rdx = targetX - refX;
      const rdy = targetY - refY;
      const rdz = targetZ - refZ;
      const rDist = Math.hypot(rdx, rdy, rdz);
      const maxSpeed = phase === 'cross' ? 1.15 : 2.05;
      const alphaRef = phase === 'cross' ? 0.12 : 0.10;
      const stepScale = rDist > 1e-6 ? Math.min(alphaRef, (maxSpeed * dt) / rDist) : alphaRef;
      const refVx = (rdx * stepScale) / dt;
      const refVy = (rdy * stepScale) / dt;
      const refVz = (rdz * stepScale) / dt;
      refX += rdx * stepScale;
      refY += rdy * stepScale;
      refZ += rdz * stepScale;

      // 3. Yaw + local pos integrals
      if (airborne) {
        yawInt = clamp(yawInt + a.yaw * dt, -0.6, 0.6);
        if (Math.abs(targetX - p.x) < 0.35 && phase !== 'cross') {
          intX = clamp(intX + (targetX - p.x) * dt, -0.45, 0.45);
        } else {
          intX *= 0.96;
        }
        if (Math.abs(targetY - p.y) < 0.35) {
          intY = clamp(intY + (targetY - p.y) * dt, -0.45, 0.45);
        } else {
          intY *= 0.96;
        }
        if (Math.abs(targetZ - p.z) < 0.35) {
          intZ = clamp(intZ + (targetZ - p.z) * dt, -0.45, 0.45);
        } else {
          intZ *= 0.96;
        }
      } else if (p.y <= 0.095) {
        intY = Math.max(0, intY);
      }

      const distX = slowDx + 0.85 * (fastDx - slowDx);
      const distY = slowDy + 0.55 * (fastDy - slowDy);
      const distZ = slowDz + 0.85 * (fastDz - slowDz);

      const ex = refX - p.x;
      const ey = refY - p.y;
      const ez = refZ - p.z;

      const kpx = 5.2, kdx = 3.8, kix = 1.8;
      const kpz = committedCross ? 7.2 : 5.8;
      const kdz = committedCross ? 4.6 : 4.1;
      const kiz = 2.0;
      const kpy = committedCross ? 7.0 : 6.0;
      const kdy = 4.0, kiy = 2.2;

      const axCmd = clamp(
        kpx * ex + kdx * (0.35 * refVx - velX) + kix * intX + dragGain * velX - distX,
        -4.5,
        4.5
      );
      const azCmd = clamp(
        kpz * ez + kdz * (0.35 * refVz - velZ) + kiz * intZ + dragGain * velZ - distZ,
        -4.5,
        4.5
      );
      const ayCmd = clamp(
        kpy * ey + kdy * (0.35 * refVy - velY) + kiy * intY + dragGain * velY - distY,
        -4.2,
        4.6
      );

      const ayTotal = Math.max(2.0, g + ayCmd);
      const bodyAx = cy * axCmd - sy * azCmd;
      const bodyAz = sy * axCmd + cy * azCmd;

      let maxTilt = 0.48;
      if (targetIndex === 3 && p.y < 0.18) {
        const hErr = Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z);
        maxTilt = p.y < 0.105 && hErr < 0.135 ? 0.13 : 0.165;
      }

      const pitch = clamp(Math.atan2(bodyAx, ayTotal), -maxTilt, maxTilt);
      const roll = clamp(Math.atan2(-bodyAz, Math.hypot(ayTotal, bodyAx)), -maxTilt, maxTilt);

      const tiltCos = Math.max(0.65, Math.cos(a.roll) * Math.cos(a.pitch));
      let thrust = clamp(ayTotal / (accGain * tiltCos), 0, 1);

      if (
        targetIndex === 3 &&
        phase === 'descend' &&
        p.y <= 0.088 &&
        Math.hypot(obs.pad.centre.x - p.x, obs.pad.centre.z - p.z) < 0.125 &&
        Math.hypot(velX, velZ) < 0.32
      ) {
        thrust = Math.min(thrust, 0.18);
      }

      lastThrust = thrust;
      const yawRate = clamp(-3.2 * a.yaw - 2.0 * yawInt - 0.25 * r.yaw, -2, 2);

      actionOut.thrust = thrust;
      actionOut.roll = roll;
      actionOut.pitch = pitch;
      actionOut.yawRate = yawRate;
      return actionOut;
    },
  };
}
