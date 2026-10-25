export default function makeController() {
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const DT = 0.01;
  const G = 9.81;
  const DRAG = 0.42;

  let initialized = false;
  let prevPos = { x: 0, y: 0, z: 0 };
  let prevVel = { x: 0, y: 0, z: 0 };
  let prevAtt = { roll: 0, pitch: 0, yaw: 0 };
  let prevRates = { roll: 0, pitch: 0, yaw: 0 };
  let lastAction = { thrust: 0.48, roll: 0, pitch: 0, yawRate: 0 };

  // Simulator inner loop integral mirror
  let simInt = { roll: 0, pitch: 0 };
  let effAlpha = { roll: 0, pitch: 0, yaw: 0 };
  let effThrust = 0.48;

  // Observers: torque bias, thrust scale, wind
  let kBias = { roll: 0, pitch: 0, yaw: 0 };
  let fastBias = { roll: 0, pitch: 0 };
  let thrustScale = 20.5;
  let windFast = { x: 0, z: 0 };
  let windMid = { x: 0, z: 0 };
  let windBase = { x: 0, z: 0 };

  // Position integral for precision gate alignment & pad centering
  let posInt = { x: 0, y: 0, z: 0 };

  // Navigation state
  let targetGate = -1;
  let phase = 'align'; // 'align', 'lunge', 'pad_approach', 'pad_descend', 'pad_pin'
  let centeredCount = 0;
  let gustWaitSteps = 0;

  return {
    step(obs) {
      const p = obs.pos;
      const v = obs.vel;
      const att = obs.att;
      const rates = obs.rates;

      if (!initialized) {
        initialized = true;
        prevPos = { ...p };
        prevVel = { ...v };
        prevAtt = { ...att };
        prevRates = { ...rates };
      } else {
        // 1. Update inner attitude loop mirror (2 substeps of 0.005)
        const subDt = 0.005;
        const lagTau = 0.038;
        const aLag = 1 - Math.exp(-subDt / lagTau);
        for (let s = 0; s < 2; s++) {
          const frac = (s + 0.5) / 2;
          const rRoll = prevAtt.roll + frac * (att.roll - prevAtt.roll);
          const rPitch = prevAtt.pitch + frac * (att.pitch - prevAtt.pitch);
          const wRoll = prevRates.roll + frac * (rates.roll - prevRates.roll);
          const wPitch = prevRates.pitch + frac * (rates.pitch - prevRates.pitch);
          const wYaw = prevRates.yaw + frac * (rates.yaw - prevRates.yaw);

          const errRoll = lastAction.roll - rRoll;
          const errPitch = lastAction.pitch - rPitch;
          simInt.roll = clamp(simInt.roll + errRoll * subDt, -0.35, 0.35);
          simInt.pitch = clamp(simInt.pitch + errPitch * subDt, -0.35, 0.35);

          const cmdAlphaRoll = 65 * errRoll - 13 * wRoll + 22 * simInt.roll;
          const cmdAlphaPitch = 65 * errPitch - 13 * wPitch + 22 * simInt.pitch;
          const cmdAlphaYaw = 8 * (lastAction.yawRate - wYaw);

          effAlpha.roll += aLag * (cmdAlphaRoll - effAlpha.roll);
          effAlpha.pitch += aLag * (cmdAlphaPitch - effAlpha.pitch);
          effAlpha.yaw += aLag * (cmdAlphaYaw - effAlpha.yaw);
          effThrust += aLag * (lastAction.thrust - effThrust);
        }

        // 2. Torque bias observer
        if (effThrust > 0.14 && p.y > 0.075) {
          for (const axis of ['roll', 'pitch', 'yaw']) {
            const measAlpha = (rates[axis] - prevRates[axis]) / DT + 0.4 * 0.5 * (rates[axis] + prevRates[axis]);
            const predAlpha = effAlpha[axis] + kBias[axis] * effThrust;
            const err = measAlpha - predAlpha;
            const maxB = axis === 'yaw' ? 3.5 : 28.0;
            const gain = obs.t < 0.8 ? 0.18 : 0.09;
            kBias[axis] = clamp(kBias[axis] + gain * (err / Math.max(0.22, effThrust)), -maxB, maxB);
            if (axis !== 'yaw') {
              fastBias[axis] = clamp(0.75 * fastBias[axis] + 0.25 * err, -8.0, 8.0);
            }
          }
        }

        // 3. Thrust scale & Wind observers
        const midRoll = 0.5 * (prevAtt.roll + att.roll);
        const midPitch = 0.5 * (prevAtt.pitch + att.pitch);
        const midYaw = 0.5 * (prevAtt.yaw + att.yaw);
        const bodyX = Math.sin(midPitch) * Math.cos(midRoll);
        const bodyZ = -Math.sin(midRoll);
        const dirX = Math.cos(midYaw) * bodyX + Math.sin(midYaw) * bodyZ;
        const dirY = Math.cos(midPitch) * Math.cos(midRoll);
        const dirZ = -Math.sin(midYaw) * bodyX + Math.cos(midYaw) * bodyZ;

        const estMass = clamp(15.3 / thrustScale, 0.6, 0.95);
        const dragCoeff = DRAG / estMass;

        if (p.y > 0.085) {
          const axMeas = (v.x - prevVel.x) / DT;
          const ayMeas = (v.y - prevVel.y) / DT;
          const azMeas = (v.z - prevVel.z) / DT;

          const uDy = effThrust * dirY;
          if (uDy > 0.18) {
            const reqThrustAccY = ayMeas + G + dragCoeff * 0.5 * (v.y + prevVel.y);
            const errS = reqThrustAccY / uDy - thrustScale;
            const gainS = obs.t < 1.0 ? 0.14 : 0.07;
            thrustScale = clamp(thrustScale + gainS * errS, 15.0, 27.0);
          }

          const wAccMeasX = axMeas - thrustScale * effThrust * dirX + dragCoeff * 0.5 * (v.x + prevVel.x);
          const wAccMeasZ = azMeas - thrustScale * effThrust * dirZ + dragCoeff * 0.5 * (v.z + prevVel.z);

          windFast.x = clamp(windFast.x + 0.34 * (wAccMeasX - windFast.x), -11.0, 11.0);
          windFast.z = clamp(windFast.z + 0.34 * (wAccMeasZ - windFast.z), -11.0, 11.0);
          windMid.x = clamp(windMid.x + 0.08 * (wAccMeasX - windMid.x), -11.0, 11.0);
          windMid.z = clamp(windMid.z + 0.08 * (wAccMeasZ - windMid.z), -11.0, 11.0);

          // No gusts occur before t = 2.0s in task definition
          if (obs.t < 1.98) {
            const gainBase = obs.t < 0.7 ? 0.15 : 0.06;
            windBase.x = clamp(windBase.x + gainBase * (wAccMeasX - windBase.x), -4.0, 4.0);
            windBase.z = clamp(windBase.z + gainBase * (wAccMeasZ - windBase.z), -4.0, 4.0);
          } else if (Math.hypot(windFast.x - windBase.x, windFast.z - windBase.z) < 1.0) {
            windBase.x = clamp(windBase.x + 0.005 * (wAccMeasX - windBase.x), -4.0, 4.0);
            windBase.z = clamp(windBase.z + 0.005 * (wAccMeasZ - windBase.z), -4.0, 4.0);
          }
        }
      }

      const estMass = clamp(15.3 / thrustScale, 0.6, 0.95);
      const dragCoeff = DRAG / estMass;
      const gustEdge = Math.hypot(windFast.x - windMid.x, windFast.z - windMid.z);
      const gustStrength = Math.hypot(windFast.x - windBase.x, windFast.z - windBase.z);
      const gustActive = obs.t >= 1.98 && (gustStrength > 1.35 || gustEdge > 0.90);

      if (gustActive) gustWaitSteps++;
      else gustWaitSteps = 0;

      let axDes = 0, ayDes = 0, azDes = 0;
      let maxTilt = 0.58;
      let pinPad = false;
      let forceBrakeX = false;

      if (obs.gateIndex < 3) {
        const g = obs.nextGate.centre;
        if (targetGate !== obs.gateIndex) {
          targetGate = obs.gateIndex;
          phase = 'align';
          centeredCount = 0;
          posInt.x = 0;
          posInt.y = 0;
          posInt.z = 0;
        }

        const dy = p.y - g.y;
        const dz = p.z - g.z;
        const distToPlane = g.x - p.x;

        if (Math.abs(dy) < 0.18 && Math.abs(v.y) < 0.35) posInt.y = clamp(posInt.y + (g.y - p.y) * DT, -0.18, 0.18);
        else posInt.y *= 0.96;
        if (Math.abs(dz) < 0.12 && Math.abs(v.z) < 0.30) posInt.z = clamp(posInt.z + (g.z - p.z) * DT, -0.18, 0.18);
        else posInt.z *= 0.96;

        const tCross = clamp(distToPlane / Math.max(0.45, v.x), 0.03, 0.22);
        const predY = dy + v.y * tCross;
        const predZ = dz + v.z * tCross;

        // Funnel alignX: stay 0.28m back while unaligned in y/z or during active gusts; step to 0.18m once rough-aligned
        const roughAligned = Math.abs(dz) < 0.025 && Math.abs(v.z) < 0.12 && Math.abs(dy) < 0.08 && !gustActive;
        const alignDist = roughAligned ? 0.18 : 0.28;
        const alignX = g.x - alignDist;

        const deadlineNear = obs.t > 8.8 + obs.gateIndex * 1.5;
        const canLungeWind = !gustActive || (gustEdge < 0.65 && (gustWaitSteps > 135 || deadlineNear));
        const zTol = deadlineNear ? 0.015 : 0.011;
        const predZTol = deadlineNear ? 0.016 : 0.012;

        const isCentered =
          Math.abs(dz) < zTol &&
          Math.abs(predZ) < predZTol &&
          Math.abs(v.z) < 0.06 &&
          Math.abs(dy) < 0.060 &&
          Math.abs(predY) < 0.070 &&
          Math.abs(v.y) < 0.18 &&
          p.x > g.x - 0.28 &&
          p.x < g.x &&
          canLungeWind;

        if (phase === 'align') {
          if (isCentered) centeredCount++;
          else centeredCount = Math.max(0, centeredCount - 1);
          if (centeredCount >= 3) {
            phase = 'lunge';
          }
        } else if (phase === 'lunge') {
          const unsafeZ = Math.abs(predZ) > 0.021 || Math.abs(dz) > 0.022;
          const unsafeY = Math.abs(predY) > 0.095 || Math.abs(dy) > 0.095;
          const gustTransient = distToPlane > 0.05 && gustEdge > 1.15;
          if (distToPlane > 0.02 && (unsafeZ || unsafeY || gustTransient)) {
            phase = 'align';
            centeredCount = 0;
          }
        }

        if (phase === 'align') {
          const ex = alignX - p.x;
          const ey = g.y - p.y;
          const ez = g.z - p.z;
          // Only integrate x when already near alignX and low speed so it never causes approach overshoot
          if (Math.abs(ex) < 0.10 && Math.abs(v.x) < 0.20) posInt.x = clamp(posInt.x + ex * DT, -0.10, 0.10);
          else posInt.x *= 0.90;

          axDes = clamp(7.2 * ex - 5.2 * v.x + 2.5 * posInt.x, -6.0, 5.2);
          ayDes = clamp(10.5 * ey - 5.4 * v.y + 5.5 * posInt.y, -4.5, 5.5);
          azDes = clamp(14.5 * ez - 6.6 * v.z + 8.5 * posInt.z, -6.5, 6.5);

          // Brake gusts & prevent crossing g.x when not centered
          const notSafeNow = Math.abs(dz) > 0.018 || Math.abs(predZ) > 0.019 || Math.abs(dy) > 0.085 || Math.abs(predY) > 0.090 || gustEdge > 1.1;
          if (notSafeNow && p.x + 0.25 * Math.max(0, v.x) > g.x - 0.06) {
            axDes = clamp(-12.0 * (p.x - (g.x - 0.24)) - 8.5 * Math.max(0, v.x) - 2.5, -9.0, -2.0);
            forceBrakeX = true;
          } else if (gustActive && p.x + 0.22 * Math.max(0, v.x) > g.x - 0.12) {
            axDes = clamp(-9.0 * (p.x - alignX) - 6.5 * Math.max(0, v.x) - 1.5, -7.5, -1.0);
          }
        } else {
          // Lunge rapidly through gate while tightly holding y and z
          const lungeOffset = obs.gateIndex === 2 ? 0.24 : 0.30;
          const lungeBoost = obs.gateIndex === 2 ? 1.8 : 2.4;
          const targetX = g.x + lungeOffset;
          const ex = targetX - p.x;
          const ey = g.y - p.y;
          const ez = g.z - p.z;
          axDes = clamp(10.0 * ex - 3.8 * v.x + lungeBoost, -5.0, 6.0);
          ayDes = clamp(12.0 * ey - 5.6 * v.y + 5.5 * posInt.y, -5.0, 6.0);
          azDes = clamp(16.0 * ez - 7.2 * v.z + 9.0 * posInt.z, -6.8, 6.8);
        }
      } else {
        // Landing on pad
        const pad = obs.pad.centre;
        if (phase !== 'pad_approach' && phase !== 'pad_descend' && phase !== 'pad_pin') {
          phase = 'pad_approach';
          posInt.x = 0;
          posInt.y = 0;
          posInt.z = 0;
        }
        const ex = pad.x - p.x;
        const ez = pad.z - p.z;
        const horizErr = Math.hypot(ex, ez);
        const horizSpeed = Math.hypot(v.x, v.z);
        const totalSpeed = Math.hypot(v.x, v.y, v.z);

        if (horizErr < 0.25) {
          posInt.x = clamp(posInt.x + ex * DT, -0.15, 0.15);
          posInt.z = clamp(posInt.z + ez * DT, -0.15, 0.15);
        } else {
          posInt.x *= 0.95;
          posInt.z *= 0.95;
        }

        if (phase === 'pad_approach') {
          const wallDanger = p.x + 0.35 * Math.max(0, v.x) > 2.30 || p.z + 0.35 * Math.max(0, v.z) > 2.30;
          // Keep approach altitude >= 0.20m so fast dive from Gate 2 never overshoots into the floor
          const targetY = clamp(0.16 + 0.55 * horizErr, 0.20, 0.85);
          const ey = targetY - p.y;
          axDes = clamp(8.5 * ex - 5.4 * v.x + 4.0 * posInt.x, wallDanger ? -12.0 : -7.5, 5.5);
          azDes = clamp(8.5 * ez - 5.4 * v.z + 4.0 * posInt.z, wallDanger ? -12.0 : -7.5, 5.5);
          ayDes = clamp(8.0 * ey - 5.2 * v.y, wallDanger ? -1.5 : -4.0, 5.5);
          if (horizErr < 0.12 && horizSpeed < 0.35 && p.y < 0.28 && Math.abs(v.y) < 0.40) {
            phase = 'pad_descend';
          }
        }
        if (phase === 'pad_descend') {
          const targetVy = p.y > 0.13 ? -0.40 : -0.24;
          axDes = clamp(9.0 * ex - 5.2 * v.x + 5.0 * posInt.x, -4.0, 4.0);
          azDes = clamp(9.0 * ez - 5.2 * v.z + 5.0 * posInt.z, -4.0, 4.0);
          ayDes = clamp(9.0 * (targetVy - v.y) + 4.0 * (0.04 - p.y), -3.5, 3.5);
          maxTilt = 0.26;
          if (p.y <= 0.094 && horizErr <= 0.142 && totalSpeed < 0.68) {
            phase = 'pad_pin';
          } else if (horizErr > 0.143) {
            phase = 'pad_approach';
          }
        }
        if (phase === 'pad_pin') {
          if (horizErr > 0.146) {
            phase = 'pad_approach';
          } else {
            pinPad = true;
          }
        }

        // Strict floor tilt protection regardless of phase so floor contact never exceeds 0.36 rad
        if (p.y < 0.20) {
          maxTilt = Math.min(maxTilt, p.y < 0.12 ? 0.18 : 0.26);
        }
      }

      // Brake gusts: fast wind + lead compensation on gust edges
      const windEstX = windFast.x + 0.45 * (windFast.x - windMid.x);
      const windEstZ = windFast.z + 0.45 * (windFast.z - windMid.z);

      let thrust, desRoll, desPitch;
      if (pinPad) {
        // Pin pad with low thrust and keep attitude flat (< 0.18 rad)
        thrust = 0.06;
        const pad = obs.pad.centre;
        const axPad = clamp(3.5 * (pad.x - p.x) - 2.0 * v.x, -0.8, 0.8);
        const azPad = clamp(3.5 * (pad.z - p.z) - 2.0 * v.z, -0.8, 0.8);
        const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
        desPitch = clamp((cy * axPad - sy * azPad) / G, -0.08, 0.08);
        desRoll = clamp(-(sy * axPad + cy * azPad) / G, -0.08, 0.08);
      } else {
        let aThrustX = axDes - windEstX + dragCoeff * v.x;
        if (forceBrakeX) {
          aThrustX = Math.min(aThrustX, -4.0 - 5.5 * Math.max(0, v.x));
        }
        const aThrustZ = azDes - windEstZ + dragCoeff * v.z;
        const aThrustY = clamp(G + ayDes + dragCoeff * v.y, 2.0, 18.0);

        const ux = aThrustX / aThrustY;
        const uz = aThrustZ / aThrustY;
        const cy = Math.cos(att.yaw), sy = Math.sin(att.yaw);
        const tanPitch = cy * ux - sy * uz;
        desPitch = clamp(Math.atan(tanPitch), -maxTilt, maxTilt);
        const tanRoll = -(sy * ux + cy * uz) * Math.cos(att.pitch);
        desRoll = clamp(Math.atan(tanRoll), -maxTilt, maxTilt);

        const dyActual = Math.max(0.5, Math.cos(att.roll) * Math.cos(att.pitch));
        thrust = clamp(aThrustY / (thrustScale * dyActual), 0.05, 1.0);
      }

      // Invert inner attitude loop with high-bandwidth outer PD + torque bias cancellation
      const kpAtt = 135, kdAtt = 18.5;
      const desAlphaRoll = kpAtt * (desRoll - att.roll) - kdAtt * rates.roll;
      const desAlphaPitch = kpAtt * (desPitch - att.pitch) - kdAtt * rates.pitch;
      const biasRoll = kBias.roll * thrust + 0.5 * fastBias.roll;
      const biasPitch = kBias.pitch * thrust + 0.5 * fastBias.pitch;

      const rollCmd = clamp(
        att.roll + (desAlphaRoll - biasRoll - 22 * simInt.roll + 13 * rates.roll) / 65,
        -0.6,
        0.6
      );
      const pitchCmd = clamp(
        att.pitch + (desAlphaPitch - biasPitch - 22 * simInt.pitch + 13 * rates.pitch) / 65,
        -0.6,
        0.6
      );

      const desAlphaYaw = -30 * att.yaw - 9 * rates.yaw;
      const biasYaw = kBias.yaw * thrust;
      const yawRateCmd = clamp(
        rates.yaw + (desAlphaYaw - biasYaw + 0.4 * rates.yaw) / 8,
        -2,
        2
      );

      prevPos = { ...p };
      prevVel = { ...v };
      prevAtt = { ...att };
      prevRates = { ...rates };
      lastAction = { thrust, roll: rollCmd, pitch: pitchCmd, yawRate: yawRateCmd };

      return lastAction;
    },
  };
}
