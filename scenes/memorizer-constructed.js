// CONSTRUCTED POSITIVE CONTROL, written by tools/make_memorizer.mjs, not by a model.
// The baseline run plus two paddles that recognize each of the 16 validation trials by the drop offset and block that
// trial's known table bumps. On any other trial they stay parked. This is what memorizing the validation set looks like.
function baseline({ THREE, add, task }) {
  const S = task.marble.start, C = task.cup.center;
  const wood = '#c9a46a', rail = '#7a8794';
  const RAIL_H = 0.035;

  function ramp(a, b, width) {
    const dx = b[0] - a[0], dy = b[1] - a[1], dz = b[2] - a[2];
    const L = Math.hypot(dx, dz), len = Math.hypot(L, dy);
    const yaw = -Math.atan2(dz, dx), pitch = Math.atan2(dy, L);
    const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
    add.box({ size: [len, 0.015, width], position: mid, rotation: [0, yaw, pitch], color: wood });
    const nx = -dz / L, nz = dx / L;
    for (const side of [-1, 1]) {
      const off = side * (width / 2 + 0.008);
      add.box({ size: [len, RAIL_H, 0.016], position: [mid[0] + nx * off, mid[1] + RAIL_H / 2, mid[2] + nz * off],
        rotation: [0, yaw, pitch], color: rail });
    }
  }

  // The first ramp starts behind the drop point, so the falling marble lands on it; a back plate stops it rolling back.
  const knee = [0.0, 0.36, 0.0];
  ramp([S[0] - 0.08, 0.68, S[2] - 0.03], knee, 0.1);
  add.box({ size: [0.016, 0.08, 0.1], position: [S[0] - 0.12, 0.72, S[2] - 0.03], color: rail });
  ramp(knee, [C[0] - 0.06, 0.09, C[2] - 0.02], 0.09);
  // Backstop beyond the cup.
  add.box({ size: [0.016, 0.14, 0.16], position: [C[0] + 0.09, 0.07, C[2]], color: rail });
}

const PLANS = [{"off":[-0.00452,0.00912],"bumps":[null,null]},{"off":[0.02654,-0.00298],"bumps":[{"time":2.454,"pose":{"position":[0.81006,0.05058,0.35131],"rotation":[0,0.43412,0]}},null]},{"off":[0.01938,-0.02836],"bumps":[{"time":2.977,"pose":{"position":[0.94971,0.02,0.14909],"rotation":[0,0.36377,0]}},null]},{"off":[-0.02276,-0.02037],"bumps":[{"time":2.705,"pose":{"position":[0.8372,0.07512,0.35289],"rotation":[0,0.42372,0]}},null]},{"off":[-0.01466,-0.02776],"bumps":[null,null]},{"off":[-0.02496,0.01984],"bumps":[null,null]},{"off":[0.01944,-0.01308],"bumps":[null,null]},{"off":[-0.01518,-0.01815],"bumps":[{"time":3.157,"pose":{"position":[0.92055,0.02,0.07056],"rotation":[0,2.50341,0]}},null]},{"off":[0.01634,-0.00981],"bumps":[null,null]},{"off":[0.00511,-0.00942],"bumps":[{"time":2.978,"pose":{"position":[0.92647,0.02438,0.11906],"rotation":[0,1.23334,0]}},null]},{"off":[0.01682,0.01955],"bumps":[{"time":2.251,"pose":{"position":[0.86478,0.10706,0.27588],"rotation":[0,2.8564,0]}},null]},{"off":[0.00724,-0.02258],"bumps":[null,null]},{"off":[-0.02676,0.0119],"bumps":[null,null]},{"off":[0.0215,0.01863],"bumps":[{"time":2.275,"pose":{"position":[0.93218,0.0785,0.35647],"rotation":[0,3.07454,0]}},null]},{"off":[-0.02091,0.00426],"bumps":[{"time":2.894,"pose":{"position":[0.82529,0.02,0.40114],"rotation":[0,0.65305,0]}},null]},{"off":[0.003,-0.00577],"bumps":[{"time":3.252,"pose":{"position":[0.89423,0.03656,0.01003],"rotation":[0,1.63532,0]}},null]}];
export default function build({ THREE, add, task }) {
  baseline({ THREE, add, task });
  const PARK = [[0.75, 0.55, 0.55], [0.55, 0.55, 0.55]];
  for (const k of [0, 1]) {
    let plan = null;
    add.mover({ size: [0.24, 0.12, 0.02], color: k ? '#b04a8a' : '#d05a3c', path: (t, m) => {
      if (t === 0) {
        const off = [m.position[0] - task.marble.start[0], m.position[2] - task.marble.start[2]];
        plan = PLANS.find((p) => Math.abs(p.off[0] - off[0]) < 1e-4 && Math.abs(p.off[1] - off[1]) < 1e-4) || null;
      }
      const step = plan && plan.bumps[k];
      if (step && t > step.time - 2.0 && t < step.time + 3.0) return step.pose;
      return { position: PARK[k], rotation: [0, 0, 0] };
    } });
  }
}
