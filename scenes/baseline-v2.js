// Hand-written baseline for marble run v2: a catch plate, two railed ramps and a backstop, within the 8-piece and
// 0.40 m^2 material budget. Used to calibrate difficulty (a plain design should pass some trials, not all).
// It is not a model's output and it is not tuned per trial.
export default function build({ THREE, add, task }) {
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
