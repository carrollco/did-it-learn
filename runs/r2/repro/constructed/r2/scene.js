export default function build({ THREE, add, task }) {
  function addBoxBetween(p1, p2, height, thickness, upOffset, color) {
    const dx = p2[0] - p1[0];
    const dy = p2[1] - p1[1];
    const dz = p2[2] - p1[2];
    const L = Math.hypot(dx, dy, dz);
    const h = Math.hypot(dx, dz);
    const ry = -Math.atan2(dz, dx);
    const rz = Math.atan2(dy, h);

    const ux = -(dy / L) * (dx / h);
    const uy = h / L;
    const uz = -(dy / L) * (dz / h);

    const mx = (p1[0] + p2[0]) / 2 + ux * upOffset;
    const my = (p1[1] + p2[1]) / 2 + uy * upOffset;
    const mz = (p1[2] + p2[2]) / 2 + uz * upOffset;

    add.box({
      size: [L, height, thickness],
      position: [mx, my, mz],
      rotation: [0, ry, rz],
      color,
    });
  }

  function addTaperedRamp(p1, p2, floorW, floorT, wallH, wallT, innerW1, innerW2) {
    const dx = p2[0] - p1[0];
    const dz = p2[2] - p1[2];
    const h = Math.hypot(dx, dz);
    const sx = -dz / h;
    const sz = dx / h;

    // Floor
    addBoxBetween(p1, p2, floorT, floorW, -floorT / 2, '#d8b08c');

    // Left and right walls
    const wOff1 = innerW1 / 2 + wallT / 2;
    const wOff2 = innerW2 / 2 + wallT / 2;
    const upOff = wallH / 2 - floorT;
    for (const sign of [-1, 1]) {
      const wp1 = [p1[0] + sx * sign * wOff1, p1[1], p1[2] + sz * sign * wOff1];
      const wp2 = [p2[0] + sx * sign * wOff2, p2[1], p2[2] + sz * sign * wOff2];
      addBoxBetween(wp1, wp2, wallH, wallT, upOff, '#c49a76');
    }
  }

  // Segment 1: behind start to center height 0.36 (tapered from wide catch at start to narrow channel)
  addTaperedRamp([-0.93, 0.76, -0.33], [0.01, 0.358, 0.004], 0.050, 0.012, 0.068, 0.012, 0.132, 0.058);

  // Segment 2: center height 0.36 to cup rim
  addTaperedRamp([-0.01, 0.36, -0.004], [0.735, 0.06, 0.276], 0.024, 0.012, 0.071, 0.012, 0.058, 0.062);

  // Funnel walls behind cup at [0.8, 0, 0.3]
  const yaw = Math.atan2(0.3, 0.8);
  const spread = 0.45;
  add.box({
    size: [0.21, 0.13, 0.012],
    position: [0.83, 0.065, 0.38],
    rotation: [0, -yaw + spread, 0],
    color: '#88a0b8',
  });
  add.box({
    size: [0.21, 0.13, 0.012],
    position: [0.87, 0.065, 0.25],
    rotation: [0, -yaw - spread, 0],
    color: '#88a0b8',
  });
}
