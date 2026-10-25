// Hand-written test scene: a funnel under the drop point, two railed ramps, a backstop over the cup.
// It exists to exercise the harness; it is not a model's output and it is not tuned.
export default function build({ THREE, add, task }) {
  const S = task.marble.start, C = task.cup.center;
  const wood = '#c9a46a', rail = '#7a8794';

  // A railed ramp from point a to point b (centre line of the floor, in metres).
  function ramp(a, b, width, railHeight) {
    const dx = b[0] - a[0], dy = b[1] - a[1], dz = b[2] - a[2];
    const L = Math.hypot(dx, dz), len = Math.hypot(L, dy);
    const yaw = -Math.atan2(dz, dx), pitch = Math.atan2(dy, L);
    const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
    add.box({ size: [len, 0.015, width], position: mid, rotation: [0, yaw, pitch], color: wood });
    const nx = -dz / L, nz = dx / L;
    for (const side of [-1, 1]) {
      const off = side * (width / 2 + 0.008);
      add.box({ size: [len, railHeight, 0.016], position: [mid[0] + nx * off, mid[1] + railHeight / 2, mid[2] + nz * off],
        rotation: [0, yaw, pitch], color: rail });
    }
  }

  // Catch tray under the drop point.
  add.box({ size: [0.2, 0.015, 0.2], position: [S[0], 0.62, S[2]], rotation: [0, 0, -0.25], color: wood });
  for (const [x, z, sx, sz] of [[S[0] - 0.1, S[2], 0.016, 0.2], [S[0], S[2] - 0.1, 0.2, 0.016], [S[0], S[2] + 0.1, 0.2, 0.016]]) {
    add.box({ size: [sx, 0.08, sz], position: [x, 0.66, z], color: rail });
  }
  const knee = [0.0, 0.32, 0.0];
  ramp([S[0] + 0.12, 0.58, S[2]], knee, 0.09, 0.05);
  ramp(knee, [C[0] - 0.05, 0.1, C[2] - 0.02], 0.09, 0.05);
  // Backstop beyond the cup so the marble drops in instead of flying over.
  add.box({ size: [0.016, 0.2, 0.18], position: [C[0] + 0.09, 0.1, C[2]], color: rail });

  // Decor: a coffee mug and a notebook (visual only).
  const mug = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.04, 0.11, 32), new THREE.MeshStandardMaterial({ color: '#e8e2d6' }));
  mug.position.set(-0.4, 0.055, 0.32); add.decor(mug);
  const book = new THREE.Mesh(new THREE.BoxGeometry(0.3, 0.02, 0.21), new THREE.MeshStandardMaterial({ color: '#3a5a8c' }));
  book.position.set(0.3, 0.01, -0.3); book.rotation.y = 0.3; add.decor(book);
}
