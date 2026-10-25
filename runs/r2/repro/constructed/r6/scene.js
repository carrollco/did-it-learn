export default function build({ THREE, add, task }) {
  function makeSegment(p0, p1, width, floorT, wallH, wallT, overlap, floorColor, wallColor) {
    const v0 = new THREE.Vector3(...p0);
    const v1 = new THREE.Vector3(...p1);
    const diff = new THREE.Vector3().subVectors(v1, v0);
    const len = Math.min(1.1, diff.length() + overlap);
    const dir = diff.clone().normalize();
    const up = new THREE.Vector3(0, 1, 0);
    const localZ = new THREE.Vector3().crossVectors(dir, up).normalize();
    const localY = new THREE.Vector3().crossVectors(localZ, dir).normalize();
    const m = new THREE.Matrix4().makeBasis(dir, localY, localZ);
    const e = new THREE.Euler().setFromRotationMatrix(m, 'XYZ');
    const rot = [e.x, e.y, e.z];
    const mid = new THREE.Vector3().addVectors(v0, v1).multiplyScalar(0.5);

    const fPos = mid.clone().addScaledVector(localY, -floorT / 2);
    add.box({
      size: [len, floorT, width],
      position: [fPos.x, fPos.y, fPos.z],
      rotation: rot,
      color: floorColor,
    });

    for (const s of [-1, 1]) {
      const wPos = mid
        .clone()
        .addScaledVector(localY, wallH / 2 - floorT)
        .addScaledVector(localZ, s * (width / 2 + wallT / 2));
      add.box({
        size: [len, wallH, wallT],
        position: [wPos.x, wPos.y, wPos.z],
        rotation: rot,
        color: wallColor,
      });
    }
  }

  const P0 = [-0.94, 0.75, -0.335];
  const P1 = [0.0, 0.36, 0.0];
  const P2 = [0.74, 0.06, 0.278];

  // Upper ramp with tall walls from behind start to center height 0.36
  makeSegment(P0, P1, 0.094, 0.012, 0.066, 0.012, 0.012, '#d8c3a5', '#8e735b');

  // Lower ramp from center height 0.36 to cup rim
  makeSegment(P1, P2, 0.052, 0.012, 0.048, 0.012, 0.012, '#d8c3a5', '#8e735b');

  // Funnel walls behind cup
  const angle = Math.atan2(0.3, 0.8);
  add.box({
    size: [0.09, 0.15, 0.012],
    position: [0.855, 0.075, 0.35],
    rotation: [0, -angle + 0.55, 0],
    color: '#6b5344',
  });
  add.box({
    size: [0.09, 0.15, 0.012],
    position: [0.875, 0.075, 0.295],
    rotation: [0, -angle - 0.55, 0],
    color: '#6b5344',
  });
}
