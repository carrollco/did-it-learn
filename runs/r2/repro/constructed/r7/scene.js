export default function build({ THREE, add, task }) {
  const v3 = (x, y, z) => new THREE.Vector3(x, y, z);
  const eulerFromBasis = (ex, ey, ez) => {
    const m = new THREE.Matrix4().makeBasis(ex, ey, ez);
    const e = new THREE.Euler().setFromRotationMatrix(m, 'XYZ');
    return [e.x, e.y, e.z];
  };

  function addSegment(p0, p1, innerW0, innerW1, floorW, floorT, wallH, wallT, wallYOffset = 0) {
    const a = v3(...p0);
    const b = v3(...p1);
    const d = b.clone().sub(a);
    const L = d.length();
    const ex = d.clone().normalize();
    const ez = v3(-d.z, 0, d.x).normalize();
    const ey = new THREE.Vector3().crossVectors(ez, ex).normalize();
    const rot = eulerFromBasis(ex, ey, ez);
    const mid = a.clone().add(b).multiplyScalar(0.5);

    const floorPos = mid.clone().addScaledVector(ey, -floorT / 2);
    add.box({
      size: [L, floorT, floorW],
      position: [floorPos.x, floorPos.y, floorPos.z],
      rotation: rot,
      color: '#d9b382',
    });

    for (const side of [-1, 1]) {
      const w0 = a.clone()
        .addScaledVector(ey, wallH / 2 + wallYOffset)
        .addScaledVector(ez, side * (innerW0 / 2 + wallT / 2));
      const w1 = b.clone()
        .addScaledVector(ey, wallH / 2 + wallYOffset)
        .addScaledVector(ez, side * (innerW1 / 2 + wallT / 2));
      const wd = w1.clone().sub(w0);
      const wL = wd.length();
      const wex = wd.clone().normalize();
      const wez = new THREE.Vector3().crossVectors(wex, ey).normalize();
      const wRot = eulerFromBasis(wex, ey, wez);
      const wMid = w0.clone().add(w1).multiplyScalar(0.5);
      add.box({
        size: [wL, wallH, wallT],
        position: [wMid.x, wMid.y, wMid.z],
        rotation: wRot,
        color: '#8c5a32',
      });
    }
  }

  // Start is [-0.85, 0.8, -0.3], cup is [0.8, 0, 0.3]
  const pStart = [-0.96, 0.76, -0.34];
  const pMid = [-0.08, 0.36, -0.025];
  const pEnd = [0.73, 0.065, 0.275];

  addSegment(pStart, pMid, 0.135, 0.072, 0.076, 0.012, 0.060, 0.012, 0.015);
  addSegment(pMid, pEnd, 0.072, 0.068, 0.054, 0.012, 0.054, 0.012, 0.015);

  // Funnel walls behind cup
  const angle = Math.atan2(0.6, 1.65);
  const fLen = 0.19;
  const fH = 0.12;
  const fT = 0.012;
  const spread = 0.65;

  const apexX = 0.8 + 0.095 * Math.cos(angle);
  const apexZ = 0.3 + 0.095 * Math.sin(angle);

  for (const side of [-1, 1]) {
    const dirAngle = angle + Math.PI + side * spread;
    const cx = apexX + (fLen / 2) * Math.cos(dirAngle);
    const cz = apexZ + (fLen / 2) * Math.sin(dirAngle);
    add.box({
      size: [fLen, fH, fT],
      position: [cx, fH / 2, cz],
      rotation: [0, -dirAngle, 0],
      color: '#6b4423',
    });
  }
}
