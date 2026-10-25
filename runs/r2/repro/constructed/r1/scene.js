export default function build({ THREE, add, task }) {
  const thickness = 0.012;
  const floorWidth = 0.060;
  const wallHeight = 0.063;

  function addChannel(p0, p1, opts = {}) {
    const wFloor = opts.floorWidth ?? floorWidth;
    const hWall = opts.wallHeight ?? wallHeight;
    const flare = opts.flare ?? 0.50;
    const v0 = new THREE.Vector3(...p0);
    const v1 = new THREE.Vector3(...p1);
    const diff = new THREE.Vector3().subVectors(v1, v0);
    const len = Math.min(1.095, diff.length() + (opts.extraLen ?? 0.01));
    const mid = new THREE.Vector3().addVectors(v0, v1).multiplyScalar(0.5);

    const ex = diff.clone().normalize();
    const up = new THREE.Vector3(0, 1, 0);
    const ez = new THREE.Vector3().crossVectors(ex, up).normalize();
    const ey = new THREE.Vector3().crossVectors(ez, ex).normalize();

    const matFloor = new THREE.Matrix4().makeBasis(ex, ey, ez);
    const rotFloor = new THREE.Euler().setFromRotationMatrix(matFloor, 'XYZ');
    const posFloor = mid.clone().addScaledVector(ey, -thickness * 0.5);

    add.box({
      size: [len, thickness, wFloor],
      position: [posFloor.x, posFloor.y, posFloor.z],
      rotation: [rotFloor.x, rotFloor.y, rotFloor.z],
      color: '#d9b382',
    });

    for (const side of [-1, 1]) {
      const ang = side * flare;
      const c = Math.cos(ang), s = Math.sin(ang);
      const eyW = ey.clone().multiplyScalar(c).addScaledVector(ez, s);
      const ezW = ey.clone().multiplyScalar(-s).addScaledVector(ez, c);
      const matW = new THREE.Matrix4().makeBasis(ex, eyW, ezW);
      const rotW = new THREE.Euler().setFromRotationMatrix(matW, 'XYZ');

      // Place wall bottom-inner corner at floor top-outer edge (no gap, maximum top opening)
      const base = mid.clone()
        .addScaledVector(ez, side * (wFloor * 0.5 - 0.002))
        .addScaledVector(ey, -0.002)
        .addScaledVector(eyW, hWall * 0.5)
        .addScaledVector(ezW, side * (thickness * 0.5));

      add.box({
        size: [len, hWall, thickness],
        position: [base.x, base.y, base.z],
        rotation: [rotW.x, rotW.y, rotW.z],
        color: '#c49a6c',
      });
    }
  }

  // Segment 1: behind start to center height 0.36
  const P0 = [-0.94, 0.69, -0.332];
  const P1a = [0.0, 0.362, 0.0];
  addChannel(P0, P1a, { extraLen: 0.01, floorWidth: 0.060, wallHeight: 0.063, flare: 0.55 });

  // Segment 2: center height 0.36 to cup rim
  const P1b = [-0.01, 0.358, -0.0035];
  const P2 = [0.735, 0.068, 0.275];
  addChannel(P1b, P2, { extraLen: 0.01, floorWidth: 0.062, wallHeight: 0.062, flare: 0.45 });

  // Funnel walls behind cup (V-shape around the back/sides of the cup)
  const cupX = task.cup.center[0];
  const cupZ = task.cup.center[2];
  const dir = new THREE.Vector3(cupX, 0, cupZ).normalize();
  const right = new THREE.Vector3(-dir.z, 0, dir.x);

  const apex = new THREE.Vector3(cupX, 0, cupZ).addScaledVector(dir, 0.072);
  const fHeight = 0.12;
  const fLen = 0.17;
  const halfAngle = 0.48;

  for (const side of [-1, 1]) {
    const backDir = dir.clone().multiplyScalar(-Math.cos(halfAngle)).addScaledVector(right, side * Math.sin(halfAngle));
    const wallCenter = apex.clone().addScaledVector(backDir, fLen * 0.5 - 0.01);
    wallCenter.y = 0.06 + fHeight * 0.5 - 0.01;
    const yaw = Math.atan2(-backDir.z, backDir.x);
    add.box({
      size: [fLen, fHeight, thickness],
      position: [wallCenter.x, wallCenter.y, wallCenter.z],
      rotation: [0, yaw, 0],
      color: '#8c6239',
    });
  }

  // Visual support trestles under the ramp (decor only)
  const supportGroup = new THREE.Group();
  const woodMat = new THREE.MeshStandardMaterial({ color: '#7a5230', roughness: 0.7, metalness: 0.05 });
  const baseMat = new THREE.MeshStandardMaterial({ color: '#5c3d22', roughness: 0.8, metalness: 0.05 });
  const stops = [
    [-0.88, 0.655, -0.31],
    [-0.45, 0.505, -0.159],
    [0.0, 0.345, 0.0],
    [0.40, 0.19, 0.15],
  ];
  for (const [sx, sy, sz] of stops) {
    const post = new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.018, sy, 16), woodMat);
    post.position.set(sx, sy * 0.5, sz);
    post.castShadow = post.receiveShadow = true;
    supportGroup.add(post);

    const foot = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.05, 0.012, 20), baseMat);
    foot.position.set(sx, 0.006, sz);
    foot.castShadow = foot.receiveShadow = true;
    supportGroup.add(foot);
  }
  add.decor(supportGroup);
}
