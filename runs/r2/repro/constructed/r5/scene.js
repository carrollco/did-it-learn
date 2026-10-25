export default function build({ THREE, add, task }) {
  const t = 0.012;

  // Waypoints: behind start -> center height 0.36 -> cup rim
  const pStart = [-0.95, 0.70, -0.339];
  const pCenter = [0.0, 0.36, 0.0];
  const pCupRim = [0.736, 0.065, 0.276];

  function addChannel(p0, p1, floorW, wallSpan, wallH, flare, fExt0, fExt1, wExt0, wExt1, floorColor, wallColor) {
    const A = new THREE.Vector3(...p0);
    const B = new THREE.Vector3(...p1);
    const d = new THREE.Vector3().subVectors(B, A);
    const L = d.length();
    const u = d.clone().normalize();
    const up = new THREE.Vector3(0, 1, 0);
    const w = new THREE.Vector3().crossVectors(u, up).normalize();
    const v = new THREE.Vector3().crossVectors(w, u).normalize();
    const mat = new THREE.Matrix4().makeBasis(u, v, w);
    const e = new THREE.Euler().setFromRotationMatrix(mat, 'XYZ');
    const rot = [e.x, e.y, e.z];

    const fL = L + fExt0 + fExt1;
    const fMid = A.clone()
      .addScaledVector(u, (L + fExt1 - fExt0) / 2)
      .addScaledVector(v, -t / 2);
    add.box({
      size: [fL, t, floorW],
      position: [fMid.x, fMid.y, fMid.z],
      rotation: rot,
      color: floorColor,
    });

    const wL = L + wExt0 + wExt1;
    const baseMid = A.clone().addScaledVector(u, (L + wExt1 - wExt0) / 2);
    for (const s of [-1, 1]) {
      const wUp = v.clone().multiplyScalar(Math.cos(flare)).addScaledVector(w, s * Math.sin(flare));
      const wOut = w.clone().multiplyScalar(Math.cos(flare)).addScaledVector(v, -s * Math.sin(flare));
      const wMat = new THREE.Matrix4().makeBasis(u, wUp, wOut);
      const wEuler = new THREE.Euler().setFromRotationMatrix(wMat, 'XYZ');
      const wPos = baseMid.clone()
        .addScaledVector(w, s * (wallSpan / 2))
        .addScaledVector(v, -t)
        .addScaledVector(wUp, wallH / 2)
        .addScaledVector(wOut, s * (t / 2));
      add.box({
        size: [wL, wallH, t],
        position: [wPos.x, wPos.y, wPos.z],
        rotation: [wEuler.x, wEuler.y, wEuler.z],
        color: wallColor,
      });
    }
  }

  // Upper ramp with tall walls: from behind start to center height 0.36
  addChannel(pStart, pCenter, 0.045, 0.094, 0.068, 0.27, 0.0, 0.01, 0.0, 0.01, '#d8c3a5', '#8e9aaf');

  // Lower ramp with tall walls: from center height 0.36 to cup rim
  addChannel(
    [pCenter[0], pCenter[1] - 0.003, pCenter[2]],
    pCupRim,
    0.046,
    0.096,
    0.068,
    0.27,
    0.01,
    0.0,
    0.01,
    0.05,
    '#d8c3a5',
    '#8e9aaf'
  );

  // Funnel walls behind cup
  const baseYaw = Math.atan2(-(pCupRim[2] - pCenter[2]), pCupRim[0] - pCenter[0]);
  const fLen = 0.19;
  const fH = 0.115;
  const fAngle = 0.65;
  const fTilt = 0.22;
  const fApexX = 0.872;
  const fApexZ = 0.327;
  const fY = 0.08;

  for (const s of [-1, 1]) {
    const yaw = baseYaw + s * fAngle;
    const uWall = new THREE.Vector3(Math.cos(yaw), 0, -Math.sin(yaw));
    const nIn = new THREE.Vector3(-s * Math.sin(yaw), 0, -s * Math.cos(yaw));
    const up = new THREE.Vector3(0, 1, 0);
    const vWall = up.clone().multiplyScalar(Math.cos(fTilt)).addScaledVector(nIn, Math.sin(fTilt));
    const wWall = new THREE.Vector3().crossVectors(uWall, vWall).normalize();
    const mat = new THREE.Matrix4().makeBasis(uWall, vWall, wWall);
    const e = new THREE.Euler().setFromRotationMatrix(mat, 'XYZ');
    const px = fApexX - (fLen / 2) * uWall.x;
    const pz = fApexZ - (fLen / 2) * uWall.z;
    add.box({
      size: [fLen, fH, t],
      position: [px, fY, pz],
      rotation: [e.x, e.y, e.z],
      color: '#e07a5f',
    });
  }
}
