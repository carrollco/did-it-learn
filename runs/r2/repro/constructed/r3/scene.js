export default function build({ THREE, add, task }) {
  function makeBasis(p1, p2) {
    const a = new THREE.Vector3(...p1);
    const b = new THREE.Vector3(...p2);
    const d = new THREE.Vector3().subVectors(b, a);
    const len = d.length();
    const u = d.clone().normalize();
    const up = new THREE.Vector3(0, 1, 0);
    const w = new THREE.Vector3().crossVectors(u, up).normalize();
    const v = new THREE.Vector3().crossVectors(w, u).normalize();
    const mat = new THREE.Matrix4().makeBasis(u, v, w);
    const euler = new THREE.Euler().setFromRotationMatrix(mat, 'XYZ');
    const mid = new THREE.Vector3().addVectors(a, b).multiplyScalar(0.5);
    return { a, b, mid, len, u, v, w, rot: [euler.x, euler.y, euler.z] };
  }

  // Waypoints: behind start -> center height 0.36 -> cup rim
  const P0 = [-0.95, 0.66, -0.34];
  const P1 = [0.0, 0.36, 0.0];
  const P2 = [0.735, 0.062, 0.275];

  const seg1 = makeBasis(P0, P1);
  const seg2 = makeBasis(P1, P2);

  const thick = 0.012;
  const upperFloorW = 0.062;
  const lowerFloorW = 0.040;
  const wallH1 = 0.065;
  const wallH2 = 0.065;

  // 1. Upper ramp floor
  {
    const pos = seg1.mid.clone().addScaledVector(seg1.v, -thick / 2);
    add.box({
      size: [seg1.len + 0.01, thick, upperFloorW],
      position: [pos.x, pos.y, pos.z],
      rotation: seg1.rot,
      color: '#d9b38c',
    });
  }

  // 2. Lower ramp floor
  {
    const pos = seg2.mid.clone().addScaledVector(seg2.v, -thick / 2);
    add.box({
      size: [seg2.len + 0.01, thick, lowerFloorW],
      position: [pos.x, pos.y, pos.z],
      rotation: seg2.rot,
      color: '#d9b38c',
    });
  }

  // 3 & 4. Upper ramp walls: flare from half-gap 0.072 at P0 to 0.041 at P1
  const halfGap0 = 0.072;
  const halfGap1 = 0.041;
  for (const side of [-1, 1]) {
    const wA = seg1.a.clone()
      .addScaledVector(seg1.v, wallH1 / 2 - 0.004)
      .addScaledVector(seg1.w, side * (halfGap0 + thick / 2));
    const wB = seg1.b.clone()
      .addScaledVector(seg1.v, wallH1 / 2 - 0.004)
      .addScaledVector(seg1.w, side * (halfGap1 + thick / 2));
    const wSeg = makeBasis([wA.x, wA.y, wA.z], [wB.x, wB.y, wB.z]);
    // Re-orient so v matches seg1.v (tilted with the ramp floor)
    const uWall = new THREE.Vector3().subVectors(wB, wA).normalize();
    const wWall = new THREE.Vector3().crossVectors(uWall, seg1.v).normalize();
    const vWall = new THREE.Vector3().crossVectors(wWall, uWall).normalize();
    const mat = new THREE.Matrix4().makeBasis(uWall, vWall, wWall);
    const euler = new THREE.Euler().setFromRotationMatrix(mat, 'XYZ');
    add.box({
      size: [wSeg.len + 0.015, wallH1, thick],
      position: [wSeg.mid.x, wSeg.mid.y, wSeg.mid.z],
      rotation: [euler.x, euler.y, euler.z],
      color: '#c49a6c',
    });
  }

  // 5 & 6. Lower ramp walls (left and right), extending alongside cup
  const extCup = 0.065;
  const seg2WallLen = seg2.len + extCup;
  for (const side of [-1, 1]) {
    const pos = seg2.mid.clone()
      .addScaledVector(seg2.u, extCup / 2)
      .addScaledVector(seg2.v, wallH2 / 2 - 0.004)
      .addScaledVector(seg2.w, side * (halfGap1 + thick / 2));
    add.box({
      size: [seg2WallLen, wallH2, thick],
      position: [pos.x, pos.y, pos.z],
      rotation: seg2.rot,
      color: '#c49a6c',
    });
  }

  // 7 & 8. Funnel walls behind cup
  const uH = new THREE.Vector3(seg2.u.x, 0, seg2.u.z).normalize();
  const wH = new THREE.Vector3(seg2.w.x, 0, seg2.w.z).normalize();
  const cupC = new THREE.Vector3(0.8, 0, 0.3);

  const funnelH = 0.13;
  const funnelY = 0.05 + funnelH / 2;
  const apex = cupC.clone().addScaledVector(uH, 0.078);
  for (const side of [-1, 1]) {
    const wing = cupC.clone()
      .addScaledVector(uH, -0.025)
      .addScaledVector(wH, side * 0.072);
    const dir = new THREE.Vector3().subVectors(apex, wing);
    const fLen = dir.length() + 0.015;
    const fMid = new THREE.Vector3().addVectors(wing, apex).multiplyScalar(0.5);
    const yaw = Math.atan2(-dir.z, dir.x);
    add.box({
      size: [fLen, funnelH, thick],
      position: [fMid.x, funnelY, fMid.z],
      rotation: [0, yaw, 0],
      color: '#8c6239',
    });
  }
}
