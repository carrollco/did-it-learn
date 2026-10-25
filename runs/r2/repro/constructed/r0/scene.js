export default function build({ THREE, add, task }) {
  const thickness = 0.012;

  function makeSegmentFrame(A, B, lean = 0) {
    const dx = B[0] - A[0];
    const dy = B[1] - A[1];
    const dz = B[2] - A[2];
    const L = Math.hypot(dx, dy, dz);
    const Lh = Math.hypot(dx, dz);
    const u = new THREE.Vector3(dx / L, dy / L, dz / L);
    const w0 = new THREE.Vector3(-dz / Lh, 0, dx / Lh);
    const v0 = new THREE.Vector3().crossVectors(w0, u).normalize();
    // Optional lean angle around u axis
    const v = v0.clone().multiplyScalar(Math.cos(lean)).addScaledVector(w0, Math.sin(lean));
    const w = w0.clone().multiplyScalar(Math.cos(lean)).addScaledVector(v0, -Math.sin(lean));
    const m = new THREE.Matrix4().makeBasis(u, v, w);
    const euler = new THREE.Euler().setFromRotationMatrix(m, 'XYZ');
    const mid = new THREE.Vector3(
      (A[0] + B[0]) / 2,
      (A[1] + B[1]) / 2,
      (A[2] + B[2]) / 2
    );
    return { L, Lh, u, v, w, rot: [euler.x, euler.y, euler.z], mid };
  }

  // Segment 1: Upper ramp from behind start to center height 0.36
  const A1 = [-0.96, 0.70, -0.34];
  const B1 = [0.0, 0.36, 0.0];
  const seg1 = makeSegmentFrame(A1, B1);

  // Segment 2: Lower ramp from center height 0.36 to cup rim
  const A2 = [-0.01, 0.354, -0.0035];
  const B2 = [0.735, 0.062, 0.275];
  const seg2 = makeSegmentFrame(A2, B2);

  const floorW1 = 0.042;
  const floorW2 = 0.042;
  const wallH1 = 0.072;
  const wallH2 = 0.072;

  // 1. Upper ramp floor
  {
    const p = seg1.mid.clone().addScaledVector(seg1.v, -thickness / 2);
    add.box({
      size: [seg1.L, thickness, floorW1],
      position: [p.x, p.y, p.z],
      rotation: seg1.rot,
      color: '#d9c2a3',
    });
  }

  // 2 & 3. Upper ramp left & right walls (tapered from wider at start to narrower at center)
  for (const side of [-1, 1]) {
    const wStart = 0.068 * side;
    const wEnd = 0.038 * side;
    const pA = new THREE.Vector3(...A1)
      .addScaledVector(seg1.w, wStart)
      .addScaledVector(seg1.v, wallH1 / 2 - 0.006);
    const pB = new THREE.Vector3(...B1)
      .addScaledVector(seg1.w, wEnd)
      .addScaledVector(seg1.v, wallH1 / 2 - 0.006);
    const wf = makeSegmentFrame([pA.x, pA.y, pA.z], [pB.x, pB.y, pB.z]);
    add.box({
      size: [wf.L, wallH1, thickness],
      position: [wf.mid.x, wf.mid.y, wf.mid.z],
      rotation: wf.rot,
      color: '#4a6b82',
    });
  }

  // 4. Lower ramp floor
  {
    const p = seg2.mid.clone().addScaledVector(seg2.v, -thickness / 2);
    add.box({
      size: [seg2.L, thickness, floorW2],
      position: [p.x, p.y, p.z],
      rotation: seg2.rot,
      color: '#d9c2a3',
    });
  }

  // 5 & 6. Lower ramp left & right walls
  for (const side of [-1, 1]) {
    const wStart = 0.042 * side;
    const wEnd = 0.046 * side;
    const pA = new THREE.Vector3(...A2)
      .addScaledVector(seg2.w, wStart)
      .addScaledVector(seg2.v, wallH2 / 2 - 0.006);
    const B2ext = new THREE.Vector3(...B2).addScaledVector(seg2.u, 0.025);
    const pB = B2ext
      .addScaledVector(seg2.w, wEnd)
      .addScaledVector(seg2.v, wallH2 / 2 - 0.006);
    const wf = makeSegmentFrame([pA.x, pA.y, pA.z], [pB.x, pB.y, pB.z]);
    add.box({
      size: [wf.L, wallH2, thickness],
      position: [wf.mid.x, wf.mid.y, wf.mid.z],
      rotation: wf.rot,
      color: '#4a6b82',
    });
  }

  // 7 & 8. Funnel walls behind cup (angled V-catcher leaning slightly inward over cup)
  const uh = new THREE.Vector3(0.8, 0, 0.3).normalize();
  const wh = new THREE.Vector3(-uh.z, 0, uh.x);
  const cupC = new THREE.Vector3(0.8, 0, 0.3);
  const funnelH = 0.145;
  const funnelY = 0.025 + funnelH / 2;

  for (const side of [-1, 1]) {
    const pFront = cupC.clone()
      .addScaledVector(uh, -0.04)
      .addScaledVector(wh, side * 0.056);
    const pBack = cupC.clone()
      .addScaledVector(uh, 0.080)
      .addScaledVector(wh, side * 0.004);
    // Lean inward toward cup center
    const wf = makeSegmentFrame(
      [pFront.x, funnelY, pFront.z],
      [pBack.x, funnelY, pBack.z],
      -side * 0.22
    );
    add.box({
      size: [wf.L, funnelH, thickness],
      position: [wf.mid.x, wf.mid.y, wf.mid.z],
      rotation: wf.rot,
      color: '#c85a32',
    });
  }
}
