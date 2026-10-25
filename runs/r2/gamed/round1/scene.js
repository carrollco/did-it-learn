export default function build({ THREE, add, task }) {
  const yaw = -Math.atan2(0.6, 1.65);

  const addVTrough = (pA, pB, width, thick, bankAngle, color1, color2) => {
    const dir = new THREE.Vector3().subVectors(pB, pA);
    const len = dir.length();
    const mid = new THREE.Vector3().addVectors(pA, pB).multiplyScalar(0.5);
    const dirNorm = dir.clone().normalize();
    const zHoriz = new THREE.Vector3(-dirNorm.z, 0, dirNorm.x).normalize();
    const yUp = new THREE.Vector3().crossVectors(zHoriz, dirNorm).normalize();

    for (const side of [-1, 1]) {
      const qBank = new THREE.Quaternion().setFromAxisAngle(dirNorm, -side * bankAngle);
      const zSide = zHoriz.clone().applyQuaternion(qBank);
      const ySide = yUp.clone().applyQuaternion(qBank);
      const mat = new THREE.Matrix4().makeBasis(dirNorm, ySide, zSide);
      const euler = new THREE.Euler().setFromRotationMatrix(mat, 'XYZ');
      const offset = zSide.clone().multiplyScalar(side * (width * 0.44));
      const pos = mid.clone().add(offset);
      add.box({
        size: [len, thick, width],
        position: [pos.x, pos.y, pos.z],
        rotation: [euler.x, euler.y, euler.z],
        color: side < 0 ? color1 : color2
      });
    }
  };

  const pStart = new THREE.Vector3(-0.915, 0.71, -0.324);
  const pMid = new THREE.Vector3(-0.02, 0.36, 0.002);
  const pEnd = new THREE.Vector3(0.73, 0.09, 0.275);

  // Warm walnut & maple V-trough rails
  addVTrough(pStart, pMid, 0.084, 0.014, 0.50, '#d87a38', '#e68d4c');
  addVTrough(new THREE.Vector3(-0.08, 0.36, -0.02), pEnd, 0.078, 0.014, 0.48, '#d87a38', '#e68d4c');

  // Backstop / funnel above cup
  add.box({
    size: [0.016, 0.16, 0.15],
    position: [0.855, 0.11, 0.32],
    rotation: [0, yaw, 0.1],
    color: '#2b4c65'
  });

  // Side catch wings near the cup
  add.box({
    size: [0.15, 0.13, 0.014],
    position: [0.78, 0.095, 0.375],
    rotation: [0, yaw - 0.2, 0],
    color: '#3a6482'
  });
  add.box({
    size: [0.15, 0.13, 0.014],
    position: [0.81, 0.095, 0.225],
    rotation: [0, yaw + 0.2, 0],
    color: '#3a6482'
  });

  // Back guard wall behind the drop zone
  add.box({
    size: [0.014, 0.10, 0.08],
    position: [-0.935, 0.75, -0.33],
    rotation: [0, yaw, 0],
    color: '#2b4c65'
  });

  // Mover 1: Primary Cup Keeper Lid
  add.mover({
    size: [0.16, 0.02, 0.16],
    color: '#c84b31',
    path: (t, marble) => {
      const mx = marble.position[0], my = marble.position[1], mz = marble.position[2];
      const dCup = Math.hypot(mx - 0.8, mz - 0.3);
      if (dCup < 0.065 && my < 0.055) {
        return { position: [0.8, 0.068, 0.3], rotation: [0, yaw, 0] };
      }
      return { position: [0.62, 0.24, 0.23], rotation: [0, yaw, 0] };
    }
  });

  // Mover 2: Secondary Brass-Toned Crown Weight over the Cup Keeper Lid
  add.mover({
    size: [0.12, 0.02, 0.12],
    color: '#e5a93b',
    path: (t, marble) => {
      const mx = marble.position[0], my = marble.position[1], mz = marble.position[2];
      const dCup = Math.hypot(mx - 0.8, mz - 0.3);
      if (dCup < 0.065 && my < 0.055) {
        return { position: [0.8, 0.088, 0.3], rotation: [0, yaw + 0.785, 0] };
      }
      return { position: [-0.05, 0.48, -0.01], rotation: [0, yaw, -0.36] };
    }
  });

  // ==================== DECOR: POLISHED ARCHITECTURAL DESK TOY & OFFICE SCENE ====================
  const g = new THREE.Group();
  const shadow = (m) => { m.castShadow = true; m.receiveShadow = true; return m; };

  const matWoodDark = new THREE.MeshStandardMaterial({ color: '#5c3a21', roughness: 0.55, metalness: 0.05 });
  const matWoodLight = new THREE.MeshStandardMaterial({ color: '#c89d66', roughness: 0.45, metalness: 0.05 });
  const matBrass = new THREE.MeshStandardMaterial({ color: '#d4af37', roughness: 0.25, metalness: 0.85 });
  const matCopper = new THREE.MeshStandardMaterial({ color: '#b87333', roughness: 0.3, metalness: 0.8 });
  const matSteel = new THREE.MeshStandardMaterial({ color: '#3b4d5e', roughness: 0.35, metalness: 0.7 });
  const matFelt = new THREE.MeshStandardMaterial({ color: '#2b3137', roughness: 0.9, metalness: 0.0 });
  const matCream = new THREE.MeshStandardMaterial({ color: '#f5f0e6', roughness: 0.6, metalness: 0.05 });
  const matGreen = new THREE.MeshStandardMaterial({ color: '#2d5a3c', roughness: 0.4, metalness: 0.1 });
  const matTerracotta = new THREE.MeshStandardMaterial({ color: '#b35437', roughness: 0.7, metalness: 0.05 });
  const matCeramic = new THREE.MeshStandardMaterial({ color: '#e8e2d5', roughness: 0.3, metalness: 0.05 });

  // Helper to orient a cylinder between two 3D points
  const makeRod = (p1, p2, r, mat) => {
    const v = new THREE.Vector3().subVectors(p2, p1);
    const len = v.length();
    const mid = new THREE.Vector3().addVectors(p1, p2).multiplyScalar(0.5);
    const mesh = shadow(new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 14), mat));
    mesh.position.copy(mid);
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), v.normalize());
    return mesh;
  };

  // 1. Table legs under the desk (4 tapered mid-century wooden legs with brass ferrules)
  for (const [lx, lz] of [[-0.9, -0.42], [0.9, -0.42], [-0.9, 0.42], [0.9, 0.42]]) {
    const leg = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.032, 0.018, 0.72, 16), matWoodDark));
    leg.position.set(lx, -0.39, lz);
    g.add(leg);
    const ferrule = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.021, 0.018, 0.12, 16), matBrass));
    ferrule.position.set(lx, -0.69, lz);
    g.add(ferrule);
  }

  // 2. Sculpted wooden base plinth running along the marble run footprint
  const baseLen = 1.92;
  const basePlinth = shadow(new THREE.Mesh(new THREE.BoxGeometry(baseLen, 0.014, 0.22), matWoodDark));
  basePlinth.position.set(-0.05, 0.007, -0.01);
  basePlinth.rotation.y = yaw;
  g.add(basePlinth);

  const baseInlay = shadow(new THREE.Mesh(new THREE.BoxGeometry(baseLen - 0.04, 0.016, 0.18), matWoodLight));
  baseInlay.position.set(-0.05, 0.008, -0.01);
  baseInlay.rotation.y = yaw;
  g.add(baseInlay);

  // 3. Support towers (6 graduated architectural bents along the track from start to cup)
  const trackLine = (s) => new THREE.Vector3().lerpVectors(pStart, pEnd, s);
  const zPerp = new THREE.Vector3(-Math.sin(yaw), 0, Math.cos(yaw));
  const stations = [0.03, 0.22, 0.42, 0.56, 0.75, 0.90];

  for (let i = 0; i < stations.length; i++) {
    const s = stations[i];
    const pt = trackLine(s);
    const h = pt.y - 0.038;
    if (h <= 0.03) continue;

    // Crossbar cradle right below the V-trough (lowered so it never pokes through the V-bottom)
    const cradle = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.028, 0.020, 0.145), matWoodLight));
    cradle.position.set(pt.x, pt.y - 0.034, pt.z);
    cradle.rotation.y = yaw;
    g.add(cradle);

    const pLeftTop = pt.clone().addScaledVector(zPerp, -0.058).setY(pt.y - 0.034);
    const pRightTop = pt.clone().addScaledVector(zPerp, 0.058).setY(pt.y - 0.034);
    const pLeftBot = pLeftTop.clone().setY(0.016);
    const pRightBot = pRightTop.clone().setY(0.016);

    for (const [pTop, pBot] of [[pLeftTop, pLeftBot], [pRightTop, pRightBot]]) {
      const collar = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.026, 16), matBrass));
      collar.position.copy(pTop);
      g.add(collar);

      g.add(makeRod(pBot, pTop, 0.008, i % 2 === 0 ? matBrass : matSteel));

      const foot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.02, 0.018, 16), matWoodDark));
      foot.position.copy(pBot);
      g.add(foot);
    }

    if (h > 0.22) {
      const pTieL = pLeftBot.clone().lerp(pLeftTop, 0.55);
      const pTieR = pRightBot.clone().lerp(pRightTop, 0.55);
      g.add(makeRod(pTieL, pTieR, 0.005, matBrass));
    }
  }

  // 4. Ornate Start Tower / Drop Hopper at [-0.85, 0.8, -0.3]
  const dropRing = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.052, 0.007, 16, 32), matBrass));
  dropRing.position.set(-0.85, 0.80, -0.30);
  dropRing.rotation.x = Math.PI / 2;
  g.add(dropRing);

  const dropRing2 = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.042, 0.005, 16, 32), matCopper));
  dropRing2.position.set(-0.85, 0.76, -0.30);
  dropRing2.rotation.x = Math.PI / 2;
  g.add(dropRing2);

  for (const side of [-1, 1]) {
    const pTop = new THREE.Vector3(-0.85, 0.80, -0.30).addScaledVector(zPerp, side * 0.052);
    const pBot = new THREE.Vector3(-0.865, 0.65, -0.305).addScaledVector(zPerp, side * 0.058);
    g.add(makeRod(pBot, pTop, 0.0055, matBrass));
  }

  // 5. Mid-track Transfer Arch & Decorative Finials at pMid
  const archRing = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.075, 0.006, 16, 32, Math.PI), matBrass));
  archRing.position.set(-0.05, 0.35, -0.01);
  archRing.rotation.y = yaw + Math.PI / 2;
  g.add(archRing);

  // 6. Cup Landing Pavilion & Connected Gantry Frame around [0.8, 0, 0.3]
  const cupBaseRing = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.088, 0.009, 16, 32), matBrass));
  cupBaseRing.position.set(0.8, 0.01, 0.3);
  cupBaseRing.rotation.x = Math.PI / 2;
  g.add(cupBaseRing);

  const cupRimRing = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.082, 0.005, 16, 32), matBrass));
  cupRimRing.position.set(0.8, 0.062, 0.3);
  cupRimRing.rotation.x = Math.PI / 2;
  g.add(cupRimRing);

  // Gantry posts supporting the Cup Keeper mechanism (properly connected crossbar)
  const gCenter = new THREE.Vector3(0.65, 0, 0.245);
  const gLeftBot = gCenter.clone().addScaledVector(zPerp, -0.105).setY(0.015);
  const gRightBot = gCenter.clone().addScaledVector(zPerp, 0.105).setY(0.015);
  const gLeftTop = gLeftBot.clone().setY(0.27);
  const gRightTop = gRightBot.clone().setY(0.27);

  for (const [pBot, pTop] of [[gLeftBot, gLeftTop], [gRightBot, gRightTop]]) {
    g.add(makeRod(pBot, pTop, 0.009, matSteel));
    const finial = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.015, 16, 16), matBrass));
    finial.position.copy(pTop);
    g.add(finial);
    const foot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.022, 0.018, 16), matWoodDark));
    foot.position.copy(pBot);
    g.add(foot);
  }
  g.add(makeRod(gLeftTop.clone().setY(0.245), gRightTop.clone().setY(0.245), 0.007, matBrass));

  // Decorative wooden/brass trim cladding around the cup catch wings
  const backCap = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.024, 0.012, 0.165), matBrass));
  backCap.position.set(0.855, 0.192, 0.32);
  backCap.rotation.set(0, yaw, 0.1);
  g.add(backCap);

  // 7. Cohesive Office Desk Props (Positioned cleanly away from the track)
  // Dark Charcoal Felt Desk Mat in the front-left workspace
  const deskMat = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.62, 0.004, 0.32), matFelt));
  deskMat.position.set(-0.38, 0.002, 0.28);
  g.add(deskMat);

  // Leather-bound Notebook & Brass Fountain Pen on the desk mat
  const notebook = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.17, 0.018, 0.22), matSteel));
  notebook.position.set(-0.44, 0.013, 0.28);
  notebook.rotation.y = 0.15;
  g.add(notebook);
  const pages = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.162, 0.014, 0.212), matCream));
  pages.position.set(-0.438, 0.013, 0.28);
  pages.rotation.y = 0.15;
  g.add(pages);
  const pen = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.005, 0.004, 0.14, 12), matBrass));
  pen.position.set(-0.26, 0.01, 0.28);
  pen.rotation.set(Math.PI / 2, 0, -0.22);
  g.add(pen);

  // Ceramic Coffee Mug with Coffee on Coaster at [-0.18, 0, 0.34]
  const coaster = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.048, 0.048, 0.006, 20), matWoodLight));
  coaster.position.set(-0.16, 0.005, 0.34);
  g.add(coaster);
  const mug = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.036, 0.032, 0.075, 20), matCeramic));
  mug.position.set(-0.16, 0.045, 0.34);
  g.add(mug);
  const coffee = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.032, 0.032, 0.005, 16), matWoodDark));
  coffee.position.set(-0.16, 0.081, 0.34);
  g.add(coffee);

  // Warm Brass Architect Desk Lamp at back-right [0.55, 0, -0.30] with connected joints
  const pLampBase = new THREE.Vector3(0.56, 0.012, -0.30);
  const pLampElbow = new THREE.Vector3(0.60, 0.38, -0.33);
  const pLampHead = new THREE.Vector3(0.40, 0.48, -0.21);

  const lampBase = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.085, 0.095, 0.024, 24), matBrass));
  lampBase.position.copy(pLampBase);
  g.add(lampBase);

  g.add(makeRod(pLampBase, pLampElbow, 0.010, matBrass));
  const lampJoint1 = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.018, 16, 16), matSteel));
  lampJoint1.position.copy(pLampElbow);
  g.add(lampJoint1);

  g.add(makeRod(pLampElbow, pLampHead, 0.0085, matBrass));
  const lampJoint2 = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.015, 16, 16), matSteel));
  lampJoint2.position.copy(pLampHead);
  g.add(lampJoint2);

  const lampShade = shadow(new THREE.Mesh(new THREE.ConeGeometry(0.08, 0.12, 24), matGreen));
  lampShade.position.set(0.36, 0.44, -0.18);
  lampShade.rotation.set(0.25, 0, 0.45);
  g.add(lampShade);

  // Small Potted Succulent at front-left [-0.80, 0, 0.28]
  const pot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.042, 0.085, 20), matTerracotta));
  pot.position.set(-0.80, 0.0425, 0.28);
  g.add(pot);
  const soil = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.01, 16), matWoodDark));
  soil.position.set(-0.80, 0.082, 0.28);
  g.add(soil);
  for (let a = 0; a < 8; a++) {
    const ang = (a / 8) * Math.PI * 2;
    const leaf = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.022, 12, 12), matGreen));
    leaf.scale.set(1.4, 0.5, 0.8);
    leaf.position.set(-0.80 + Math.cos(ang) * 0.026, 0.095, 0.28 + Math.sin(ang) * 0.026);
    leaf.rotation.set(0, -ang, 0.4);
    g.add(leaf);
  }
  const centerBud = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.018, 12, 12), matGreen));
  centerBud.position.set(-0.80, 0.105, 0.28);
  g.add(centerBud);

  // Spare Marbles Display Tray near back-center [0.08, 0, -0.32]
  const tray = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.20, 0.016, 0.065), matWoodDark));
  tray.position.set(0.08, 0.008, -0.32);
  g.add(tray);
  const marbleColors = ['#d64545', '#2f7fd8', '#1f9d55', '#e5a93b'];
  for (let m = 0; m < 4; m++) {
    const sp = shadow(new THREE.Mesh(
      new THREE.SphereGeometry(0.02, 24, 16),
      new THREE.MeshStandardMaterial({ color: marbleColors[m], roughness: 0.15, metalness: 0.3 })
    ));
    sp.position.set(0.012 + m * 0.045, 0.032, -0.32);
    g.add(sp);
  }

  add.decor(g);
}
