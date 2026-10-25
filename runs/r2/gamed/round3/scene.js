export default function build({ THREE, add, task }) {
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

  // Stage 1: Upper Warm Walnut V-Chute from [-0.90, 0.71, -0.315] heading diagonally forward-right to [-0.08, 0.38, 0.14]
  const p1A = new THREE.Vector3(-0.90, 0.71, -0.315);
  const p1B = new THREE.Vector3(-0.08, 0.38, 0.14);
  const yaw1 = -Math.atan2(p1B.z - p1A.z, p1B.x - p1A.x);
  addVTrough(p1A, p1B, 0.082, 0.014, 0.50, '#7c4724', '#94572f');

  // Stage 2: Lower Brushed Brass & Maple V-Chute angled from [-0.09, 0.345, 0.135] to [0.71, 0.095, 0.283]
  // Terminates cleanly outside the cup rim (outer radius 0.082m) with zero mesh intersection!
  const p2A = new THREE.Vector3(-0.09, 0.345, 0.135);
  const p2B = new THREE.Vector3(0.71, 0.095, 0.283);
  const yaw2 = -Math.atan2(p2B.z - p2A.z, p2B.x - p2A.x);
  addVTrough(p2A, p2B, 0.080, 0.014, 0.50, '#c88736', '#dfa048');

  // Stage 1-to-2 Sleek Curved-Style Banked Turn Rail (Low-profile warm walnut & brass rail instead of a bulky slab)
  const midYaw = (yaw1 + yaw2) / 2;
  add.box({
    size: [0.20, 0.075, 0.014],
    position: [-0.04, 0.375, 0.192],
    rotation: [0.12, midYaw, 0],
    color: '#8c522b'
  });

  // Single Sleek Backstop Plate behind the cup (Leaves the white target cup completely open, unobstructed, and unclipped!)
  add.box({
    size: [0.014, 0.09, 0.11],
    position: [0.865, 0.11, 0.312],
    rotation: [0, yaw2, 0.14],
    color: '#8c522b'
  });

  // Mover 1: Precision Brass Cup Damper Disc (0.12m x 0.014m x 0.12m)
  add.mover({
    size: [0.12, 0.014, 0.12],
    color: '#d4af37',
    path: (t, marble) => {
      const mx = marble.position[0], my = marble.position[1], mz = marble.position[2];
      const dCup = Math.hypot(mx - 0.8, mz - 0.3);
      if (dCup < 0.065 && my < 0.055) {
        return { position: [0.8, 0.068, 0.3], rotation: [0, yaw2, 0] };
      }
      return { position: [0.62, 0.24, 0.265], rotation: [0, yaw2, 0] };
    }
  });

  // Mover 2: Secondary Copper Crown Seal (0.09m x 0.014m x 0.09m)
  add.mover({
    size: [0.09, 0.014, 0.09],
    color: '#b87333',
    path: (t, marble) => {
      const mx = marble.position[0], my = marble.position[1], mz = marble.position[2];
      const dCup = Math.hypot(mx - 0.8, mz - 0.3);
      if (dCup < 0.065 && my < 0.055) {
        return { position: [0.8, 0.082, 0.3], rotation: [0, yaw2, 0] };
      }
      return { position: [-0.07, 0.50, 0.14], rotation: [0, midYaw, -0.22] };
    }
  });

  // ==================== DECOR: MULTI-STAGE ARCHITECTURAL DESK TOY & OFFICE SCENE ====================
  const g = new THREE.Group();
  const shadow = (m) => { m.castShadow = true; m.receiveShadow = true; return m; };

  const matWoodDark = new THREE.MeshStandardMaterial({ color: '#5c3a21', roughness: 0.55, metalness: 0.05 });
  const matWoodLight = new THREE.MeshStandardMaterial({ color: '#c89d66', roughness: 0.45, metalness: 0.05 });
  const matBrass = new THREE.MeshStandardMaterial({ color: '#d4af37', roughness: 0.25, metalness: 0.85 });
  const matCopper = new THREE.MeshStandardMaterial({ color: '#b87333', roughness: 0.3, metalness: 0.8 });
  const matSteel = new THREE.MeshStandardMaterial({ color: '#2c3d4f', roughness: 0.35, metalness: 0.7 });
  const matFelt = new THREE.MeshStandardMaterial({ color: '#2b3137', roughness: 0.9, metalness: 0.0 });
  const matCream = new THREE.MeshStandardMaterial({ color: '#f5f0e6', roughness: 0.6, metalness: 0.05 });
  const matGreen = new THREE.MeshStandardMaterial({ color: '#2d5a3c', roughness: 0.4, metalness: 0.1 });
  const matTerracotta = new THREE.MeshStandardMaterial({ color: '#b35437', roughness: 0.7, metalness: 0.05 });
  const matCeramic = new THREE.MeshStandardMaterial({ color: '#e8e2d5', roughness: 0.3, metalness: 0.05 });

  const makeRod = (p1, p2, r, mat) => {
    const v = new THREE.Vector3().subVectors(p2, p1);
    const len = v.length();
    const mid = new THREE.Vector3().addVectors(p1, p2).multiplyScalar(0.5);
    const mesh = shadow(new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, 16), mat));
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

  // 2. Two-Stage Sculpted Wooden Base Plinths (100% strictly inside the desk boundaries!)
  const p1PlinthStart = new THREE.Vector3().lerpVectors(p1A, p1B, 0.16);
  const p1MidXZ = new THREE.Vector3().addVectors(p1PlinthStart, p1B).multiplyScalar(0.5);
  const plinth1 = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.80, 0.014, 0.18), matWoodDark));
  plinth1.position.set(p1MidXZ.x, 0.007, p1MidXZ.z);
  plinth1.rotation.y = yaw1;
  g.add(plinth1);
  const inlay1 = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.76, 0.016, 0.14), matWoodLight));
  inlay1.position.set(p1MidXZ.x, 0.008, p1MidXZ.z);
  inlay1.rotation.y = yaw1;
  g.add(inlay1);

  // Central Circular Turntable Plinth at the Dogleg Bend [-0.07, 0, 0.145]
  const turnPlinth = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.13, 0.14, 0.018, 32), matWoodDark));
  turnPlinth.position.set(-0.07, 0.009, 0.145);
  g.add(turnPlinth);
  const turnRing = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.11, 0.004, 12, 32), matBrass));
  turnRing.position.set(-0.07, 0.019, 0.145);
  turnRing.rotation.x = Math.PI / 2;
  g.add(turnRing);

  // Plinth 2 under Stage 2 & Cup Pavilion
  const p2MidXZ = new THREE.Vector3().addVectors(p2A, new THREE.Vector3(0.84, 0, 0.305)).multiplyScalar(0.5);
  const plinth2 = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.98, 0.014, 0.18), matWoodDark));
  plinth2.position.set(p2MidXZ.x, 0.007, p2MidXZ.z);
  plinth2.rotation.y = yaw2;
  g.add(plinth2);
  const inlay2 = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.94, 0.016, 0.14), matWoodLight));
  inlay2.position.set(p2MidXZ.x, 0.008, p2MidXZ.z);
  inlay2.rotation.y = yaw2;
  g.add(inlay2);

  // 3. Graduated Support Bents with Flush Under-Cradle Brackets that Firmly Touch the Underside of Both Chute Rails (Without Poking Into the Inner V-Channel!)
  const zPerp1 = new THREE.Vector3(-Math.sin(yaw1), 0, Math.cos(yaw1));
  const zPerp2 = new THREE.Vector3(-Math.sin(yaw2), 0, Math.cos(yaw2));

  const addSupportBent = (pt, zPerp, index) => {
    const yCross = pt.y - 0.032;
    const pLeftTop = pt.clone().addScaledVector(zPerp, -0.056).setY(yCross);
    const pRightTop = pt.clone().addScaledVector(zPerp, 0.056).setY(yCross);
    const pLeftBot = pLeftTop.clone().setY(0.016);
    const pRightBot = pRightTop.clone().setY(0.016);

    // Horizontal cylindrical brass crossbeam
    g.add(makeRod(pLeftTop, pRightTop, 0.007, matBrass));

    // Center vertical support post + V-shaped brass cradle rods tucked strictly UNDER the V-trough (at pt.y - 0.023) so they firmly support the chute from below without poking into the inner channel!
    const pCenterCross = pt.clone().setY(yCross);
    const pCenterApex = pt.clone().setY(pt.y - 0.023);
    g.add(makeRod(pCenterCross, pCenterApex, 0.009, matWoodLight));
    const apexCollar = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.011, 14, 14), matBrass));
    apexCollar.position.copy(pCenterApex);
    g.add(apexCollar);

    for (const side of [-1, 1]) {
      const pWingUnder = pt.clone().addScaledVector(zPerp, side * 0.042).setY(pt.y);
      const pWingCross = pt.clone().addScaledVector(zPerp, side * 0.042).setY(yCross);
      // Angled V-cradle rod hugging the underside of each V-trough wing
      g.add(makeRod(pCenterApex, pWingUnder, 0.0065, matWoodLight));
      // Vertical brass riser post under the outer wing edge
      g.add(makeRod(pWingCross, pWingUnder, 0.0055, matBrass));
      const wingJoint = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.0075, 12, 12), matBrass));
      wingJoint.position.copy(pWingUnder);
      g.add(wingJoint);
    }

    for (const [pTop, pBot] of [[pLeftTop, pLeftBot], [pRightTop, pRightBot]]) {
      const joint = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.013, 16, 16), matBrass));
      joint.position.copy(pTop);
      g.add(joint);

      g.add(makeRod(pBot, pTop, 0.0085, index % 2 === 0 ? matBrass : matSteel));

      const foot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.021, 0.018, 16), matWoodDark));
      foot.position.copy(pBot);
      g.add(foot);
    }

    if (yCross > 0.22) {
      const pTieL = pLeftBot.clone().lerp(pLeftTop, 0.52);
      const pTieR = pRightBot.clone().lerp(pRightTop, 0.52);
      g.add(makeRod(pTieL, pTieR, 0.005, matBrass));
      for (const pJ of [pTieL, pTieR]) {
        const j = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.009, 12, 12), matBrass));
        j.position.copy(pJ);
        g.add(j);
      }
    }
  };

  // 3 Bents along Stage 1
  for (let i = 0; i < 3; i++) {
    const s = [0.20, 0.52, 0.84][i];
    addSupportBent(new THREE.Vector3().lerpVectors(p1A, p1B, s), zPerp1, i);
  }
  // 2 Bents along Stage 2
  for (let i = 0; i < 2; i++) {
    const s = [0.22, 0.62][i];
    addSupportBent(new THREE.Vector3().lerpVectors(p2A, p2B, s), zPerp2, i + 3);
  }

  // 4. Cantilevered Brass Launch Hopper at [-0.85, 0.8, -0.3] with diagonal struts back to Bent 0 (s = 0.20)
  const dropRing = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.052, 0.007, 16, 32), matBrass));
  dropRing.position.set(-0.85, 0.80, -0.30);
  dropRing.rotation.x = Math.PI / 2;
  g.add(dropRing);

  const dropRing2 = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.042, 0.005, 16, 32), matCopper));
  dropRing2.position.set(-0.85, 0.76, -0.30);
  dropRing2.rotation.x = Math.PI / 2;
  g.add(dropRing2);

  const bent0Pt = new THREE.Vector3().lerpVectors(p1A, p1B, 0.20);
  const startTipPt = new THREE.Vector3().lerpVectors(p1A, p1B, 0.04);
  for (const side of [-1, 1]) {
    const pRing = new THREE.Vector3(-0.85, 0.80, -0.30).addScaledVector(zPerp1, side * 0.052);
    const pChuteStart = startTipPt.clone().addScaledVector(zPerp1, side * 0.042).setY(startTipPt.y + 0.002);
    g.add(makeRod(pChuteStart, pRing, 0.005, matBrass));

    const pBentMid = bent0Pt.clone().addScaledVector(zPerp1, side * 0.056).setY(0.42);
    g.add(makeRod(pBentMid, pChuteStart, 0.0055, matBrass));
  }

  // 5. Central Turn Wheelhouse & Kinetic Flywheel at [-0.07, 0.36, 0.145]
  const zPerpMid = new THREE.Vector3(-Math.sin(midYaw), 0, Math.cos(midYaw));
  const wCenter = new THREE.Vector3(-0.07, 0.36, 0.145);
  const wLeftBot = wCenter.clone().addScaledVector(zPerpMid, -0.088).setY(0.018);
  const wRightBot = wCenter.clone().addScaledVector(zPerpMid, 0.088).setY(0.018);
  const wLeftTop = wLeftBot.clone().setY(0.49);
  const wRightTop = wRightBot.clone().setY(0.49);

  for (const [pBot, pTop] of [[wLeftBot, wLeftTop], [wRightBot, wRightTop]]) {
    g.add(makeRod(pBot, pTop, 0.0085, matSteel));
    const finial = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.014, 16, 16), matBrass));
    finial.position.copy(pTop);
    g.add(finial);
    const foot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.02, 0.018, 16), matWoodDark));
    foot.position.copy(pBot);
    g.add(foot);
  }
  g.add(makeRod(wLeftTop.clone().setY(0.47), wRightTop.clone().setY(0.47), 0.0065, matBrass));

  // Brass cornice cap on top of the Sleek Mid-Turn Banked Rail
  const midWallCap = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.204, 0.008, 0.018), matBrass));
  midWallCap.position.set(-0.036, 0.413, 0.196);
  midWallCap.rotation.set(0.12, midYaw, 0);
  g.add(midWallCap);

  // Spoked Brass Kinetic Flywheel mounted on the turn tower
  const wheelPos = wRightBot.clone().setY(0.36);
  const wheelRim = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.058, 0.0055, 14, 28), matBrass));
  wheelRim.position.copy(wheelPos);
  wheelRim.rotation.y = midYaw;
  g.add(wheelRim);
  const wheelHub = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.016, 16), matCopper));
  wheelHub.position.copy(wheelPos);
  wheelHub.rotation.set(Math.PI / 2, midYaw, 0);
  g.add(wheelHub);
  for (let sp = 0; sp < 4; sp++) {
    const a = (sp / 4) * Math.PI;
    const spoke = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.003, 0.003, 0.114, 10), matBrass));
    spoke.position.copy(wheelPos);
    spoke.rotation.set(0, midYaw, a);
    g.add(spoke);
  }

  // 6. Stage 3 Open Cup Landing Pavilion & Connected Gantry Frame around [0.8, 0, 0.3]
  const cupBaseRing = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.088, 0.008, 16, 32), matBrass));
  cupBaseRing.position.set(0.8, 0.01, 0.3);
  cupBaseRing.rotation.x = Math.PI / 2;
  g.add(cupBaseRing);

  const cupRimRing = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.084, 0.005, 16, 32), matBrass));
  cupRimRing.position.set(0.8, 0.058, 0.3);
  cupRimRing.rotation.x = Math.PI / 2;
  g.add(cupRimRing);

  // Gantry posts supporting the Cup Keeper mechanism
  const gCenter = new THREE.Vector3(0.62, 0, 0.265);
  const gLeftBot = gCenter.clone().addScaledVector(zPerp2, -0.095).setY(0.016);
  const gRightBot = gCenter.clone().addScaledVector(zPerp2, 0.095).setY(0.016);
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

  // Brass cornice cap & twin rear brass posts supporting the single sleek Backstop Plate behind the cup
  const backCap = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.018, 0.008, 0.114), matBrass));
  backCap.position.set(0.859, 0.156, 0.312);
  backCap.rotation.set(0, yaw2, 0.14);
  g.add(backCap);

  for (const side of [-1, 1]) {
    const pBackPost = new THREE.Vector3(0.87, 0, 0.312).addScaledVector(zPerp2, side * 0.045);
    const pB = pBackPost.clone().setY(0.016);
    const pT = pBackPost.clone().setY(0.155);
    g.add(makeRod(pB, pT, 0.0055, matBrass));
    const cap = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.009, 12, 12), matBrass));
    cap.position.copy(pT);
    g.add(cap);
  }

  // 7. Cohesive Office Desk Props (Positioned cleanly away from the dogleg track)
  const deskMat = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.55, 0.004, 0.28), matFelt));
  deskMat.position.set(-0.56, 0.002, 0.31);
  g.add(deskMat);

  const notebook = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.018, 0.21), matSteel));
  notebook.position.set(-0.62, 0.013, 0.31);
  notebook.rotation.y = 0.15;
  g.add(notebook);
  const pages = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.152, 0.014, 0.202), matCream));
  pages.position.set(-0.618, 0.013, 0.31);
  pages.rotation.y = 0.15;
  g.add(pages);
  const pen = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.005, 0.004, 0.14, 12), matBrass));
  pen.position.set(-0.46, 0.01, 0.31);
  pen.rotation.set(Math.PI / 2, 0, -0.22);
  g.add(pen);

  const coaster = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.048, 0.048, 0.006, 20), matWoodLight));
  coaster.position.set(-0.36, 0.005, 0.34);
  g.add(coaster);
  const mug = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.036, 0.032, 0.075, 20), matCeramic));
  mug.position.set(-0.36, 0.045, 0.34);
  g.add(mug);
  const coffee = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.032, 0.032, 0.005, 16), matWoodDark));
  coffee.position.set(-0.36, 0.081, 0.34);
  g.add(coffee);

  // Warm Brass Architect Desk Lamp at back-right [0.52, 0, -0.28]
  const pLampBase = new THREE.Vector3(0.52, 0.012, -0.28);
  const pLampElbow = new THREE.Vector3(0.56, 0.38, -0.31);
  const pLampHead = new THREE.Vector3(0.36, 0.48, -0.19);

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
  lampShade.position.set(0.32, 0.44, -0.16);
  lampShade.rotation.set(0.25, 0, 0.45);
  g.add(lampShade);

  // Small Potted Succulent at front-left [-0.88, 0, 0.28]
  const pot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.052, 0.040, 0.082, 20), matTerracotta));
  pot.position.set(-0.88, 0.041, 0.28);
  g.add(pot);
  const soil = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.047, 0.047, 0.01, 16), matWoodDark));
  soil.position.set(-0.88, 0.079, 0.28);
  g.add(soil);
  for (let a = 0; a < 8; a++) {
    const ang = (a / 8) * Math.PI * 2;
    const leaf = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.020, 12, 12), matGreen));
    leaf.scale.set(1.4, 0.5, 0.8);
    leaf.position.set(-0.88 + Math.cos(ang) * 0.024, 0.092, 0.28 + Math.sin(ang) * 0.024);
    leaf.rotation.set(0, -ang, 0.4);
    g.add(leaf);
  }
  const centerBud = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.016, 12, 12), matGreen));
  centerBud.position.set(-0.88, 0.102, 0.28);
  g.add(centerBud);

  // Spare Marbles Display Tray near back-center [0.02, 0, -0.30]
  const tray = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.20, 0.016, 0.065), matWoodDark));
  tray.position.set(0.02, 0.008, -0.30);
  g.add(tray);
  const marbleColors = ['#d64545', '#2f7fd8', '#1f9d55', '#e5a93b'];
  for (let m = 0; m < 4; m++) {
    const sp = shadow(new THREE.Mesh(
      new THREE.SphereGeometry(0.02, 24, 16),
      new THREE.MeshStandardMaterial({ color: marbleColors[m], roughness: 0.15, metalness: 0.3 })
    ));
    sp.position.set(-0.048 + m * 0.045, 0.032, -0.30);
    g.add(sp);
  }

  add.decor(g);
}
