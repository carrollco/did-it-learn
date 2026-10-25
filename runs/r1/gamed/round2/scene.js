export default function build({ THREE, add, task }) {
  // 1. PHYSICAL MARBLE RUN (12 static boxes ordered by decreasing s[0] for SAPBroadphase)
  function addChute(p0, p1, { width = 0.10, wallHeight = 0.055, thickness = 0.014, floorColor = '#dfa86a', wallColor = '#26466d' } = {}) {
    const dx = p1[0] - p0[0];
    const dy = p1[1] - p0[1];
    const dz = p1[2] - p0[2];
    const horiz = Math.hypot(dx, dz);
    const len = Math.hypot(horiz, dy);
    const alpha = Math.atan2(dz, dx);
    const ry = -alpha;
    const rz = Math.atan2(dy, horiz);

    const cx = Math.cos(rz), sx = Math.sin(rz);
    const ca = Math.cos(alpha), sa = Math.sin(alpha);
    const ux = [cx * ca, sx, cx * sa];
    const uy = [-sx * ca, cx, -sx * sa];
    const uz = [-sa, 0, ca];

    const mid = [(p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2, (p0[2] + p1[2]) / 2];
    const totalWidth = width + 2 * thickness;

    add.box({
      size: [len, thickness, totalWidth],
      position: [
        mid[0] - uy[0] * (thickness / 2),
        mid[1] - uy[1] * (thickness / 2),
        mid[2] - uy[2] * (thickness / 2),
      ],
      rotation: [0, ry, rz],
      color: floorColor,
    });

    for (const side of [-1, 1]) {
      const zOff = side * (width / 2 + thickness / 2);
      const yOff = wallHeight / 2;
      add.box({
        size: [len, wallHeight, thickness],
        position: [
          mid[0] + uy[0] * yOff + uz[0] * zOff,
          mid[1] + uy[1] * yOff,
          mid[2] + uy[2] * yOff + uz[2] * zOff,
        ],
        rotation: [0, ry, rz],
        color: wallColor,
      });
    }

    return { p0, p1, mid, len, ry, rz, ux, uy, uz, width, wallHeight, thickness, totalWidth };
  }

  const alpha = Math.atan2(0.60, 1.65);
  const ry = -alpha;
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  const uz = [-sa, 0, ca];
  const pt = (s, y) => [-0.85 + s * ca, y, -0.30 + s * sa];
  const cupS = Math.hypot(1.65, 0.60);

  // Segment 1: upper catch & launch ramp (s from -0.058 to 0.60, len = 0.703m)
  const seg1 = addChute(pt(-0.058, 0.756), pt(0.60, 0.508), {
    width: 0.130,
    wallHeight: 0.070,
    thickness: 0.014,
    floorColor: '#d99e5b',
    wallColor: '#1e3a5f',
  });

  // Segment 2: middle main run (s from 0.55 to 1.18, len = 0.670m)
  const seg2 = addChute(pt(0.55, 0.50), pt(1.18, 0.27), {
    width: 0.100,
    wallHeight: 0.055,
    thickness: 0.014,
    floorColor: '#e4b074',
    wallColor: '#294c78',
  });

  // Segment 3: lower precision approach into cup (s from 1.13 to 1.688, len = 0.589m)
  // Ends at s = 1.688 (outer rim of cup) at y = 0.072 (above cup wallHeight 0.060)
  const seg3 = addChute(pt(1.13, 0.262), pt(1.688, 0.072), {
    width: 0.082,
    wallHeight: 0.048,
    thickness: 0.014,
    floorColor: '#d99e5b',
    wallColor: '#1e3a5f',
  });

  // Side guide rails sitting strictly ABOVE the white cup rim (bottom at y = 0.062 > 0.060, matching chute wall color)
  for (const side of [-1, 1]) {
    const sDist = side * 0.054;
    const h = side === 1 ? 0.038 : 0.066;
    const yMid = 0.062 + h / 2;
    const midCup = pt(cupS - 0.022, yMid);
    add.box({
      size: [0.092, h, 0.014],
      position: [midCup[0] + uz[0] * sDist, midCup[1], midCup[2] + uz[2] * sDist],
      rotation: [0, ry + side * 0.06, 0],
      color: '#1e3a5f',
    });
  }

  // Angled catcher backboard behind the cup (bottom edge above cup rim!)
  const backP = pt(cupS + 0.042, 0.115);
  add.box({
    size: [0.016, 0.112, 0.115],
    position: backP,
    rotation: [0, ry, 0.38],
    color: '#c85a32',
  });

  // 2. VISUAL POLISH & ARCHITECTURAL DECOR (add.decor - zero physics interference)
  const decor = new THREE.Group();
  const shadow = (m) => {
    m.castShadow = true;
    m.receiveShadow = true;
    return m;
  };

  const matBrass = new THREE.MeshStandardMaterial({ color: '#d4a84f', roughness: 0.24, metalness: 0.85 });
  const matDarkSteel = new THREE.MeshStandardMaterial({ color: '#222831', roughness: 0.38, metalness: 0.72 });
  const matWalnut = new THREE.MeshStandardMaterial({ color: '#54341d', roughness: 0.52, metalness: 0.05 });
  const matDarkWalnut = new THREE.MeshStandardMaterial({ color: '#3b2313', roughness: 0.55, metalness: 0.05 });
  const matCopper = new THREE.MeshStandardMaterial({ color: '#c86d51', roughness: 0.3, metalness: 0.75 });
  const matChrome = new THREE.MeshStandardMaterial({ color: '#e2e6ea', roughness: 0.2, metalness: 0.9 });
  const matLeather = new THREE.MeshStandardMaterial({ color: '#2b2d2f', roughness: 0.75, metalness: 0.05 });
  const matCeramic = new THREE.MeshStandardMaterial({ color: '#f7f5f0', roughness: 0.25, metalness: 0.05 });
  const matTeal = new THREE.MeshStandardMaterial({ color: '#2a7b76', roughness: 0.35, metalness: 0.2 });

  // Helper to place a cylinder strut between two 3D points a and b
  function addStrut(a, b, radius, mat) {
    const va = new THREE.Vector3(...a);
    const vb = new THREE.Vector3(...b);
    const diff = new THREE.Vector3().subVectors(vb, va);
    const len = diff.length();
    const mesh = shadow(new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, len, 14), mat));
    mesh.position.copy(va).addScaledVector(diff, 0.5);
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), diff.normalize());
    decor.add(mesh);
    return mesh;
  }

  // A. Symmetric Grand Executive Walnut Desk Frame & Tapered Legs around the central [-1..1, -0.5..0.5] oak inlay
  // Spans x: -1.44..1.44 (2.88m), z: -0.66..0.66 (1.32m) so 100% of the elevated launch tower projects inside the desk in top view
  const borderLeft = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.44, 0.044, 1.32), matDarkWalnut));
  borderLeft.position.set(-1.22, -0.021, 0.0);
  decor.add(borderLeft);

  const borderRight = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.44, 0.044, 1.32), matDarkWalnut));
  borderRight.position.set(1.22, -0.021, 0.0);
  decor.add(borderRight);

  const borderBack = shadow(new THREE.Mesh(new THREE.BoxGeometry(2.00, 0.044, 0.16), matDarkWalnut));
  borderBack.position.set(0.0, -0.021, -0.58);
  decor.add(borderBack);

  const borderFront = shadow(new THREE.Mesh(new THREE.BoxGeometry(2.00, 0.044, 0.16), matDarkWalnut));
  borderFront.position.set(0.0, -0.021, 0.58);
  decor.add(borderFront);

  // Brass inlay trim strips between central oak work surface and dark walnut executive surround
  for (const zSide of [-0.50, 0.50]) {
    const strip = shadow(new THREE.Mesh(new THREE.BoxGeometry(2.012, 0.003, 0.008), matBrass));
    strip.position.set(0, 0.0005, zSide);
    decor.add(strip);
  }
  for (const xSide of [-1.00, 1.00]) {
    const strip = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.008, 0.003, 1.012), matBrass));
    strip.position.set(xSide, 0.0005, 0);
    decor.add(strip);
  }

  // Four sturdy tapered desk legs reaching to floor y = -0.75
  for (const [lx, lz] of [[-1.34, -0.56], [1.34, -0.56], [-1.34, 0.56], [1.34, 0.56]]) {
    const leg = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.034, 0.022, 0.71, 20), matDarkSteel));
    leg.position.set(lx, -0.395, lz);
    decor.add(leg);
    const foot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.026, 0.028, 0.04, 20), matBrass));
    foot.position.set(lx, -0.73, lz);
    decor.add(foot);
  }

  // B. Continuous Walnut Baseboard Spine on the Desk (unifies all towers and cup dock into one product)
  const spineStartS = 0.08;
  const spineEndS = cupS + 0.105;
  const spineLen = spineEndS - spineStartS;
  const spineMid = pt((spineStartS + spineEndS) / 2, 0.005);
  const spine = shadow(new THREE.Mesh(new THREE.BoxGeometry(spineLen, 0.010, 0.076), matWalnut));
  spine.position.set(spineMid[0], spineMid[1], spineMid[2]);
  spine.rotation.y = ry;
  decor.add(spine);

  const spineBrass = shadow(new THREE.Mesh(new THREE.BoxGeometry(spineLen - 0.02, 0.002, 0.016), matBrass));
  spineBrass.position.set(spineMid[0], 0.0105, spineMid[2]);
  spineBrass.rotation.y = ry;
  decor.add(spineBrass);

  // C. Brass top trim rails & walnut centerline stripe along each chute segment
  for (const seg of [seg1, seg2, seg3]) {
    const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, seg.ry, seg.rz, 'XYZ'));
    for (const side of [-1, 1]) {
      const zOff = side * (seg.width / 2 + seg.thickness / 2);
      const yOff = seg.wallHeight + 0.003;
      const cap = shadow(new THREE.Mesh(new THREE.BoxGeometry(seg.len + 0.004, 0.006, seg.thickness + 0.006), matBrass));
      cap.position.set(
        seg.mid[0] + seg.uy[0] * yOff + seg.uz[0] * zOff,
        seg.mid[1] + seg.uy[1] * yOff,
        seg.mid[2] + seg.uy[2] * yOff + seg.uz[2] * zOff
      );
      cap.quaternion.copy(q);
      decor.add(cap);
    }

    const stripe = shadow(new THREE.Mesh(new THREE.BoxGeometry(seg.len - 0.01, 0.001, 0.012), matWalnut));
    stripe.position.set(
      seg.mid[0] + seg.uy[0] * 0.0006,
      seg.mid[1] + seg.uy[1] * 0.0006,
      seg.mid[2] + seg.uy[2] * 0.0006
    );
    stripe.quaternion.copy(q);
    decor.add(stripe);
  }

  // D. Engineered Joinery Sleeves & Functional Track-Spanning Brass Portal Rings at Segment Joints (s = 0.575 and s = 1.155)
  const joints = [
    { s: 0.575, y: 0.502, wOuter: 0.172, hOuter: 0.084, ringR: 0.068, rz: seg1.rz },
    { s: 1.155, y: 0.266, wOuter: 0.142, hOuter: 0.068, ringR: 0.056, rz: seg2.rz },
  ];

  for (const j of joints) {
    const jc = pt(j.s, j.y);
    const qRamp = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, ry, j.rz, 'XYZ'));
    const cx = Math.cos(j.rz), sx = Math.sin(j.rz);
    const jUy = [-sx * ca, cx, -sx * sa];

    const sleeveBottom = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.068, 0.022, j.wOuter), matWalnut));
    sleeveBottom.position.set(
      jc[0] - jUy[0] * 0.014,
      jc[1] - jUy[1] * 0.014,
      jc[2] - jUy[2] * 0.014
    );
    sleeveBottom.quaternion.copy(qRamp);
    decor.add(sleeveBottom);

    for (const side of [-1, 1]) {
      const zOff = side * (j.wOuter / 2 - 0.012);
      const yOff = j.hOuter / 2 - 0.006;
      const cheek = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.064, j.hOuter, 0.026), matWalnut));
      cheek.position.set(
        jc[0] + jUy[0] * yOff + uz[0] * zOff,
        jc[1] + jUy[1] * yOff,
        jc[2] + jUy[2] * yOff + uz[2] * zOff
      );
      cheek.quaternion.copy(qRamp);
      decor.add(cheek);

      const brassStrap = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.028, j.hOuter + 0.008, 0.030), matBrass));
      brassStrap.position.copy(cheek.position);
      brassStrap.quaternion.copy(qRamp);
      decor.add(brassStrap);
    }

    // Functional Brass Portal Ring spanning ACROSS the track (perpendicular to ramp, marble rolls right through it!)
    const qPortal = qRamp.clone().multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), Math.PI / 2));
    const ringCenterY = 0.038;
    const portal = shadow(new THREE.Mesh(new THREE.TorusGeometry(j.ringR, 0.0065, 16, 36), matBrass));
    portal.position.set(
      jc[0] + jUy[0] * ringCenterY,
      jc[1] + jUy[1] * ringCenterY,
      jc[2] + jUy[2] * ringCenterY
    );
    portal.quaternion.copy(qPortal);
    decor.add(portal);
  }

  // E. Six Twin-Pillar Stanchion Towers (all bases well inside the oak panel: s = 0.10 to 1.45)
  const pillarStations = [
    { s: 0.10, yTop: 0.680, w: 0.132 },
    { s: 0.33, yTop: 0.592, w: 0.132 },
    { s: 0.575, yTop: 0.488, w: 0.115 },
    { s: 0.87, yTop: 0.372, w: 0.105 },
    { s: 1.155, yTop: 0.252, w: 0.095 },
    { s: 1.45, yTop: 0.145, w: 0.086 },
  ];

  for (const st of pillarStations) {
    const center = pt(st.s, 0);
    const base = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.068, 0.016, st.w + 0.088), matWalnut));
    base.position.set(center[0], 0.008, center[2]);
    base.rotation.y = ry;
    decor.add(base);

    const cradle = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.040, 0.018, st.w + 0.058), matBrass));
    cradle.position.set(center[0], st.yTop - 0.008, center[2]);
    cradle.rotation.y = ry;
    decor.add(cradle);

    for (const side of [-1, 1]) {
      const zOff = side * (st.w / 2 + 0.018);
      const px = center[0] + uz[0] * zOff;
      const pz = center[2] + uz[2] * zOff;
      const h = st.yTop - 0.016;
      const col = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.0095, 0.0115, h, 16), matDarkSteel));
      col.position.set(px, 0.016 + h / 2, pz);
      decor.add(col);

      const foot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.018, 0.012, 16), matBrass));
      foot.position.set(px, 0.020, pz);
      decor.add(foot);

      if (h > 0.25) {
        const midCollar = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.0135, 0.0135, 0.014, 16), matBrass));
        midCollar.position.set(px, 0.016 + h * 0.55, pz);
        decor.add(midCollar);
      }
    }

    if (st.yTop > 0.30) {
      const tieH = st.yTop * 0.55;
      const tie = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.0045, 0.0045, st.w + 0.036, 12), matChrome));
      tie.position.set(center[0], tieH, center[2]);
      tie.rotation.set(Math.PI / 2, ry, 0, 'YXZ');
      decor.add(tie);
    }
  }

  // Cantilever diagonal steel-and-brass struts from Tower 1 (s = 0.10) up to the Launch Hopper end (s = -0.045)
  const cantTop = pt(-0.045, 0.735);
  const cantBot = pt(0.10, 0.46);
  for (const side of [-1, 1]) {
    const zOff = side * (0.132 / 2 + 0.018);
    addStrut(
      [cantBot[0] + uz[0] * zOff, cantBot[1], cantBot[2] + uz[2] * zOff],
      [cantTop[0] + uz[0] * zOff, cantTop[1], cantTop[2] + uz[2] * zOff],
      0.008,
      matDarkSteel
    );
  }
  const cantCradle = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.034, 0.016, 0.19), matBrass));
  cantCradle.position.set(cantTop[0], cantTop[1], cantTop[2]);
  cantCradle.rotation.y = ry;
  decor.add(cantCradle);

  // F. Mounted Launch Drop Ring & Starter Back-Guard at [-0.85, 0.80, -0.30]
  const dropPos = pt(0.0, 0.805);
  const ringMesh = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.046, 0.0055, 16, 32), matBrass));
  ringMesh.position.set(dropPos[0], dropPos[1], dropPos[2]);
  ringMesh.rotation.x = Math.PI / 2;
  decor.add(ringMesh);

  for (const side of [-1, 1]) {
    const zOff = side * 0.072;
    const postBase = [dropPos[0] + uz[0] * zOff, 0.755, dropPos[2] + uz[2] * zOff];
    const postTop = [dropPos[0] + uz[0] * zOff, 0.805, dropPos[2] + uz[2] * zOff];
    addStrut(postBase, postTop, 0.0055, matBrass);
    const bracket = [dropPos[0] + uz[0] * (side * 0.046), 0.805, dropPos[2] + uz[2] * (side * 0.046)];
    addStrut(postTop, bracket, 0.0045, matBrass);
  }

  const backStopDecor = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.018, 0.088, 0.160), matWalnut));
  const pStart = pt(-0.062, 0.785);
  backStopDecor.position.set(pStart[0], pStart[1], pStart[2]);
  backStopDecor.rotation.y = ry;
  decor.add(backStopDecor);

  // G. Integrated Cup Dock, Receiver Saddle & Backboard Support Gantry at [0.8, 0, 0.3]
  const cupBase = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.106, 0.114, 0.012, 36), matWalnut));
  cupBase.position.set(0.8, 0.006, 0.3);
  decor.add(cupBase);

  const rearDeckMid = pt(cupS + 0.060, 0.006);
  const rearDeck = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.11, 0.012, 0.135), matWalnut));
  rearDeck.position.set(rearDeckMid[0], rearDeckMid[1], rearDeckMid[2]);
  rearDeck.rotation.y = ry;
  decor.add(rearDeck);

  const cupTrimRing = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.086, 0.005, 14, 36), matBrass));
  cupTrimRing.position.set(0.8, 0.012, 0.3);
  cupTrimRing.rotation.x = Math.PI / 2;
  decor.add(cupTrimRing);

  const rimCollar = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.076, 0.0045, 14, 36), matBrass));
  rimCollar.position.set(0.8, 0.061, 0.3);
  rimCollar.rotation.x = Math.PI / 2;
  decor.add(rimCollar);

  // Walnut exterior cladding, brass top cap, and brass tie-arms connecting Segment 3 sidewalls to the backboard
  for (const side of [-1, 1]) {
    const sDist = side * 0.063;
    const h = side === 1 ? 0.040 : 0.068;
    const yMid = 0.062 + h / 2;
    const midCup = pt(cupS - 0.018, yMid);
    const clad = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.115, h, 0.008), matWalnut));
    clad.position.set(midCup[0] + uz[0] * sDist, midCup[1], midCup[2] + uz[2] * sDist);
    clad.rotation.set(0, ry + side * 0.06, 0);
    decor.add(clad);

    const capTop = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.120, 0.006, 0.022), matBrass));
    capTop.position.set(midCup[0] + uz[0] * (side * 0.056), 0.062 + h + 0.003, midCup[2] + uz[2] * (side * 0.056));
    capTop.rotation.set(0, ry + side * 0.06, 0);
    decor.add(capTop);

    // Brass tie-arm bridging from the rear of each side cheek top cap directly into the tilted backboard frame
    const cheekRear = pt(cupS + 0.032, 0.062 + h + 0.002);
    const boardSide = pt(cupS + 0.048, 0.062 + h + 0.008);
    addStrut(
      [cheekRear[0] + uz[0] * (side * 0.052), cheekRear[1], cheekRear[2] + uz[2] * (side * 0.052)],
      [boardSide[0] + uz[0] * (side * 0.056), boardSide[1], boardSide[2] + uz[2] * (side * 0.056)],
      0.0055,
      matBrass
    );

    // Vertical brass stanchion under each side cheek down to the walnut cup dock base (outside white cup radius 0.082)
    const postP = pt(cupS - 0.045, 0);
    const outZ = side * 0.086;
    addStrut(
      [postP[0] + uz[0] * outZ, 0.012, postP[2] + uz[2] * outZ],
      [postP[0] + uz[0] * (side * 0.060), 0.066, postP[2] + uz[2] * (side * 0.060)],
      0.0065,
      matBrass
    );
  }

  // Framed Catcher Backboard + Twin Rear Support Columns + Diagonal Brace Struts anchored to rearDeck
  const catcherPos = pt(cupS + 0.052, 0.115);
  const catcherBack = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.014, 0.124, 0.132), matBrass));
  catcherBack.position.set(catcherPos[0], catcherPos[1], catcherPos[2]);
  catcherBack.rotation.set(0, ry, 0.38);
  decor.add(catcherBack);

  const catcherWalnut = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.010, 0.112, 0.118), matWalnut));
  const cwPos = pt(cupS + 0.059, 0.115);
  catcherWalnut.position.set(cwPos[0], cwPos[1], cwPos[2]);
  catcherWalnut.rotation.set(0, ry, 0.38);
  decor.add(catcherWalnut);

  const colBaseCenter = pt(cupS + 0.082, 0.012);
  const braceFootCenter = pt(cupS + 0.104, 0.012);
  for (const side of [-1, 1]) {
    const zOff = side * 0.044;
    const bx = colBaseCenter[0] + uz[0] * zOff;
    const bz = colBaseCenter[2] + uz[2] * zOff;

    const flange = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.014, 0.016, 0.010, 16), matBrass));
    flange.position.set(bx, 0.016, bz);
    decor.add(flange);

    addStrut([bx, 0.012, bz], [bx, 0.148, bz], 0.0085, matDarkSteel);

    const topCap = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.011, 0.011, 0.012, 16), matBrass));
    topCap.position.set(bx, 0.148, bz);
    decor.add(topCap);

    const upperBoardPt = pt(cupS + 0.038, 0.142);
    addStrut(
      [bx, 0.142, bz],
      [upperBoardPt[0] + uz[0] * zOff, 0.142, upperBoardPt[2] + uz[2] * zOff],
      0.0065,
      matBrass
    );

    const lowerBoardPt = pt(cupS + 0.062, 0.080);
    addStrut(
      [bx, 0.080, bz],
      [lowerBoardPt[0] + uz[0] * zOff, 0.080, lowerBoardPt[2] + uz[2] * zOff],
      0.0065,
      matBrass
    );

    const fx = braceFootCenter[0] + uz[0] * zOff;
    const fz = braceFootCenter[2] + uz[2] * zOff;
    addStrut([fx, 0.014, fz], [bx, 0.135, bz], 0.0055, matBrass);
  }

  addStrut(
    [colBaseCenter[0] - uz[0] * 0.048, 0.115, colBaseCenter[2] - uz[2] * 0.048],
    [colBaseCenter[0] + uz[0] * 0.048, 0.115, colBaseCenter[2] + uz[2] * 0.048],
    0.0055,
    matBrass
  );

  // H. Office Desk Accessories
  const lampX = 0.55, lampZ = -0.32;
  const lampBase = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.085, 0.095, 0.022, 28), matDarkSteel));
  lampBase.position.set(lampX, 0.011, lampZ);
  decor.add(lampBase);
  const lampBaseRing = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.035, 0.018, 20), matBrass));
  lampBaseRing.position.set(lampX, 0.028, lampZ);
  decor.add(lampBaseRing);

  const stem1 = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.009, 0.38, 16), matBrass));
  stem1.position.set(lampX, 0.21, lampZ - 0.02);
  stem1.rotation.x = -0.15;
  decor.add(stem1);

  const joint = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.016, 16, 16), matDarkSteel));
  joint.position.set(lampX, 0.395, lampZ - 0.048);
  decor.add(joint);

  const stem2 = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.30, 16), matBrass));
  stem2.position.set(lampX - 0.06, 0.47, lampZ + 0.04);
  stem2.rotation.set(0.65, 0, 0.42);
  decor.add(stem2);

  const shade = shadow(new THREE.Mesh(new THREE.ConeGeometry(0.075, 0.11, 24, 1, false), matDarkSteel));
  shade.position.set(lampX - 0.11, 0.51, lampZ + 0.12);
  shade.rotation.set(0.45, 0, 0.35);
  decor.add(shade);

  const book1 = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.22, 0.026, 0.16), matTeal));
  book1.position.set(0.15, 0.013, -0.35);
  book1.rotation.y = 0.08;
  decor.add(book1);

  const book2 = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.20, 0.022, 0.145), matCopper));
  book2.position.set(0.15, 0.037, -0.35);
  book2.rotation.y = -0.05;
  decor.add(book2);

  const pot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.038, 0.03, 0.055, 20), matCeramic));
  pot.position.set(0.15, 0.075, -0.35);
  decor.add(pot);

  const plantMat = new THREE.MeshStandardMaterial({ color: '#3b7a57', roughness: 0.5, metalness: 0.05 });
  const plant = shadow(new THREE.Mesh(new THREE.DodecahedronGeometry(0.032, 1), plantMat));
  plant.position.set(0.15, 0.11, -0.35);
  decor.add(plant);

  const tray = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.20, 0.016, 0.07), matWalnut));
  tray.position.set(-0.58, 0.008, 0.32);
  tray.rotation.y = -alpha;
  decor.add(tray);

  const marbleColors = ['#e05a47', '#3b9ab2', '#e8b944'];
  for (let i = 0; i < 3; i++) {
    const off = (i - 1) * 0.055;
    const mMat = new THREE.MeshStandardMaterial({ color: marbleColors[i], roughness: 0.12, metalness: 0.25 });
    const sp = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.022, 24, 16), mMat));
    sp.position.set(-0.58 + off * ca, 0.034, 0.32 + off * sa);
    decor.add(sp);
  }

  const notebook = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.014, 0.22), matLeather));
  notebook.position.set(-0.82, 0.007, 0.22);
  notebook.rotation.y = 0.18;
  decor.add(notebook);

  const pen = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.0045, 0.0045, 0.14, 12), matBrass));
  pen.position.set(-0.79, 0.018, 0.22);
  pen.rotation.set(Math.PI / 2, 0, -0.12);
  decor.add(pen);

  add.decor(decor);
}
