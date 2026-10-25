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

    return { mid, len, ry, rz, uy, uz, width, wallHeight, thickness, totalWidth };
  }

  const alpha = Math.atan2(0.60, 1.65);
  const ry = -alpha;
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  const uz = [-sa, 0, ca];
  const pt = (s, y) => [-0.85 + s * ca, y, -0.30 + s * sa];
  const cupS = Math.hypot(1.65, 0.60);

  // Segment 1: upper catch & launch ramp (s from -0.065 to 0.60, len = 0.710m)
  const seg1 = addChute(pt(-0.065, 0.755), pt(0.60, 0.508), {
    width: 0.132,
    wallHeight: 0.072,
    thickness: 0.014,
    floorColor: '#d99e5b',
    wallColor: '#1e3a5f',
  });

  // Segment 2: middle main run (s from 0.55 to 1.18, len = 0.670m)
  const seg2 = addChute(pt(0.55, 0.50), pt(1.18, 0.27), {
    width: 0.102,
    wallHeight: 0.056,
    thickness: 0.014,
    floorColor: '#e4b074',
    wallColor: '#294c78',
  });

  // Segment 3: lower precision approach into cup (s from 1.13 to 1.70, len = 0.600m)
  const seg3 = addChute(pt(1.13, 0.262), pt(1.70, 0.076), {
    width: 0.080,
    wallHeight: 0.048,
    thickness: 0.014,
    floorColor: '#d99e5b',
    wallColor: '#1e3a5f',
  });

  // Compact side guide cheeks at the cup entry neck (s = cupS - 0.035), keeping the cup itself open to closeup camera
  for (const side of [-1, 1]) {
    const sDist = side * 0.052;
    const h = side === 1 ? 0.045 : 0.075; // Lower on front (+1, camera side) so cup interior & marble are clearly visible
    const yMid = side === 1 ? 0.082 : 0.095;
    const midCup = pt(cupS - 0.025, yMid);
    add.box({
      size: [0.095, h, 0.014],
      position: [midCup[0] + uz[0] * sDist, midCup[1], midCup[2] + uz[2] * sDist],
      rotation: [0, ry + side * 0.06, 0],
      color: '#294c78',
    });
  }

  // Angled catcher plate behind the cup (s = cupS + 0.042, tilted forward to deflect marble straight down into cup)
  const backP = pt(cupS + 0.042, 0.112);
  add.box({
    size: [0.016, 0.115, 0.115],
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

  const matBrass = new THREE.MeshStandardMaterial({ color: '#d4a84f', roughness: 0.25, metalness: 0.85 });
  const matDarkSteel = new THREE.MeshStandardMaterial({ color: '#222831', roughness: 0.4, metalness: 0.7 });
  const matWalnut = new THREE.MeshStandardMaterial({ color: '#5c3a21', roughness: 0.55, metalness: 0.05 });
  const matCopper = new THREE.MeshStandardMaterial({ color: '#c86d51', roughness: 0.3, metalness: 0.75 });
  const matChrome = new THREE.MeshStandardMaterial({ color: '#e2e6ea', roughness: 0.2, metalness: 0.9 });
  const matLeather = new THREE.MeshStandardMaterial({ color: '#2b2d2f', roughness: 0.75, metalness: 0.05 });
  const matCeramic = new THREE.MeshStandardMaterial({ color: '#f7f5f0', roughness: 0.25, metalness: 0.05 });
  const matTeal = new THREE.MeshStandardMaterial({ color: '#2a7b76', roughness: 0.35, metalness: 0.2 });

  // A. Brass top trim rails & wooden inlay stripe along each chute segment
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

  // B. Support towers & twin-pillar stanchions under the track
  const pillarStations = [
    { s: -0.05, yTop: 0.74, w: 0.135 },
    { s: 0.27, yTop: 0.62, w: 0.135 },
    { s: 0.57, yTop: 0.49, w: 0.11 },
    { s: 0.88, yTop: 0.37, w: 0.105 },
    { s: 1.15, yTop: 0.252, w: 0.092 },
    { s: 1.48, yTop: 0.14, w: 0.085 },
  ];

  for (const st of pillarStations) {
    const center = pt(st.s, 0);
    const base = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.065, 0.014, st.w + 0.09), matWalnut));
    base.position.set(center[0], 0.007, center[2]);
    base.rotation.y = ry;
    decor.add(base);

    const cradle = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.036, 0.018, st.w + 0.06), matBrass));
    cradle.position.set(center[0], st.yTop - 0.008, center[2]);
    cradle.rotation.y = ry;
    decor.add(cradle);

    for (const side of [-1, 1]) {
      const zOff = side * (st.w / 2 + 0.018);
      const px = center[0] + uz[0] * zOff;
      const pz = center[2] + uz[2] * zOff;
      const h = st.yTop - 0.014;
      const col = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.009, 0.011, h, 16), matDarkSteel));
      col.position.set(px, 0.014 + h / 2, pz);
      decor.add(col);

      const foot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.018, 0.012, 16), matBrass));
      foot.position.set(px, 0.018, pz);
      decor.add(foot);

      if (h > 0.25) {
        const midCollar = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.013, 0.013, 0.014, 16), matBrass));
        midCollar.position.set(px, 0.014 + h * 0.55, pz);
        decor.add(midCollar);
      }
    }

    if (st.yTop > 0.32) {
      const tieH = st.yTop * 0.55;
      const tie = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.0045, 0.0045, st.w + 0.036, 12), matChrome));
      tie.position.set(center[0], tieH, center[2]);
      tie.rotation.set(Math.PI / 2, ry, 0, 'YXZ');
      decor.add(tie);
    }
  }

  // C. Launch Drop Ring & Starter Back-Guard at [-0.85, 0.80, -0.30]
  const dropPos = pt(0.0, 0.80);
  const ringMesh = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.048, 0.005, 16, 32), matBrass));
  ringMesh.position.set(dropPos[0], 0.805, dropPos[2]);
  ringMesh.rotation.x = Math.PI / 2;
  decor.add(ringMesh);

  const backStopDecor = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.018, 0.09, 0.16), matWalnut));
  const pStart = pt(-0.068, 0.795);
  backStopDecor.position.set(pStart[0], pStart[1], pStart[2]);
  backStopDecor.rotation.y = ry;
  decor.add(backStopDecor);

  // D. Decorative Kinetic-Style Brass Flywheel & Escapement Arch over Segment 2 (s = 0.88)
  const archCenter = pt(0.88, 0.375);
  for (const side of [-1, 1]) {
    const zOff = side * 0.068;
    const post = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 0.15, 12), matBrass));
    post.position.set(archCenter[0] + uz[0] * zOff, archCenter[1] + 0.075, archCenter[2] + uz[2] * zOff);
    decor.add(post);
  }
  const axle = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.004, 0.004, 0.15, 12), matChrome));
  axle.position.set(archCenter[0], archCenter[1] + 0.14, archCenter[2]);
  axle.rotation.set(Math.PI / 2, ry, 0, 'YXZ');
  decor.add(axle);

  const wheel = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.036, 0.005, 12, 28), matBrass));
  wheel.position.set(archCenter[0] + uz[0] * 0.08, archCenter[1] + 0.14, archCenter[2] + uz[2] * 0.08);
  wheel.rotation.y = ry;
  decor.add(wheel);

  // E. Cup Base Coaster, Brass Rim Frame & Catcher Backing around [0.8, 0, 0.3]
  const cupBase = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.105, 0.112, 0.008, 32), matWalnut));
  cupBase.position.set(0.8, 0.004, 0.3);
  decor.add(cupBase);

  const cupTrimRing = shadow(new THREE.Mesh(new THREE.TorusGeometry(0.086, 0.0045, 12, 32), matBrass));
  cupTrimRing.position.set(0.8, 0.01, 0.3);
  cupTrimRing.rotation.x = Math.PI / 2;
  decor.add(cupTrimRing);

  const catcherPos = pt(cupS + 0.052, 0.112);
  const catcherBack = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.125, 0.128), matBrass));
  catcherBack.position.set(catcherPos[0], catcherPos[1], catcherPos[2]);
  catcherBack.rotation.set(0, ry, 0.38);
  decor.add(catcherBack);

  // F. Office Desk Accessories
  // 1) Architect Desk Lamp at back-right [0.55, 0, -0.32]
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

  // 2) Stack of Hardcover Books & Potted Succulent at [0.15, 0, -0.35]
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

  // 3) Wooden Spare-Marble Display Stand & Notebook in front-left [-0.58, 0, 0.32]
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
