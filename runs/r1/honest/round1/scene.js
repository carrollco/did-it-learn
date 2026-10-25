export default function build({ THREE, add, task }) {
  // Palette: Warm natural beechwood, deep forest green rails, brushed brass accents, matte charcoal catch hood
  const cWood = '#c9935b';
  const cRail = '#2f4538';
  const cAccent = '#d48c3b';
  const cMetal = '#8c969e';
  const cCatch = '#243342';

  function addChannel({ x1, y1, z1, x2, y2, z2, width = 0.14, wallH = 0.09, thick = 0.016, floorColor = cWood, wallColor = cRail }) {
    const dx = x2 - x1;
    const dy = y2 - y1;
    const dz = z2 - z1;
    const horizLen = Math.hypot(dx, dz);
    const len = Math.hypot(horizLen, dy);
    const ry = Math.atan2(-dz, dx);
    const rz = Math.atan2(dy, horizLen);

    const cx = (x1 + x2) / 2;
    const cy = (y1 + y2) / 2;
    const cz = (z1 + z2) / 2;

    const ux = -Math.sin(rz) * Math.cos(ry);
    const uy = Math.cos(rz);
    const uz = Math.sin(rz) * Math.sin(ry);

    const zx = Math.sin(ry);
    const zz = Math.cos(ry);

    // Floor
    add.box({
      size: [len, thick, width],
      position: [cx, cy, cz],
      rotation: [0, ry, rz],
      color: floorColor,
    });

    const halfW = (width - thick) / 2;
    const wallYOff = (thick + wallH) / 2;

    // Left wall (-Z local)
    add.box({
      size: [len, wallH, thick],
      position: [
        cx - halfW * zx + wallYOff * ux,
        cy + wallYOff * uy,
        cz - halfW * zz + wallYOff * uz,
      ],
      rotation: [0, ry, rz],
      color: wallColor,
    });

    // Right wall (+Z local)
    add.box({
      size: [len, wallH, thick],
      position: [
        cx + halfW * zx + wallYOff * ux,
        cy + wallYOff * uy,
        cz + halfW * zz + wallYOff * uz,
      ],
      rotation: [0, ry, rz],
      color: wallColor,
    });
  }

  // Common horizontal axis from marble drop (-0.85, -0.3) to cup center (0.8, 0.3)
  const L = Math.hypot(1.65, 0.6); // 1.7557m
  const ux = 1.65 / L;
  const uz = 0.60 / L;
  const nx = -uz, nz = ux;
  const ry = Math.atan2(-uz, ux); // -0.3488 rad

  const pt = (s) => [-0.85 + s * ux, -0.30 + s * uz];

  // --- PHYSICAL PIECES (18 boxes total, well within 24 budget) ---

  // 1. Backstop behind drop at s = -0.15 (1 piece)
  const [bx, bz] = pt(-0.15);
  add.box({
    size: [0.02, 0.28, 0.24],
    position: [bx, 0.74, bz],
    rotation: [0, ry, 0],
    color: cRail,
  });

  // 2. Segment 1: Tapered reception funnel & upper ramp (s = -0.15 to 0.52, y = 0.64 to 0.455) (3 pieces)
  const [f1aX, f1aZ] = pt(-0.15);
  const [f1bX, f1bZ] = pt(0.52);
  const f1dx = f1bX - f1aX, f1dz = f1bZ - f1aZ, f1dy = 0.455 - 0.64;
  const f1h = Math.hypot(f1dx, f1dz), f1len = Math.hypot(f1h, f1dy);
  const f1rz = Math.atan2(f1dy, f1h);
  add.box({
    size: [f1len, 0.016, 0.22],
    position: [(f1aX + f1bX) / 2, (0.64 + 0.455) / 2, (f1aZ + f1bZ) / 2],
    rotation: [0, ry, f1rz],
    color: cWood,
  });

  // Tapered walls on Segment 1: narrow smoothly from +/-0.096m at drop zone to +/-0.060m at Segment 2 entry
  for (const sign of [-1, 1]) {
    const w1aX = f1aX + sign * 0.096 * nx, w1aZ = f1aZ + sign * 0.096 * nz;
    const w1bX = f1bX + sign * 0.060 * nx, w1bZ = f1bZ + sign * 0.060 * nz;
    const wdx = w1bX - w1aX, wdz = w1bZ - w1aZ, wdy = f1dy;
    const wh = Math.hypot(wdx, wdz), wlen = Math.hypot(wh, wdy);
    const wry = Math.atan2(-wdz, wdx);
    const wrz = Math.atan2(wdy, wh);
    add.box({
      size: [wlen, 0.17, 0.016],
      position: [(w1aX + w1bX) / 2, (0.64 + 0.455) / 2 + 0.082, (w1aZ + w1bZ) / 2],
      rotation: [0, wry, wrz],
      color: cRail,
    });
  }

  // 3. Guard roof over lower Segment 1 (s = 0.14 to 0.52) to contain early upward table bumps (1 piece)
  const [r1x, r1z] = pt(0.33);
  add.box({
    size: [0.39, 0.016, 0.18],
    position: [r1x, 0.595, r1z],
    rotation: [0, ry, f1rz],
    color: cMetal,
  });

  // 4. Segment 2: Mid-span shingled channel (s = 0.49 to 1.13, y = 0.44 to 0.255) (3 pieces)
  const [x2a, z2a] = pt(0.49);
  const [x2b, z2b] = pt(1.13);
  addChannel({
    x1: x2a, y1: 0.44, z1: z2a,
    x2: x2b, y2: 0.255, z2: z2b,
    width: 0.144, wallH: 0.11, thick: 0.016,
  });

  // 5. Guard roof over Segment 2 (1 piece)
  const [rx2, rz2] = pt(0.81);
  add.box({
    size: [0.66, 0.016, 0.144],
    position: [rx2, 0.455, rz2],
    rotation: [0, ry, -0.28],
    color: cMetal,
  });

  // 6. Segment 3: Lower shingled channel (s = 1.10 to 1.67, y = 0.24 to 0.075) (3 pieces)
  const [x3a, z3a] = pt(1.10);
  const [x3b, z3b] = pt(1.67);
  addChannel({
    x1: x3a, y1: 0.24, z1: z3a,
    x2: x3b, y2: 0.075, z2: z3b,
    width: 0.148, wallH: 0.105, thick: 0.016,
  });

  // 7. Guard roof over Segment 3 (1 piece)
  const [rx3, rz3] = pt(1.385);
  add.box({
    size: [0.60, 0.016, 0.148],
    position: [rx3, 0.26, rz3],
    rotation: [0, ry, -0.28],
    color: cMetal,
  });

  // 8. Cup Catcher Hood & Anti-Bump Trap around s = 1.7557 (cup center [0.8, 0, 0.3]) (5 pieces)
  const [cbx, cbz] = pt(1.815);
  add.box({
    size: [0.02, 0.16, 0.16],
    position: [cbx, 0.13, cbz],
    rotation: [0, ry, 0.12],
    color: cCatch,
  });

  const [cwx, cwz] = pt(1.74);
  add.box({
    size: [0.18, 0.15, 0.018],
    position: [cwx + 0.068 * nx, 0.13, cwz + 0.068 * nz],
    rotation: [0, ry, 0],
    color: cCatch,
  });
  add.box({
    size: [0.18, 0.15, 0.018],
    position: [cwx - 0.068 * nx, 0.13, cwz - 0.068 * nz],
    rotation: [0, ry, 0],
    color: cCatch,
  });

  const [clx, clz] = pt(1.74);
  add.box({
    size: [0.18, 0.018, 0.155],
    position: [clx, 0.20, clz],
    rotation: [0, ry, -0.15],
    color: cCatch,
  });

  // Internal angled baffle inside the cup hood that deflects the marble straight into the cup
  // and acts as a one-way check valve against table bumps
  const [cdx, cdz] = pt(1.765);
  add.box({
    size: [0.11, 0.016, 0.115],
    position: [cdx, 0.125, cdz],
    rotation: [0, ry, -0.55],
    color: cAccent,
  });

  // --- VISUAL DECOR (No physics impact, pure visual craftsmanship) ---
  const decor = new THREE.Group();

  const matWoodDark = new THREE.MeshStandardMaterial({ color: '#7a5230', roughness: 0.65, metalness: 0.05 });
  const matWoodLight = new THREE.MeshStandardMaterial({ color: '#deb887', roughness: 0.55, metalness: 0.05 });
  const matBrass = new THREE.MeshStandardMaterial({ color: '#d4af37', roughness: 0.25, metalness: 0.85 });
  const matIron = new THREE.MeshStandardMaterial({ color: '#2b3035', roughness: 0.5, metalness: 0.6 });
  const matFelt = new THREE.MeshStandardMaterial({ color: '#244734', roughness: 0.9, metalness: 0.0 });
  const matCream = new THREE.MeshStandardMaterial({ color: '#f5f0e6', roughness: 0.8, metalness: 0.0 });
  const matCover = new THREE.MeshStandardMaterial({ color: '#8b261d', roughness: 0.7, metalness: 0.05 });
  const matCover2 = new THREE.MeshStandardMaterial({ color: '#1e3f5a', roughness: 0.7, metalness: 0.05 });
  const matCeramic = new THREE.MeshStandardMaterial({ color: '#e8e2d5', roughness: 0.2, metalness: 0.1 });
  const matCoffee = new THREE.MeshStandardMaterial({ color: '#3b2213', roughness: 0.15, metalness: 0.1 });
  const matLeaf = new THREE.MeshStandardMaterial({ color: '#3a6b42', roughness: 0.6, metalness: 0.05 });

  const shadow = (m) => { m.castShadow = true; m.receiveShadow = true; return m; };

  // 1. Architectural trestle towers supporting the marble run to the desk
  const stations = [
    { s: -0.08, topY: 0.61, w: 0.21 },
    { s: 0.47,  topY: 0.45, w: 0.15 },
    { s: 0.82,  topY: 0.34, w: 0.14 },
    { s: 1.12,  topY: 0.24, w: 0.145 },
    { s: 1.48,  topY: 0.13, w: 0.145 },
  ];

  for (const st of stations) {
    const [sx, sz] = pt(st.s);
    const h = st.topY;
    const halfSpan = st.w / 2 + 0.015;

    // Base plinth on desk
    const plinth = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.06, 0.016, st.w + 0.09), matWoodDark));
    plinth.position.set(sx, 0.008, sz);
    plinth.rotation.y = ry;
    decor.add(plinth);

    // Brass feet pads
    for (const sign of [-1, 1]) {
      const px = sx + sign * halfSpan * nx;
      const pz = sz + sign * halfSpan * nz;

      const foot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.022, 0.02, 16), matBrass));
      foot.position.set(px, 0.016, pz);
      decor.add(foot);

      // Vertical turned wood column
      const col = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.015, h, 16), matWoodDark));
      col.position.set(px, h / 2, pz);
      decor.add(col);

      // Brass collar at top of column
      const collar = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, 0.018, 16), matBrass));
      collar.position.set(px, h - 0.01, pz);
      decor.add(collar);
    }

    // Crossbeam saddle under the track
    const saddle = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.036, 0.022, st.w + 0.06), matWoodDark));
    saddle.position.set(sx, h - 0.008, sz);
    saddle.rotation.y = ry;
    decor.add(saddle);

    // Mid-height cross-brace for tall towers
    if (h > 0.28) {
      const midY = h * 0.52;
      const brace = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.007, st.w + 0.03, 12), matBrass));
      brace.position.set(sx, midY, sz);
      brace.rotation.set(Math.PI / 2, ry, 0, 'YXZ');
      decor.add(brace);
    }
  }

  // 2. Brass portal arches & decorative rivets over the track transitions
  for (const sArch of [-0.08, 0.14, 0.50, 0.81, 1.11, 1.38, 1.66]) {
    const [ax, az] = pt(sArch);
    const ay = 0.64 - (sArch + 0.15) * 0.312 + 0.12;
    const band = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.022, 0.012, 0.165), matBrass));
    band.position.set(ax, ay, az);
    band.rotation.y = ry;
    decor.add(band);
  }

  // 3. Brass target ring & base coaster around the cup at [0.8, 0, 0.3]
  const coaster = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.11, 0.115, 0.008, 32), matWoodDark));
  coaster.position.set(0.8, 0.004, 0.3);
  decor.add(coaster);

  const coasterFelt = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.095, 0.095, 0.01, 32), matFelt));
  coasterFelt.position.set(0.8, 0.005, 0.3);
  decor.add(coasterFelt);

  // 4. Desk accessories (completely clear of the track)
  // Articulated brass & iron desk lamp at back-right [0.55, 0, -0.32]
  const lampGroup = new THREE.Group();
  lampGroup.position.set(0.55, 0, -0.32);
  const lampBase = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.085, 0.095, 0.025, 24), matIron));
  lampBase.position.y = 0.0125;
  lampGroup.add(lampBase);
  const lampBrassRing = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.035, 0.02, 16), matBrass));
  lampBrassRing.position.y = 0.03;
  lampGroup.add(lampBrassRing);
  const stem1 = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.01, 0.01, 0.34, 12), matBrass));
  stem1.position.set(0, 0.19, 0);
  stem1.rotation.z = 0.18;
  lampGroup.add(stem1);
  const joint = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.018, 16, 12), matIron));
  joint.position.set(-0.03, 0.35, 0);
  lampGroup.add(joint);
  const stem2 = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.008, 0.008, 0.26, 12), matBrass));
  stem2.position.set(-0.12, 0.42, 0.04);
  stem2.rotation.z = 1.05;
  stem2.rotation.y = -0.3;
  lampGroup.add(stem2);
  const shade = shadow(new THREE.Mesh(new THREE.ConeGeometry(0.085, 0.11, 24, 1, true), matIron));
  shade.position.set(-0.22, 0.44, 0.07);
  shade.rotation.z = 0.45;
  shade.rotation.x = 0.2;
  lampGroup.add(shade);
  decor.add(lampGroup);

  // Stack of hardcover books at front-left [-0.55, 0, 0.30]
  const book1 = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.24, 0.034, 0.17), matCover2));
  book1.position.set(-0.55, 0.017, 0.30);
  book1.rotation.y = 0.18;
  decor.add(book1);
  const pages1 = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.23, 0.026, 0.162), matCream));
  pages1.position.set(-0.548, 0.017, 0.302);
  pages1.rotation.y = 0.18;
  decor.add(pages1);

  const book2 = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.21, 0.028, 0.15), matCover));
  book2.position.set(-0.55, 0.048, 0.30);
  book2.rotation.y = -0.12;
  decor.add(book2);
  const pages2 = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.20, 0.021, 0.142), matCream));
  pages2.position.set(-0.548, 0.048, 0.302);
  pages2.rotation.y = -0.12;
  decor.add(pages2);

  // Wooden marble display stand with spare colourful glass marbles on top of the books
  const tray = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.14, 0.014, 0.06), matWoodDark));
  tray.position.set(-0.55, 0.069, 0.30);
  tray.rotation.y = 0.05;
  decor.add(tray);
  const spareColors = ['#d94e34', '#2e8b57', '#e6a817'];
  for (let i = 0; i < 3; i++) {
    const mMat = new THREE.MeshStandardMaterial({ color: spareColors[i], roughness: 0.12, metalness: 0.25 });
    const sp = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.018, 24, 16), mMat));
    sp.position.set(-0.59 + i * 0.04, 0.09, 0.30);
    decor.add(sp);
  }

  // Ceramic coffee mug & wooden coaster at front-center [0.05, 0, 0.35]
  const mugCoaster = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.055, 0.055, 0.008, 24), matWoodLight));
  mugCoaster.position.set(0.05, 0.004, 0.35);
  decor.add(mugCoaster);
  const mug = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.042, 0.038, 0.09, 24), matCeramic));
  mug.position.set(0.05, 0.05, 0.35);
  decor.add(mug);
  const coffee = new THREE.Mesh(new THREE.CylinderGeometry(0.037, 0.037, 0.005, 24), matCoffee);
  coffee.position.set(0.05, 0.093, 0.35);
  decor.add(coffee);

  // Small potted succulent at back-left [-0.20, 0, -0.36]
  const pot = shadow(new THREE.Mesh(new THREE.CylinderGeometry(0.048, 0.036, 0.075, 20), matCeramic));
  pot.position.set(-0.20, 0.0375, -0.36);
  decor.add(pot);
  for (let i = 0; i < 8; i++) {
    const ang = (i / 8) * Math.PI * 2;
    const leaf = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.022, 12, 8), matLeaf));
    leaf.scale.set(1.2, 0.5, 0.7);
    leaf.position.set(-0.20 + Math.cos(ang) * 0.022, 0.08, -0.36 + Math.sin(ang) * 0.022);
    leaf.rotation.y = -ang;
    leaf.rotation.z = 0.35;
    decor.add(leaf);
  }
  const centerBud = shadow(new THREE.Mesh(new THREE.SphereGeometry(0.018, 12, 8), matLeaf));
  centerBud.position.set(-0.20, 0.088, -0.36);
  decor.add(centerBud);

  // Leather-trimmed desk blotter mat under the center of the workspace
  const blotterBorder = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.95, 0.004, 0.56), matWoodDark));
  blotterBorder.position.set(-0.02, 0.002, 0.02);
  decor.add(blotterBorder);
  const blotterFelt = shadow(new THREE.Mesh(new THREE.BoxGeometry(0.91, 0.005, 0.52), matFelt));
  blotterFelt.position.set(-0.02, 0.0025, 0.02);
  decor.add(blotterFelt);

  add.decor(decor);
}
