export default function build({ THREE, add, task }) {
  // Ramp with tall walls from behind start via center height 0.36 to cup rim plus funnel walls behind cup
  const thickness = 0.012;

  function addRampSegment(p0, p1, floorWidth, wallHeight, spacing0, spacing1, wallExtendEnd = 0) {
    const dx = p1[0] - p0[0];
    const dy = p1[1] - p0[1];
    const dz = p1[2] - p0[2];
    const horizLen = Math.hypot(dx, dz);
    const len = Math.hypot(dx, dy, dz);

    const ry = -Math.atan2(dz, dx);
    const rz = Math.atan2(dy, horizLen);

    const cx = (p0[0] + p1[0]) / 2;
    const cy = (p0[1] + p1[1]) / 2;
    const cz = (p0[2] + p1[2]) / 2;

    const dirX = dx / len;
    const dirY = dy / len;
    const dirZ = dz / len;

    const upX = -Math.sin(rz) * Math.cos(ry);
    const upY = Math.cos(rz);
    const upZ = Math.sin(rz) * Math.sin(ry);

    const sideX = Math.sin(ry);
    const sideZ = Math.cos(ry);

    add.box({
      size: [len, thickness, floorWidth],
      position: [
        cx - upX * (thickness / 2),
        cy - upY * (thickness / 2),
        cz - upZ * (thickness / 2),
      ],
      rotation: [0, ry, rz],
      color: '#d4a373',
    });

    const wallOffsetUp = wallHeight / 2 - thickness;
    const avgSpacing = (spacing0 + spacing1) / 2;
    const deltaSpacing = spacing1 - spacing0;
    const extLen = len + wallExtendEnd;
    const wallLen = Math.hypot(extLen, deltaSpacing / 2);
    const shiftAlong = wallExtendEnd / 2;

    for (const s of [-1, 1]) {
      const zOff = s * (avgSpacing / 2 + thickness / 2);
      const flareAngle = Math.atan2(s * (deltaSpacing / 2), extLen);
      const qRamp = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, ry, rz, 'XYZ'));
      const qFlare = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), -flareAngle);
      const qTotal = qRamp.multiply(qFlare);
      const eul = new THREE.Euler().setFromQuaternion(qTotal, 'XYZ');

      add.box({
        size: [wallLen, wallHeight, thickness],
        position: [
          cx + dirX * shiftAlong + upX * wallOffsetUp + sideX * zOff,
          cy + dirY * shiftAlong + upY * wallOffsetUp,
          cz + dirZ * shiftAlong + upZ * wallOffsetUp + sideZ * zOff,
        ],
        rotation: [eul.x, eul.y, eul.z],
        color: '#ccd5ae',
      });
    }
  }

  // Segment 1: behind start to center height 0.36
  addRampSegment([-0.93, 0.70, -0.33], [0.0, 0.36, 0.0], 0.064, 0.065, 0.112, 0.076, 0);

  // Segment 2: center height 0.36 to cup rim, with side walls extending 0.045 m over the front of the cup
  addRampSegment([0.0, 0.36, 0.0], [0.74, 0.065, 0.278], 0.052, 0.065, 0.076, 0.076, 0.045);

  // Funnel walls behind cup ([0.8, 0, 0.3]), meeting the extended side walls to form a closed funnel over the cup
  const alpha = Math.atan2(0.278, 0.74);
  const ux = Math.cos(alpha), uz = Math.sin(alpha);
  const vx = -Math.sin(alpha), vz = Math.cos(alpha);

  // Funnel wall endpoints in (u, v) relative to cup center [0.8, 0.3]:
  // Front end at u0 = -0.025, v0 = +/- 0.048 (meeting the extended lower ramp walls)
  // Back vertex at u1 = +0.068, v1 = 0 (just inside the back cup rim r = 0.070)
  const u0 = -0.025, v0 = 0.048;
  const u1 = 0.068, v1 = 0.0;
  const du = u1 - u0, dv = v0 - v1;
  const funnelLen = Math.hypot(du, dv) + 0.015;
  const funnelHeight = 0.13;
  const armAngle = Math.atan2(dv, du);

  for (const s of [-1, 1]) {
    const uMid = (u0 + u1) / 2;
    const vMid = s * (v0 + v1) / 2;
    // Offset outward by thickness/2 so inner face is on the line from (u0, s*v0) to (u1, 0)
    const nU = Math.sin(armAngle) * (thickness / 2);
    const nV = s * Math.cos(armAngle) * (thickness / 2);
    const px = 0.8 + ux * (uMid + nU) + vx * (vMid + nV);
    const pz = 0.3 + uz * (uMid + nU) + vz * (vMid + nV);
    const wallYaw = -alpha + s * armAngle;

    add.box({
      size: [funnelLen, funnelHeight, thickness],
      position: [px, 0.055 + funnelHeight / 2, pz],
      rotation: [0, wallYaw, 0],
      color: '#e9edc9',
    });
  }
}
