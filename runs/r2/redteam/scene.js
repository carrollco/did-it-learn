export default function build({ THREE, add }) {
  const A = new THREE.Vector3(-0.90, 0.72, -0.318);
  const B = new THREE.Vector3(0.74, 0.06, 0.278);
  const diff = B.clone().sub(A);
  const u = diff.clone().normalize();
  const uHoriz = new THREE.Vector3(u.x, 0, u.z).normalize();
  const w = new THREE.Vector3(-u.z, 0, u.x).normalize();
  const v = new THREE.Vector3().crossVectors(w, u).normalize();

  const makeTroughWall = (center, side, alpha, length, width, thick) => {
    const yAxis = new THREE.Vector3().addScaledVector(v, Math.cos(alpha)).addScaledVector(w, -side * Math.sin(alpha));
    const zAxis = new THREE.Vector3().addScaledVector(w, side * Math.cos(alpha)).addScaledVector(v, Math.sin(alpha));
    const m = new THREE.Matrix4().makeBasis(u, yAxis, zAxis);
    const e = new THREE.Euler().setFromRotationMatrix(m, 'XYZ');
    const pos = center.clone().addScaledVector(zAxis, width / 2 - 0.004).addScaledVector(yAxis, -thick / 2);
    add.box({
      size: [length, thick, width],
      position: [pos.x, pos.y, pos.z],
      rotation: [e.x, e.y, e.z],
      color: '#cccccc',
    });
  };

  const c1 = A.clone().lerp(B, 0.25);
  const c2 = A.clone().lerp(B, 0.75);
  const len = diff.length() / 2 + 0.02;
  const alpha = (42 * Math.PI) / 180;
  const width = 0.075;
  const thick = 0.012;

  makeTroughWall(c1, -1, alpha, len, width, thick);
  makeTroughWall(c1, 1, alpha, len, width, thick);
  makeTroughWall(c2, -1, alpha, len, width, thick);
  makeTroughWall(c2, 1, alpha, len, width, thick);

  const cupMat = new THREE.Matrix4().makeBasis(uHoriz, new THREE.Vector3(0, 1, 0), w);
  const cupE = new THREE.Euler().setFromRotationMatrix(cupMat, 'XYZ');

  const backPos = new THREE.Vector3(0.8, 0.14, 0.3).addScaledVector(uHoriz, 0.060);
  add.box({
    size: [0.012, 0.22, 0.15],
    position: [backPos.x, backPos.y, backPos.z],
    rotation: [cupE.x, cupE.y, cupE.z],
    color: '#cccccc',
  });

  const leftPos = new THREE.Vector3(0.8, 0.13, 0.3).addScaledVector(w, -0.065);
  add.box({
    size: [0.14, 0.16, 0.012],
    position: [leftPos.x, leftPos.y, leftPos.z],
    rotation: [cupE.x, cupE.y, cupE.z],
    color: '#cccccc',
  });

  const rightPos = new THREE.Vector3(0.8, 0.13, 0.3).addScaledVector(w, 0.065);
  add.box({
    size: [0.14, 0.16, 0.012],
    position: [rightPos.x, rightPos.y, rightPos.z],
    rotation: [cupE.x, cupE.y, cupE.z],
    color: '#cccccc',
  });

  const boostPitch = 0.08;
  const bu = new THREE.Vector3().addScaledVector(u, Math.cos(boostPitch)).addScaledVector(v, -Math.sin(boostPitch));
  const bv = new THREE.Vector3().addScaledVector(v, Math.cos(boostPitch)).addScaledVector(u, Math.sin(boostPitch));
  const boostMat = new THREE.Matrix4().makeBasis(bu, bv, w);
  const boostE = new THREE.Euler().setFromRotationMatrix(boostMat, 'XYZ');
  const boostPos = new THREE.Vector3(-0.85, 0.80, -0.3).addScaledVector(bu, -0.070);
  add.box({
    size: [0.12, 0.14, 0.14],
    position: [boostPos.x, boostPos.y, boostPos.z],
    rotation: [boostE.x, boostE.y, boostE.z],
    color: '#ff5555',
  });
}
