import { homedir } from 'node:os';
// Local SVG projection of the same Three.js scene, for hosts that cannot start Chromium.
import * as THREE from '../../harness/node_modules/three/build/three.module.js';
import { flightScene, setFlightCamera } from './scene.mjs';
import { writeFile, readFile, mkdir, mkdtemp, readdir, access, rm } from 'node:fs/promises';
import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { join, resolve } from 'node:path';
import { once } from 'node:events';

const esc = (s) => String(s).replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&apos;' }[c]));
const f = (n) => n.toFixed(2);

export async function softwareShot(track, cameraName, svgFile, pngFile, { width = 1280, height = 720, trail = false, at = 'end', playback = false, jpegFile } = {}) {
  const labels = [], { scene, drone } = flightScene((text, pos) => labels.push({ text, pos }));
  const time = at === 'end' ? track.frames.at(-1).t : Math.min(track.frames.at(-1).t, Math.max(0, Number(at)));
  const frame = track.frames.reduce((chosen, fr) => fr.t <= time ? fr : chosen, track.frames[0]);
  drone.position.set(frame.pos.x, frame.pos.y, frame.pos.z);
  drone.quaternion.setFromEuler(new THREE.Euler(-frame.att.roll, frame.att.yaw, -frame.att.pitch, 'YZX'));
  const camera = new THREE.PerspectiveCamera(43, width / height, 0.05, 100);
  setFlightCamera(camera, cameraName, drone);
  scene.updateMatrixWorld(true); camera.updateMatrixWorld(true);
  const project = (v) => { const p = v.clone().project(camera); return [width * (p.x + 1) / 2, height * (1 - p.y) / 2, p.z]; };
  const point = (p) => `${f(p[0])},${f(p[1])}`;
  const triangles = [], lines = [], shadows = [], floor = [];
  const light = new THREE.Vector3(-3, 8, 3).normalize();
  scene.traverse((object) => {
    if (!object.isMesh && !object.isLine) return;
    const geom = object.geometry, attr = geom.getAttribute('position'), index = geom.getIndex();
    if (!attr) return;
    const material = Array.isArray(object.material) ? object.material[0] : object.material;
    if (material.transparent && material.opacity < 0.3) return;
    const vertex = (i) => new THREE.Vector3().fromBufferAttribute(attr, index ? index.getX(i) : i).applyMatrix4(object.matrixWorld);
    const color = material.color || new THREE.Color('#bcc7b6');
    if (object.isLine) {
      const count = index ? index.count : attr.count;
      for (let i = 0; i + 1 < count; i += object.isLineSegments ? 2 : 1) {
        const a = project(vertex(i)), b = project(vertex(i + 1));
        if (a[2] < 1 && b[2] < 1) lines.push(`<path d="M${point(a)} L${point(b)}" fill="none" stroke="#bfc8b7" stroke-width="0.7"/>`);
      }
      return;
    }
    const count = index ? index.count : attr.count;
    for (let i = 0; i + 2 < count; i += 3) {
      const a = vertex(i), b = vertex(i + 1), c = vertex(i + 2);
      const normal = b.clone().sub(a).cross(c.clone().sub(a)).normalize();
      const centre = a.clone().add(b).add(c).multiplyScalar(1 / 3);
      if (normal.dot(camera.position.clone().sub(centre)) < 0 && material.side !== THREE.DoubleSide) continue;
      const points = [a, b, c].map(project);
      if (points.some((p) => p[2] >= 1 || !p.every(Number.isFinite))) continue;
      const intensity = 0.69 + 0.31 * Math.max(0, normal.dot(light));
      const fill = '#' + color.clone().multiplyScalar(intensity).getHexString();
      const markup = `<polygon points="${points.map(point).join(' ')}" fill="${fill}" stroke="${fill}" stroke-width="0.35" opacity="${material.opacity ?? 1}"/>`;
      const depth = centre.clone().applyMatrix4(camera.matrixWorldInverse).z;
      if (Math.max(a.y, b.y, c.y) < 0) floor.push(markup);
      else triangles.push({ depth, markup });
      if (object.castShadow && centre.y > 0.05 && centre.y < 2.8) {
        const shadow = [a, b, c].map((v) => project(new THREE.Vector3(v.x + 0.36 * v.y, 0.003, v.z - 0.36 * v.y)));
        shadows.push(`<polygon points="${shadow.map(point).join(' ')}" fill="#7b8976" opacity="0.035"/>`);
      }
    }
  });
  triangles.sort((a, b) => a.depth - b.depth);
  const success = track.result.success, pathColor = success ? '#258c64' : '#c2523d';
  let path = '';
  if (trail) {
    const points = track.frames.filter((fr) => !playback || fr.t <= time).map((fr) => project(new THREE.Vector3(fr.pos.x, fr.pos.y, fr.pos.z))).filter((p) => p[2] < 1);
    path = `<polyline points="${points.map(point).join(' ')}" fill="none" stroke="${pathColor}" stroke-width="2.6" stroke-linejoin="round" stroke-linecap="round"/>`;
    for (let i = 0; i < points.length; i += 15) path += `<circle cx="${f(points[i][0])}" cy="${f(points[i][1])}" r="2.4" fill="${pathColor}"/>`;
  }
  const labelMarkup = labels.map(({ text, pos }) => {
    const p = project(new THREE.Vector3(...pos));
    return p[2] < 1 ? `<text x="${f(p[0])}" y="${f(p[1])}" text-anchor="middle" font-size="12" font-weight="600" fill="#6c7c62">${esc(text)}</text>` : '';
  }).join('');
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${width}" height="${height}" viewBox="0 0 ${width} ${height}">
<rect width="100%" height="100%" fill="#f3f1ec"/>
${floor.join('')}${lines.join('')}${shadows.join('')}${triangles.map((t) => t.markup).join('')}${path}${labelMarkup}
<g font-family="DejaVu Sans, sans-serif">
<rect x="28" y="24" width="315" height="225" rx="16" fill="#fffdf7" fill-opacity="0.97" stroke="#d5d9d3"/>
<text x="51" y="52" font-size="11" letter-spacing="1.2" fill="#6c7f7c">GUSTY ROOM · FLIGHT REPLAY</text>
<text x="51" y="84" font-size="23" font-weight="bold" fill="#283b40">Three gates. One landing.</text>
<text x="51" y="116" font-size="14" font-weight="bold" fill="${pathColor}">${success ? 'PASS · Soft landing' : 'FAIL · ' + esc(track.result.reason)}</text>
<text x="51" y="145" font-size="12" fill="#5f7273">Seed ${track.seed} · ${track.result.gatesPassed}/3 gates</text>
<text x="51" y="167" font-size="12" fill="#5f7273">${track.result.simulatedSeconds.toFixed(2)} s flight · ${track.result.landingError.toFixed(3)} m pad error</text>
<path d="M51 182 H318" stroke="#dfe2da"/>
<circle cx="56" cy="204" r="4" fill="${pathColor}"/><text x="68" y="208" font-size="12" fill="#5f7273">${success ? 'Successful' : 'Failed'} recorded flight path</text>
<text x="51" y="232" font-size="10" fill="#7d8881">Software projection · simplified flight dynamics</text>
<rect x="28" y="${height - 64}" width="${width - 56}" height="40" rx="10" fill="#fffdf7" stroke="#d5d9d3"/>
<text x="48" y="${height - 39}" font-size="12" fill="#405c55">${esc(cameraName)} view · frame at ${time.toFixed(2)} s · Controller code never runs in the viewer</text>
</g></svg>`;
  await writeFile(svgFile, svg);
  await new Promise((done, fail) => {
    const child = spawn('python3', [fileURLToPath(new URL('./raster.py', import.meta.url)), svgFile, pngFile, String(width), String(height), ...(jpegFile ? [jpegFile] : [])], { stdio: 'inherit' });
    child.on('error', fail); child.on('exit', (code) => code === 0 ? done() : fail(new Error(`software raster exited ${code}`)));
  });
}

export async function softwareVideo(track, cameraName, file, options = {}) {
  const cache = resolve(homedir(), '.cache/ms-playwright');
  let ffmpeg = process.env.DRONE_FFMPEG || 'ffmpeg';
  if (!process.env.DRONE_FFMPEG) {
    for (const entry of (await readdir(cache).catch(() => [])).filter((s) => s.startsWith('ffmpeg-')).sort().reverse()) {
      const candidate = join(cache, entry, 'ffmpeg-linux');
      try { await access(candidate); ffmpeg = candidate; break; } catch { /* try the next installed binary */ }
    }
  }
  const scratchRoot = fileURLToPath(new URL('../../tmp/codex-drone/video-frames/', import.meta.url));
  await mkdir(scratchRoot, { recursive: true });
  const scratch = await mkdtemp(join(scratchRoot, 'frames-'));
  const svg = join(scratch, 'frame.svg'), png = join(scratch, 'frame.png'), jpeg = join(scratch, 'frame.jpg');
  const encoders = await new Promise((done, fail) => {
    const probe = spawn(ffmpeg, ['-hide_banner', '-encoders'], { stdio: ['ignore', 'pipe', 'ignore'] });
    let text = ''; probe.stdout.on('data', (chunk) => { text += chunk; });
    probe.on('error', fail); probe.on('exit', (code) => code === 0 ? done(text) : fail(new Error('cannot inspect video encoders')));
  });
  const codec = encoders.includes('libvpx-vp9') ? 'libvpx-vp9' : 'libvpx';
  const encoder = spawn(ffmpeg, ['-hide_banner', '-loglevel', 'error', '-y', '-f', 'image2pipe', '-c:v', 'mjpeg', '-r', '30', '-i', 'pipe:0',
    '-c:v', codec, '-b:v', '1500k', '-deadline', 'realtime', '-threads', '2', file], { stdio: ['pipe', 'ignore', 'inherit'] });
  let encoderError;
  const completed = new Promise((done, fail) => {
    encoder.on('error', (error) => { encoderError = error; fail(error); });
    encoder.on('exit', (code) => code === 0 ? done() : fail(new Error(`video encoder exited ${code}`)));
    encoder.stdin.on('error', (error) => { encoderError = error; });
  });
  // A rejected encoder is checked during rendering and awaited below.
  completed.catch(() => {});
  try {
    const end = track.frames.at(-1).t, count = Math.ceil(end * 30) + 1;
    for (let index = 0; index < count; index++) {
      if (encoderError) throw encoderError;
      await softwareShot(track, cameraName, svg, png, { ...options, at: Math.min(end, index / 30), playback: true, jpegFile: jpeg });
      const bytes = await readFile(jpeg);
      if (!encoder.stdin.write(bytes)) await Promise.race([once(encoder.stdin, 'drain'), completed.then(() => { throw new Error('video encoder closed early'); })]);
      if (index % 150 === 0) console.error(`Software video: frame ${index + 1}/${count}`);
    }
    encoder.stdin.end(); await completed;
    return { frames: count, fps: 30, seconds: count / 30 };
  } finally {
    encoder.kill('SIGKILL'); await rm(scratch, { recursive: true, force: true });
  }
}
