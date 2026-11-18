
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { DronePhysics } from './flight_physics.js';
import { FlightController } from './flight_controller.js';

const BUILD_VERSION = 'build-2026-11-18-physics-pass-1';
console.log('[AETHON-X] game.js', BUILD_VERSION);

const AXIS = {
  yawSign: -1,     
  pitchSign: 1,     
  rollSign: -1,     
  worldYSign: -1,   
  armRestDown: new THREE.Vector3(0, -1, 0), 
};


function makeNoise2D(seed = 1337) {
  const perm = new Uint8Array(512);
  let s = seed >>> 0;
  const rand = () => { s = (s * 1664525 + 1013904223) >>> 0; return s / 4294967296; };
  const p = Array.from({ length: 256 }, (_, i) => i);
  for (let i = 255; i > 0; i--) {
    const j = Math.floor(rand() * (i + 1));
    [p[i], p[j]] = [p[j], p[i]];
  }
  for (let i = 0; i < 512; i++) perm[i] = p[i & 255];

  function fade(t) { return t * t * t * (t * (t * 6 - 15) + 10); }
  function lerp(a, b, t) { return a + t * (b - a); }
  function grad(hash, x, y) {
    const h = hash & 3;
    const u = h < 2 ? x : y, v = h < 2 ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2 * v : 2 * v);
  }
  function noise(x, y) {
    const X = Math.floor(x) & 255, Y = Math.floor(y) & 255;
    x -= Math.floor(x); y -= Math.floor(y);
    const u = fade(x), v = fade(y);
    const aa = perm[X + perm[Y]], ab = perm[X + perm[Y + 1]];
    const ba = perm[X + 1 + perm[Y]], bb = perm[X + 1 + perm[Y + 1]];
    return lerp(
      lerp(grad(aa, x, y), grad(ba, x - 1, y), u),
      lerp(grad(ab, x, y - 1), grad(bb, x - 1, y - 1), u), v);
  }
  return function fbm(x, y, octaves = 4, lacunarity = 2.1, gain = 0.5) {
    let amp = 1, freq = 1, sum = 0, norm = 0;
    for (let i = 0; i < octaves; i++) {
      sum += amp * noise(x * freq, y * freq);
      norm += amp;
      amp *= gain; freq *= lacunarity;
    }
    return sum / norm;
  };
}

const RUN = {
  baseSpeed: 15,         
  maxSpeed: 50,           
  accelPerMeter: 0.010,   
  laneRangeX: 11,         
  altMin: 3, altMax: 26,  
  chunkLength: 40,        
  chunksAhead: 6,         
  chunksBehind: 2,        
  corridorHalf: 15,       
  steerAccel: 85,         
  steerDampPerSec: 4.2,   
                           
  obstacleMaxYHeadroom: 4,
};

const noise = makeNoise2D(2026);

const grainNoise = makeNoise2D(9001);

function smoothstep(edge0, edge1, x) {
  const t = THREE.MathUtils.clamp((x - edge0) / (edge1 - edge0), 0, 1);
  return t * t * (3 - 2 * t);
}

function surfaceGrain(x, z) {
  const coarse = grainNoise(x * 0.05, z * 0.05, 2);   
  const fine = grainNoise(x * 0.6 + 50, z * 0.6 + 50, 2); 
  return { patch: coarse, speckle: fine };
}

function hashSeed(str, index) {
  let h = 2166136261 ^ index;
  for (let i = 0; i < str.length; i++) {
    h = Math.imul(h ^ str.charCodeAt(i), 16777619);
  }
  return h >>> 0;
}
function mulberry32(seed) {
  let a = seed >>> 0;
  return function () {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function disposeGroup(group) {
  group.traverse((o) => {
    if (o.isMesh) {
      o.geometry?.dispose();
      if (Array.isArray(o.material)) o.material.forEach((m) => m.dispose());
      else o.material?.dispose();
    }
  });
}

const LANE_SLOTS = [-8, 0, 8];

const ENEMY_CLOTH_MAT = new THREE.MeshStandardMaterial({ color: 0x111214, roughness: 0.88, metalness: 0.04 });
const ENEMY_GEAR_MAT = new THREE.MeshStandardMaterial({ color: 0x0a0a0c, roughness: 0.55, metalness: 0.35 });

function createEnemyMesh(rng) {
  const g = new THREE.Group();
  const legH = 0.82, torsoH = 0.66, headR = 0.155;

  const legGeo = new THREE.BoxGeometry(0.16, legH, 0.16);
  const legL = new THREE.Mesh(legGeo, ENEMY_CLOTH_MAT); legL.position.set(-0.11, legH / 2, 0);
  const legR = new THREE.Mesh(legGeo, ENEMY_CLOTH_MAT); legR.position.set(0.11, legH / 2, 0);
  g.add(legL, legR);

  const torso = new THREE.Mesh(new THREE.BoxGeometry(0.44, torsoH, 0.24), ENEMY_CLOTH_MAT);
  torso.position.set(0, legH + torsoH / 2, 0);
  g.add(torso);

  const head = new THREE.Mesh(new THREE.SphereGeometry(headR, 10, 8), ENEMY_CLOTH_MAT);
  head.position.set(0, legH + torsoH + headR, 0);
  g.add(head);

  const armGeo = new THREE.BoxGeometry(0.13, 0.58, 0.13);
  const armL = new THREE.Mesh(armGeo, ENEMY_CLOTH_MAT);
  armL.position.set(-0.32, legH + torsoH - 0.28, 0.04); armL.rotation.z = 0.18;
  const armR = new THREE.Mesh(armGeo, ENEMY_CLOTH_MAT);
  armR.position.set(0.32, legH + torsoH - 0.28, 0.04); armR.rotation.z = -0.18;
  g.add(armL, armR);

  const rifle = new THREE.Mesh(new THREE.BoxGeometry(0.07, 0.07, 0.92), ENEMY_GEAR_MAT);
  rifle.position.set(0.24, legH + torsoH - 0.1, 0.12);
  rifle.rotation.x = 0.3 + rng() * 0.1;
  g.add(rifle);

  g.traverse((o) => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; } });


  g.scale.setScalar(10);

  g.userData.isEnemy = true;
  return g;
}

function makeGroundStrip(zStart, zEnd, heightFn, colorFn) {
  const length = Math.abs(zEnd - zStart);

  const segsZ = 30, segsX = 36;
  const geo = new THREE.PlaneGeometry(RUN.corridorHalf * 2.6, length, segsX, segsZ);
  geo.rotateX(-Math.PI / 2);
  const zMid = (zStart + zEnd) / 2;
  geo.translate(0, 0, zMid);
  const pos = geo.attributes.position;
  const colors = new Float32Array(pos.count * 3);
  const tmpColor = new THREE.Color();
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i), z = pos.getZ(i);
    const y = heightFn(x, z);
    pos.setY(i, y);
    const c = colorFn(x, z, y);

    const { patch, speckle } = surfaceGrain(x, z);
    tmpColor.copy(c);
    const patchShade = 1 + patch * 0.08;       
    const speckleShade = 1 + speckle * 0.05;   
    tmpColor.r = THREE.MathUtils.clamp(tmpColor.r * patchShade * speckleShade, 0, 1);
    tmpColor.g = THREE.MathUtils.clamp(tmpColor.g * patchShade * speckleShade, 0, 1);
    tmpColor.b = THREE.MathUtils.clamp(tmpColor.b * patchShade * speckleShade, 0, 1);
    colors[i * 3] = tmpColor.r; colors[i * 3 + 1] = tmpColor.g; colors[i * 3 + 2] = tmpColor.b;
  }
  geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  geo.computeVertexNormals();
  const mat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.92, metalness: 0.03 });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.receiveShadow = true;
  return mesh;
}

function pickChunkSlots(rng) {

  const count = rng() < 0.28 ? 0 : (rng() < 0.6 ? 1 : 2);
  const order = [0, 1, 2].sort(() => rng() - 0.5);
  return order.slice(0, count);
}

const TERRAIN_FLAVORS = {

  mountains: {
    name: 'Mountains', seed: 'mountains',
    fog: new THREE.FogExp2(0x9db4c9, 0.0022), sky: 0xaecbe0,
    baseline(distance) { return noise(distance * 0.004, 11, 3) * 9; },
    build(group, obstacles, index, zStart, zEnd, rng) {
      const base = this.baseline((zStart + zEnd) / -2);
      const rockColor = new THREE.Color(0x6b5f52), snowColor = new THREE.Color(0xf2f5f8),
            valleyColor = new THREE.Color(0x4a5a3d);
      const heightFn = (x, z) => {
        const dist = -z;
        const b = noise(dist * 0.004, 11, 3) * 9;
        const detail = noise(x * 0.05 + dist * 0.02, dist * 0.05, 3) * 3.2;
        const wallRise = Math.max(0, (Math.abs(x) - RUN.corridorHalf * 0.9)) * 0.9;
        
        const scree = noise(x * 0.4 + dist * 0.3, dist * 0.4 - x * 0.1, 2) * 0.45;
        return b - 5 + detail + wallRise + scree;
      };
      const colorFn = (x, z, y) => {
        const t = THREE.MathUtils.clamp((y - base + 5) / 14, 0, 1);
        const snowT = smoothstep(0.68, 0.82, t);
        const rockT = smoothstep(0.26, 0.42, t);
        const col = valleyColor.clone().lerp(rockColor, rockT).lerp(snowColor, snowT);

        const patch = noise(x * 0.15, z * 0.15, 2);
        if (t < 0.72 && patch > 0.32) col.lerp(rockColor, 0.4);
        if (snowT > 0.15 && snowT < 0.9 && patch < -0.3) col.lerp(rockColor, 0.25);
        return col;
      };
      group.add(makeGroundStrip(zStart, zEnd, heightFn, colorFn));


      const pebbleMat = new THREE.MeshStandardMaterial({ color: 0x8a7c6a, roughness: 1 });
      const scrubMat = new THREE.MeshStandardMaterial({ color: 0x5c6a3f, roughness: 1 });
      for (let i = 0; i < 10; i++) {
        const px = (rng() - 0.5) * RUN.corridorHalf * 2.4;
        const pz = zStart - rng() * RUN.chunkLength;
        const ph = heightFn(px, pz);
        if (rng() < 0.7) {
          const geo = new THREE.DodecahedronGeometry(0.25 + rng() * 0.4, 0);
          const mesh = new THREE.Mesh(geo, pebbleMat);
          mesh.position.set(px, ph + 0.15, pz);
          mesh.rotation.set(rng() * Math.PI, rng() * Math.PI, rng() * Math.PI);
          group.add(mesh);
        } else {
          const geo = new THREE.ConeGeometry(0.3 + rng() * 0.2, 0.6 + rng() * 0.5, 5);
          const mesh = new THREE.Mesh(geo, scrubMat);
          mesh.position.set(px, ph + 0.3, pz);
          group.add(mesh);
        }
      }

      if (index < 2) return; 
      const rockMat = new THREE.MeshStandardMaterial({ color: 0x746355, roughness: 0.95 });
      for (const slotX of pickChunkSlots(rng)) {
        const laneX = LANE_SLOTS[slotX] + (rng() - 0.5) * 3;
        const h = 14 + rng() * 8;
        const geo = new THREE.ConeGeometry(2.6 + rng() * 1.2, h, 6);
        const mesh = new THREE.Mesh(geo, rockMat);
        const zCenter = zStart - RUN.chunkLength * (0.25 + rng() * 0.5);
        mesh.position.set(laneX, base + h / 2 - 4, zCenter);
        mesh.rotation.y = rng() * Math.PI;
        mesh.castShadow = true;
        group.add(mesh);
        obstacles.push({
          x: laneX, zCenter, halfLen: 3, radiusX: 3,
          minY: base - 5, maxY: Math.min(base + h - 4, RUN.altMax - RUN.obstacleMaxYHeadroom),
          label: 'ROCK SPIRE STRUCK',
        });
      }
      if (rng() < 0.3) {
        const archMat = new THREE.MeshStandardMaterial({ color: 0x8a7a68, roughness: 0.9 });
        const midY = base + (RUN.altMin + RUN.altMax) / 2;
        const geo = new THREE.BoxGeometry(RUN.corridorHalf * 1.6, 3.2, 3.2);
        const mesh = new THREE.Mesh(geo, archMat);
        const zCenter = zStart - RUN.chunkLength * 0.75;
        mesh.position.set(0, midY, zCenter);
        mesh.castShadow = true;
        group.add(mesh);
        obstacles.push({
          x: 0, zCenter, halfLen: 2, radiusX: RUN.corridorHalf * 0.8,
          minY: midY - RUN.altMin, maxY: midY - RUN.altMin + 3.2,
          label: 'STONE ARCH STRUCK',
        });
      }
      const bgMat = new THREE.MeshStandardMaterial({ color: 0x8fa2ab, roughness: 1 });
      for (let i = 0; i < 2; i++) {
        const side = i === 0 ? -1 : 1;
        const bx = side * (RUN.corridorHalf * 2.4 + rng() * 40);
        const bh = 60 + rng() * 90;
        const geo = new THREE.ConeGeometry(30 + rng() * 20, bh, 5);
        const mesh = new THREE.Mesh(geo, bgMat);
        mesh.position.set(bx, base + bh / 2 - 20, (zStart + zEnd) / 2);
        group.add(mesh);
      }
    },
  },

  city: {
    name: 'City', seed: 'city',
    fog: new THREE.FogExp2(0x1b2028, 0.0042), sky: 0x28313c,
    baseline() { return 0; },
    build(group, obstacles, index, zStart, zEnd, rng) {
      const streetColor = new THREE.Color(0x2a2f36), stripeColor = new THREE.Color(0xd9c878),
            sidewalkColor = new THREE.Color(0x4a4f56), curbColor = new THREE.Color(0x6a6f76);
      const sidewalkEdge = RUN.corridorHalf * 0.78;

      const heightFn = (x) => {
        const d = Math.abs(x) - sidewalkEdge;
        if (d < 0) return 0;
        return Math.min(0.22, d * 1.2);
      };
      const colorFn = (x, z) => {
        const ax = Math.abs(x);
        if (ax > sidewalkEdge) {
          const slab = (Math.abs(Math.sin(x * 0.9)) < 0.06 || Math.abs(Math.sin(z * 0.5)) < 0.04);
          return slab ? curbColor : sidewalkColor;
        }
        if (ax < 0.6) {
          return (Math.sin(z * 0.55) > 0.35) ? stripeColor : streetColor;
        }
        const wear = noise(x * 0.08, z * 0.03, 2);
        return wear > 0.3 ? streetColor.clone().multiplyScalar(0.85) : streetColor;
      };
      group.add(makeGroundStrip(zStart, zEnd, heightFn, colorFn));

      const poleMat = new THREE.MeshStandardMaterial({ color: 0x2c2f33, roughness: 0.6, metalness: 0.4 });
      const lampMat = new THREE.MeshStandardMaterial({ color: 0xffe9b0, emissive: 0xffb84d, emissiveIntensity: 0.9, roughness: 0.4 });
      for (let i = 0; i < 2; i++) {
        const side = i === 0 ? -1 : 1;
        const lx = side * (sidewalkEdge + 1.2);
        const lz = zStart - RUN.chunkLength * (0.3 + rng() * 0.4);
        const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.09, 0.12, 5.5, 6), poleMat);
        pole.position.set(lx, 2.75, lz);
        group.add(pole);
        const lamp = new THREE.Mesh(new THREE.SphereGeometry(0.28, 8, 8), lampMat);
        lamp.position.set(lx, 5.4, lz);
        group.add(lamp);
      }

      if (index < 2) return;
      const buildingMat = new THREE.MeshStandardMaterial({ color: 0x59626e, roughness: 0.7, metalness: 0.15 });
      const winMat = new THREE.MeshStandardMaterial({ color: 0xffd27a, emissive: 0x553300, roughness: 0.4 });
      for (const slotX of pickChunkSlots(rng)) {
        const laneX = LANE_SLOTS[slotX] + (rng() - 0.5) * 2;
        const h = 12 + rng() * 20;
        const w = 4 + rng() * 2.5;
        const geo = new THREE.BoxGeometry(w, h, w);
        const mesh = new THREE.Mesh(geo, rng() < 0.5 ? buildingMat : winMat);
        const zCenter = zStart - RUN.chunkLength * (0.25 + rng() * 0.5);
        mesh.position.set(laneX, h / 2, zCenter);
        mesh.castShadow = true; mesh.receiveShadow = true;
        group.add(mesh);
        if (rng() < 0.5) {
          const acGeo = new THREE.BoxGeometry(w * 0.35, 0.9, w * 0.35);
          const ac = new THREE.Mesh(acGeo, buildingMat);
          ac.position.set(laneX + w * 0.2, h + 0.5, zCenter);
          group.add(ac);
        }

        {
          const winRows = Math.max(2, Math.floor(h / 3));
          const winCols = Math.max(2, Math.floor(w / 1.3));
          const winGeo = new THREE.BoxGeometry(Math.min(0.55, w / winCols * 0.55), 0.7, 0.05);
          const winInst = new THREE.InstancedMesh(winGeo, winMat, winRows * winCols * 2);
          const dummy = new THREE.Object3D();
          let wi = 0;
          for (let r = 0; r < winRows; r++) {
            for (let c = 0; c < winCols; c++) {
              if (rng() < 0.18) continue; // a few dark/unlit windows
              const wx = laneX + (c - (winCols - 1) / 2) * (w / winCols);
              const wy = 1.4 + r * ((h - 2) / winRows);
              dummy.position.set(wx, wy, zCenter + w / 2 + 0.03);
              dummy.updateMatrix(); winInst.setMatrixAt(wi++, dummy.matrix);
              dummy.position.set(wx, wy, zCenter - w / 2 - 0.03);
              dummy.updateMatrix(); winInst.setMatrixAt(wi++, dummy.matrix);
            }
          }
          winInst.count = wi;
          group.add(winInst);
        }
        obstacles.push({
          x: laneX, zCenter, halfLen: w / 2 + 0.6, radiusX: w / 2 + 0.6,
          minY: 0, maxY: Math.min(h, RUN.altMax - RUN.obstacleMaxYHeadroom),
          label: 'BUILDING STRUCK',
        });
      }
      if (rng() < 0.32) {
        const bridgeMat = new THREE.MeshStandardMaterial({ color: 0x707a86, roughness: 0.6, metalness: 0.3 });
        const midY = (RUN.altMin + RUN.altMax) / 2;
        const geo = new THREE.BoxGeometry(RUN.corridorHalf * 1.7, 2.4, 3.2);
        const mesh = new THREE.Mesh(geo, bridgeMat);
        const zCenter = zStart - RUN.chunkLength * 0.75;
        mesh.position.set(0, midY, zCenter);
        mesh.castShadow = true;
        group.add(mesh);
        obstacles.push({
          x: 0, zCenter, halfLen: 2, radiusX: RUN.corridorHalf * 0.85,
          minY: midY - 1.2, maxY: midY + 1.2,
          label: 'SKY-BRIDGE STRUCK',
        });
      }
      const bgMat = new THREE.MeshStandardMaterial({ color: 0x3a4552, roughness: 0.9 });
      for (let i = 0; i < 3; i++) {
        const side = rng() < 0.5 ? -1 : 1;
        const bx = side * (RUN.corridorHalf * 1.9 + rng() * 30);
        const bh = 20 + rng() * 50;
        const geo = new THREE.BoxGeometry(8 + rng() * 6, bh, 8 + rng() * 6);
        const mesh = new THREE.Mesh(geo, bgMat);
        mesh.position.set(bx, bh / 2, zStart - rng() * RUN.chunkLength);
        group.add(mesh);
      }
    },
  },

  forest: {
    name: 'Forest', seed: 'forest',
    fog: new THREE.FogExp2(0x203524, 0.0038), sky: 0x4a6b52,
    baseline(distance) { return noise(distance * 0.012, 5) * 4; },
    build(group, obstacles, index, zStart, zEnd, rng) {
      const groundColor = new THREE.Color(0x2e4a2a), mossColor = new THREE.Color(0x3a5a34),
            litterColor = new THREE.Color(0x4a3d24), dirtColor = new THREE.Color(0x40331f);
      const heightFn = (x, z) => {
        const rolling = this.baseline(-z) + noise(x * 0.06, -z * 0.06, 3) * 1.4;
        const litter = noise(x * 0.5 - z * 0.2, z * 0.5, 2) * 0.3;
        return rolling + litter;
      };
      const colorFn = (x, z, y) => {
        const mossT = smoothstep(0.25, 0.55, Math.sin(x * 0.7 + y));
        let col = groundColor.clone().lerp(mossColor, mossT);
        const patch = noise(x * 0.1, z * 0.1, 2);
        if (patch > 0.38) col.lerp(litterColor, 0.5);
        else if (patch < -0.42) col.lerp(dirtColor, 0.4);
        return col;
      };
      group.add(makeGroundStrip(zStart, zEnd, heightFn, colorFn));


      const bushMat = new THREE.MeshStandardMaterial({ color: 0x2c4a28, roughness: 1 });
      const rockMat2 = new THREE.MeshStandardMaterial({ color: 0x555248, roughness: 1 });
      for (let i = 0; i < 8; i++) {
        const px = (rng() - 0.5) * RUN.corridorHalf * 2.2;
        const pz = zStart - rng() * RUN.chunkLength;
        const ph = heightFn(px, pz);
        if (rng() < 0.65) {
          const geo = new THREE.SphereGeometry(0.5 + rng() * 0.5, 6, 5);
          const mesh = new THREE.Mesh(geo, bushMat);
          mesh.position.set(px, ph + 0.4, pz);
          mesh.scale.y = 0.7;
          group.add(mesh);
        } else {
          const geo = new THREE.DodecahedronGeometry(0.2 + rng() * 0.3, 0);
          const mesh = new THREE.Mesh(geo, rockMat2);
          mesh.position.set(px, ph + 0.15, pz);
          group.add(mesh);
        }
      }

      if (index < 2) return;
      const base = this.baseline((zStart + zEnd) / -2);
      const trunkMat = new THREE.MeshStandardMaterial({ color: 0x4b3626, roughness: 0.9 });
      const leafMats = [0x2f6b32, 0x356b2f, 0x2a5e2e].map(
        (c) => new THREE.MeshStandardMaterial({ color: c, roughness: 0.85 }));
      for (const slotX of pickChunkSlots(rng)) {
        const laneX = LANE_SLOTS[slotX] + (rng() - 0.5) * 3;
        const scale = 0.9 + rng() * 0.7;
        const zCenter = zStart - RUN.chunkLength * (0.2 + rng() * 0.6);
        const trunk = new THREE.Mesh(new THREE.CylinderGeometry(0.5, 0.7, 6, 7), trunkMat);
        trunk.position.set(laneX, base + 3 * scale, zCenter);
        trunk.scale.set(scale, scale, scale);
        trunk.castShadow = true;
        group.add(trunk);
        const leaves = new THREE.Mesh(new THREE.ConeGeometry(4.2, 11, 8),
          leafMats[Math.floor(rng() * leafMats.length)]);
        leaves.position.set(laneX, base + 9.5 * scale, zCenter);
        leaves.scale.set(scale, scale, scale);
        leaves.castShadow = true;
        group.add(leaves);
        obstacles.push({
          x: laneX, zCenter, halfLen: 2.8 * scale, radiusX: 2.6 * scale,
          minY: base - 2, maxY: Math.min(base + 15 * scale, RUN.altMax - RUN.obstacleMaxYHeadroom),
          label: 'TREE STRUCK',
        });
      }
      if (rng() < 0.3) {
        const logMat = new THREE.MeshStandardMaterial({ color: 0x5a4030, roughness: 0.9 });
        const geo = new THREE.CylinderGeometry(1.2, 1.2, RUN.corridorHalf * 1.5, 8);
        geo.rotateZ(Math.PI / 2);
        const mesh = new THREE.Mesh(geo, logMat);
        const zCenter = zStart - RUN.chunkLength * 0.7;
        mesh.position.set(0, base + 2.2, zCenter);
        mesh.castShadow = true;
        group.add(mesh);
        obstacles.push({
          x: 0, zCenter, halfLen: 1.4, radiusX: RUN.corridorHalf * 0.75,
          minY: base + 0.8, maxY: base + 3.6,
          label: 'FALLEN LOG STRUCK',
        });
      }
      const bgTrunk = trunkMat, bgLeaf = leafMats[0];
      for (let i = 0; i < 4; i++) {
        const side = i % 2 === 0 ? -1 : 1;
        const bx = side * (RUN.corridorHalf * 1.5 + rng() * 20);
        const bz = zStart - rng() * RUN.chunkLength;
        const s = 0.8 + rng() * 0.8;
        const t = new THREE.Mesh(new THREE.CylinderGeometry(0.5, 0.7, 6, 6), bgTrunk);
        t.position.set(bx, base + 3 * s, bz); t.scale.setScalar(s);
        const l = new THREE.Mesh(new THREE.ConeGeometry(4, 10, 6), bgLeaf);
        l.position.set(bx, base + 9 * s, bz); l.scale.setScalar(s);
        group.add(t, l);
      }
    },
  },

  lake: {
    name: 'Lake', seed: 'lake',
    fog: new THREE.FogExp2(0x8fb0c0, 0.0026), sky: 0xbcd8e6,
    baseline() { return 0; },
    build(group, obstacles, index, zStart, zEnd, rng) {
      const t = performance.now() / 1000;
      const shoreColor = new THREE.Color(0x5a7a3f), waterEdge = new THREE.Color(0x1c5f8a),
            waterDeep = new THREE.Color(0x123f5c), foamColor = new THREE.Color(0xdcf0f5),
            sandColor = new THREE.Color(0x8a7c56);
      const shoreLine = RUN.corridorHalf * 0.72;
      const heightFn = (x, z) => {
        const edge = Math.abs(x) - shoreLine;
        if (edge < 0) {

          const swell = Math.sin(z * 0.25 + t * 0.6) * 0.08;
          const chop = noise(x * 0.4 + t * 0.3, z * 0.4 - t * 0.25, 2) * 0.05;
          return -0.4 + swell + chop;
        }
        return edge * 0.6 + noise(x * 0.05, z * 0.05, 2) * 2 + noise(x * 0.5, z * 0.5, 2) * 0.2;
      };
      const colorFn = (x) => {
        const edge = Math.abs(x) - shoreLine;
        if (edge < 0) {

          const depthT = smoothstep(0, shoreLine * 0.8, shoreLine - Math.abs(x));
          const col = waterEdge.clone().lerp(waterDeep, depthT);
          if (edge > -0.9) col.lerp(foamColor, smoothstep(-0.9, -0.1, edge));
          return col;
        }
        const sandT = smoothstep(0, 2.5, edge);
        return sandColor.clone().lerp(shoreColor, sandT);
      };
      const waterMesh = makeGroundStrip(zStart, zEnd, heightFn, colorFn);
      waterMesh.material.transparent = true;
      waterMesh.material.opacity = 0.9;
      waterMesh.material.roughness = 0.2;
      waterMesh.material.metalness = 0.25;
      group.add(waterMesh);

      const padMat = new THREE.MeshStandardMaterial({ color: 0x2f6b3a, roughness: 0.8 });
      const reedMat = new THREE.MeshStandardMaterial({ color: 0x5c7a3a, roughness: 0.9 });
      for (let i = 0; i < 6; i++) {
        const side = rng() < 0.5 ? -1 : 1;
        const px = side * (shoreLine - 1.5 - rng() * 2.5);
        const pz = zStart - rng() * RUN.chunkLength;
        if (rng() < 0.5) {
          const pad = new THREE.Mesh(new THREE.CylinderGeometry(0.4 + rng() * 0.3, 0.4 + rng() * 0.3, 0.05, 8), padMat);
          pad.position.set(px, -0.35, pz);
          group.add(pad);
        } else {
          const reed = new THREE.Mesh(new THREE.ConeGeometry(0.06, 1.2 + rng() * 0.8, 4), reedMat);
          reed.position.set(px, 0.3, pz);
          group.add(reed);
        }
      }

      if (index < 2) return;
      const rockMat = new THREE.MeshStandardMaterial({ color: 0x6b6355, roughness: 0.9 });
      for (const slotX of pickChunkSlots(rng)) {
        const laneX = LANE_SLOTS[slotX] + (rng() - 0.5) * 2.5;
        const h = 8 + rng() * 8;
        const geo = new THREE.CylinderGeometry(1.4 + rng(), 2.2 + rng(), h, 7);
        const mesh = new THREE.Mesh(geo, rockMat);
        const zCenter = zStart - RUN.chunkLength * (0.25 + rng() * 0.5);
        mesh.position.set(laneX, h / 2 - 1, zCenter);
        mesh.castShadow = true;
        group.add(mesh);
        obstacles.push({
          x: laneX, zCenter, halfLen: 2.4, radiusX: 2.4,
          minY: -2, maxY: Math.min(h - 1, RUN.altMax - RUN.obstacleMaxYHeadroom),
          label: 'ROCK PILLAR STRUCK',
        });
      }
      if (rng() < 0.3) {
        const bridgeMat = new THREE.MeshStandardMaterial({ color: 0x6b4a30, roughness: 0.85 });
        const midY = 6 + rng() * 4;
        const geo = new THREE.BoxGeometry(RUN.corridorHalf * 1.6, 1.6, 3);
        const mesh = new THREE.Mesh(geo, bridgeMat);
        const zCenter = zStart - RUN.chunkLength * 0.75;
        mesh.position.set(0, midY, zCenter);
        mesh.castShadow = true;
        group.add(mesh);
        obstacles.push({
          x: 0, zCenter, halfLen: 2, radiusX: RUN.corridorHalf * 0.82,
          minY: midY - 0.8, maxY: midY + 0.8,
          label: 'BRIDGE STRUCK',
        });
      }
      const bgMat = new THREE.MeshStandardMaterial({ color: 0x6f8f56, roughness: 1 });
      for (let i = 0; i < 2; i++) {
        const side = i === 0 ? -1 : 1;
        const bx = side * (RUN.corridorHalf * 2.2 + rng() * 25);
        const bh = 14 + rng() * 20;
        const geo = new THREE.ConeGeometry(18 + rng() * 10, bh, 6);
        const mesh = new THREE.Mesh(geo, bgMat);
        mesh.position.set(bx, bh / 2 - 4, (zStart + zEnd) / 2);
        group.add(mesh);
      }
    },
  },
};


class EndlessTrack {
  constructor(scene, flavor) {
    this.scene = scene;
    this.flavor = flavor;
    this.chunks = [];
    this.nextIndex = 0;
    this._fill(0);
  }
  baselineY(distance) { return this.flavor.baseline(distance); }
  reset() {
    for (const c of this.chunks) { this.scene.remove(c.group); disposeGroup(c.group); }
    this.chunks = [];
    this.nextIndex = 0;
    this._fill(0);
  }
  _fill(distance) {
    const aheadIdx = Math.floor(distance / RUN.chunkLength) + RUN.chunksAhead;
    while (this.nextIndex <= aheadIdx) this._genChunk(this.nextIndex++);
  }
  _genChunk(index) {
    const zStart = -index * RUN.chunkLength;
    const zEnd = -(index + 1) * RUN.chunkLength;
    const rng = mulberry32(hashSeed(this.flavor.seed, index));
    const group = new THREE.Group();
    const obstacles = [];
    this.flavor.build(group, obstacles, index, zStart, zEnd, rng);

    const enemies = [];
    if (index >= 2) {
      const tryCount = rng() < 0.35 ? 0 : (rng() < 0.7 ? 1 : 2);
      for (let i = 0; i < tryCount; i++) {
        const zCenter = zStart - RUN.chunkLength * (0.15 + rng() * 0.7);
        const ex = (rng() - 0.5) * RUN.corridorHalf * 1.7;
        const blocked = obstacles.some((ob) =>
          Math.abs(ob.zCenter - zCenter) < ob.halfLen + 2 && Math.abs(ob.x - ex) < ob.radiusX + 1.5);
        if (blocked) continue;
        const ey = this.baselineY(-zCenter);
        const mesh = createEnemyMesh(rng);
        mesh.position.set(ex, ey, zCenter);
        mesh.rotation.y = rng() * Math.PI * 2;
        group.add(mesh);
        enemies.push({ mesh, x: ex, zCenter, alive: true });
      }
    }

    this.scene.add(group);
    this.chunks.push({ index, zStart, zEnd, group, obstacles, enemies });
  }
  update(distance) {
    this._fill(distance);
    const minKeepIndex = Math.floor(distance / RUN.chunkLength) - RUN.chunksBehind;
    while (this.chunks.length && this.chunks[0].index < minKeepIndex) {
      const c = this.chunks.shift();
      this.scene.remove(c.group);
      disposeGroup(c.group);
    }
  }
  checkCollision(dronePos) {
    for (const c of this.chunks) {
      for (const ob of c.obstacles) {
        if (Math.abs(ob.zCenter - dronePos.z) > ob.halfLen + 1.3) continue;
        if (Math.abs(dronePos.x - ob.x) < ob.radiusX + 1.3
            && dronePos.y > ob.minY && dronePos.y < ob.maxY) {
          return ob.label || 'OBSTACLE STRUCK';
        }
      }
    }
    return null;
  }

  checkEnemyMiss(dronePos, onMiss) {
    for (const c of this.chunks) {
      for (const en of c.enemies) {
        if (en.alive && dronePos.z < en.zCenter - 3) {
          en.alive = false;
          en.mesh.visible = false;
          onMiss(en);
        }
      }
    }
  }

  liveEnemyMeshes() {
    const meshes = [];
    for (const c of this.chunks) for (const en of c.enemies) if (en.alive) meshes.push(en.mesh);
    return meshes;
  }
  enemyForMesh(mesh) {
    for (const c of this.chunks) for (const en of c.enemies) if (en.mesh === mesh) return en;
    return null;
  }
}

class MouseKeyboardSource {
  constructor() {
    this.state = { angleX: 0, angleY: 0, angleZ: 0, vertZ: 1, posX: 0, posY: 0, posZ: 0, connected: true };
    this.keys = {};
    window.addEventListener('keydown', (e) => { this.keys[e.code] = true; });
    window.addEventListener('keyup', (e) => { this.keys[e.code] = false; });
    this._targetPitch = 0; this._targetRoll = 0; this._yaw = 0;
  }
  update(dt) {
    const k = this.keys;
    const pitchIn = (k['KeyW'] || k['ArrowUp'] ? 1 : 0) - (k['KeyS'] || k['ArrowDown'] ? 1 : 0);
    const rollIn = (k['KeyD'] || k['ArrowRight'] ? 1 : 0) - (k['KeyA'] || k['ArrowLeft'] ? 1 : 0);
    const yawIn = (k['KeyE'] ? 1 : 0) - (k['KeyQ'] ? 1 : 0);
    const thrustIn = (k['Space'] ? 1 : 0) - (k['ShiftLeft'] || k['ShiftRight'] ? 1 : 0);
    const blend = 1 - Math.exp(-5 * dt);
    this._targetPitch = THREE.MathUtils.lerp(this._targetPitch, pitchIn * 18, blend);
    this._targetRoll = THREE.MathUtils.lerp(this._targetRoll, rollIn * 18, blend);
    this._yaw += yawIn * 55 * dt;
    this.state.angleX = this._targetPitch;
    this.state.angleY = this._targetRoll;
    this.state.angleZ = this._yaw;
    this.state.vertZ = 1 + thrustIn * 0.1;
    return this.state;
  }
  reset() { this._targetPitch = 0; this._targetRoll = 0; this._yaw = 0; this.state.angleX = this.state.angleY = this.state.angleZ = 0; }
}

class BridgeSource {

  constructor(url) {
    this.state = { angleX: 0, angleY: 0, angleZ: 0, vertZ: 1, posX: 0, posY: 0, posZ: 0, connected: false };
    this.url = url;
    this._connect();
  }
  _connect() {
    try {
      this.ws = new WebSocket(this.url);
      this.ws.onmessage = (ev) => {
        try {
          const d = JSON.parse(ev.data);
          Object.assign(this.state, d);
        } catch (e) { /* ignore malformed frame */ }
      };
      this.ws.onclose = () => { this.state.connected = false; setTimeout(() => this._connect(), 1500); };
      this.ws.onerror = () => { this.state.connected = false; };
    } catch (e) {
      this.state.connected = false;
      setTimeout(() => this._connect(), 1500);
    }
  }
  update(_dt) { return this.state; }
  reset(_spawn) { /* position is hardware-driven — nothing to reset locally */ }
}

class PalmAimSource {
  constructor(url) {
    this.state = { connected: false, handFound: false, lookYaw: 0, lookPitch: 0 };
    this._pendingFires = 0;
    this.url = url;

    this._previewImg = document.getElementById('palmPreview');
    this._previewWrap = document.getElementById('palmPreviewWrap');
    if (this._previewWrap) this._previewWrap.style.display = 'block';
    this._connect();
  }
  _connect() {
    try {
      this.ws = new WebSocket(this.url);
      this.ws.onmessage = (ev) => {
        try {
          const d = JSON.parse(ev.data);
          this.state.connected = true;
          if (this._previewWrap) this._previewWrap.style.display = 'block';
          this.state.handFound = !!d.handFound;

          if (d.handFound) {
            this.state.lookYaw = d.lookYaw;
            this.state.lookPitch = d.lookPitch;
          }
          if (d.fire) this._pendingFires++;

          if (d.frame && this._previewImg) {
            this._previewImg.src = 'data:image/jpeg;base64,' + d.frame;
          }
        } catch (e) { /* ignore malformed frame */ }
      };
      this.ws.onclose = () => {
        this.state.connected = false;
        this.state.handFound = false;
        if (this._previewWrap) this._previewWrap.style.display = 'none';
        setTimeout(() => this._connect(), 1500);
      };
      this.ws.onerror = () => { this.state.connected = false; };
    } catch (e) {
      this.state.connected = false;
      setTimeout(() => this._connect(), 1500);
    }
  }

  poll() {
    const fireEdges = this._pendingFires;
    this._pendingFires = 0;
    return {
      connected: this.state.connected,
      handFound: this.state.handFound,
      lookYaw: this.state.lookYaw,
      lookPitch: this.state.lookPitch,
      fireEdges,
    };
  }
}

class WebSerialSource {

  constructor() {
    this.state = { angleX: 0, angleY: 0, angleZ: 0, vertZ: 1, posX: 0, posY: 0, posZ: 0, connected: false };
    this.vel = { x: 0, y: 0, z: 0 };
    this._lastT = performance.now() / 1000;
    this._buf = '';
    this._connect();
  }
  async _connect() {
    if (!('serial' in navigator)) {
      console.warn('[WebSerial] navigator.serial unavailable — use Chrome/Edge over HTTPS or localhost.');
      return;
    }
    try {
      const port = await navigator.serial.requestPort();
      await port.open({ baudRate: 115200 });
      this.state.connected = true;
      const decoder = new TextDecoderStream();
      port.readable.pipeTo(decoder.writable);
      const reader = decoder.readable.getReader();
      this._pump(reader);
    } catch (e) {
      console.warn('[WebSerial] connection failed:', e);
    }
  }
  async _pump(reader) {
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      if (!value) continue;
      this._buf += value;
      let idx;
      while ((idx = this._buf.indexOf('\n')) >= 0) {
        const line = this._buf.slice(0, idx).trim();
        this._buf = this._buf.slice(idx + 1);
        this._processLine(line);
      }
    }
  }
  _processLine(raw) {
    if (!raw || raw.length < 5) return;
    let start = 0;
    for (let i = 0; i < raw.length; i++) {
      if ('0123456789-+'.includes(raw[i])) { start = i; break; }
    }
    const parts = raw.slice(start).split(',');
    if (parts.length < 6) return;
    const f = parts.map(Number);
    if (f.slice(0, 6).some(Number.isNaN)) return;

    let [nx, ny, nz, ngx, ngy, ngz] = f;
    nx = -nx; ny = -ny; nz = -nz; 

    let nwaz;
    if (parts.length >= 9) {
      nwaz = f[8];
    } else {
      const p = THREE.MathUtils.degToRad(nx), r = THREE.MathUtils.degToRad(ny);
      nwaz = Math.cos(p) * Math.cos(r);
    }
    const nlanded = parts.length >= 13 ? !!Number(f[12]) : false;

    const now = performance.now() / 1000;
    const dt = Math.max(0, Math.min(0.1, now - this._lastT));
    this._lastT = now;
    const pitch = THREE.MathUtils.degToRad(nx), roll = THREE.MathUtils.degToRad(ny);
    const G = 9.81, DRAG = 0.18;
    const axW = Math.sin(pitch) * G, ayW = -Math.sin(roll) * G, azW = (nwaz - 1.0) * G;

    const s = this.state;
    s.angleX = nx; s.angleY = ny; s.angleZ = nz; s.vertZ = nwaz; s.connected = true;
    if (nlanded) {
      this.vel = { x: 0, y: 0, z: 0 };
    } else {
      this.vel.x = this.vel.x * (1 - DRAG) + axW * dt;
      this.vel.y = this.vel.y * (1 - DRAG) + ayW * dt;
      this.vel.z = this.vel.z * (1 - DRAG) + azW * dt;
      s.posX += this.vel.x * dt;
      s.posY += this.vel.y * dt;
      s.posZ += this.vel.z * dt;
    }
  }
  update(_dt) { return this.state; }
  reset(spawn) { this.state.posX = 0; this.state.posY = 0; this.state.posZ = spawn; this.vel = { x: 0, y: 0, z: 0 }; }
}

const GUN = {
  FOLD_SPEED: 2.2,
  FOLD_J2_DEG: 150.0, FOLD_J4_DEG: -70.0, FOLD_J6_DEG: 40.0, // AXIS NOTE: ported constants, sign/axis per-joint may need a tweak once seen rendered
  AIM_J2_DEG: 60.0, AIM_J4_BASE_DEG: -25.0, AIM_J4_REACH_DEG: -55.0, AIM_J6_DEG: 30.0,
  AIM_DISTANCE: 40.0,
};

function findJoint(root, name) {
  let found = null;
  root.traverse((n) => { if (!found && n.name === name) found = n; });
  return found;
}

const PISTOL_FRAME_MAT = new THREE.MeshStandardMaterial({ color: 0x14171c, roughness: 0.35, metalness: 0.55 });
const PISTOL_SLIDE_MAT = new THREE.MeshStandardMaterial({ color: 0x848e9c, roughness: 0.24, metalness: 0.88 });

function createPistolMesh() {
  const g = new THREE.Group();
  const frame = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.11, 0.16), PISTOL_FRAME_MAT);
  g.add(frame);
  const slide = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.05, 0.24), PISTOL_SLIDE_MAT);
  slide.position.set(0, 0.045, -0.02);
  g.add(slide);
  const barrel = new THREE.Mesh(new THREE.CylinderGeometry(0.012, 0.012, 0.1, 10), PISTOL_SLIDE_MAT);
  barrel.rotation.x = Math.PI / 2;
  barrel.position.set(0, 0.045, -0.16);
  g.add(barrel);
  const grip = new THREE.Mesh(new THREE.BoxGeometry(0.045, 0.13, 0.06), PISTOL_FRAME_MAT);
  grip.position.set(0, -0.09, 0.06);
  grip.rotation.x = -0.25;
  g.add(grip);
  const flash = new THREE.Mesh(
    new THREE.SphereGeometry(0.045, 8, 8),
    new THREE.MeshBasicMaterial({ color: 0xffcc55, transparent: true, opacity: 0.95 }),
  );
  flash.position.set(0, 0.045, -0.22);
  flash.visible = false;
  g.add(flash);
  g.userData.flash = flash;
  g.traverse((o) => { if (o.isMesh) o.castShadow = true; });
  return g;
}

class ArmRig {
  constructor(droneRoot) {
    this.j1 = findJoint(droneRoot, 'J1_ShoulderAbduction');
    this.j2 = findJoint(droneRoot, 'J2_ShoulderFlexion');
    this.j4 = findJoint(droneRoot, 'J4_ElbowFlexion');
    this.j6 = findJoint(droneRoot, 'J6_WristFlexion');
    this.j1L = findJoint(droneRoot, 'J1_ShoulderAbduction_L');
    this.j2L = findJoint(droneRoot, 'J2_ShoulderFlexion_L');
    this.j4L = findJoint(droneRoot, 'J4_ElbowFlexion_L');
    this.j6L = findJoint(droneRoot, 'J6_WristFlexion_L');
    this.fold = 1.0; this.foldTarget = 1.0;

    this.weaponsOn = false;
    this.weaponR = createPistolMesh();
    this.weaponL = createPistolMesh();
    this.weaponR.visible = false;
    this.weaponL.visible = false;
    const palmR = findJoint(droneRoot, 'Palm');
    const palmL = findJoint(droneRoot, 'Palm_L');
    if (palmR) { this.weaponR.position.set(0, -0.12, 0.02); palmR.add(this.weaponR); }
    if (palmL) { this.weaponL.position.set(0, -0.12, 0.02); palmL.add(this.weaponL); }
  }
  deploy() { this.foldTarget = 0.0; }
  retract() {
    this.foldTarget = 1.0;

    this._setWeaponsOn(false);
  }
  get isDeployed() { return this.fold <= 0.02; }

  toggleWeapons() {
    if (!this.isDeployed) return;
    this._setWeaponsOn(!this.weaponsOn);
  }
  _setWeaponsOn(on) {
    this.weaponsOn = on;
    this.weaponR.visible = on;
    this.weaponL.visible = on;
  }
  muzzleFlash() {
    for (const w of [this.weaponL, this.weaponR]) {
      const flash = w?.userData.flash;
      if (!flash) continue;
      flash.visible = true;
      clearTimeout(flash.userData?._timer);
      flash.userData._timer = setTimeout(() => { flash.visible = false; }, 60);
    }
  }

  update(dt, aimDirWorld) {
    const diff = this.foldTarget - this.fold;
    const step = GUN.FOLD_SPEED * dt;
    this.fold = Math.abs(diff) <= step ? this.foldTarget : this.fold + Math.sign(diff) * step;
    const f = this.fold;
    // Belt-and-braces: if anything folds the arm while armed (e.g. a
    // future auto-retract), make sure the pistols still vanish inside
    // the drone rather than clipping through the folded arm.
    if (f > 0.02 && this.weaponsOn) this._setWeaponsOn(false);

    const dir = f <= 0.02 ? aimDirWorld : null;
    this._poseOne(this.j1, this.j2, this.j4, this.j6, f, dir);
    this._poseOne(this.j1L, this.j2L, this.j4L, this.j6L, f, dir);
  }
  _poseOne(j1, j2, j4, j6, f, aimDirWorld) {
    if (!j1) return;
    if (f > 0.02) {
      j1.quaternion.identity();
      if (j2) j2.rotation.x = THREE.MathUtils.degToRad(f * GUN.FOLD_J2_DEG);
      if (j4) j4.rotation.x = THREE.MathUtils.degToRad(f * GUN.FOLD_J4_DEG);
      if (j6) j6.rotation.x = THREE.MathUtils.degToRad(f * GUN.FOLD_J6_DEG);
      return;
    }
    if (!aimDirWorld) return;
    const parent = j1.parent;
    const worldQuat = new THREE.Quaternion();
    parent.getWorldQuaternion(worldQuat);
    const aimDirLocal = aimDirWorld.clone().applyQuaternion(worldQuat.clone().invert()).normalize();

    const quat = new THREE.Quaternion().setFromUnitVectors(AXIS.armRestDown, aimDirLocal);
    j1.quaternion.copy(quat);

    const reach = THREE.MathUtils.clamp(0.5 * (1.0 - aimDirLocal.y), 0, 1); // AXIS NOTE: .y here mirrors Panda's aim_dir_local.z (see AXIS.armRestDown)
    if (j2) j2.rotation.x = THREE.MathUtils.degToRad(reach * GUN.AIM_J2_DEG);
    if (j4) j4.rotation.x = THREE.MathUtils.degToRad(GUN.AIM_J4_BASE_DEG + reach * GUN.AIM_J4_REACH_DEG);
    if (j6) j6.rotation.x = THREE.MathUtils.degToRad(reach * GUN.AIM_J6_DEG);
  }
}

// ─────────────────────────────────────────────────────────────────
//  Main game
// ─────────────────────────────────────────────────────────────────
class Game {
  constructor() {
    this.terrainKey = null;
    this.connMode = 'mouse';
    this.source = null;
    this.track = null;
    this.armRig = null;
    this.crashed = false;
    this.kills = 0;
    this.missed = 0;
    this.camMode = 'chase';
    this.camYaw = 0; this.camPitch = 18; this.camDist = 16;
    // Aim direction for the arm rig (and shots), NOT the camera —
    // camera angle in third-person (chase/orbit) is now fixed on
    // purpose, so the pointer never gets locked/hidden there and the
    // retry button is always clickable after a crash. lookYaw/
    // lookPitch are driven by palm tracking (PalmAimSource, see
    // _animate) when enabled; at rest they sit at 0/0, meaning the
    // arm just aims straight ahead. Only FPV's camera moves, and it's
    // slaved entirely to the drone's own tilt, not to these.
    this.lookYaw = 0; this.lookPitch = 0;
    this._clock = new THREE.Clock();
    this._t0 = performance.now() / 1000;

    // Endless-runner state: constant-then-ramping forward speed,
    // steered laterally/vertically by the same tilt math the original
    // script used for dead-reckoned acceleration — here it drives lane
    // position and altitude instead of open-world position.
    this.distance = 0;
    this.speed = RUN.baseSpeed;
    this.laneX = 0;
    this.altY = (RUN.altMin + RUN.altMax) / 2;
    this.physics = new DronePhysics();
    this.flightController = new FlightController();
    this._physicsReady = false;

    this._initMenu();
  }

  _initMenu() {
    const grid = document.getElementById('terrainGrid');
    grid.querySelectorAll('.terrain-card').forEach((card) => {
      card.addEventListener('click', () => {
        grid.querySelectorAll('.terrain-card').forEach((c) => c.classList.remove('selected'));
        card.classList.add('selected');
        this.terrainKey = card.dataset.terrain;
        this._refreshStartBtn();
      });
    });

    const connSel = document.getElementById('connMode');
    const bridgeAddr = document.getElementById('bridgeAddr');
    const palmAimEnabled = document.getElementById('palmAimEnabled');
    const palmAddr = document.getElementById('palmAddr');
    const palmHint = document.getElementById('palmHint');
    connSel.addEventListener('change', () => {
      this.connMode = connSel.value;
      bridgeAddr.style.display = this.connMode === 'bridge' ? 'inline-block' : 'none';
    });
    palmAimEnabled.addEventListener('change', () => {
      palmAddr.style.display = palmAimEnabled.checked ? 'inline-block' : 'none';
      palmHint.style.display = palmAimEnabled.checked ? 'block' : 'none';
    });

    document.getElementById('startBtn').addEventListener('click', () => this._start());
  }

  _refreshStartBtn() {
    const btn = document.getElementById('startBtn');
    if (this.terrainKey) { btn.disabled = false; btn.textContent = 'LAUNCH'; }
  }

  _start() {
    document.getElementById('menu').style.display = 'none';
    document.getElementById('hud').style.display = 'block';
    document.getElementById('crosshair').style.display = 'block';
    document.getElementById('controlsHint').style.display = 'block';

    this.connMode = document.getElementById('connMode').value;
    const bridgeAddr = document.getElementById('bridgeAddr').value || 'ws://localhost:8765';
    const palmAddr = document.getElementById('palmAddr').value || 'ws://localhost:8766';
    const palmAimEnabled = document.getElementById('palmAimEnabled').checked;

    // The drone flies automatically in every mode — mouse/keyboard,
    // Bluetooth bridge, and Web Serial all just supply steering
    // (pitch/roll/yaw) the same way, so the endless-runner behavior
    // (constant/ramping forward speed, lane/altitude dodging) is
    // identical regardless of which one is selected. Webcam palm aim
    // isn't a flight source at all and is independent of this choice
    // — it layers on top of whichever one is picked, only overriding
    // look direction + fire (mouse-look/click work identically
    // underneath as a fallback whenever no hand is tracked); see the
    // palmAim polling in _animate().
    if (this.connMode === 'bridge') this.source = new BridgeSource(bridgeAddr);
    else if (this.connMode === 'serial') this.source = new WebSerialSource();
    else this.source = new MouseKeyboardSource();

    this.palmAim = palmAimEnabled ? new PalmAimSource(palmAddr) : null;

    this._initScene();
    document.getElementById('retryBtn').addEventListener('click', () => this._restart());

    const canvas = this.renderer.domElement;

    window.addEventListener('keydown', (e) => {
      if (e.code === 'KeyG') this.armRig?.deploy();
      if (e.code === 'KeyH') this.armRig?.retract();
      if (e.code === 'KeyF') this.armRig?.toggleWeapons();
      if (e.code === 'KeyX') this._fire();
      if (e.code === 'KeyV') {
        // Cycles chase -> orbit -> first-person (cockpit) -> chase.
        // All three are fixed-angle now except FPV, which is slaved to
        // the drone's own orientation — no pointer lock to request or
        // release on any of them anymore.
        const order = ['chase', 'orbit', 'fpv'];
        this.camMode = order[(order.indexOf(this.camMode) + 1) % order.length];
      }
      if (e.code === 'Escape') location.reload();
    });
    window.addEventListener('mousedown', (e) => {
      if (e.button === 0) this._fire();
    });
    window.addEventListener('wheel', (e) => {
      this.camDist = THREE.MathUtils.clamp(this.camDist + e.deltaY * 0.02, 6, 60);
    });
    window.addEventListener('mousemove', (e) => {
      // Absolute screen position -> arm aim direction (see _aimDir).
      // No pointer lock here — the cursor stays visible at all times
      // (so it's never hidden behind the retry button after a crash),
      // and this never touches the camera in any mode: chase/orbit
      // are fixed-angle, FPV is slaved to the drone's own tilt. Palm
      // tracking (in _animate) overrides these same two fields
      // whenever a hand is currently detected, so mouse is the
      // always-available fallback and palm takes over automatically
      // the moment it sees a hand.
      const nx = (e.clientX / window.innerWidth) * 2 - 1;
      const ny = (e.clientY / window.innerHeight) * 2 - 1;
      this.lookYaw = nx * 180;
      this.lookPitch = THREE.MathUtils.clamp(-ny * 89, -89, 89);
    });

    this._animate();
  }

  _initScene() {
    const canvas = document.createElement('canvas');
    document.body.appendChild(canvas);
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    // ACES tone mapping gives highlights a much richer rolloff than the
    // flat default — one of the cheapest, safest wins for a "polished"
    // look, and it benefits the whole scene, not just the drone.
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.05;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;

    // A soft procedural environment (three.js's built-in RoomEnvironment
    // — no external HDR needed) gives the glossier materials below
    // something to actually reflect. Without any environment, raising
    // metalness/lowering roughness on a material has almost nothing to
    // show for it beyond direct-light highlights; this is what turns
    // that into visible soft reflections, the core of a "polished"
    // product-render look.
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    this._envTexture = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    pmrem.dispose();

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(62, innerWidth / innerHeight, 0.1, 2000);
    this.scene.environment = this._envTexture;

    this.flavor = TERRAIN_FLAVORS[this.terrainKey];
    this.scene.fog = this.flavor.fog;
    this.scene.background = new THREE.Color(this.flavor.sky);
    this.track = new EndlessTrack(this.scene, this.flavor);
    const spawnY = this.track.baselineY(0) + this.altY;
    this.physics.reset(new THREE.Vector3(0, spawnY, 0));
    this.flightController.reset(spawnY);
    this._physicsReady = true;

    // ── Lighting rig ─────────────────────────────────────────────
    // A single hemisphere + sun reads flat. This adds the same kind of
    // multi-light setup a hand-built "hero" render uses: a soft sky/
    // ground ambient, one key light with shadows, a cooler fill light
    // to open up the shadow side, a faint under-light so the belly and
    // joints don't go fully black, and two small colored rim lights
    // that ride along with the drone (children of droneRoot, not the
    // scene) so it always gets a subtle accent kick regardless of
    // which way it's facing or what the terrain lighting is doing.
    const hemi = new THREE.HemisphereLight(this.flavor.sky, 0x2a2a2a, 0.85);
    this.scene.add(hemi);
    const sun = new THREE.DirectionalLight(0xfff2d8, 1.25);
    sun.position.set(60, 100, 40);
    sun.castShadow = true;
    sun.shadow.mapSize.set(1536, 1536);
    sun.shadow.camera.left = -70; sun.shadow.camera.right = 70;
    sun.shadow.camera.top = 70; sun.shadow.camera.bottom = -70;
    sun.shadow.bias = -0.0015;
    this.scene.add(sun);
    this._sun = sun;
    const fill = new THREE.DirectionalLight(0x6f8fbf, 0.5);
    fill.position.set(-40, 30, -30);
    this.scene.add(fill);
    const under = new THREE.DirectionalLight(0x88aabb, 0.25);
    under.position.set(0, -20, 10);
    this.scene.add(under);

    this.droneRoot = new THREE.Group();
    this.scene.add(this.droneRoot);
    const rimBlue = new THREE.PointLight(0x4fc3f7, 2.2, 14);
    rimBlue.position.set(-2, 1.5, -1.5);
    this.droneRoot.add(rimBlue);
    const rimAmber = new THREE.PointLight(0xf9a825, 1.8, 12);
    rimAmber.position.set(2, 0.5, 1.5);
    this.droneRoot.add(rimAmber);

    const loader = new GLTFLoader();
    const TARGET_SIZE = 5.5; // desired drone wingspan/length in world units
    // Cache-bust: browsers cache .glb binaries aggressively across
    // reloads (sometimes surviving a normal hard-refresh). Bumping
    // BUILD_VERSION whenever the model or this file changes forces the
    // browser to treat it as a brand-new URL instead of possibly
    // reusing a stale cached copy.
    loader.load(`assets/drone.glb?${BUILD_VERSION}`, (gltf) => {
      this.droneModel = gltf.scene;
      let _tunedMatCount = 0;
      this.droneModel.traverse((n) => {
        if (!n.isMesh) return;
        n.castShadow = true;
        n.receiveShadow = true;
        // Material polish pass: the GLB's baked materials are a flat,
        // slightly matte default (roughness ~0.55). Pushed hard toward
        // a glossy, semi-metallic "coated composite" finish — a subtle
        // nudge here reads as almost no change at all on a fast-moving
        // model, so this deliberately overshoots into an unmistakably
        // different sheen rather than a barely-there tweak. The
        // existing semi-transparent low-roughness material (the
        // joint/lens look) is already glossy and correct, so it's
        // left untouched rather than pushed further.
        const mats = Array.isArray(n.material) ? n.material : [n.material];
        for (const mat of mats) {
          if (!mat || !mat.isMeshStandardMaterial) continue;
          if (mat.transparent || mat.opacity < 1) continue;
          mat.roughness = 0.16;
          mat.metalness = 0.62;
          mat.envMapIntensity = 1.6;
          _tunedMatCount++;
        }
      });
      console.log('[AETHON-X] material polish applied to', _tunedMatCount, 'material(s)');

      // ── Robotic gun-arm recolor: metallic red ────────────────────
      // The shoulder→elbow→wrist→palm arm assembly (rooted at
      // ShoulderSocket / ShoulderSocket_L) shares the same baked
      // material as the rest of the drone body/struts in the GLB, so
      // its meshes must get their own cloned material instead of
      // having the shared one mutated — otherwise the whole airframe
      // would turn red along with the arms. Matches the "metallic
      // red" arm finish used elsewhere (0xee2222, low roughness, high
      // metalness). The transparent elbow lens/joint material is left
      // untouched, same as the polish pass above.
      const armRedMat = new THREE.MeshStandardMaterial({
        color: 0xee2222, roughness: 0.28, metalness: 0.75, envMapIntensity: 1.6,
      });
      let _armMeshCount = 0;
      for (const rootName of ['ShoulderSocket', 'ShoulderSocket_L']) {
        const armRoot = findJoint(this.droneModel, rootName);
        if (!armRoot) continue;
        armRoot.traverse((n) => {
          if (!n.isMesh) return;
          const mats = Array.isArray(n.material) ? n.material : [n.material];
          const isLens = mats.some((mat) => mat && mat.isMeshStandardMaterial && (mat.transparent || mat.opacity < 1));
          if (isLens) return;
          n.material = Array.isArray(n.material) ? mats.map(() => armRedMat) : armRedMat;
          n.castShadow = true;
          n.receiveShadow = true;
          _armMeshCount++;
        });
      }
      console.log('[AETHON-X] arm recolor (metallic red) applied to', _armMeshCount, 'mesh(es)');

      // ── Propeller spin rig ──────────────────────────────────────
      // Each Propeller_N group node's own transform (baked as a glTF
      // "matrix", not separate T/R/S — easy to miss on a quick node
      // dump) already places its local origin exactly at that motor's
      // hub, with the blade meshes positioned around it. So no
      // recentering is needed — rotating the group's own local X axis
      // (confirmed from the raw mesh data: blade span sits along
      // local X, thin on Y/Z) spins the blades correctly in place.
      this.propellers = [];
      this.droneModel.traverse((n) => {
        if (/^Propeller_\d+$/.test(n.name)) this.propellers.push(n);
      });

      // The GLB's baked node scale was authored for Panda3D's own scene
      // (a 12x body scale, plus the 8x we added on the arm subtree on
      // top of that) — dropped in raw, the model is enormous relative
      // to this game's world units. Measure the model's actual
      // bounding box and rescale it uniformly so its largest dimension
      // matches TARGET_SIZE, regardless of whatever scale is baked in.
      const box = new THREE.Box3().setFromObject(this.droneModel);
      const sizeVec = new THREE.Vector3();
      box.getSize(sizeVec);
      const largestDim = Math.max(sizeVec.x, sizeVec.y, sizeVec.z) || 1;
      const fitScale = TARGET_SIZE / largestDim;
      this.droneModel.scale.setScalar(fitScale);
      // Align X/Z to the model's own middle, and its bottom (not its
      // geometric center) to local y=0 — altitude/collision checks
      // treat dronePos.y as "how high the vehicle's belly is above the
      // ground", so the pivot needs to sit at the mesh's bottom.
      const center = new THREE.Vector3();
      box.getCenter(center);
      this.droneModel.position.set(
        -center.x * fitScale,
        -box.min.y * fitScale,
        -center.z * fitScale,
      );

      this.droneRoot.add(this.droneModel);
      this.armRig = new ArmRig(this.droneModel);
    }, undefined, (err) => console.error('drone.glb failed to load:', err));
  }

  _restart() {
    document.getElementById('crashOverlay').style.display = 'none';
    this.crashed = false;
    this.distance = 0;
    this.speed = RUN.baseSpeed;
    this.laneX = 0;
    this.altY = (RUN.altMin + RUN.altMax) / 2;
    const spawnY = this.track.baselineY(0) + this.altY;
    this.physics.reset(new THREE.Vector3(0, spawnY, 0));
    this.flightController.reset(spawnY);
    this.source?.reset?.();
    this.kills = 0;
    this.missed = 0;
    this.lookYaw = 0;
    this.lookPitch = 0;
    this.track.reset();
  }

  // Palm aim is a true spherical direction: yaw spans -180..+180 and
  // pitch spans almost -90..+90. This is independent of the camera in
  // third-person, while FPV deliberately uses the same direction so
  // the player can look and shoot exactly where the grey palm is aiming.
  _aimDir() {
    const yawR = THREE.MathUtils.degToRad(this.lookYaw);
    const pitchR = THREE.MathUtils.degToRad(this.lookPitch);
    return new THREE.Vector3(0, 0, -1).applyEuler(new THREE.Euler(pitchR, yawR, 0, 'YXZ')).normalize();
  }

  // Left click fires both arm-mounted pistols along the same aim
  // direction the arm rig itself points at while deployed (see
  // _aimDir), so the shot always goes where the arm visually points —
  // camera angle no longer has anything to do with it. Needs both the
  // arms out (isDeployed) AND the pistols actually drawn (weaponsOn) —
  // G/H swing the bare hand in and out, F draws or holsters the guns
  // once the hand is out.
  _fire() {
    if (this.crashed || !this.armRig || !this.armRig.isDeployed || !this.armRig.weaponsOn || !this.track) return;
    this.armRig.muzzleFlash();
    const targets = this.track.liveEnemyMeshes();
    if (!targets.length) return;
    const raycaster = new THREE.Raycaster();
    raycaster.set(this.camera.position, this._aimDir());
    const hits = raycaster.intersectObjects(targets, true);
    if (!hits.length) return;
    let root = hits[0].object;
    while (root && !targets.includes(root)) root = root.parent;
    if (!root) return;
    const enemy = this.track.enemyForMesh(root);
    if (enemy && enemy.alive) {
      enemy.alive = false;
      enemy.mesh.visible = false;
      this.kills++;
    }
  }

  _crash(reason) {
    if (this.crashed) return;
    this.crashed = true;
    document.getElementById('crashReason').textContent =
      `${reason} — ${Math.round(this.distance)} m`;
    document.getElementById('crashOverlay').style.display = 'flex';
  }

  _updateCamera(dronePos) {
    if (this.camMode === 'chase') {
      // Fixed trailing angle on purpose — this used to free-look via
      // pointer-lock mouse movement, which both hid the cursor (so the
      // retry button couldn't be clicked after a crash) and coupled
      // the camera to palm-tracking aim. Position still trails the
      // drone; the view always faces straight down the corridor.
      this.camera.position.set(dronePos.x * 0.5, dronePos.y + 4.2, dronePos.z + this.camDist);
      this.camera.lookAt(dronePos.x * 0.5, dronePos.y + 2, dronePos.z - 20);
    } else if (this.camMode === 'fpv') {
      // First-person/cockpit view: the camera rides at the drone nose,
      // but its LOOK direction is controlled by the same grey-palm aim
      // vector as the arms. This branch is the only camera mode affected
      // by palm aim. The drone body orientation is still used for the
      // camera's position, so FPV remains attached to the aircraft.
      const noseOffset = new THREE.Vector3(0, 0.35, -1.2).applyQuaternion(this.droneRoot.quaternion);
      this.camera.position.copy(dronePos).add(noseOffset);
      const palmLook = this._aimDir();
      this.camera.lookAt(this.camera.position.clone().add(palmLook.multiplyScalar(100)));
    } else {
      const yawR = THREE.MathUtils.degToRad(this.camYaw), pitchR = THREE.MathUtils.degToRad(this.camPitch);
      this.camera.position.set(
        dronePos.x + this.camDist * Math.cos(pitchR) * Math.sin(yawR),
        dronePos.y + this.camDist * Math.sin(pitchR),
        dronePos.z + this.camDist * Math.cos(pitchR) * Math.cos(yawR),
      );
      this.camera.lookAt(dronePos);
    }
  }

  _animate = () => {
    requestAnimationFrame(this._animate);
    const dt = Math.min(0.1, this._clock.getDelta());
    const s = this.source.update(dt);

    if (!this.crashed) {
      // Runner speed is a cruise target. The aircraft must accelerate
      // through thrust, attitude and aerodynamic drag to reach it.
      this.flightController.targetForwardSpeed = Math.min(RUN.maxSpeed, RUN.baseSpeed + this.distance * RUN.accelPerMeter);
      this.flightController.targetAltitude = Math.max(
        this.track.baselineY(this.distance) + RUN.altMin,
        Math.min(this.track.baselineY(this.distance) + RUN.altMax, this.flightController.targetAltitude)
      );
      const groundY = this.track.baselineY(this.distance) + 0.15;
      this.physics.update(dt, (h, body) => this.flightController.compute(h, body, s, groundY));
      this.distance = Math.max(this.distance, -this.physics.position.z);
      this.speed = Math.max(0, -this.physics.velocity.z);
      this.laneX = this.physics.position.x;
      this.altY = this.physics.position.y - this.track.baselineY(this.distance);
    }

    const dronePos = this.physics.position;

    if (!this.crashed) {
      this.droneRoot.position.copy(dronePos);
      this.droneRoot.quaternion.copy(this.physics.orientation);

      this.track.update(this.distance);
      this.track.checkEnemyMiss(dronePos, () => { this.missed++; });
      const reason = this.track.checkCollision(dronePos);
      if (reason) this._crash(reason);
    }

    // Spin the propellers — idles even when crashed/stationary (real
    // rotors don't instantly stop), spins faster with forward speed
    // so it reads as thrust-driven rather than a fixed idle animation.
    // Adjacent motors alternate direction, matching how a real
    // quadcopter cancels reaction torque.
    if (this.propellers) {
      for (let i = 0; i < this.propellers.length; i++) {
        const dir = i % 2 === 0 ? 1 : -1;
        const rpmVisual = (this.physics.motorOmega[i] || 0) * 0.035;
        this.propellers[i].rotation.x += dir * rpmVisual * dt;
      }
    }

    // Palm-tracking aim (if enabled) overrides lookYaw/lookPitch whenever
    // the grey LEFT hand is visible. The values are full spherical angles
    // from the palm's 3-D normal. Third-person cameras ignore this aim;
    // FPV consumes the same direction so camera + arms stay aligned.
    if (this.palmAim) {
      const p = this.palmAim.poll();
      if (p.handFound) { this.lookYaw = p.lookYaw; this.lookPitch = p.lookPitch; }
      for (let i = 0; i < p.fireEdges; i++) this._fire();
    }

    this.armRig?.update(dt, this._aimDir());
    this._updateCamera(dronePos);
    if (this._sun) {
      this._sun.position.set(dronePos.x + 60, dronePos.y + 100, dronePos.z + 40);
      this._sun.target.position.copy(dronePos);
      this._sun.target.updateMatrixWorld();
    }

    this._updateHud(s, dronePos);
    this.renderer.render(this.scene, this.camera);
  };

  _updateHud(s, dronePos) {
    const hud = document.getElementById('hud');
    const connLabel = s.connected ? 'CONNECTED' : 'WAITING...';
    const deployed = !!this.armRig?.isDeployed;
    const armed = !!this.armRig?.weaponsOn;
    document.getElementById('crosshair').classList.toggle('armed', deployed && armed);

    let aimLine = '';
    if (this.palmAim) {
      const p = this.palmAim.state;
      const palmStatus = !p.connected ? 'BRIDGE OFFLINE' : (p.handFound ? 'TRACKING' : 'NO HAND');
      const palmColor = !p.connected ? '#f87171' : (p.handFound ? '#4ade80' : '#f59e0b');
      aimLine = `<span class="label">aim</span> <span style="color:${palmColor}">PALM — ${palmStatus}</span><br>`;
    }

    hud.innerHTML =
      `<div class="status" style="color:${s.connected ? '#4ade80' : '#f59e0b'}">${connLabel}</div>` +
      `<span class="label">terrain</span> ${this.flavor.name} &nbsp;` +
      `<span class="label">dist</span> ${Math.round(this.distance)} m<br>` +
      `<span class="label">speed</span> ${this.speed.toFixed(1)} m/s<br>` +
      `<span class="label">pitch</span> ${s.angleX.toFixed(1)}&deg; &nbsp;` +
      `<span class="label">roll</span> ${s.angleY.toFixed(1)}&deg;<br>` +
      `<span class="label">lane</span> ${this.laneX.toFixed(1)} m &nbsp;` +
      `<span class="label">alt</span> ${dronePos.y.toFixed(1)} m<br>` +
      `<span class="label">arms</span> ${deployed ? 'DEPLOYED' : 'RETRACTED'} &nbsp;` +
      `<span class="label">pistols</span> ${armed ? 'DRAWN' : 'STOWED'}<br>` +
      aimLine +
      `<span class="label" style="color:#4ade80">kills</span> ${this.kills} &nbsp;` +
      `<span class="label" style="color:#f87171">missed</span> ${this.missed}`;
  }
}

window.addEventListener('resize', () => {
  const g = window.__aethonGame;
  if (g?.camera && g?.renderer) {
    g.camera.aspect = innerWidth / innerHeight;
    g.camera.updateProjectionMatrix();
    g.renderer.setSize(innerWidth, innerHeight);
  }
});

window.__aethonGame = new Game();
