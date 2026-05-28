import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

// ── Robot physical constants (cm) ──────────────────────────────────────────
const L1 = 24;   // base height (O → 1J2)
const L2 = 21;   // upper arm  (1J2 → 2J3)
const L3 = 35;   // forearm    (2J3 → E)
const BASE_W = 23.5;  // base box width (X)
const BASE_D = 23.5;  // base box depth (Y)
const BASE_H = 13;    // base box visual height — physical enclosure (NOT the shaft length L1=24)

// Geometry radii (cm)
const R_SHAFT = 1.0;
const R_LINK2 = 1.4;
const R_LINK3 = 1.1;
const R_JOINT = 2.0;
const GRIPPER_SZ = 3.0;

function toRad(d) { return d * Math.PI / 180; }

/**
 * Forward Kinematics — Robot frame: X=left(+), Y=forward(arm), Z=up
 *
 * s2 = 0°  → J2 points straight up   (+Z)
 * s3 = 0°  → J3 points horizontal   (+Y direction, absolute)
 * s1 = 0°  → arm plane aligned with +Y axis
 *
 * X axis convention (per user spec):
 *   base rotates LEFT  → positive X
 *   base rotates RIGHT → negative X
 *   Achieved by negating the sin(r1) term on X.
 *
 * At calibration (s1=0, s2=0, s3=0):
 *   2J3 = (0, 0, 45)   E = (0, 35, 45)  ← matches coordinate-system.txt
 */
function computeFK(s1, s2, s3) {
  const r1 = toRad(s1), r2 = toRad(s2), r3 = toRad(s3);

  // 2J3: top of J2, affected by J1 rotation (s1) and J2 lean (s2).
  // X is negated so that left rotation (positive s1) gives positive X.
  const J23 = {
    x: -L2 * Math.sin(r2) * Math.sin(r1),
    y:  L2 * Math.sin(r2) * Math.cos(r1),
    z:  L1 + L2 * Math.cos(r2),
  };

  // E: J3 angle is measured relative to J2's frame (90° offset built in).
  // When J2 leans by s2 and J3 hasn't moved (s3=0), J3 physically rotates
  // with J2 — so its absolute direction is governed by (s2 - s3).
  //
  // Derivation confirmed by solve_d3 in main.py:
  //   value = (L1 + L2·cos(s2)) / L3  → this equals sin(s2 - s3) when E.z = 0
  //   → E.z = J23.z - L3·sin(s2 - s3)
  //
  // Verification at calibration (s1=0, s2=0, s3=0):
  //   E = (0, 35, 45) ✓
  // X is negated (same sign convention as J23.x).
  const r23 = r2 - r3;   // compound angle: J3 absolute direction in arm plane
  return {
    O:   { x: 0, y: 0, z: 0 },
    J12: { x: 0, y: 0, z: L1 },
    J23,
    E: {
      x: J23.x - L3 * Math.cos(r23) * Math.sin(r1),
      y: J23.y + L3 * Math.cos(r23) * Math.cos(r1),
      z: J23.z - L3 * Math.sin(r23),
    },
  };
}

/**
 * Convert robot frame (X, Y, Z) → Three.js frame (X_t, Y_t, Z_t)
 * Robot Z (up)      → Three.js Y (up)
 * Robot Y (forward) → Three.js Z (into screen at s1=0)
 * Robot X (right)   → Three.js X (right)
 */
function r2t({ x, y, z }) { return new THREE.Vector3(x, z, y); }

/**
 * Reposition a unit-height cylinder mesh between two 3D points.
 * The mesh must have been created with CylinderGeometry height=1.
 */
function updateLink(mesh, p1, p2) {
  const dir = new THREE.Vector3().subVectors(p2, p1);
  const len = dir.length();
  if (len < 0.01) return;
  mesh.position.copy(new THREE.Vector3().addVectors(p1, p2).multiplyScalar(0.5));
  mesh.scale.set(1, len, 1);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir.normalize());
}

// ── Helpers for HUD number formatting ────────────────────────────────────────
const fmt = (v) => (v >= 0 ? '+' : '') + Math.round(v * 10); // cm → mm, signed

export default function RobotVisualization({ state }) {
  const mountRef = useRef(null);
  const threeRef = useRef({});          // holds Three.js objects across renders
  const [hud, setHud] = useState(null); // drives the HTML overlay

  // ── Scene initialisation (runs once on mount) ─────────────────────────────
  useEffect(() => {
    const el = mountRef.current;
    if (!el) return;

    // ── Renderer ─────────────────────────────────────────────────────────────
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(el.clientWidth, el.clientHeight);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.2;
    renderer.domElement.style.display = 'block'; // prevent 4px inline-block gap
    el.appendChild(renderer.domElement);

    // ── Scene ─────────────────────────────────────────────────────────────────
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x080e1a);
    scene.fog = new THREE.FogExp2(0x080e1a, 0.006);

    // ── Camera ────────────────────────────────────────────────────────────────
    const camera = new THREE.PerspectiveCamera(45, el.clientWidth / el.clientHeight, 0.5, 600);
    // Observer at robot-frame (100, 100, 40): r2t maps Z→Y, Y→Z → Three.js (100, 40, 100)
    camera.position.set(100, 40, 100);
    camera.lookAt(0, L1 + L2 / 2, 0);

    // ── Orbit Controls ────────────────────────────────────────────────────────
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, L1 + L2 / 2, 0);
    controls.enableDamping = true;
    controls.dampingFactor = 0.07;
    controls.minDistance = 40;
    controls.maxDistance = 280;
    controls.maxPolarAngle = Math.PI * 0.92;
    controls.update();

    // ── Lights ────────────────────────────────────────────────────────────────
    scene.add(new THREE.AmbientLight(0x1a2a44, 2.0));

    const sun = new THREE.DirectionalLight(0xffffff, 2.5);
    sun.position.set(80, 120, 60);
    sun.castShadow = true;
    sun.shadow.mapSize.set(2048, 2048);
    sun.shadow.camera.near = 1;
    sun.shadow.camera.far = 300;
    sun.shadow.camera.left = -80;
    sun.shadow.camera.right = 80;
    sun.shadow.camera.top = 80;
    sun.shadow.camera.bottom = -80;
    scene.add(sun);

    const fill = new THREE.PointLight(0x0055ff, 1.0, 200);
    fill.position.set(-60, 60, -40);
    scene.add(fill);

    const rimLight = new THREE.PointLight(0x00d4ff, 0.6, 200);
    rimLight.position.set(0, 100, -60);
    scene.add(rimLight);

    // ── Ground grid ────────────────────────────────────────────────────────────
    const gridHelper = new THREE.GridHelper(140, 28, 0x1a3050, 0x0d1f30);
    scene.add(gridHelper);

    // Ground plane (shadow receiver)
    const groundMesh = new THREE.Mesh(
      new THREE.PlaneGeometry(140, 140),
      new THREE.MeshStandardMaterial({ color: 0x080e1a, roughness: 1, metalness: 0 })
    );
    groundMesh.rotation.x = -Math.PI / 2;
    groundMesh.receiveShadow = true;
    scene.add(groundMesh);

    // ── World Axes ────────────────────────────────────────────────────────────
    const axesHelper = new THREE.AxesHelper(20);
    scene.add(axesHelper);

    // Axis labels (sprite-based text via canvas)
    function makeAxisLabel(text, color) {
      const canvas = document.createElement('canvas');
      canvas.width = 64; canvas.height = 32;
      const ctx = canvas.getContext('2d');
      ctx.fillStyle = color;
      ctx.font = 'bold 22px Arial';
      ctx.fillText(text, 8, 24);
      const tex = new THREE.CanvasTexture(canvas);
      const mat = new THREE.SpriteMaterial({ map: tex, depthTest: false });
      return new THREE.Sprite(mat);
    }
    const lblX = makeAxisLabel('X', '#ff4444'); lblX.position.set(22, 0, 0); lblX.scale.set(5, 2.5, 1); scene.add(lblX);
    const lblY = makeAxisLabel('Z', '#44ff44'); lblY.position.set(0, 22, 0); lblY.scale.set(5, 2.5, 1); scene.add(lblY);
    const lblZ = makeAxisLabel('Y', '#4488ff'); lblZ.position.set(0, 0, 22); lblZ.scale.set(5, 2.5, 1); scene.add(lblZ);

    // ── Base box (23.5 × L1 × 23.5 cm) ───────────────────────────────────────
    const baseMat = new THREE.MeshStandardMaterial({
      color: 0x1e3a5f, metalness: 0.65, roughness: 0.35,
      transparent: true, opacity: 0.80,
    });
    const baseBox = new THREE.Mesh(new THREE.BoxGeometry(BASE_W, BASE_H, BASE_D), baseMat);
    baseBox.position.set(0, BASE_H / 2, 0);
    baseBox.castShadow = true;
    baseBox.receiveShadow = true;
    scene.add(baseBox);

    // Wireframe overlay on base box
    const baseEdges = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(BASE_W, BASE_H, BASE_D)),
      new THREE.LineBasicMaterial({ color: 0x00d4ff, transparent: true, opacity: 0.5 })
    );
    baseEdges.position.copy(baseBox.position);
    scene.add(baseEdges);

    // Origin sphere (O)
    const originSph = new THREE.Mesh(
      new THREE.SphereGeometry(1.2, 12, 12),
      new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0xffffff, emissiveIntensity: 0.5 })
    );
    originSph.position.set(0, 0, 0);
    scene.add(originSph);

    // ── J1 Shaft (fixed: O → 1J2) ─────────────────────────────────────────────
    const shaftMat = new THREE.MeshStandardMaterial({ color: 0x8899bb, metalness: 0.9, roughness: 0.15 });
    const shaftMesh = new THREE.Mesh(new THREE.CylinderGeometry(R_SHAFT, R_SHAFT, 1, 12), shaftMat);
    updateLink(shaftMesh, r2t({ x: 0, y: 0, z: 0 }), r2t({ x: 0, y: 0, z: L1 }));
    shaftMesh.castShadow = true;
    scene.add(shaftMesh);

    // Joint sphere at 1J2 (fixed)
    const jointMat = new THREE.MeshStandardMaterial({
      color: 0xffd700, metalness: 0.75, roughness: 0.2,
      emissive: 0xffd700, emissiveIntensity: 0.12,
    });
    const sphJ12 = new THREE.Mesh(new THREE.SphereGeometry(R_JOINT, 16, 16), jointMat);
    sphJ12.position.copy(r2t({ x: 0, y: 0, z: L1 }));
    sphJ12.castShadow = true;
    scene.add(sphJ12);

    // ── J2 Link (dynamic: 1J2 → 2J3) ─────────────────────────────────────────
    const link2Mat = new THREE.MeshStandardMaterial({ color: 0x00d4ff, metalness: 0.5, roughness: 0.3, emissive: 0x003040, emissiveIntensity: 0.4 });
    const link2Mesh = new THREE.Mesh(new THREE.CylinderGeometry(R_LINK2, R_LINK2, 1, 12), link2Mat);
    link2Mesh.castShadow = true;
    scene.add(link2Mesh);

    // Joint sphere at 2J3 (dynamic)
    const sphJ23 = new THREE.Mesh(new THREE.SphereGeometry(R_JOINT, 16, 16), jointMat.clone());
    sphJ23.castShadow = true;
    scene.add(sphJ23);

    // ── J3 Link (dynamic: 2J3 → E) ────────────────────────────────────────────
    const link3Mat = new THREE.MeshStandardMaterial({ color: 0xff8c00, metalness: 0.5, roughness: 0.3, emissive: 0x301800, emissiveIntensity: 0.4 });
    const link3Mesh = new THREE.Mesh(new THREE.CylinderGeometry(R_LINK3, R_LINK3, 1, 12), link3Mat);
    link3Mesh.castShadow = true;
    scene.add(link3Mesh);

    // ── End-effector / Gripper at E (dynamic) ─────────────────────────────────
    const gripMat = new THREE.MeshStandardMaterial({
      color: 0x00ff88, metalness: 0.4, roughness: 0.4,
      emissive: 0x00ff88, emissiveIntensity: 0.3,
    });
    const gripMesh = new THREE.Mesh(new THREE.BoxGeometry(GRIPPER_SZ, GRIPPER_SZ, GRIPPER_SZ), gripMat);
    gripMesh.castShadow = true;
    scene.add(gripMesh);

    // Gripper pulse ring (torus)
    const ringMat = new THREE.MeshBasicMaterial({ color: 0x00ff88, transparent: true, opacity: 0.4, side: THREE.DoubleSide });
    const ringMesh = new THREE.Mesh(new THREE.TorusGeometry(GRIPPER_SZ * 0.8, 0.3, 8, 24), ringMat);
    scene.add(ringMesh);

    // ── Arm skeleton line (O→J12→J23→E) ──────────────────────────────────────
    const skelGeo = new THREE.BufferGeometry();
    const skelPos = new Float32Array(4 * 3);
    skelGeo.setAttribute('position', new THREE.BufferAttribute(skelPos, 3));
    skelGeo.setDrawRange(0, 4);
    const skelLine = new THREE.Line(skelGeo, new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.15 }));
    scene.add(skelLine);

    // ── Projection shadow (circle on ground) ──────────────────────────────────
    const shadowCircle = new THREE.Mesh(
      new THREE.CircleGeometry(3, 16),
      new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.35, side: THREE.DoubleSide })
    );
    shadowCircle.rotation.x = -Math.PI / 2;
    shadowCircle.position.y = 0.1;
    scene.add(shadowCircle);

    // ── Store refs ───────────────────────────────────────────────────────────
    threeRef.current = {
      renderer, scene, camera, controls,
      link2Mesh, link3Mesh, sphJ23,
      gripMesh, gripMat, ringMesh, ringMat,
      skelLine, shadowCircle,
    };

    // ── Animation loop ────────────────────────────────────────────────────────
    let animId;
    let isActive = true;   // cleared on cleanup to prevent render-after-dispose
    let t = 0;
    function animate() {
      if (!isActive) return;   // StrictMode safety: stop if cleaned up
      animId = requestAnimationFrame(animate);
      t += 0.016;
      controls.update();
      // Pulse the gripper ring
      const { ringMesh: rm, ringMat: rmt } = threeRef.current;
      if (rm && rmt) {
        const pulse = 0.3 + 0.1 * Math.sin(t * 3);
        rmt.opacity = pulse;
        rm.scale.setScalar(1 + 0.06 * Math.sin(t * 3));
      }
      renderer.render(scene, camera);
    }
    animate();

    // ── Resize observer ───────────────────────────────────────────────────────
    const ro = new ResizeObserver(() => {
      if (!el) return;
      const w = el.clientWidth, h = el.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    });
    ro.observe(el);

    return () => {
      isActive = false;   // stop animation loop before any async frame fires
      cancelAnimationFrame(animId);
      ro.disconnect();
      controls.dispose();
      renderer.dispose();
      if (el.contains(renderer.domElement)) el.removeChild(renderer.domElement);
    };
  }, []); // run once

  // ── Update arm on every state change ─────────────────────────────────────
  useEffect(() => {
    const { link2Mesh, link3Mesh, sphJ23, gripMesh, gripMat, ringMesh, ringMat, skelLine, shadowCircle } = threeRef.current;
    if (!link2Mesh) return;

    const s1 = state?.s1 ?? 0;
    const s2 = state?.s2 ?? 0;
    const s3 = state?.s3 ?? 0;
    const isOpen = state?.gripper_state === 'open';

    const { O, J12, J23, E } = computeFK(s1, s2, s3);

    const p0 = r2t(O);
    const p1 = r2t(J12);   // always (0, L1, 0) in Three.js
    const p2 = r2t(J23);
    const p3 = r2t(E);

    // Update arm links
    updateLink(link2Mesh, p1, p2);
    updateLink(link3Mesh, p2, p3);

    // Update joint at 2J3
    sphJ23.position.copy(p2);

    // Update gripper
    gripMesh.position.copy(p3);
    ringMesh.position.copy(p3);

    const gripColor = isOpen ? 0x00ff88 : 0xff3344;
    gripMat.color.set(gripColor);
    gripMat.emissive.set(isOpen ? 0x004422 : 0x440011);
    ringMat.color.set(gripColor);

    // Update skeleton line
    const pos = skelLine.geometry.attributes.position;
    pos.setXYZ(0, p0.x, p0.y, p0.z);
    pos.setXYZ(1, p1.x, p1.y, p1.z);
    pos.setXYZ(2, p2.x, p2.y, p2.z);
    pos.setXYZ(3, p3.x, p3.y, p3.z);
    pos.needsUpdate = true;

    // Ground projection shadow under E
    shadowCircle.position.x = p3.x;
    shadowCircle.position.z = p3.z;

    // HUD: coordinates in mm (× 10), display in robot frame (X,Y,Z)
    // Y is negated per coordinate-system convention: arm extends toward the
    // observer at s1=0, which is the negative-Y direction in the world frame.
    setHud({
      J23: { x: fmt(J23.x), y: fmt(-J23.y), z: fmt(J23.z) },
      E:   { x: fmt(E.x),   y: fmt(-E.y),   z: fmt(E.z)   },
      s1: s1.toFixed(1), s2: s2.toFixed(1), s3: s3.toFixed(1),
      open: isOpen,
    });
  }, [state]);

  return (
    <div style={{ position: 'relative', width: '100%', height: '400px', borderRadius: '10px', overflow: 'hidden' }}>
      {/* Three.js canvas mount point */}
      <div ref={mountRef} style={{ width: '100%', height: '100%' }} />

      {/* ── HUD Coordinate Monitor ───────────────────────────────────────────── */}
      {hud && (
        <div className="viz-hud">
          <div className="viz-hud-title">📐 Coordinates (mm)</div>

          <div className="viz-hud-row">
            <span className="viz-pt-label">O</span>
            <span className="viz-coords-fixed">(+0, +0, +0)</span>
            <span className="viz-pt-note">origin</span>
          </div>

          <div className="viz-hud-row">
            <span className="viz-pt-label">1J2</span>
            <span className="viz-coords-fixed">(+0, +0, +240)</span>
            <span className="viz-pt-note">fixed</span>
          </div>

          <div className="viz-hud-row">
            <span className="viz-pt-label viz-label-j23">2J3</span>
            <span className="viz-coords-live viz-val-j23">
              ({hud.J23.x}, {hud.J23.y}, {hud.J23.z})
            </span>
          </div>

          <div className="viz-hud-row">
            <span className="viz-pt-label viz-label-e">E</span>
            <span className="viz-coords-live viz-val-e">
              ({hud.E.x}, {hud.E.y}, {hud.E.z})
            </span>
          </div>

          <div className="viz-hud-divider" />

          <div className="viz-hud-angles">
            <span className="viz-angle"><span>θ₁</span><b>{hud.s1}°</b></span>
            <span className="viz-angle"><span>θ₂</span><b>{hud.s2}°</b></span>
            <span className="viz-angle"><span>θ₃</span><b>{hud.s3}°</b></span>
          </div>

          <div className={`viz-gripper-badge ${hud.open ? 'viz-gripper-open' : 'viz-gripper-closed'}`}>
            {hud.open ? '◯ OPEN' : '● CLOSED'}
          </div>
        </div>
      )}

      {/* ── Legend ───────────────────────────────────────────────────────────── */}
      <div className="viz-legend">
        <span className="viz-legend-item"><span className="viz-dot" style={{ background: '#8899bb' }} />J1 Shaft</span>
        <span className="viz-legend-item"><span className="viz-dot" style={{ background: '#00d4ff' }} />J2 Link</span>
        <span className="viz-legend-item"><span className="viz-dot" style={{ background: '#ff8c00' }} />J3 Link</span>
        <span className="viz-legend-item"><span className="viz-dot" style={{ background: '#ffd700' }} />Joints</span>
        <span className="viz-legend-item"><span className="viz-dot" style={{ background: '#00ff88' }} />End Eff.</span>
        <span className="viz-legend-hint">🖱 Drag to orbit · Scroll to zoom</span>
      </div>
    </div>
  );
}
