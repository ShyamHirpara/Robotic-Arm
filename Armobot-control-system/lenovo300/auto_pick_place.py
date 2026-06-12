#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
auto_pick_place.py  —  Vision-Based Automatic Pick-and-Place
=============================================================
Uses arm_mat_calibration.npz + the distance_estimator detection
pipeline to autonomously pick objects from the mat and place them
in a fixed drop zone to the LEFT of the mat (base=+70 deg).

Required calibration files (same directory):
  camera_calibration.npz      -- from camera_calibration.py
  aruco_plane_calibration.npz -- from aruco_plane_calibrator.py
  arm_mat_calibration.npz     -- from arm_mat_calibration.py

Coordinate flow:
  Camera pixel (cx,cy)
    -> pixel_to_mat_mm()   -> mat (x,y) mm   [aruco plane homography]
    -> mat_to_arm_xy()     -> arm (X,Y) mm   [H_mat_to_arm homography]
    -> compute_pick_ik()   -> (base,sh,el)   [2-link planar IK]
    -> PicoController      -> TCP command    [Pico W servo control]

Keys:
  SPACE   -- pick one object (single shot)
  A       -- toggle auto-loop mode (keep picking until mat is clear)
  R       -- toggle reachability overlay (green=reachable, red=out of range)
  Q       -- quit

Run:
  python auto_pick_place.py              # live mode
  python auto_pick_place.py --dry-run    # simulate (no TCP needed)
  python auto_pick_place.py --test-ik    # print IK table for mat grid
"""

import cv2 as cv
import numpy as np
import os
import sys
import time
import math
import socket
import json
import argparse

# ── Encoding fix for Windows terminal ────────────────────────────────────────
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import config
from distance_estimator import (
    load_calibration,
    load_plane_calibration,
    make_aruco_detector,
    detect_mat_quad,
    pixel_to_mat_mm,
    point_in_quad,
    draw_mat_overlay,
    draw_object_hud,
)
from object_detector import detect_objects


# ═══════════════════════════════════════════════════════════════════════════════
# PHYSICAL CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

PICO_IP   = "192.168.137.50"   # arm on the control PC's 'RoboticArm_PC' hotspot
PICO_PORT = 81

# Arm geometry (cm) — base at table level, shoulder pivot at L1 = 24 cm
ARM_HEIGHT_CM = 0.0
L1, L2, L3   = 24.0, 21.0, 35.0      # shoulder height / upper arm / forearm

# Pick heights (cm above table surface)
Z_APPROACH_CM = 10.0    # hover above object — clears a 5 cm cube with margin
Z_PICK_CM     =  2.5    # gripper at cube mid-height  (5 cm tall / 2)
Z_ASCEND_CM   = 10.0    # rise after gripping before lateral move

# Timing (seconds)
MOVE_SETTLE = 2.0       # pause after each joint move to let arm settle
GRIP_WAIT   = 1.2       # pause after gripper open/close command

# Home (observation) configuration
HOME = {"base": 0.0, "shoulder": -20.0, "elbow": -65.0}

# Drop zone — left of mat, fixed joint angles (no IK needed)
DROP_ZONE = {"base": 70.0, "shoulder": -10.0, "elbow": -20.0}

# Calibration file paths
ARM_MAT_CALIB_FILE = os.path.join(_HERE, "arm_mat_calibration.npz")


# ═══════════════════════════════════════════════════════════════════════════════
# PICO CONTROLLER
# ═══════════════════════════════════════════════════════════════════════════════

class PicoController:
    """
    Direct TCP connection to Pico W on port 81.
    Auto-reconnects if the Node.js backend steals the slot.
    """
    MAX_RETRIES     = 2      # keep low — rapid retries can crash Pico TCP state
    RECONNECT_DELAY = 3.0    # > backend's 2 s retry so we can re-steal slot

    def __init__(self, dry_run: bool = False):
        self.dry_run = dry_run
        self.sock    = None
        self.buf     = ""
        self.state   = dict(HOME)
        self._order  = 0

    # ── Internal socket helpers ───────────────────────────────────────────────

    def _open_socket(self):
        if self.sock:
            try: self.sock.close()
            except OSError: pass
            self.sock = None
        self.buf = ""
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(10.0)
        s.connect((PICO_IP, PICO_PORT))
        s.settimeout(0.05)
        self.sock = s

    def _recv_lines(self):
        if self.dry_run or not self.sock:
            return []
        try:
            data = self.sock.recv(4096)
            if data:
                self.buf += data.decode(errors="replace")
            else:
                raise ConnectionResetError("Pico closed connection.")
        except socket.timeout:
            pass
        lines = []
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            lines.append(line.strip())
        return lines

    # ── Public API ────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """
        Connect to the Pico W and secure the TCP slot.
        Returns True on success, False if connection is unavailable.
        On failure, caller can fall back to dry-run mode.
        """
        if self.dry_run:
            print("[DRY-RUN] PicoController: no TCP connection.")
            return True
        print(f"[TCP] Connecting to Pico at {PICO_IP}:{PICO_PORT} ...")
        try:
            self._open_socket()
            print(f"[TCP] Connected. Stabilising {self.RECONNECT_DELAY:.1f} s ...")
            time.sleep(self.RECONNECT_DELAY)
            self._open_socket()   # re-grab slot after backend's first retry
            print("[TCP] Slot secured.")
            return True
        except (TimeoutError, ConnectionRefusedError, OSError) as exc:
            print(f"[TCP] Cannot connect to Pico: {exc}")
            return False

    def disconnect(self):
        if self.sock:
            try: self.sock.close()
            except OSError: pass
            self.sock = None

    def move_to(self, base: float, shoulder: float, elbow: float,
                gripper: str = "close", timeout_s: float = 20.0) -> bool:
        """
        Send joint command to Pico and wait for 'done' acknowledgment.

        On timeout (e.g. Node.js backend stole the TCP slot), automatically
        reconnects and retries the same command up to MAX_RETRIES times.
        Returns True on success, False after all retries are exhausted.
        """
        if self.dry_run:
            print(f"  [SIM] base={base:.1f}  sh={shoulder:.1f}  "
                  f"el={elbow:.1f}  grip={gripper}")
            self.state = {"base": base, "shoulder": shoulder, "elbow": elbow}
            time.sleep(0.1)
            return True

        self._order += 1
        order = self._order
        cmd = (f"/position_order={order}/axis_1={base:.2f}"
               f"/axis_2={shoulder:.2f}/axis_3={elbow:.2f}"
               f"/gripper={gripper}\n")

        for attempt in range(1, self.MAX_RETRIES + 1):
            try:
                # Re-grab slot before each attempt (displaces backend if it reconnected)
                if not self.sock:
                    self._open_socket()
                self.sock.sendall(cmd.encode())

                t0 = time.time()
                while time.time() - t0 < timeout_s:
                    for line in self._recv_lines():
                        # Pico response: /position_order=N/status=complete
                        if (f"position_order={order}" in line and
                                "status=complete" in line.lower()):
                            self.state = {"base": base,
                                          "shoulder": shoulder,
                                          "elbow": elbow}
                            return True
                        # Also accept if Pico echoes 'done' (older firmware)
                        if (f"position_order={order}" in line and
                                "done" in line.lower()):
                            self.state = {"base": base,
                                          "shoulder": shoulder,
                                          "elbow": elbow}
                            return True
                    time.sleep(0.05)

                # Timeout — backend likely stole the slot; reconnect + retry
                print(f"  [TCP] Timeout order {order} (attempt {attempt}/{self.MAX_RETRIES}). "
                      f"Re-grabbing slot ...")
                time.sleep(self.RECONNECT_DELAY)
                try:
                    self._open_socket()
                except OSError:
                    pass

            except (OSError, ConnectionResetError) as exc:
                print(f"  [TCP] Attempt {attempt}: {exc}. "
                      f"Reconnecting in {self.RECONNECT_DELAY:.1f} s ...")
                time.sleep(self.RECONNECT_DELAY)
                try:
                    self._open_socket()
                except OSError:
                    pass

        print(f"  [TCP] Failed after {self.MAX_RETRIES} retries.")
        return False


# ═══════════════════════════════════════════════════════════════════════════════
# INVERSE KINEMATICS  (2-link planar, elbow-down)
# ═══════════════════════════════════════════════════════════════════════════════

def compute_pick_ik(x_arm_mm: float, y_arm_mm: float,
                    z_target_cm: float
                    ) -> "tuple[float|None, float|None, float|None]":
    """
    Compute (base_deg, shoulder_deg, elbow_deg) to place the arm tip
    (end of L3) at arm-frame position (x_arm_mm, y_arm_mm) at height
    z_target_cm above the table.

    Uses elbow-DOWN configuration (arm folds toward the table).

    FK model (from arm_mat_calibration.py):
        r_cm = L2*cos(s2) + L3*cos(s3)
        z_cm = ARM_HEIGHT + L1 + L2*sin(s2) + L3*sin(s3)

    Solving the 2-equation system via the inter-link angle:
        cos(delta) = (D^2 - L2^2 - L3^2) / (2*L2*L3)   where D = |target - shoulder|
        s2 = atan2(B*r + A*h, A*r - B*h)               where A=L2+L3*cos(d), B=L3*sin(d)
        s3 = s2 - delta                                  (elbow-down)

    Returns (None, None, None) if the target is outside the arm's reach.
    """
    # Base: geometric facing angle (pure rotation to face the target)
    base_deg = -math.degrees(math.atan2(x_arm_mm, -y_arm_mm))

    # Horizontal reach from arm rotation axis (cm)
    r_cm = math.sqrt(x_arm_mm ** 2 + y_arm_mm ** 2) / 10.0

    # Height of target relative to shoulder pivot (negative = below shoulder)
    h_shoulder = ARM_HEIGHT_CM + L1          # shoulder pivot above table (cm)
    h_rel      = z_target_cm - h_shoulder    # cm, typically negative

    # Distance from shoulder to target
    D = math.sqrt(r_cm ** 2 + h_rel ** 2)

    # Reachability: |L2-L3| <= D <= L2+L3
    if D > (L2 + L3) or D < abs(L2 - L3):
        return None, None, None

    # Inter-link angle  Δ = s2 - s3
    cos_d = (D ** 2 - L2 ** 2 - L3 ** 2) / (2.0 * L2 * L3)
    cos_d = max(-1.0, min(1.0, cos_d))
    delta = math.acos(cos_d)

    # Shoulder angle via linear combination
    A  = L2 + L3 * math.cos(delta)
    B  = L3 * math.sin(delta)
    dn = A ** 2 + B ** 2

    s2 = math.atan2(B * r_cm + A * h_rel,
                    A * r_cm - B * h_rel)
    s3 = s2 - delta     # elbow-down: elbow is more negative than shoulder

    return base_deg, math.degrees(s2), math.degrees(s3)


def is_reachable(x_arm_mm: float, y_arm_mm: float,
                 z_cm: float = Z_PICK_CM) -> bool:
    """True if the arm can physically place its tip at (x,y,z)."""
    b, _, _ = compute_pick_ik(x_arm_mm, y_arm_mm, z_cm)
    return b is not None


# ═══════════════════════════════════════════════════════════════════════════════
# CALIBRATION LOADER
# ═══════════════════════════════════════════════════════════════════════════════

def load_arm_calibration() -> dict:
    """
    Load arm_mat_calibration.npz and return a dict with the key transforms.
    """
    if not os.path.exists(ARM_MAT_CALIB_FILE):
        print(f"[ERROR] {ARM_MAT_CALIB_FILE} not found.")
        print("  Run arm_mat_calibration.py first.")
        sys.exit(1)

    d   = np.load(ARM_MAT_CALIB_FILE)
    cal = {
        "H_mat_to_arm": d["H_mat_to_arm"].astype(np.float32),
        "mat_w_mm":     float(d["mat_w_mm"]),
        "mat_h_mm":     float(d["mat_h_mm"]),
        "lat_scale":    float(d["lat_scale"]),
        "lat_offset":   float(d["lat_offset"]),
    }
    print(f"[INFO] Arm-mat calibration: mat={cal['mat_w_mm']:.0f}x"
          f"{cal['mat_h_mm']:.0f} mm  "
          f"scale={cal['lat_scale']:.3f}  offset={cal['lat_offset']:.3f}")
    return cal


def mat_to_arm_xy(mat_x_mm: float, mat_y_mm: float,
                  H: np.ndarray) -> "tuple[float, float]":
    """Apply the H_mat_to_arm homography: mat (mm) -> arm XY (mm)."""
    pt  = np.array([[[float(mat_x_mm), float(mat_y_mm)]]], dtype=np.float32)
    out = cv.perspectiveTransform(pt, H)
    return float(out[0, 0, 0]), float(out[0, 0, 1])


# ═══════════════════════════════════════════════════════════════════════════════
# TARGET SELECTION
# ═══════════════════════════════════════════════════════════════════════════════

def pick_best_target(detections: list, H_mat_to_arm: np.ndarray) -> "dict|None":
    """
    From a list of detections (each with mat_x_mm, mat_y_mm), select the
    nearest reachable object to pick.

    The chosen detection dict is augmented with arm_x_mm / arm_y_mm.
    Returns None if no object is pickable.
    """
    candidates = []
    for det in detections:
        mx = det.get("mat_x_mm")
        my = det.get("mat_y_mm")
        if mx is None or my is None:
            det["reachable"] = False
            continue

        ax, ay = mat_to_arm_xy(mx, my, H_mat_to_arm)
        det["arm_x_mm"] = ax
        det["arm_y_mm"] = ay

        if not is_reachable(ax, ay, Z_PICK_CM):
            det["reachable"] = False
            continue

        det["reachable"] = True
        r_mm = math.sqrt(ax ** 2 + ay ** 2)
        candidates.append((r_mm, det))

    if not candidates:
        return None

    candidates.sort(key=lambda t: t[0])    # nearest first
    return candidates[0][1]


# ═══════════════════════════════════════════════════════════════════════════════
# PICK SEQUENCE
# ═══════════════════════════════════════════════════════════════════════════════

def execute_pick(arm: PicoController,
                 ax_mm: float, ay_mm: float,
                 label: str = "object") -> bool:
    """
    Full 7-step pick-and-place sequence for one object.

    Steps:
      1  Open gripper at HOME
      2  APPROACH — hover 10 cm above object (gripper open)
      3  DESCEND  — lower to 2.5 cm above mat  (gripper open)
      4  GRIP     — close gripper, wait
      5  ASCEND   — rise back to 10 cm         (gripper closed)
      6  TRANSPORT — move to DROP_ZONE         (gripper closed)
      7  DROP     — open gripper, wait
      8  HOME     — return to observation position

    Returns True on success.
    """
    print(f"\n[PICK] label={label}  arm=({ax_mm:.0f}, {ay_mm:.0f}) mm")

    # Compute IK for approach (z=10 cm) and pick (z=2.5 cm)
    base, sh_ap, el_ap = compute_pick_ik(ax_mm, ay_mm, Z_APPROACH_CM)
    _,    sh_pk, el_pk = compute_pick_ik(ax_mm, ay_mm, Z_PICK_CM)

    if base is None:
        print("  [SKIP] IK failed — object is outside arm reach.")
        return False

    print(f"  APPROACH joints: base={base:.1f}  sh={sh_ap:.1f}  el={el_ap:.1f}")
    print(f"  PICK    joints: base={base:.1f}  sh={sh_pk:.1f}  el={el_pk:.1f}")

    # 1 — Open gripper (stay at HOME)
    print("  [1/7] Open gripper ...")
    arm.move_to(HOME["base"], HOME["shoulder"], HOME["elbow"], gripper="open")
    time.sleep(MOVE_SETTLE)

    # 2 — APPROACH
    print("  [2/7] Approaching ...")
    ok = arm.move_to(base, sh_ap, el_ap, gripper="open")
    if not ok:
        print("  [WARN] No ACK for Approach (Node.js may hold slot) — continuing on timer.")
    time.sleep(MOVE_SETTLE)

    # 3 — DESCEND
    print("  [3/7] Descending to pick height ...")
    ok = arm.move_to(base, sh_pk, el_pk, gripper="open")
    if not ok:
        print("  [WARN] No ACK for Descend — continuing on timer.")
    time.sleep(MOVE_SETTLE)

    # 4 — GRIP
    print("  [4/7] Gripping ...")
    arm.move_to(base, sh_pk, el_pk, gripper="close")
    time.sleep(GRIP_WAIT)

    # 5 — ASCEND
    print("  [5/7] Ascending ...")
    arm.move_to(base, sh_ap, el_ap, gripper="close")
    time.sleep(MOVE_SETTLE)

    # 6 — TRANSPORT to drop zone
    print("  [6/7] Transporting to drop zone ...")
    arm.move_to(DROP_ZONE["base"], DROP_ZONE["shoulder"], DROP_ZONE["elbow"],
                gripper="close")
    time.sleep(MOVE_SETTLE)

    # 7 — DROP
    print("  [7/7] Dropping ...")
    arm.move_to(DROP_ZONE["base"], DROP_ZONE["shoulder"], DROP_ZONE["elbow"],
                gripper="open")
    time.sleep(GRIP_WAIT)

    # 8 — Return HOME
    print("  [DONE] Returning home ...")
    _return_home(arm)
    return True


def _return_home(arm: PicoController) -> None:
    """Return arm to HOME observation position."""
    arm.move_to(HOME["base"], HOME["shoulder"], HOME["elbow"], gripper="open")
    time.sleep(MOVE_SETTLE)


# ═══════════════════════════════════════════════════════════════════════════════
# DISPLAY HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

_STATE_COLORS = {
    "IDLE":         (100, 100, 100),
    "DETECTING":    (0,   200, 255),
    "PICKING":      (0,   220,  60),
}


def draw_state_badge(frame: np.ndarray, state: str, auto_mode: bool) -> None:
    """Draw coloured state label in the top-left corner."""
    color = _STATE_COLORS.get(state, (180, 180, 180))
    text  = f"  {state}  "
    font  = cv.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv.getTextSize(text, font, 0.75, 2)
    cv.rectangle(frame, (8, 8), (tw + 16, th + 24), color, -1)
    cv.putText(frame, text, (12, th + 14), font, 0.75, (10, 10, 10), 2, cv.LINE_AA)
    if auto_mode:
        cv.putText(frame, "AUTO", (tw + 24, th + 14),
                   font, 0.65, (0, 200, 255), 2, cv.LINE_AA)


def draw_target_highlight(frame: np.ndarray, det: dict) -> None:
    """Draw a green bullseye on the chosen target object."""
    cx, cy = det["center"]
    ax     = det.get("arm_x_mm", 0)
    ay     = det.get("arm_y_mm", 0)
    cv.circle(frame, (cx, cy), 22, (0, 255, 100), 3)
    cv.circle(frame, (cx, cy), 7,  (0, 255, 100), -1)
    cv.putText(frame, f"TARGET ({ax:.0f},{ay:.0f}) mm",
               (cx + 26, cy - 8), cv.FONT_HERSHEY_SIMPLEX,
               0.44, (0, 255, 100), 1, cv.LINE_AA)


def draw_reachability_overlay(frame: np.ndarray, plane: dict,
                               H_mat_to_arm: np.ndarray) -> None:
    """
    Draw a 9x9 dot grid over the mat.
    Green dot = reachable at Z_PICK_CM.
    Red dot   = outside arm range.
    """
    W   = plane["mat_w_mm"]
    Hm  = plane["mat_h_mm"]
    Hinv = plane["H_inv"]     # mat mm -> pixel
    N    = 9

    for iy in range(N):
        for ix in range(N):
            mx = (ix + 0.5) * W / N
            my = (iy + 0.5) * Hm / N
            ax, ay = mat_to_arm_xy(mx, my, H_mat_to_arm)
            reach  = is_reachable(ax, ay, Z_PICK_CM)
            # Project mat mm -> pixel
            pt = Hinv @ np.array([mx, my, 1.0])
            px = int(pt[0] / pt[2])
            py = int(pt[1] / pt[2])
            color = (0, 200, 80) if reach else (40, 40, 220)
            cv.circle(frame, (px, py), 5, color, -1)


def draw_legend(frame: np.ndarray) -> None:
    fh = frame.shape[0]
    cv.rectangle(frame, (0, fh - 30), (frame.shape[1], fh), (20, 20, 20), -1)
    legend = ("  [SPACE] pick one    [A] auto-loop    "
              "[R] reachability overlay    [Q] quit")
    cv.putText(frame, legend, (6, fh - 9),
               cv.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv.LINE_AA)


# ═══════════════════════════════════════════════════════════════════════════════
# IK TEST MODE
# ═══════════════════════════════════════════════════════════════════════════════

def run_ik_test(cal: dict) -> None:
    """Print IK joint angles for a 5x5 grid across the full mat."""
    H  = cal["H_mat_to_arm"]
    W  = cal["mat_w_mm"]
    Hm = cal["mat_h_mm"]

    print(f"\n{'='*78}")
    print(f"  IK Test  —  5x5 grid across mat ({W:.0f} x {Hm:.0f} mm)")
    print(f"  Approach z = {Z_APPROACH_CM} cm     Pick z = {Z_PICK_CM} cm")
    print(f"  Arm: L1={L1}  L2={L2}  L3={L3}  arm_h={ARM_HEIGHT_CM}  (all cm)")
    print(f"{'='*78}")
    hdr = (f"  {'mat_x':>6} {'mat_y':>6}  {'armX':>6} {'armY':>6}  "
           f"{'base':>6} {'sh_ap':>6} {'el_ap':>6}  "
           f"{'sh_pk':>6} {'el_pk':>6}  REACH")
    print(hdr)
    print(f"{'─'*78}")

    for iy in range(5):
        for ix in range(5):
            mx = (ix + 0.5) * W / 5
            my = (iy + 0.5) * Hm / 5
            ax, ay = mat_to_arm_xy(mx, my, H)

            b_ap, sh_ap, el_ap = compute_pick_ik(ax, ay, Z_APPROACH_CM)
            b_pk, sh_pk, el_pk = compute_pick_ik(ax, ay, Z_PICK_CM)

            reach = "YES" if (b_ap is not None and b_pk is not None) else " NO"
            if b_ap is None:
                b_ap = sh_ap = el_ap = float("nan")
            if b_pk is None:
                sh_pk = el_pk = float("nan")

            print(f"  {mx:6.0f} {my:6.0f}  {ax:6.0f} {ay:6.0f}  "
                  f"{b_ap:6.1f} {sh_ap:6.1f} {el_ap:6.1f}  "
                  f"{sh_pk:6.1f} {el_pk:6.1f}  {reach}")
    print(f"{'='*78}\n")


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN LOOP
# ═══════════════════════════════════════════════════════════════════════════════

def run_pick_place(dry_run: bool = False) -> None:
    """
    Main application loop.

    Camera runs continuously for live feedback.
    Pick sequences run synchronously (camera feed pauses during pick).
    """
    # ── Load calibrations ─────────────────────────────────────────────────────
    K, dist   = load_calibration()
    plane     = load_plane_calibration()
    arm_cal   = load_arm_calibration()
    H_mat_arm = arm_cal["H_mat_to_arm"]

    if plane is None:
        print("[ERROR] aruco_plane_calibration.npz not found.")
        print("  Run aruco_plane_calibrator.py first.")
        return

    detect_fn = make_aruco_detector()

    # ── Connect arm ──────────────────────────────────────────────────────────
    arm = PicoController(dry_run=dry_run)
    if not arm.connect():
        print("[WARN] Pico not reachable — switching to DRY-RUN mode.")
        print("       Join the 'RoboticArm_PC' hotspot (arm at 192.168.137.50) and retry for live mode.")
        arm.dry_run = True   # safe fallback: camera runs, arm simulated

    # ── Open camera ──────────────────────────────────────────────────────────
    cap = cv.VideoCapture(config.CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {config.CAMERA_INDEX}")
        arm.disconnect()
        return
    cap.set(cv.CAP_PROP_FRAME_WIDTH,  config.FRAME_W)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, config.FRAME_H)
    cap.set(cv.CAP_PROP_FPS,          config.FPS)

    print("\n[INFO] Pick-and-Place ready.")
    print(f"  Approach z={Z_APPROACH_CM} cm   Pick z={Z_PICK_CM} cm")
    print(f"  Drop: base={DROP_ZONE['base']}  sh={DROP_ZONE['shoulder']}"
          f"  el={DROP_ZONE['elbow']}")
    print("  NOTE: Stop the Node.js backend (server.js) before running to avoid")
    print("        TCP slot competition. Or ensure Pico is on ARMOBOT WiFi.")
    print("  Keys: [SPACE] pick  [A] auto  [R] reach overlay  [Q] quit\n")

    # Move arm to HOME observation position (short timeout — arm may already be there)
    arm.move_to(HOME["base"], HOME["shoulder"], HOME["elbow"],
                gripper="open", timeout_s=10.0)
    time.sleep(MOVE_SETTLE)

    # ── State ─────────────────────────────────────────────────────────────────
    state       = "IDLE"
    auto_mode   = False
    show_reach  = False
    mat_quad    = None     # cached mat boundary quad
    pick_queued = False
    target_det  = None     # currently chosen pick target (survives across loop iters)

    while True:
        ret, frame = cap.read()
        if not ret:
            # Transient camera grab failure (MSMF) — retry a few times before quitting
            for _ in range(5):
                ret, frame = cap.read()
                if ret:
                    break
                time.sleep(0.05)
            if not ret:
                print("[WARN] Camera grab failed repeatedly — exiting.")
                break

        # ── ArUco + object detection ──────────────────────────────────────────
        frame_ud = cv.undistort(frame, K, dist)
        output   = frame_ud.copy()
        gray     = cv.cvtColor(frame_ud, cv.COLOR_BGR2GRAY)

        quad, n_markers = detect_mat_quad(gray, detect_fn, mat_quad, plane=plane)
        if quad is not None:
            mat_quad = quad
        draw_mat_overlay(output, quad, n_markers)

        all_dets = detect_objects(frame_ud)
        dets     = [d for d in all_dets
                    if point_in_quad(d["center"][0], d["center"][1], quad)]

        # Enrich each detection with mat/arm coordinates
        for i, det in enumerate(dets):
            cx, cy = det["center"]

            # Mat position (mm)
            if quad is not None and n_markers >= 1:
                mx, my = pixel_to_mat_mm(cx, cy, plane["H"])
                det["mat_x_mm"] = mx
                det["mat_y_mm"] = my
            else:
                det["mat_x_mm"] = det["mat_y_mm"] = None

            # Arm position + reachability
            if det["mat_x_mm"] is not None:
                ax, ay = mat_to_arm_xy(mx, my, H_mat_arm)
                det["arm_x_mm"] = ax
                det["arm_y_mm"] = ay
                det["reachable"] = is_reachable(ax, ay, Z_PICK_CM)
            else:
                det["arm_x_mm"] = det["arm_y_mm"] = None
                det["reachable"] = False

            # Draw standard HUD
            draw_object_hud(output, det, i + 1, False, False, K, dist)

            # Arm position + reach badge below object label
            if det["arm_x_mm"] is not None:
                x, _  = det["pt1"]
                _, y2 = det["pt2"]
                ax_d  = det["arm_x_mm"]
                ay_d  = det["arm_y_mm"]
                ok    = det["reachable"]
                col   = (0, 210, 60) if ok else (30, 30, 210)
                cv.putText(output,
                           f"arm({ax_d:.0f},{ay_d:.0f}) {'[PICK OK]' if ok else '[OUT OF REACH]'}",
                           (x + 2, y2 + 36),
                           cv.FONT_HERSHEY_SIMPLEX, 0.36, col, 1, cv.LINE_AA)

        # ── Optional reachability dot grid ───────────────────────────────────
        if show_reach and plane is not None:
            draw_reachability_overlay(output, plane, H_mat_arm)

        # ── State machine ─────────────────────────────────────────────────────

        if state == "IDLE":
            if pick_queued or auto_mode:
                state       = "DETECTING"
                pick_queued = False

        elif state == "DETECTING":
            target_det = pick_best_target(dets, H_mat_arm)
            if target_det is not None:
                state = "PICKING"
            else:
                if not auto_mode:
                    print("[INFO] No reachable object detected. Press [SPACE] to retry.")
                    state = "IDLE"
                # In auto_mode: keep scanning silently

        elif state == "PICKING":
            # Highlight target on this frame before blocking
            draw_state_badge(output, "PICKING", auto_mode)
            draw_target_highlight(output, target_det)
            draw_legend(output)
            cv.imshow("ARMOBOT  Auto Pick & Place", output)
            cv.waitKey(1)

            # Execute blocking pick sequence
            execute_pick(arm,
                         target_det["arm_x_mm"],
                         target_det["arm_y_mm"],
                         target_det.get("label", "object"))

            state      = "DETECTING" if auto_mode else "IDLE"
            target_det = None   # clear after pick completes
            continue    # skip re-draw below (arm just moved back to HOME)

        # ── Draw HUD ─────────────────────────────────────────────────────────
        draw_state_badge(output, state, auto_mode)
        if target_det is not None:
            draw_target_highlight(output, target_det)
        draw_legend(output)

        cv.imshow("ARMOBOT  Auto Pick & Place", output)

        # ── Key handling ─────────────────────────────────────────────────────
        key = cv.waitKey(1) & 0xFF
        if   key == ord("q"):
            break
        elif key == ord(" "):
            pick_queued = True
            state       = "DETECTING"
            print("[KEY] Single pick triggered.")
        elif key == ord("a"):
            auto_mode = not auto_mode
            print(f"[KEY] Auto-mode {'ON' if auto_mode else 'OFF'}")
            if auto_mode:
                state = "DETECTING"
        elif key == ord("r"):
            show_reach = not show_reach
            print(f"[KEY] Reachability overlay {'ON' if show_reach else 'OFF'}")

    # ── Cleanup ───────────────────────────────────────────────────────────────
    cap.release()
    cv.destroyAllWindows()
    _return_home(arm)
    arm.disconnect()
    print("[INFO] Session ended.")


# ═══════════════════════════════════════════════════════════════════════════════
# ENTRY POINT
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="ARMOBOT Vision-Based Auto Pick-and-Place")
    parser.add_argument("--dry-run",  action="store_true",
                        help="Simulate arm movements — no TCP required")
    parser.add_argument("--test-ik",  action="store_true",
                        help="Print 5x5 IK table for the mat and exit")
    args = parser.parse_args()

    if args.test_ik:
        cal = load_arm_calibration()
        run_ik_test(cal)
        return

    run_pick_place(dry_run=args.dry_run)


if __name__ == "__main__":
    main()
