"""
calculate_distance.py  —  Object XYZ + Joint-1 (Base) Tracking
===============================================================
Shows:
  • 4 ArUco corner markers (IDs 0-3) as the detection boundary
  • Each detected object inside that boundary with its bounding box
  • Camera-relative X, Y, Z in mm for every object

Joint-1 (base) control  (from test/config.py + test/vision.py logic):
  dx = X_mm of nearest object  (lateral offset from camera centre)
  If |dx| > DEAD_MM:
      d_j1 = clamp( -dx / STEPPER1,  ±adaptive_limit(dx) )
      base += d_j1  →  sent to Pico W on port 81

Stepper constants  (mirrors test/config.py + related_file.py):
  STEPPER1        = 6   mm per degree  (base)
  DEAD_MM         = 25  mm  (deadzone — don't move inside this)
  MIN_STEP        = 2   °  (ignore micro-steps smaller than this)
  MAX_STEP_SMALL  = 4   °  (|dx| ≤ 40 mm)
  MAX_STEP_MED    = 10  °  (40 < |dx| ≤ 80 mm)
  MAX_STEP_LARGE  = 18  °  (|dx| > 80 mm)

Keys:
    q  — quit
    s  — save snapshot
    a  — toggle arm control ON / OFF
    c  — print current detections to terminal
    r  — reset base angle to 0°
"""

import cv2 as cv
import numpy as np
import os, sys, time, socket

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import config
from object_detector import detect_objects
from distance_estimator import (
    load_calibration,
    load_plane_calibration,
    make_aruco_detector,
    detect_mat_quad,
    point_in_quad,
    build_object_points,
    get_image_corners,
    triangle_similarity_distance,
)

_FONT = cv.FONT_HERSHEY_SIMPLEX


# ══════════════════════════════════════════════════════════════════════════════
# ARM CONTROL CONSTANTS  (mirrors test/config.py + related_file.py)
# ══════════════════════════════════════════════════════════════════════════════

PICO_IP    = "192.168.4.1"
PICO_PORT  = 81

STEPPER1       = 6     # mm per degree  (base joint 1)   -- ACTIVE
# STEPPER2     = 9     # mm per degree  (shoulder joint 2) -- COMMENTED OUT
# STEPPER3     = 6     # mm per degree  (elbow    joint 3) -- COMMENTED OUT

DEAD_MM        = 15    # mm deadzone for X axis
# Y_DEAD_MM    = 7     # NOT USED
# Z_DEAD_MM    = 20    # NOT USED

MIN_STEP       = 2     # ignore steps smaller than this (degrees)
MAX_STEP_SMALL = 4     # degree cap when |dx| <= 40 mm
MAX_STEP_MED   = 10    # degree cap when 40 < |dx| <= 80 mm
MAX_STEP_LARGE = 18    # degree cap when |dx| > 80 mm

HOME_SHOULDER  = -20.0  # shoulder held fixed at home (not moved)
HOME_ELBOW     = -65.0  # elbow    held fixed at home (not moved)
# HOME_GRIPPER = "open"  # gripper  held fixed open   (not moved)
BASE_MIN       = -175.0
BASE_MAX      =  175.0


def adaptive_limit(distance_mm: float) -> int:
    """Return step cap (°) based on |dx| — same as related_file.py."""
    d = abs(distance_mm)
    if d > 80:
        return MAX_STEP_LARGE
    if d > 40:
        return MAX_STEP_MED
    return MAX_STEP_SMALL


# ══════════════════════════════════════════════════════════════════════════════
# TCP SENDER  (fire-and-forget, no blocking wait for ACK)
# ══════════════════════════════════════════════════════════════════════════════

class _PicoSender:
    """Minimal TCP sender for joint-1 incremental commands."""

    def __init__(self):
        self._sock  = None
        self._order = 0

    def connect(self) -> bool:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3.0)
            s.connect((PICO_IP, PICO_PORT))
            self._sock = s
            print(f"[TCP] Connected to Pico at {PICO_IP}:{PICO_PORT}")
            return True
        except (TimeoutError, ConnectionRefusedError, OSError) as e:
            print(f"[TCP] Cannot connect: {e}")
            self._sock = None
            return False

    def send_base(self, base_deg: float) -> bool:
        """
        Send ONLY joint-1 (base) position to Pico.
        Joints 2, 3 and gripper are HELD at home values (not controlled).
        Fire-and-forget — does NOT wait for status=complete.
        """
        self._order += 1

        # ── Joint 1 (base) — ACTIVE ─────────────────────────────────────────
        axis1_cmd = f"/axis_1={base_deg:.2f}"

        # ── Joint 2 (shoulder) — COMMENTED OUT (held at home) ───────────────
        # axis2_cmd = f"/axis_2={shoulder_deg:.2f}"   # uncomment to enable
        # axis2_cmd = f"/axis_2={HOME_SHOULDER:.2f}"     # fixed home value

        # ── Joint 3 (elbow) — COMMENTED OUT (held at home) ──────────────────
        # axis3_cmd = f"/axis_3={elbow_deg:.2f}"       # uncomment to enable
        # axis3_cmd = f"/axis_3={HOME_ELBOW:.2f}"        # fixed home value

        # ── Gripper — COMMENTED OUT (held open) ──────────────────────────────
        # gripper_cmd = "/gripper=close"               # uncomment to enable
        gripper_cmd = "/gripper=open"                  # fixed open

        cmd = (f"/position_order={self._order}"
               + axis1_cmd
               + axis2_cmd
               + axis3_cmd
               + gripper_cmd + "\n")

        try:
            if self._sock is None:
                self.connect()
            if self._sock:
                self._sock.sendall(cmd.encode())
                return True
        except (OSError, BrokenPipeError) as e:
            print(f"[TCP] Send error: {e} -- reconnecting next frame")
            self._sock = None
        return False

    def disconnect(self):
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None


# ══════════════════════════════════════════════════════════════════════════════
# XYZ via solvePnP
# ══════════════════════════════════════════════════════════════════════════════

def get_xyz_mm(det, K, dist_coeffs):
    """Returns (x_mm, y_mm, z_mm) — tvec from solvePnP × 10 (cm→mm)."""
    if det.get("w_cm") is None:
        return None, None, None

    x, y = det["pt1"]
    w, h = det["pixel_w"], det["pixel_h"]
    cx, cy = det["center"]

    z_init = triangle_similarity_distance(w, h, det["w_cm"], det["h_cm"], K, cx, cy)
    if not (0 < z_init < 500):
        z_init = 30.0

    obj_pts = build_object_points(det["w_cm"], det["h_cm"])
    img_pts = get_image_corners(x, y, w, h)

    ok, _, tvec = cv.solvePnP(
        obj_pts, img_pts, K, dist_coeffs,
        rvec=np.zeros((3, 1), np.float64),
        tvec=np.array([[0.], [0.], [z_init]]),
        useExtrinsicGuess=True,
        flags=cv.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None, None, None

    return (float(tvec[0][0]) * 10.0,
            float(tvec[1][0]) * 10.0,
            float(tvec[2][0]) * 10.0)


# ══════════════════════════════════════════════════════════════════════════════
# DRAW HELPERS
# ══════════════════════════════════════════════════════════════════════════════

_ARUCO_COLORS = [(0, 230, 80), (0, 210, 255), (255, 120, 0), (255, 30, 200)]
_ARUCO_LABELS = ["ID:0 TL", "ID:1 TR", "ID:2 BR", "ID:3 BL"]


def draw_aruco_boundary(output, quad, n_markers):
    if quad is None:
        return
    pts = quad.astype(np.int32)
    overlay = output.copy()
    cv.fillConvexPoly(overlay, pts, (0, 180, 50))
    cv.addWeighted(overlay, 0.10, output, 0.90, 0, output)
    for i in range(4):
        cv.line(output, tuple(pts[i]), tuple(pts[(i+1) % 4]),
                (0, 210, 60), 2, cv.LINE_AA)
    for pt, lbl, col in zip(pts, _ARUCO_LABELS, _ARUCO_COLORS):
        cv.circle(output, tuple(pt), 8, col, -1)
        cv.circle(output, tuple(pt), 10, (255, 255, 255), 1)
        ox, oy = pt[0] + 12, pt[1] + 5
        cv.putText(output, lbl, (ox+1, oy+1), _FONT, 0.42, (0,0,0), 2, cv.LINE_AA)
        cv.putText(output, lbl, (ox, oy),     _FONT, 0.42, col,     1, cv.LINE_AA)
    fh, fw = output.shape[:2]
    col  = (0, 220, 60) if n_markers == 4 else (0, 180, 255)
    cv.putText(output, f"ArUco: {n_markers}/4",
               (fw - 140, fh - 44), _FONT, 0.45, col, 1, cv.LINE_AA)


def draw_object(output, det, idx, x_mm, y_mm, z_mm, is_target: bool = False):
    """Bounding box + XYZ tag. Target object gets a thicker highlight."""
    bx1, by1 = det["pt1"]
    bx2, by2 = det["pt2"]
    label     = det["label"]
    color     = det["color_bgr"]
    thickness = 3 if is_target else 2

    cv.rectangle(output, (bx1, by1), (bx2, by2), color, thickness, cv.LINE_AA)
    if is_target:
        cv.rectangle(output, (bx1-2, by1-2), (bx2+2, by2+2),
                     (0, 255, 255), 1, cv.LINE_AA)

    # Label above box
    cv.putText(output, f"#{idx} {label}", (bx1+2, by1-8),
               _FONT, 0.50, color, 2, cv.LINE_AA)

    # XYZ below box
    if x_mm is not None:
        tag = f"X:{int(x_mm):+d}  Y:{int(y_mm):+d}  Z:{int(z_mm):d} mm"
        cv.putText(output, tag, (bx1+3, by2+19), _FONT, 0.48, (0,0,0), 2, cv.LINE_AA)
        cv.putText(output, tag, (bx1+2, by2+18), _FONT, 0.48, color,   1, cv.LINE_AA)


def draw_arm_hud(output, arm_active: bool, base_deg: float,
                 dx: float, d_j1: int):
    """
    Draw the joint-1 control HUD panel in the top-left corner.
    Shows dx, d_j1, current base angle — same data as related_file.py prints.
    """
    # Background strip
    cv.rectangle(output, (0, 0), (460, 58), (20, 20, 20), -1)

    # Row 1 — arm mode
    mode_col  = (0, 230, 80) if arm_active else (60, 60, 200)
    mode_str  = "ARM CTRL: ON  [A]=off" if arm_active else "ARM CTRL: OFF [A]=on"
    cv.putText(output, mode_str, (6, 16), _FONT, 0.44, mode_col, 1, cv.LINE_AA)

    # Row 2 — control values
    if arm_active:
        info = (f"dx={int(dx):+d} mm   d_J1={d_j1:+d}°   "
                f"base={base_deg:.1f}°   dead={DEAD_MM} mm   stp={STEPPER1} mm/°")
    else:
        info = f"base={base_deg:.1f}°   [R]=reset   stepper={STEPPER1} mm/°"
    cv.putText(output, info, (6, 38), _FONT, 0.38, (200, 200, 200), 1, cv.LINE_AA)

    # dx bar (only when active)
    if arm_active and abs(dx) > 0:
        bar_w = min(int(abs(dx) * 1.5), 200)
        bar_col = (0, 80, 255) if abs(dx) > DEAD_MM else (0, 200, 80)
        bx = 6 if dx < 0 else 230
        cv.rectangle(output, (bx, 44), (bx + bar_w, 54), bar_col, -1)
        cv.putText(output, "dx", (6, 53), _FONT, 0.30, (180,180,180), 1)


# ══════════════════════════════════════════════════════════════════════════════
# MAIN LOOP
# ══════════════════════════════════════════════════════════════════════════════

def run_camera(cam_index: int = config.CAMERA_INDEX) -> None:
    K, dist_coeffs = load_calibration()
    plane          = load_plane_calibration()
    detect_aruco   = make_aruco_detector()
    mat_quad       = None

    pico = _PicoSender()
    pico.connect()          # try once; will retry silently on each send

    cap = cv.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {cam_index}")
        return
    cap.set(cv.CAP_PROP_FRAME_WIDTH,  config.FRAME_W)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, config.FRAME_H)
    cap.set(cv.CAP_PROP_FPS,          config.FPS)

    print("\n[INFO] calculate_distance.py — XYZ + Joint-1 tracking")
    print(f"       STEPPER1={STEPPER1} mm/°   DEAD_MM={DEAD_MM} mm")
    print("       Keys: [q]=quit  [s]=save  [a]=arm ctrl  [r]=reset base  [c]=print\n")

    WIN        = "ARMOBOT  XYZ + Base Tracking"
    arm_active = False      # press [A] to enable arm movement
    base_deg   = 0.0        # current base angle (tracked here)
    dx         = 0.0        # last computed lateral error (mm)
    d_j1       = 0          # last computed joint-1 delta (°)
    prev_time  = time.time()
    last_cmd_t = 0.0
    CMD_INTERVAL = 0.15     # seconds between TCP commands (matches test/config STEP_INTERVAL)

    cv.namedWindow(WIN, cv.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            for _ in range(5):
                ret, frame = cap.read()
                if ret:
                    break
                time.sleep(0.05)
            if not ret:
                print("[WARN] Camera grab failed — exiting.")
                break

        frame_ud = cv.undistort(frame, K, dist_coeffs)
        output   = frame_ud.copy()
        gray     = cv.cvtColor(frame_ud, cv.COLOR_BGR2GRAY)

        # ── ArUco boundary ────────────────────────────────────────────────────
        quad, n_markers = detect_mat_quad(gray, detect_aruco, mat_quad, plane=plane)
        if quad is not None:
            mat_quad = quad
        draw_aruco_boundary(output, quad, n_markers)

        # ── Object detection (inside ArUco boundary) ──────────────────────────
        all_dets   = detect_objects(frame_ud)
        detections = [d for d in all_dets
                      if point_in_quad(d["center"][0], d["center"][1], quad)]

        # ── Compute XYZ for each; pick nearest as tracking target ─────────────
        xyz_list = []
        for det in detections:
            xm, ym, zm = get_xyz_mm(det, K, dist_coeffs)
            det["x_mm"] = xm
            det["y_mm"] = ym
            det["z_mm"] = zm
            xyz_list.append((zm if zm is not None else 9999, det))

        xyz_list.sort(key=lambda t: t[0])   # nearest first (smallest Z)

        # ── Joint-1 control (base tracking) ──────────────────────────────────
        dx    = 0.0
        d_j1  = 0

        if xyz_list:
            _, target_det = xyz_list[0]
            x_mm = target_det.get("x_mm")

            if x_mm is not None:
                dx = x_mm   # lateral error from camera centre (mm)

                if abs(dx) > DEAD_MM:
                    # Same formula as related_file.py / test/vision.py
                    raw = int(-dx / STEPPER1)
                    limit = adaptive_limit(dx)
                    d_j1 = max(-limit, min(limit, raw))

                    if abs(d_j1) < MIN_STEP:
                        d_j1 = 0

                # Send command at controlled rate
                now = time.time()
                if arm_active and d_j1 != 0 and (now - last_cmd_t) >= CMD_INTERVAL:
                    base_deg = max(BASE_MIN, min(BASE_MAX, base_deg + d_j1))
                    pico.send_base(base_deg)
                    last_cmd_t = now
                # ── Terminal status print ─────────────────────────────────────
                    curr_deg     = base_deg - d_j1          # degree BEFORE this move
                    curr_pos_mm  = curr_deg * STEPPER1      # mm BEFORE this move
                    tgt_deg      = base_deg                 # degree AFTER this move
                    tgt_pos_mm   = base_deg * STEPPER1      # mm AFTER this move
                    dist_deg     = d_j1                     # delta degrees
                    dist_pos_mm  = d_j1 * STEPPER1         # delta mm

                    print(
                        f"[BASE] "
                        f"curr_deg={curr_deg:+.1f} deg  "
                        f"curr_pos={curr_pos_mm:+.0f} mm"
                        f"  |  "
                        f"tgt_deg={tgt_deg:+.1f} deg  "
                        f"tgt_pos={tgt_pos_mm:+.0f} mm"
                        f"  |  "
                        f"dist_deg={dist_deg:+d} deg  "
                        f"dist_pos={dist_pos_mm:+.0f} mm"
                    )

        # ── Draw objects ──────────────────────────────────────────────────────
        for i, (_, det) in enumerate(xyz_list):
            is_target = (i == 0)
            draw_object(output, det, i+1,
                        det.get("x_mm"), det.get("y_mm"), det.get("z_mm"),
                        is_target=is_target)

        # ── Arm HUD ───────────────────────────────────────────────────────────
        draw_arm_hud(output, arm_active, base_deg, dx, d_j1)

        # ── Status bar ────────────────────────────────────────────────────────
        now       = time.time()
        fps       = 1.0 / max(now - prev_time, 1e-6)
        prev_time = now
        fh, fw    = output.shape[:2]
        cv.rectangle(output, (0, fh-28), (fw, fh), (20,20,20), -1)
        cv.putText(output,
                   f"  {len(detections)} obj in ROI / {len(all_dets)} total"
                   f"   FPS:{fps:.0f}"
                   "   [q]=quit [s]=save [a]=arm [r]=reset [c]=print",
                   (6, fh-8), _FONT, 0.38, (200,200,200), 1, cv.LINE_AA)

        cv.imshow(WIN, output)

        key = cv.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break
        elif key == ord('s'):
            ts    = time.strftime("%Y%m%d_%H%M%S")
            fname = f"xyz_snapshot_{ts}.jpg"
            cv.imwrite(fname, output)
            print(f"[INFO] Saved {fname}")
        elif key == ord('a') or key == ord('A'):
            arm_active = not arm_active
            print(f"[ARM] Control {'ENABLED' if arm_active else 'DISABLED'}")
        elif key == ord('r') or key == ord('R'):
            base_deg = 0.0
            print("[ARM] Base angle reset to 0°")
        elif key == ord('c'):
            print(f"\n-- Detections ({len(detections)}) --  base={base_deg:.1f}°")
            for i, (_, d) in enumerate(xyz_list):
                xm = d.get("x_mm"); ym = d.get("y_mm"); zm = d.get("z_mm")
                tag = (f"X:{int(xm):+d}  Y:{int(ym):+d}  Z:{int(zm):d} mm"
                       if xm is not None else "XYZ: --")
                print(f"  #{i+1}  {d['label']:<12s}  {tag}"
                      + (" ← TARGET" if i == 0 else ""))
            print()

    pico.disconnect()
    cap.release()
    cv.destroyAllWindows()
    print("[INFO] Session ended.")


if __name__ == "__main__":
    run_camera()
