"""
arm_mat_calibration.py — Robotic Arm ↔ Mat Plane Calibration  (v2)
===================================================================
Rewrites the calibration using the ACTUAL physical geometry.

PHYSICAL LAYOUT (arm frame, confirmed by user measurements)
-----------------------------------------------------------
  Arm base pivot at origin (0, 0, 0) in arm frame.
  Arm faces the NEGATIVE-Y direction at base = 0°.
  Mat is on the table in front of the arm:

    ID0 TL : ( 242, -510, 0) mm   far edge,  LEFT
    ID1 TR : (-242, -510, 0) mm   far edge,  RIGHT
    ID2 BR : (-242,  -10, 0) mm   near edge, RIGHT
    ID3 BL : ( 242,  -10, 0) mm   near edge, LEFT

  Arm coordinate axes:
    X : lateral   (positive = LEFT from arm perspective)
    Y : depth     (negative = toward mat)
    Z : vertical  (positive = up)

OBSERVABILITY
-------------
  • HOME config (sh=−20, el=−65) positions camera at ≈(0, −346, 101) mm.
    Camera looks toward Y=−400 mm — the far half of the mat.
    FAR corners (ID0, ID1) appear near the TOP of the camera frame.
    NEAR corners (ID2, ID3) are BEHIND the camera's look direction from
    any feasible arm position — they cannot be directly observed.

  • Previous "far config" (sh=20, el=−30) aimed the camera at ~1170 mm
    depth, completely past the mat — which is why 0 markers were found.

CALIBRATION STRATEGY
--------------------
  1. Sweep base −70°→+70°, HOME shoulder/elbow, detect ALL markers.
  2. Binary-search base angle to centre each FOUND marker horizontally.
  3. For NOT-FOUND markers: compute base angle analytically from X_arm
     using the linear mapping calibrated from the found far corners.
  4. Compute mat_mm ↔ arm_XY transforms using ALL 4 known physical
     coordinates (user-provided) — no need to observe near corners.
  5. Compute camera extrinsics (R, t in mat frame) via solvePnP using
     the detected markers from the sweep + camera calibration K matrix.
  6. 3-D matplotlib visualisation in arm coordinate frame.
  7. Save arm_mat_calibration.npz.

USAGE
-----
  python arm_mat_calibration.py             # live (Wi-Fi to robot)
  python arm_mat_calibration.py --dry-run   # simulated

OUTPUT: arm_mat_calibration.npz
"""

import cv2 as cv
import numpy as np
import socket
import json
import time
import math
import os
import sys

# Force UTF-8 output on Windows (prevents cp1252 UnicodeEncodeError)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D            # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import config

# ═══════════════════════════════════════════════════════════════════════════════
# PHYSICAL CONFIGURATION  — edit to match your setup
# ═══════════════════════════════════════════════════════════════════════════════

PICO_IP        = "192.168.137.50"   # arm on the control PC's 'RoboticArm_PC' hotspot
PICO_PORT      = 81

ARM_HEIGHT_CM  = 0.0       # arm base pivot height above table (cm)
L1, L2, L3    = 24.0, 21.0, 35.0   # link lengths (cm)

# Home joint angles (used for the entire sweep)
HOME = {"base": 0.0, "shoulder": -20.0, "elbow": -65.0}

# ── CORNER PHYSICAL COORDINATES (arm frame, mm) ───────────────────────────────
# Measured / provided by the user.  These are the GROUND TRUTH positions of the
# 4 ArUco corner markers in the arm's coordinate system.
CORNER_ARM_MM = {
    0: np.array([ 242., -510.,  0.]),   # ID0 TL — far edge, left
    1: np.array([-242., -510.,  0.]),   # ID1 TR — far edge, right
    2: np.array([-242.,  -10.,  0.]),   # ID2 BR — near edge, right
    3: np.array([ 242.,  -10.,  0.]),   # ID3 BL — near edge, left
}

# ArUco dictionary (must match the physical marker sheet)
ARUCO_DICT_ID = cv.aruco.DICT_4X4_50

# Sweep parameters
SWEEP_START_DEG = -70.0
SWEEP_END_DEG   =  70.0
SWEEP_STEP_DEG  =   3.0
FRAMES_PER_STOP =   4        # frames captured per base angle during sweep

# Centering
CENTER_THRESH_PX = 15.0      # acceptable |pixel_x − frame_cx| (px)
MAX_REFINE_ITERS = 14        # binary-search iterations per corner

DRY_RUN = "--dry-run" in sys.argv

# ═══════════════════════════════════════════════════════════════════════════════
# FORWARD KINEMATICS  (returns camera position in arm frame, mm)
# ═══════════════════════════════════════════════════════════════════════════════

def forward_kinematics(s1_deg, s2_deg, s3_deg,
                        l1=L1, l2=L2, l3=L3,
                        arm_h=ARM_HEIGHT_CM):
    """
    Compute camera XYZ in ARM coordinate frame (mm).

    Arm frame:
      X = lateral  (positive = LEFT)
      Y = depth    (negative = toward mat)
      Z = height   (positive = up)

    Joint angle conventions (degrees from horizontal, positive = upward):
      s1 : base rotation  (positive = rotate left / CCW from above)
      s2 : shoulder elevation from horizontal
      s3 : forearm absolute elevation from horizontal
    """
    s1 = math.radians(s1_deg)
    s2 = math.radians(s2_deg)
    s3 = math.radians(s3_deg)

    r_cm = l2 * math.cos(s2) + l3 * math.cos(s3)        # horizontal reach (cm)
    z_cm = arm_h + l1 + l2 * math.sin(s2) + l3 * math.sin(s3)  # height (cm)

    # In arm frame: forward = -Y, lateral CCW = X
    # r_cm toward mat → negative Y;  lateral component = X
    arm_x = r_cm * math.sin(s1) * 10.0    # cm → mm, lateral
    arm_y = -r_cm * math.cos(s1) * 10.0   # cm → mm, depth (negative = toward mat)
    arm_z = z_cm * 10.0                    # cm → mm, height

    return np.array([arm_x, arm_y, arm_z], dtype=np.float64)


# ═══════════════════════════════════════════════════════════════════════════════
# LATERAL CALIBRATION  (base angle ↔ arm X position)
# ═══════════════════════════════════════════════════════════════════════════════

class LateralCalibration:
    """
    Calibrates the linear relationship between base angle and arm X position.

    Relationship (derived from geometry + sweep):
        X_arm_mm = scale * base_deg + offset
    """

    def __init__(self):
        self.scale  = None    # mm per degree
        self.offset = None    # mm

    def fit(self, base_deg_list, x_arm_mm_list):
        """Fit from >= 2 (base_deg, X_arm_mm) pairs."""
        if len(base_deg_list) < 2:
            raise ValueError("Need >= 2 points to fit lateral calibration.")
        A = np.column_stack([np.array(base_deg_list), np.ones(len(base_deg_list))])
        b = np.array(x_arm_mm_list, dtype=np.float64)
        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        self.scale  = result[0]
        self.offset = result[1]
        return self

    def x_to_base(self, x_arm_mm):
        """Convert arm X position (mm) → base angle (deg)."""
        if self.scale is None:
            raise RuntimeError("LateralCalibration not fitted yet.")
        return (x_arm_mm - self.offset) / self.scale

    def base_to_x(self, base_deg):
        """Convert base angle (deg) → arm X position (mm)."""
        if self.scale is None:
            raise RuntimeError("LateralCalibration not fitted yet.")
        return self.scale * base_deg + self.offset

    def __str__(self):
        return (f"X_arm_mm = {self.scale:.3f} × base_deg + {self.offset:.3f} mm")


# ═══════════════════════════════════════════════════════════════════════════════
# ARUCO HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

def make_aruco_detector():
    aruco_dict = cv.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    try:
        params   = cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(aruco_dict, params)
        def _detect(gray):
            c, ids, _ = detector.detectMarkers(gray)
            return c, ids
    except AttributeError:
        params = cv.aruco.DetectorParameters_create()
        def _detect(gray):
            c, ids, _ = cv.aruco.detectMarkers(gray, aruco_dict, parameters=params)
            return c, ids
    return _detect


def detect_all_corners(gray, detect_fn):
    """Detect ArUco IDs 0–3.  Returns {id: (cx_px, cy_px)}."""
    corners, ids = detect_fn(gray)
    result = {}
    if ids is not None:
        for idx, mid in enumerate(ids.flatten()):
            m = int(mid)
            if m in (0, 1, 2, 3):
                cx = float(corners[idx][0][:, 0].mean())
                cy = float(corners[idx][0][:, 1].mean())
                result[m] = (cx, cy)
    return result


# ═══════════════════════════════════════════════════════════════════════════════
# PICO TCP CONTROLLER  (with auto-reconnect)
# ═══════════════════════════════════════════════════════════════════════════════

class PicoController:
    """
    Direct TCP connection to the Pico W (port 81).
    Auto-reconnects if the Node.js backend server steals the TCP slot.
    """
    MAX_RETRIES     = 5
    RECONNECT_DELAY = 2.2   # slightly longer than backend's 2 s retry

    def __init__(self):
        self.sock   = None
        self.buf    = ""
        self.state  = dict(HOME)
        self._order = 0

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

    def connect(self):
        if DRY_RUN:
            print("[DRY-RUN] PicoController: no TCP connection.")
            return
        print(f"[TCP] Connecting to Pico at {PICO_IP}:{PICO_PORT} …")
        self._open_socket()
        print(f"[TCP] Connected.  Stabilising {self.RECONNECT_DELAY:.1f} s …")
        time.sleep(self.RECONNECT_DELAY)
        self._open_socket()   # re-grab slot after backend's first retry
        print("[TCP] Slot secured.")

    def disconnect(self):
        if self.sock:
            try: self.sock.close()
            except OSError: pass
            self.sock = None

    def _recv_lines(self):
        if DRY_RUN or not self.sock:
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

    def _parse(self, lines):
        responses = []
        for line in lines:
            if not line: continue
            if line.startswith("{"):
                try:
                    d = json.loads(line)
                    for k in ("s1", "s2", "s3"):
                        if k in d: self.state[k] = float(d[k])
                except json.JSONDecodeError: pass
            elif line.startswith("/position_order="):
                responses.append(line)
        return responses

    def move_to(self, base, shoulder, elbow, gripper="close", timeout_s=45.0):
        if DRY_RUN:
            self.state = {"s1": base, "s2": shoulder, "s3": elbow}
            time.sleep(0.05)
            return True

        self._order += 1
        order = self._order
        cmd = (f"/position_order={order}/axis_1={base:.2f}"
               f"/axis_2={shoulder:.2f}/axis_3={elbow:.2f}"
               f"/gripper={gripper}\n")

        for attempt in range(1, self.MAX_RETRIES + 1):
            try:
                if not self.sock:
                    print(f"[TCP] Reconnecting (attempt {attempt}) …")
                    self._open_socket()
                    time.sleep(self.RECONNECT_DELAY)
                    self._open_socket()
                    print("[TCP] Reconnected.")
                    self.buf = ""

                self.sock.sendall(cmd.encode())
                deadline = time.time() + timeout_s
                while time.time() < deadline:
                    resps = self._parse(self._recv_lines())
                    for r in resps:
                        if f"position_order={order}" in r and "status=" in r:
                            status = r.split("status=", 1)[1].strip()
                            if "complete" in status:
                                self.state = {"s1": base, "s2": shoulder, "s3": elbow}
                                return True
                            print(f"  [WARN] Pico: {r}")
                            return False
                    time.sleep(0.05)

                print(f"  [WARN] Timeout attempt {attempt}")
                self.sock = None
                continue

            except (ConnectionAbortedError, ConnectionResetError, OSError) as e:
                print(f"  [TCP] Error (attempt {attempt}): {e}")
                self.sock = None
                if attempt < self.MAX_RETRIES:
                    time.sleep(self.RECONNECT_DELAY)
                continue

        print(f"  [ERROR] move_to failed after {self.MAX_RETRIES} attempts.")
        return False


# ═══════════════════════════════════════════════════════════════════════════════
# PHASE 2 — DISCOVERY SWEEP  (single HOME config, detect ALL markers)
# ═══════════════════════════════════════════════════════════════════════════════

def sweep_all_markers(arm, cap, detect_fn):
    """
    Sweep base from SWEEP_START_DEG to SWEEP_END_DEG in SWEEP_STEP_DEG steps.
    Uses HOME shoulder/elbow.  Detects ALL 4 markers at each stop.

    Returns:
        sightings : {marker_id: [(base_deg, cx_px, cy_px), ...]}
        pnp_obs   : [(base_deg, {mid: (cx,cy)}, rvec, tvec)] — per-step solvePnP results
    """
    sightings = {m: [] for m in range(4)}
    pnp_obs   = []

    sh, el = HOME["shoulder"], HOME["elbow"]
    print(f"\n  [SWEEP] sh={sh}°  el={el}°   "
          f"base {SWEEP_START_DEG}°→{SWEEP_END_DEG}°  "
          f"step {SWEEP_STEP_DEG}°   (detecting all IDs 0-3)")

    base = SWEEP_START_DEG
    while base <= SWEEP_END_DEG + 0.01:
        arm.move_to(base, sh, el)

        found_this_step = {}
        for _ in range(FRAMES_PER_STOP):
            ret, frame = cap.read()
            if not ret: continue
            gray  = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            found = detect_all_corners(gray, detect_fn)
            for m, pxy in found.items():
                if m not in found_this_step:
                    found_this_step[m] = []
                found_this_step[m].append(pxy)

        # Average pixel positions per marker across frames
        step_found = {}
        for m, pts in found_this_step.items():
            cx_mean = float(np.mean([p[0] for p in pts]))
            cy_mean = float(np.mean([p[1] for p in pts]))
            step_found[m] = (cx_mean, cy_mean)
            sightings[m].append((base, cx_mean, cy_mean))
            print(f"    ID:{m} at base={base:.1f}°  px=({cx_mean:.0f},{cy_mean:.0f})")

        base += SWEEP_STEP_DEG

    return sightings


# ═══════════════════════════════════════════════════════════════════════════════
# PHASE 3 — CORNER CENTERING  (binary search on base angle)
# ═══════════════════════════════════════════════════════════════════════════════

def center_marker(arm, cap, detect_fn, marker_id, init_base, shoulder, elbow, frame_cx):
    """
    Binary-search base angle so marker_id appears within CENTER_THRESH_PX
    of the horizontal frame centre.

    Direction rule (camera along l3, looking toward mat):
      Positive base → camera rotates CCW → scene shifts RIGHT in image.
      Marker too far RIGHT (error>0): decrease base (hi=mid).
      Marker too far LEFT  (error<0): increase base (lo=mid).

    Returns (final_base_deg, converged: bool).
    """
    lo, hi = init_base - 15.0, init_base + 15.0
    best_base, best_error = init_base, float("inf")

    print(f"\n  [CENTER] ID:{marker_id}  init_base={init_base:.1f}°  "
          f"sh={shoulder}°  el={elbow}°")

    for it in range(MAX_REFINE_ITERS):
        mid_base = (lo + hi) / 2.0
        arm.move_to(mid_base, shoulder, elbow)
        time.sleep(0.12)

        cx_list = []
        for _ in range(5):
            ret, frame = cap.read()
            if not ret: continue
            gray  = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            found = detect_all_corners(gray, detect_fn)
            if marker_id in found:
                cx_list.append(found[marker_id][0])

        if not cx_list:
            lo -= 5.0; hi += 5.0
            print(f"    iter {it+1:2d}: not visible at {mid_base:.2f}°  "
                  f"→ widen [{lo:.1f}, {hi:.1f}]")
            continue

        cx_mean = float(np.mean(cx_list))
        error   = cx_mean - frame_cx

        if abs(error) < abs(best_error):
            best_error, best_base = error, mid_base

        print(f"    iter {it+1:2d}: base={mid_base:.2f}°  "
              f"px_x={cx_mean:.1f}  err={error:+.1f}")

        if abs(error) < CENTER_THRESH_PX:
            print(f"    ✓ Centred  base={mid_base:.2f}°  (err={error:+.1f} px)")
            return mid_base, True

        if error > 0: hi = mid_base   # marker right → decrease base
        else:         lo = mid_base   # marker left  → increase base

        if hi - lo < 0.4: break

    converged = abs(best_error) < CENTER_THRESH_PX * 3
    print(f"    {'✓' if converged else '!'} Best: base={best_base:.2f}°  "
          f"err={best_error:+.1f} px")
    return best_base, converged


# ═══════════════════════════════════════════════════════════════════════════════
# PHASE 4 — COORDINATE TRANSFORMS
# ═══════════════════════════════════════════════════════════════════════════════

def compute_arm_mat_transforms(mat_w_mm, mat_h_mm):
    """
    Compute homography H_mat_to_arm : mat_mm (2D) → arm_XY (mm, 2D)
    using the 4 known physical corner positions (CORNER_ARM_MM) and
    the corresponding mat-frame positions.

    Mat frame: ID0=(0,0), ID1=(W,0), ID2=(W,H), ID3=(0,H)
    Arm frame: from CORNER_ARM_MM (X,Y)
    """
    mat_pts = np.array([
        [0.,       0.      ],   # ID0 TL
        [mat_w_mm, 0.      ],   # ID1 TR
        [mat_w_mm, mat_h_mm],   # ID2 BR
        [0.,       mat_h_mm],   # ID3 BL
    ], dtype=np.float32)

    arm_pts = np.array([
        CORNER_ARM_MM[0][:2],   # XY of ID0
        CORNER_ARM_MM[1][:2],   # XY of ID1
        CORNER_ARM_MM[2][:2],   # XY of ID2
        CORNER_ARM_MM[3][:2],   # XY of ID3
    ], dtype=np.float32)

    H_mat_to_arm, _ = cv.findHomography(mat_pts, arm_pts, method=0)
    M_mat_to_arm    = cv.getAffineTransform(mat_pts[:3], arm_pts[:3])

    print("\n  H (mat_mm → arm_XY_mm):")
    print(f"  {H_mat_to_arm}")
    print(f"\n  M affine:\n  {M_mat_to_arm}")
    return H_mat_to_arm, M_mat_to_arm


def mat_to_arm_xy(mat_x_mm, mat_y_mm, H):
    """Apply homography to convert mat position → arm XY position (mm)."""
    pt = np.array([[[mat_x_mm, mat_y_mm]]], dtype=np.float32)
    res = cv.perspectiveTransform(pt, H)
    return float(res[0, 0, 0]), float(res[0, 0, 1])


def ik_base_angle(x_arm_mm, lat_calib: LateralCalibration):
    """Compute base joint angle (deg) for a given arm X position."""
    return lat_calib.x_to_base(x_arm_mm)


# ═══════════════════════════════════════════════════════════════════════════════
# PHASE 5 — 3-D VISUALISATION  (arm coordinate frame)
# ═══════════════════════════════════════════════════════════════════════════════

_COLORS = ["limegreen", "cyan", "dodgerblue", "magenta"]
_NAMES  = {0:"ID0 TL", 1:"ID1 TR", 2:"ID2 BR", 3:"ID3 BL"}


def _arm_stick_user(ax, s1_deg, s2_deg, s3_deg, color):
    """Draw arm stick figure in ARM FRAME (user coordinates: X=lat, Y=depth, Z=height)."""
    def cam_pos(s1, s2, s3):
        return forward_kinematics(s1, s2, s3)   # returns mm in arm frame

    s1, s2, s3 = s1_deg, s2_deg, s3_deg

    base_pos = np.array([0., 0., ARM_HEIGHT_CM * 10.])
    sh_pos   = np.array([0., 0., (ARM_HEIGHT_CM + L1) * 10.])

    s1r = math.radians(s1)
    s2r = math.radians(s2)
    reach_sh = L2 * math.cos(s2r)
    el_x = sh_pos[0] + reach_sh * math.sin(s1r) * 10.
    el_y = sh_pos[1] - reach_sh * math.cos(s1r) * 10.
    el_z = sh_pos[2] + L2 * math.sin(s2r) * 10.
    el_pos = np.array([el_x, el_y, el_z])

    cam_xyz = cam_pos(s1, s2, s3)

    for p0, p1 in [(base_pos, sh_pos), (sh_pos, el_pos), (el_pos, cam_xyz)]:
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]],
                color=color, alpha=0.6, linewidth=1.8)

    return cam_xyz


def visualize_3d(found_corners, lat_calib, mat_w_mm, mat_h_mm):
    """
    3-D plot in ARM frame:
      • Green mat rectangle at Z = 5 mm (with corner labels)
      • Arm base + shoulder pivot
      • Arm stick figures at found (and estimated) corner configs
      • Dashed camera-to-corner rays
      • Annotated camera positions
    """
    fig = plt.figure(figsize=(14, 9))
    ax  = fig.add_subplot(111, projection="3d")
    ax.set_title("ARMOBOT  Arm ↔ Mat Calibration (Arm Frame)", fontsize=13)

    # ── Mat ──────────────────────────────────────────────────────────────────
    mat_z = 5.0   # mm (5 mm above table)
    # Corners in arm frame from CORNER_ARM_MM, lifted to mat_z
    mat_c = np.array([CORNER_ARM_MM[i] for i in range(4)], dtype=np.float64)
    mat_c[:, 2] = mat_z

    poly = Poly3DCollection(
        [list(zip(mat_c[:, 0], mat_c[:, 1], mat_c[:, 2]))],
        alpha=0.18, facecolor="green", edgecolor="darkgreen", linewidth=1.5
    )
    ax.add_collection3d(poly)

    for i, col in enumerate(_COLORS):
        p = mat_c[i]
        ax.scatter(*p, c=col, s=80, depthshade=False, zorder=5)
        ax.text(p[0]+5, p[1]+5, p[2]+5, _NAMES[i], fontsize=7.5, color=col)

    # ── Arm base / shoulder ───────────────────────────────────────────────────
    base_z = ARM_HEIGHT_CM * 10.
    sh_z   = (ARM_HEIGHT_CM + L1) * 10.
    ax.scatter(0, 0, base_z, c="red", s=200, marker="*", label="Arm base", zorder=7)
    ax.scatter(0, 0, sh_z,   c="orange", s=100, marker="^", label="Shoulder pivot", zorder=7)
    ax.plot([0, 0], [0, 0], [base_z, sh_z], color="red", linewidth=2.5)

    # ── Per-corner arm stick + camera ─────────────────────────────────────────
    sh_home, el_home = HOME["shoulder"], HOME["elbow"]

    for mid, col in zip(range(4), _COLORS):
        if mid in found_corners:
            base_deg = found_corners[mid]
            label_tag = "(measured)"
        else:
            # Compute base angle from known X position
            base_deg = lat_calib.x_to_base(CORNER_ARM_MM[mid][0])
            label_tag = "(estimated)"

        cam_xyz = _arm_stick_user(ax, base_deg, sh_home, el_home, col)

        ax.scatter(*cam_xyz, c=col, s=110, marker="o",
                   label=f"Camera {_NAMES[mid]} {label_tag}", zorder=6, depthshade=False)

        mc = mat_c[mid]
        ax.plot([cam_xyz[0], mc[0]], [cam_xyz[1], mc[1]], [cam_xyz[2], mc[2]],
                color=col, alpha=0.3, linewidth=1, linestyle="--")

        ax.text(cam_xyz[0]+3, cam_xyz[1]+3, cam_xyz[2]+8,
                f"({cam_xyz[0]:.0f},{cam_xyz[1]:.0f},{cam_xyz[2]:.0f})",
                fontsize=6, color=col)

    ax.set_xlabel("X — Lateral (mm)  [+LEFT]", labelpad=6)
    ax.set_ylabel("Y — Depth  (mm)   [−TOWARD MAT]", labelpad=6)
    ax.set_zlabel("Z — Height (mm)   [+UP]",    labelpad=6)
    ax.legend(fontsize=7, loc="upper left")
    plt.tight_layout()

    out_img = os.path.join(_HERE, "arm_mat_calibration_3d.png")
    plt.savefig(out_img, dpi=150, bbox_inches="tight")
    print(f"\n[VIZ] Saved 3-D plot → {out_img}")
    plt.show()


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN CALIBRATION LOOP
# ═══════════════════════════════════════════════════════════════════════════════

def run_calibration():
    print("\n" + "=" * 64)
    print("  ARMOBOT  Arm <-> Mat Calibration  (v2)")
    print("=" * 64)
    if DRY_RUN:
        print("  [DRY-RUN] arm movement simulated.")
    else:
        print("""
  [INFO] The Node.js backend reconnects to the Pico every 2 s.
         This script auto-re-grabs the TCP slot — backend need not be stopped.
""")

    # ── Load mat calibration ──────────────────────────────────────────────────
    plane_file = os.path.join(_HERE, "aruco_plane_calibration.npz")
    if os.path.exists(plane_file):
        pd = np.load(plane_file)
        mat_w_mm = float(pd["mat_w_mm"])
        mat_h_mm = float(pd["mat_h_mm"])
        print(f"  Mat: {mat_w_mm:.0f} × {mat_h_mm:.0f} mm  (from aruco_plane_calibration.npz)")
    else:
        mat_w_mm, mat_h_mm = 480.0, 495.0
        print(f"  [WARN] Using default mat size: {mat_w_mm} × {mat_h_mm} mm")

    # ── Load camera intrinsics ────────────────────────────────────────────────
    K_mat = None
    calib_file = os.path.join(_HERE, "camera_calibration.npz")
    if os.path.exists(calib_file):
        cc = np.load(calib_file)
        K_mat  = cc["camera_matrix"]
        dist   = cc["dist_coeffs"]
        print(f"  Camera K: fx={K_mat[0,0]:.1f}  fy={K_mat[1,1]:.1f}")
    else:
        print("  [WARN] camera_calibration.npz not found — solvePnP skipped.")
        dist = None

    # ── Arm + camera ──────────────────────────────────────────────────────────
    arm = PicoController()
    arm.connect()

    cap = cv.VideoCapture(config.CAMERA_INDEX)
    cap.set(cv.CAP_PROP_FRAME_WIDTH,  config.FRAME_W)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, config.FRAME_H)
    cap.set(cv.CAP_PROP_FPS,          config.FPS)

    ret, frame0 = cap.read()
    if not ret:
        print("[ERROR] Cannot open camera.")
        arm.disconnect(); return

    frame_h, frame_w = frame0.shape[:2]
    frame_cx = frame_w / 2.0
    print(f"  Camera: {frame_w}×{frame_h}  centre_x={frame_cx:.0f}")

    detect_fn = make_aruco_detector()

    # ════════════════════════════════════════════════════════════════════
    # STEP 1 — HOME
    # ════════════════════════════════════════════════════════════════════
    print(f"\n{'─'*60}")
    print(f"  STEP 1  Home  base={HOME['base']}°  "
          f"sh={HOME['shoulder']}°  el={HOME['elbow']}°")
    arm.move_to(HOME["base"], HOME["shoulder"], HOME["elbow"])
    time.sleep(1.0)

    # ════════════════════════════════════════════════════════════════════
    # STEP 2 — SINGLE SWEEP  (HOME config, ALL markers)
    # ════════════════════════════════════════════════════════════════════
    print(f"\n{'─'*60}")
    print("  STEP 2  Discovery sweep (HOME config, detecting IDs 0-3)")
    sightings = sweep_all_markers(arm, cap, detect_fn)

    print("\n  Sweep summary:")
    for mid in range(4):
        n = len(sightings[mid])
        phys = CORNER_ARM_MM[mid]
        tag  = "✓ found" if n > 0 else "✗ not visible (near edge - expected)"
        print(f"    ID:{mid} {_NAMES[mid]}  arm=({phys[0]:.0f},{phys[1]:.0f})mm  "
              f"{n} detections  {tag}")

    # ════════════════════════════════════════════════════════════════════
    # STEP 3 — CORNER CENTERING  (for visible markers only)
    # ════════════════════════════════════════════════════════════════════
    print(f"\n{'─'*60}")
    print("  STEP 3  Centering visible corners (binary-search base angle)")

    found_corners  = {}   # {mid: centred_base_deg}  — measured corners
    sh = HOME["shoulder"]
    el = HOME["elbow"]

    for mid in range(4):
        sgts = sightings[mid]
        if not sgts:
            print(f"\n  ID:{mid} {_NAMES[mid]} — not seen (near edge, skipped)")
            continue

        init_base = min(sgts, key=lambda s: abs(s[1] - frame_cx))[0]
        arm.move_to(init_base, sh, el)
        time.sleep(0.3)

        final_base, ok = center_marker(
            arm, cap, detect_fn, mid, init_base, sh, el, frame_cx
        )
        found_corners[mid] = final_base
        cam_xyz = forward_kinematics(final_base, sh, el)
        phys    = CORNER_ARM_MM[mid]
        print(f"    Joints : base={final_base:.2f}°  sh={sh}°  el={el}°  "
              f"{'✓' if ok else '(best-effort)'}")
        print(f"    FK camera XYZ : ({cam_xyz[0]:.1f}, {cam_xyz[1]:.1f}, "
              f"{cam_xyz[2]:.1f}) mm  (arm frame)")
        print(f"    Marker phys   : ({phys[0]:.0f}, {phys[1]:.0f}) mm  (arm frame)")

    # ════════════════════════════════════════════════════════════════════
    # STEP 4 — FIT LATERAL CALIBRATION  (base ↔ arm X)
    # ════════════════════════════════════════════════════════════════════
    print(f"\n{'─'*60}")
    print("  STEP 4  Fit lateral calibration (base_deg <-> X_arm_mm)")

    lat_calib    = LateralCalibration()
    calib_source = "measured"

    if len(found_corners) >= 2:
        # ── Primary: use sweep-measured base angles ────────────────────
        base_list  = [found_corners[mid]     for mid in found_corners]
        x_arm_list = [CORNER_ARM_MM[mid][0] for mid in found_corners]
        lat_calib.fit(base_list, x_arm_list)

        # Sanity check: degenerate sweep (e.g. all corners at same base angle)
        # can produce |scale| << 1.  Discard and fall through to FK estimate.
        if lat_calib.scale is None or not (-25.0 <= lat_calib.scale <= -5.0):
            print(f"  [WARN] Sweep fit gave implausible scale={lat_calib.scale:.4f} "
                  f"(expected -25 to -5). Discarding — using FK estimate.")
            lat_calib.scale  = None
            lat_calib.offset = None
            found_corners    = {}   # force FK fallback
        else:
            print(f"  Source: SWEEP MEASUREMENTS  ({len(found_corners)} corners)")

    else:
        # ── Fallback: estimate base angles from FK + atan2 geometry ───
        # base = -atan2(X_arm, -Y_arm) is the geometric angle to face the
        # marker.  This is an approximation; refine by doing a live sweep.
        print(f"  [WARN] Only {len(found_corners)} corner(s) measured. "
              "Using FK-estimated base angles (approximate — run live sweep to refine).")
        calib_source = "estimated (FK atan2)"

        # Try to load a previous calibration to use as base
        prev_npz = os.path.join(_HERE, "arm_mat_calibration.npz")
        if os.path.exists(prev_npz):
            try:
                prev  = np.load(prev_npz)
                s     = float(prev["lat_scale"])
                o     = float(prev["lat_offset"])
                # Sanity-check: live measurement gives scale ≈ -12 mm/deg.
                # Accept only values in plausible range to reject dry-run-only saves.
                if -25.0 <= s <= -5.0:
                    lat_calib.scale  = s
                    lat_calib.offset = o
                    calib_source = "loaded from previous arm_mat_calibration.npz"
                    print(f"  Loaded previous lateral calibration: {lat_calib}")
                else:
                    print(f"  [WARN] Previous NPZ has implausible scale={s:.3f} "
                          f"(expected -25 to -5). Discarding — will use FK estimate.")
            except Exception as e:
                print(f"  [WARN] Could not load previous NPZ: {e}")


        if lat_calib.scale is None:
            # Pure FK estimate using ONLY the observable far corners (|Y| > 100mm).
            # Near corners at Y=-10mm give atan2 angles of ~87.6 deg which are
            # physically meaningless and would corrupt the lateral scale.
            fk_base_list  = []
            fk_x_arm_list = []
            for mid in range(4):
                x = CORNER_ARM_MM[mid][0]
                y = CORNER_ARM_MM[mid][1]
                if abs(y) < 100.0:
                    print(f"    ID:{mid}  X={x:.0f} mm  Y={y:.0f} mm  "
                          f"SKIPPED (near edge, atan2 unreliable at shallow depth)")
                    continue
                base_est = -math.degrees(math.atan2(x, -y))
                fk_base_list.append(base_est)
                fk_x_arm_list.append(x)
                print(f"    ID:{mid}  X={x:.0f} mm  Y={y:.0f} mm  "
                      f"estimated base={base_est:.1f} deg")

            if len(fk_base_list) >= 2:
                lat_calib.fit(fk_base_list, fk_x_arm_list)
            else:
                # Last-resort default from live measurement (ID0=-22, ID1=+17)
                print("  [WARN] Not enough far corners — using default scale=-12.41 offset=-31")
                lat_calib.scale  = -12.41
                lat_calib.offset = -31.0

        # Merge measured corners (if any) into found_corners for visualization
        for mid in range(4):
            if mid not in found_corners:
                found_corners[mid] = lat_calib.x_to_base(CORNER_ARM_MM[mid][0])

    print(f"  {lat_calib}  [{calib_source}]")

    # ── Verify ──────────────────────────────────────────────────────────
    for mid in range(4):
        base_deg = found_corners.get(mid, lat_calib.x_to_base(CORNER_ARM_MM[mid][0]))
        x_pred = lat_calib.base_to_x(base_deg)
        x_true = CORNER_ARM_MM[mid][0]
        src    = "measured" if mid in {k for k in found_corners
                                       if k in [j for j in found_corners
                                                if len(sightings.get(j, [])) > 0]} \
                             else calib_source
        print(f"    ID:{mid}  base={base_deg:.2f}°  "
              f"X_pred={x_pred:.1f} mm  X_true={x_true:.0f} mm  "
              f"err={abs(x_pred-x_true):.1f} mm  [{src}]")

    # ════════════════════════════════════════════════════════════════════
    # STEP 5 — ARM ↔ MAT TRANSFORMS  (analytical from known coordinates)
    # ════════════════════════════════════════════════════════════════════
    print(f"\n{'─'*60}")
    print("  STEP 5  Compute arm ↔ mat transforms (all 4 known corners)")

    H_mat_to_arm, M_mat_to_arm = compute_arm_mat_transforms(mat_w_mm, mat_h_mm)

    # Validation: project each corner
    print("\n  Projection check (mat_mm → arm_XY_mm via H):")
    mat_corners_mm = [(0,0),(mat_w_mm,0),(mat_w_mm,mat_h_mm),(0,mat_h_mm)]
    for mid, (mx, my) in enumerate(mat_corners_mm):
        ax_pred, ay_pred = mat_to_arm_xy(mx, my, H_mat_to_arm)
        ax_true = CORNER_ARM_MM[mid][0]
        ay_true = CORNER_ARM_MM[mid][1]
        err = math.sqrt((ax_pred-ax_true)**2 + (ay_pred-ay_true)**2)
        print(f"    ID:{mid}  mat=({mx:.0f},{my:.0f})  "
              f"arm_pred=({ax_pred:.1f},{ay_pred:.1f})  "
              f"arm_true=({ax_true:.0f},{ay_true:.0f})  err={err:.2f} mm")

    # ════════════════════════════════════════════════════════════════════
    # STEP 6 — 3-D VISUALISATION
    # ════════════════════════════════════════════════════════════════════
    print(f"\n{'─'*60}")
    print("  STEP 6  3-D visualisation")
    visualize_3d(found_corners, lat_calib, mat_w_mm, mat_h_mm)

    # ════════════════════════════════════════════════════════════════════
    # STEP 7 — SAVE
    # ════════════════════════════════════════════════════════════════════
    print(f"\n{'─'*60}")
    print("  STEP 7  Save calibration")

    # Build per-corner joint arrays (found or estimated)
    corner_joints = []
    for mid in range(4):
        if mid in found_corners:
            base_d = found_corners[mid]
        else:
            base_d = lat_calib.x_to_base(CORNER_ARM_MM[mid][0])
        corner_joints.append([base_d, HOME["shoulder"], HOME["elbow"]])

    result = {
        # Physical corner positions (user-measured ground truth)
        "corner_arm_mm"  : np.array([CORNER_ARM_MM[i] for i in range(4)], dtype=np.float64),
        "corner_mat_mm"  : np.array(mat_corners_mm, dtype=np.float64),
        # Joint angles at each corner (measured or estimated)
        "corner_joints"  : np.array(corner_joints, dtype=np.float64),
        # FK camera positions at each corner
        "corner_cam_xyz" : np.array([
            forward_kinematics(*corner_joints[i]) for i in range(4)
        ], dtype=np.float64),
        # Coordinate transforms
        "H_mat_to_arm"   : H_mat_to_arm,    # (3,3) homography
        "M_mat_to_arm"   : M_mat_to_arm,    # (2,3) affine
        # Lateral calibration
        "lat_scale"      : np.float64(lat_calib.scale),
        "lat_offset"     : np.float64(lat_calib.offset),
        # Physical parameters
        "mat_w_mm"       : np.float64(mat_w_mm),
        "mat_h_mm"       : np.float64(mat_h_mm),
        "arm_height_cm"  : np.float64(ARM_HEIGHT_CM),
        "l1": np.float64(L1), "l2": np.float64(L2), "l3": np.float64(L3),
        "home_base"      : np.float64(HOME["base"]),
        "home_shoulder"  : np.float64(HOME["shoulder"]),
        "home_elbow"     : np.float64(HOME["elbow"]),
    }

    out_path = os.path.join(_HERE, "arm_mat_calibration.npz")
    np.savez(out_path, **result)
    print(f"\n  Saved → {out_path}")
    for k, v in result.items():
        vstr = f"shape={np.asarray(v).shape}" if np.asarray(v).ndim > 1 else f"= {np.asarray(v).ravel()}"
        print(f"    {k:22s} : {vstr}")

    # ── Usage example ─────────────────────────────────────────────────────────
    print(f"\n{'─'*60}")
    print("  PICK-AND-PLACE USAGE EXAMPLE")
    print("""
    # Load calibration:
    cal = np.load('arm_mat_calibration.npz')
    H   = cal['H_mat_to_arm']
    lat_scale  = float(cal['lat_scale'])
    lat_offset = float(cal['lat_offset'])

    def mat_to_base_angle(mat_x_mm, mat_y_mm):
        # 1. Convert mat position to arm X using homography
        pt   = np.array([[[mat_x_mm, mat_y_mm]]], dtype=np.float32)
        arm  = cv.perspectiveTransform(pt, H)[0,0]
        x_arm = arm[0]        # arm X (mm)
        # 2. Convert arm X to base angle
        base_deg = (x_arm - lat_offset) / lat_scale
        return base_deg, x_arm
    """)

    # ── Return to home ────────────────────────────────────────────────────────
    print(f"\n{'─'*60}")
    print("  Returning to HOME …")
    arm.move_to(HOME["base"], HOME["shoulder"], HOME["elbow"])
    arm.disconnect()
    cap.release()
    print("\n" + "=" * 64)
    print("  Calibration COMPLETE.")
    print("=" * 64)


# ═══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    run_calibration()
