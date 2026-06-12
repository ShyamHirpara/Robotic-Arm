"""
base_scan_stop.py  --  Smooth 3-joint object search & approach
=====================================================================
ALL THREE joints move smoothly via the Pico TCP jog bridge (port 81):
continuous real-time motion with live position feedback -- no
step-stop-step.  The base scans/centres horizontally, the shoulder
centres vertically, the elbow advances toward the object while the
shoulder keeps it centred, until distance <= APPROACH_STOP_CM.

Pipeline per frame:
  1. Detect the 4-corner ArUco mat boundary (IDs 0-3, hybrid quad),
     re-detected EVERY frame (the camera moves with the arm).
  2. Detect white objects inside the boundary.
  3. Measure distance (cm) and mat position (mm).

State machine:
  IDLE    : motors stopped, camera live.  Waits for S / C.
  SCAN    : base jogs continuously between BASE_MIN..BASE_MAX; stops
            on a find ONLY when >= SCAN_MIN_MARKERS ArUco markers are
            freshly visible.  THE MARKERS ARE ONLY NEEDED HERE: once
            the object is found it is tracked frame-to-frame by its
            position -- no ArUco required during centering/approach
            (close-up views lose the markers and that is fine).
  CENTER  : base jogs slowly toward the vertical centre line, stops
            in real time within tolerance.
  SHOULDER: shoulder jogs toward the horizontal centre line, stops in
            real time (same smooth servo as the base).
  APPROACH: elbow jogs toward the object with an ADAPTIVE direction:
            if the measured distance grows by ELBOW_TREND_CM the
            elbow auto-reverses (3 reversals without progress ->
            hold).  If Y drifts past APPROACH_Y_TOL the elbow pauses
            and the shoulder re-centres first.  Ends when distance
            <= APPROACH_STOP_CM.
  ARRIVED : all motors stopped at the object.  Watchdog: X drift ->
            CENTER, Y drift -> SHOULDER, distance grew -> APPROACH,
            object gone -> rescan.

NO AUTOMATIC POSITIONING: the script never moves shoulder/elbow on
its own at startup -- you control everything with the keys.  S scans
from wherever the arm currently is.

REAL-TIME JOG:
  START_<axis>_<dir> / STOP / SPEED_<axis>_<sps> on TCP port 81.
  The firmware streams {"s1","s2","s3"} JSON every 100 ms while
  jogging -- that is the live position feedback.  Only one joint jogs
  at a time (the firmware jog loop is blocking) -- the state machine
  is sequential anyway.

USAGE:
  python base_scan_stop.py                 # live (Wi-Fi: RoboticArm_AP)
  python base_scan_stop.py --dry-run       # camera only, no robot

KEYS:
  S        move shoulder/elbow to the view_pose.json position, then
           scan (base sweeps left/right at 75% firmware speed)
  C        calibrate -- firmware limit-switch homing (/calibrate);
           arm ends at 0/0/0, then press S when ready
  R        rescan from the current pose (no pose move)
  SPACE    pause/resume        ESC / Q  quit
"""

import argparse
import json
import math
import os
import socket
import sys
import time

import cv2 as cv
import numpy as np
import requests

# Force UTF-8 output on Windows (prevents cp1252 UnicodeEncodeError)
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import config
from object_detector import detect_objects
from distance_estimator import (
    load_calibration,
    load_plane_calibration,
    make_aruco_detector,
    detect_mat_quad,
    draw_mat_overlay,
    point_in_quad,
    triangle_similarity_distance,
)

# ════════════════════════════ TUNING ════════════════════════════════════════

PICO_URL = "http://192.168.4.1/"
PICO_TCP_PORT = 81

# Base sweep range (deg).  Firmware hardware limits are -170..+150.
BASE_MIN = -70.0
BASE_MAX = 70.0

# Flip to -1 if the base turns AWAY from the object while centering.
BASE_DIRECTION = -1

# ── Joint step scales (deg of joint per step/s of jog speed) ─────────────────
J1_DEG_PER_SPS = 360.0 / 7971.0    # base
J2_DEG_PER_SPS = 360.0 / 10500.0   # shoulder
J3_DEG_PER_SPS = 360.0 / 10000.0   # elbow

# ── Smooth jog speeds (steps/sec) ────────────────────────────────────────────
SCAN_SPS = 200               # SCAN ONLY: 75% of firmware default (600 sps)
CENTER_SPS = 70              # base ~3.2 deg/s while centering (unchanged)
SHOULDER_SPS = 90            # shoulder ~3.1 deg/s (smooth vertical servo)
ELBOW_SPS = 80               # elbow ~2.9 deg/s (smooth approach)

JOG_STALL_S = 0.8            # jogging but position frozen -> treat as stopped
STOP_EARLY_FACTOR = 1.5      # stop jog at tol*this (compensates frame latency)

# ── Shoulder (joint 2) vertical centering.  Firmware limits -20..+70 ─────────
SHOULDER_MIN = -20.0
SHOULDER_MAX = 40.0
SHOULDER_DIRECTION = -1      # flip if shoulder corrects the wrong way

# ── Elbow approach (joint 3).  Firmware limits -75..+85 ──────────────────────
ELBOW_MIN = -75.0
ELBOW_MAX = 40.0
ELBOW_APPROACH_DIRECTION = -1  # starting direction (auto-reverses on trend)
ELBOW_TREND_CM = 2.0           # distance grew this much -> reverse elbow
ELBOW_PROGRESS_CM = 0.5        # distance shrank this much -> real progress
APPROACH_STOP_CM = 15.0        # stop when object is this close
APPROACH_Y_TOL = 60            # px; pause elbow and fix Y beyond this

# Object tracking after the find: ArUco markers are NOT required any
# more -- the object is followed frame-to-frame by nearest position.
TRACK_MAX_JUMP_PX = 300        # max per-frame jump of the tracked object

DETECT_HITS = 2              # consecutive frames with object to confirm find
SCAN_MIN_MARKERS = 2         # ArUco markers required to confirm a SCAN find
ROI_INSET_PX = 45            # object centre must be this many px INSIDE the
                             # boundary -- rejects the white paper squares of
                             # the ArUco markers sitting at the quad corners
FLUSH_FRAMES = 4             # stale frames discarded after blocking moves

CENTER_TOL_PX = 20           # |pixel error| below this counts as centred
CENTER_LOCK_HITS = 2         # consecutive centred checks before next stage
CENTER_SAMPLES = 3           # frames averaged per stationary evaluation
LOST_LIMIT = 10              # frames without object -> rescan

# Watchdog (ARRIVED state): react when the object is moved or replaced
RELOCK_PX = 60
RELOCK_HITS = 5
LOCKED_LOST_LIMIT = 15

MOVE_TIMEOUT = 25            # s, /stepper blocks on the Pico until done
STATUS_EVERY = 30            # frames between [STATUS] console prints

# Dry-run simulated start pose (live mode always uses the real position)
HOME_SHOULDER = -20.0
HOME_ELBOW = -65.0

# Scan pose applied when S is pressed (edit the json to change it)
VIEW_POSE_FILE = os.path.join(_HERE, "view_pose.json")

(STATE_IDLE, STATE_SCAN, STATE_CENTER, STATE_SHOULDER, STATE_APPROACH,
 STATE_ARRIVED, STATE_PAUSED) = (
    "IDLE", "SCAN", "CENTER", "SHOULDER", "APPROACH", "ARRIVED", "PAUSED")

STATE_COLORS = {
    STATE_IDLE:     (255, 255, 0),   # cyan
    STATE_SCAN:     (0, 200, 255),   # orange
    STATE_CENTER:   (255, 200, 0),   # cyan-blue
    STATE_SHOULDER: (255, 0, 200),   # magenta
    STATE_APPROACH: (0, 140, 255),   # warm orange
    STATE_ARRIVED:  (0, 220, 60),    # green
    STATE_PAUSED:   (160, 160, 160), # grey
}

_AXIS_DEG_PER_SPS = {1: J1_DEG_PER_SPS, 2: J2_DEG_PER_SPS, 3: J3_DEG_PER_SPS}


# ════════════════════════════ ARM CLIENT ════════════════════════════════════

class BaseArm:
    """
    Pico W client.  Two channels:
      HTTP (port 80)   -- /calibrate, /current_position (and /stepper).
      TCP  (port 81)   -- real-time jog for ALL joints:
                          START_<axis>_<dir> / STOP / SPEED_<axis>_<sps>;
                          firmware streams {"s1","s2","s3"} JSON every
                          100 ms while jogging (live position feedback).
    """

    def __init__(self, url=PICO_URL, dry_run=False):
        self.url = url if url.endswith("/") else url + "/"
        self.dry_run = dry_run
        self.base_deg = 0.0
        self.shoulder_deg = 0.0
        self.elbow_deg = 0.0
        self.sock = None
        self.jogging = False
        self.jog_axis = 0
        self.jog_dir = 0
        self._sps = SCAN_SPS
        self._rx = ""
        self._stall_pos = 0.0
        self._stall_t = 0.0
        self._speed_sent_t = None    # awaiting SPEED_OK ack from firmware
        self._speed_warned = False

    # ── connection ────────────────────────────────────────────────────────

    def _host(self):
        return self.url.split("//", 1)[-1].strip("/").split(":")[0]

    def connect(self):
        if self.dry_run:
            print("[ARM] DRY-RUN: no robot connection, motors simulated.")
            self.shoulder_deg = HOME_SHOULDER
            self.elbow_deg = HOME_ELBOW
            return True
        pos = self.get_position()
        if pos is None:
            print(f"[ARM] ERROR: Pico unreachable at {self.url}")
            print("      Connect to Wi-Fi 'RoboticArm_AP' (pass 12345678).")
            return False
        self.base_deg = pos.get("joint1", 0.0)
        self.shoulder_deg = pos.get("joint2", 0.0)
        self.elbow_deg = pos.get("joint3", 0.0)
        print(f"[ARM] Connected. base={self.base_deg:+.1f}  "
              f"sh={self.shoulder_deg:+.1f}  el={self.elbow_deg:+.1f} deg")
        return True

    def jog_connect(self):
        """Open the TCP jog channel (port 81)."""
        if self.dry_run:
            return True
        try:
            self.sock = socket.create_connection(
                (self._host(), PICO_TCP_PORT), timeout=5)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock.setblocking(False)
            print("[ARM] Jog TCP connected (port 81). NOTE: stop the Node "
                  "backend if it is running -- it steals this port.")
            return True
        except Exception as e:
            print(f"[ARM] ERROR: jog TCP connect failed: {e}")
            return False

    def close(self):
        if self.jogging:
            self.jog_stop(resync=False)
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    # ── HTTP ──────────────────────────────────────────────────────────────

    def get_position(self):
        if self.dry_run:
            return {"joint1": self.base_deg, "joint2": self.shoulder_deg,
                    "joint3": self.elbow_deg}
        try:
            r = requests.get(self.url + "current_position", timeout=3)
            return r.json()
        except Exception:
            return None

    def calibrate(self):
        """
        Firmware limit-switch calibration (/calibrate).  All joints seek
        their switches, then return to 0 deg.  Blocking (up to ~2 min).
        """
        if self.dry_run:
            print("[ARM] DRY-RUN: calibration simulated (joints -> 0).")
            self.base_deg = self.shoulder_deg = self.elbow_deg = 0.0
            return True
        print("[ARM] CALIBRATING -- joints home to limit switches, then "
              "return to 0 deg.  Please wait ...")
        try:
            requests.get(self.url + "calibrate", timeout=180)
        except Exception as e:
            print(f"[ARM] Calibrate failed: {e}")
            return False
        pos = self.get_position()
        if pos:
            self.base_deg = pos.get("joint1", 0.0)
            self.shoulder_deg = pos.get("joint2", 0.0)
            self.elbow_deg = pos.get("joint3", 0.0)
        print(f"[ARM] Calibration complete. base={self.base_deg:+.1f}  "
              f"sh={self.shoulder_deg:+.1f}  el={self.elbow_deg:+.1f} deg")
        return True

    def move_joint(self, num, target_deg):
        """Blocking absolute HTTP move (used by S to reach the scan pose)."""
        cur = self._axis_angle(num)
        if abs(target_deg - cur) < 0.05:
            return True
        if self.dry_run:
            time.sleep(0.10)
        else:
            try:
                requests.get(
                    self.url + f"stepper?num={num}&angle={target_deg:.2f}",
                    timeout=MOVE_TIMEOUT)
            except Exception as e:
                print(f"[ARM] move joint{num} -> {target_deg:.1f} failed: {e}")
                return False
        if num == 1:
            self.base_deg = target_deg
        elif num == 2:
            self.shoulder_deg = target_deg
        else:
            self.elbow_deg = target_deg
        return True

    # ── TCP: smooth real-time jog (all joints) ────────────────────────────

    def _send(self, line):
        if self.sock:
            try:
                self.sock.sendall((line + "\n").encode())
            except Exception as e:
                print(f"[ARM] TCP send failed: {e}")

    def _axis_angle(self, axis):
        return {1: self.base_deg, 2: self.shoulder_deg,
                3: self.elbow_deg}.get(axis, 0.0)

    def jog_speed(self, axis, sps):
        self._sps = sps
        if not self.dry_run:
            self._send(f"SPEED_{axis}_{int(sps)}")
            if not self._speed_warned:
                self._speed_sent_t = time.time()

    def jog_start(self, axis, direction):
        """Start continuous joint rotation. +1 = increasing angle."""
        self.jog_axis = axis
        self.jog_dir = 1 if direction > 0 else -1
        self.jogging = True
        self._stall_pos = self._axis_angle(axis)
        self._stall_t = time.time()
        if not self.dry_run:
            self._send(f"START_{axis}_{1 if self.jog_dir > 0 else 0}")

    def jog_stop(self, resync=True):
        """Stop the jog NOW.  Brief pause lets the Pico exit its jog loop."""
        was = self.jogging
        self.jogging = False
        if self.dry_run or not was:
            return
        self._send("STOP")
        time.sleep(0.20)          # let jog_motion() exit before next command
        if resync:
            pos = self.get_position()
            if pos:
                self.base_deg = pos.get("joint1", self.base_deg)
                self.shoulder_deg = pos.get("joint2", self.shoulder_deg)
                self.elbow_deg = pos.get("joint3", self.elbow_deg)

    def jog_poll(self, dt):
        """Refresh live position. Dry-run integrates; live parses TCP JSON."""
        if self.dry_run:
            if self.jogging:
                d = self.jog_dir * self._sps * \
                    _AXIS_DEG_PER_SPS[self.jog_axis] * dt
                if self.jog_axis == 1:
                    self.base_deg = max(-170.0, min(150.0, self.base_deg + d))
                elif self.jog_axis == 2:
                    self.shoulder_deg = max(-20.0, min(70.0,
                                                       self.shoulder_deg + d))
                else:
                    self.elbow_deg = max(-75.0, min(85.0, self.elbow_deg + d))
            return
        if not self.sock:
            return
        try:
            data = self.sock.recv(4096)
            if data:
                self._rx += data.decode(errors="replace")
        except OSError:
            pass                   # no data ready (non-blocking socket)
        while "\n" in self._rx:
            line, self._rx = self._rx.split("\n", 1)
            line = line.strip()
            if line == "SPEED_OK":
                self._speed_sent_t = None     # firmware accepted the speed
            elif line.startswith("{"):
                try:
                    d = json.loads(line)
                    if "s1" in d:
                        self.base_deg = float(d["s1"])
                    if "s2" in d:
                        self.shoulder_deg = float(d["s2"])
                    if "s3" in d:
                        self.elbow_deg = float(d["s3"])
                except Exception:
                    pass
        # Old firmware never acks SPEED -- warn once: jog runs FAST (600 sps)
        if (self._speed_sent_t is not None and not self._speed_warned
                and time.time() - self._speed_sent_t > 1.5):
            self._speed_warned = True
            self._speed_sent_t = None
            print("=" * 60)
            print("[ARM] WARNING: Pico IGNORED the SPEED command!")
            print("      The Pico is still running the OLD main.py, so all")
            print("      jogs run at the firmware default speed (FAST).")
            print("      Flash the updated Armobot-control-system\\main.py")
            print("      via Thonny and restart the Pico.")
            print("=" * 60)
        # Stall watchdog: firmware stopped the jog itself (limit/emergency)
        if self.jogging:
            cur = self._axis_angle(self.jog_axis)
            if abs(cur - self._stall_pos) > 0.2:
                self._stall_pos = cur
                self._stall_t = time.time()
            elif time.time() - self._stall_t > JOG_STALL_S:
                print(f"[ARM] Jog J{self.jog_axis} stalled (firmware limit?) "
                      f"-- marked stopped")
                self.jogging = False


# ════════════════════════════ CAMERA ════════════════════════════════════════

def open_camera(preferred):
    """Try the preferred index first, then common fallbacks."""
    tried = []
    for idx in [preferred, 0, 1, 2]:
        if idx in tried:
            continue
        tried.append(idx)
        cap = cv.VideoCapture(idx, cv.CAP_DSHOW)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                cap.set(cv.CAP_PROP_FRAME_WIDTH, config.FRAME_W)
                cap.set(cv.CAP_PROP_FRAME_HEIGHT, config.FRAME_H)
                cap.set(cv.CAP_PROP_FPS, config.FPS)
                cap.set(cv.CAP_PROP_BUFFERSIZE, 1)
                print(f"[CAM] Opened camera {idx} @ "
                      f"{int(cap.get(cv.CAP_PROP_FRAME_WIDTH))}x"
                      f"{int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))}")
                return cap
        cap.release()
    print(f"[CAM] ERROR: no camera found (tried {tried})")
    return None


def flush_camera(cap, n=FLUSH_FRAMES):
    """Discard buffered frames so the next read reflects the new pose."""
    for _ in range(n):
        cap.grab()


def load_view_pose():
    """Return (shoulder, elbow) from view_pose.json, or None if missing."""
    if not os.path.exists(VIEW_POSE_FILE):
        return None
    try:
        with open(VIEW_POSE_FILE) as f:
            d = json.load(f)
        return float(d["shoulder"]), float(d["elbow"])
    except Exception as e:
        print(f"[POSE] Could not read {VIEW_POSE_FILE}: {e}")
        return None


# ════════════════════════════ MEASUREMENT ═══════════════════════════════════

def fresh_mat_homography(quad, plane):
    """Pixel -> mat-mm homography refit from the CURRENT quad each frame."""
    if quad is None:
        return None
    if plane is not None:
        W, H = plane["mat_w_mm"], plane["mat_h_mm"]
    else:
        W, H = 480.0, 495.0
    dst = np.array([[0, 0], [W, 0], [W, H], [0, H]], dtype=np.float32)
    H_now, _ = cv.findHomography(quad.astype(np.float32), dst, method=0)
    return H_now


def measure_target(det, K, H_now):
    """Attach distance_cm and mat (x,y) mm to a detection (None if unknown)."""
    cx, cy = det["center"]
    dist_cm = None
    if det.get("w_cm") is not None:
        dist_cm = triangle_similarity_distance(
            det["pixel_w"], det["pixel_h"], det["w_cm"], det["h_cm"], K, cx, cy)
        if not (0 < dist_cm < 500):
            dist_cm = None
    mat_mm = None
    if H_now is not None:
        pt = H_now @ np.array([cx, cy, 1.0])
        mat_mm = (float(pt[0] / pt[2]), float(pt[1] / pt[2]))
    det["distance_cm"] = dist_cm
    det["mat_mm"] = mat_mm
    return det


def pick_target(detections):
    """Largest object inside the boundary = most reliable target."""
    if not detections:
        return None
    return max(detections, key=lambda d: d["area"])


# ════════════════════════════ DRAWING ═══════════════════════════════════════

def draw_target(frame, det):
    color = det["color_bgr"]
    cv.rectangle(frame, det["pt1"], det["pt2"], color, 2)
    x, y = det["pt1"]
    lines = [det["label"]]
    if det.get("distance_cm") is not None:
        lines.append(f"dist {det['distance_cm']:.1f} cm")
    if det.get("mat_mm") is not None:
        lines.append(f"mat ({det['mat_mm'][0]:.0f},{det['mat_mm'][1]:.0f}) mm")
    for i, txt in enumerate(lines):
        cv.putText(frame, txt, (x, y - 10 - 18 * (len(lines) - 1 - i)),
                   cv.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv.LINE_AA)
    cv.drawMarker(frame, det["center"], color, cv.MARKER_CROSS, 16, 2)


def draw_hud(frame, state, arm, sweep_dir, err_px, err_y, msg=""):
    h, w = frame.shape[:2]
    color = STATE_COLORS.get(state, (255, 255, 255))
    cv.rectangle(frame, (0, 0), (w, 34), (20, 20, 20), -1)
    jog = f" JOG-J{arm.jog_axis}" if arm.jogging else ""
    text = (f"[{state}]{jog}  b={arm.base_deg:+.1f} "
            f"s={arm.shoulder_deg:+.1f} e={arm.elbow_deg:+.1f}")
    if state == STATE_SCAN:
        text += f"  sweep={'+' if sweep_dir > 0 else '-'}"
    if err_px is not None:
        ey = f"{err_y:+.0f}" if err_y is not None else "--"
        text += f"  err=({err_px:+.0f},{ey})px"
    if msg:
        text += f"  |  {msg}"
    cv.putText(frame, text, (8, 23), cv.FONT_HERSHEY_SIMPLEX, 0.55,
               color, 2, cv.LINE_AA)
    # frame-centre crosshair (the target the object must reach)
    cv.line(frame, (w // 2, 34), (w // 2, h), (90, 90, 90), 1, cv.LINE_AA)
    cv.line(frame, (0, h // 2), (w, h // 2), (90, 90, 90), 1, cv.LINE_AA)
    cv.putText(frame,
               "S scan  C calibrate  R rescan  SPACE pause  ESC/Q quit",
               (8, h - 10), cv.FONT_HERSHEY_SIMPLEX, 0.45,
               (200, 200, 200), 1, cv.LINE_AA)


# ════════════════════════════ MAIN ══════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="Smooth 3-joint object scan & approach")
    ap.add_argument("--dry-run", action="store_true",
                    help="camera only, simulate motors")
    ap.add_argument("--camera", type=int, default=config.CAMERA_INDEX,
                    help="webcam index")
    ap.add_argument("--url", default=PICO_URL, help="Pico base URL")
    args = ap.parse_args()

    # ── Calibration & detector ────────────────────────────────────────────
    K, dist_coeffs = load_calibration()
    plane = load_plane_calibration()
    detect_fn = make_aruco_detector()
    print(f"[CTRL] jog speeds: scan {SCAN_SPS} sps "
          f"(~{SCAN_SPS * J1_DEG_PER_SPS:.1f} deg/s)  "
          f"center {CENTER_SPS} sps  shoulder {SHOULDER_SPS} sps  "
          f"elbow {ELBOW_SPS} sps")

    # ── Camera ────────────────────────────────────────────────────────────
    cap = open_camera(args.camera)
    if cap is None:
        return 1

    # ── Arm (no automatic positioning -- S and C control everything) ──────
    arm = BaseArm(args.url, dry_run=args.dry_run)
    if not arm.connect():
        return 1
    if not arm.jog_connect():
        return 1

    # ── State machine ─────────────────────────────────────────────────────
    state = STATE_IDLE               # wait for S (scan) or C (calibrate)
    prev_state = STATE_IDLE
    sweep_dir = 1 if (BASE_MAX - arm.base_deg) >= (arm.base_deg - BASE_MIN) else -1
    stop_hits = 0                    # consecutive valid find frames (SCAN)
    center_cx = []                   # stationary cx samples (CENTER)
    lock_hits = 0
    lost_frames = 0
    sh_cy = []                       # stationary cy samples (SHOULDER)
    sh_lock = 0
    sh_lost = 0
    sh_sat = 0
    ap_lost = 0                      # APPROACH: object-missing frames
    ap_nodist = 0                    # APPROACH: frames without distance
    ap_dir = ELBOW_APPROACH_DIRECTION  # current elbow direction (adaptive)
    ap_flips = 0                     # direction reversals without progress
    ap_best = None                   # best (smallest) distance so far
    track_pt = None                  # last known object centre (tracking)
    err_px = None
    err_y = None
    msg = ""
    frame_count = 0
    saturated = 0
    dry_hint = False
    locked_off = 0
    locked_lost = 0
    last_t = time.time()

    def go_rescan(reason):
        """Back to SCAN from the current pose (no automatic positioning)."""
        nonlocal state, stop_hits, msg, track_pt
        if arm.jogging:
            arm.jog_stop()
        state = STATE_SCAN
        stop_hits = 0
        track_pt = None
        msg = reason

    WIN = "Object Approach  (4-ArUco boundary)"
    cv.namedWindow(WIN, cv.WINDOW_NORMAL)
    msg = "press S to scan, C to calibrate"
    print("[RUN] Ready.  S = start scan   C = calibrate   ESC/Q = quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        now = time.time()
        dt = now - last_t
        last_t = now
        arm.jog_poll(dt)

        # ── Boundary + detection (re-detected EVERY frame) ────────────────
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        quad, n_markers = detect_mat_quad(gray, detect_fn, None, plane=plane)
        draw_mat_overlay(frame, quad, n_markers)
        H_now = fresh_mat_homography(quad, plane)

        all_dets = detect_objects(frame)
        in_quad = None
        if quad is not None:
            # Objects whose centre is WELL INSIDE the boundary (positive
            # distance = inside).  The white paper squares of the ArUco
            # markers sit ON the corners, so they are rejected here.
            q32 = quad.astype(np.float32)
            in_quad = [d for d in all_dets
                       if cv.pointPolygonTest(
                           q32, (float(d["center"][0]),
                                 float(d["center"][1])), True) >= ROI_INSET_PX]

        target = None
        tracking = state in (STATE_CENTER, STATE_SHOULDER,
                             STATE_APPROACH, STATE_ARRIVED)
        if tracking and track_pt is not None:
            # After the find the boundary is NOT required (close-up views
            # lose the markers).  Follow the detection nearest to the
            # object's last known position.
            cands = in_quad if in_quad else all_dets
            best, best_d2 = None, None
            for d in cands:
                dx = d["center"][0] - track_pt[0]
                dy = d["center"][1] - track_pt[1]
                d2 = dx * dx + dy * dy
                if best_d2 is None or d2 < best_d2:
                    best, best_d2 = d, d2
            if best is not None and best_d2 <= TRACK_MAX_JUMP_PX ** 2:
                target = best
        elif in_quad:
            # searching: boundary required (defines the workspace)
            target = pick_target(in_quad)

        if target is not None:
            track_pt = target["center"]
            measure_target(target, K, H_now)
            draw_target(frame, target)

        frame_cx = frame.shape[1] / 2.0
        frame_cy = frame.shape[0] / 2.0
        err_px = (target["center"][0] - frame_cx) if target else None
        err_y = (target["center"][1] - frame_cy) if target else None
        dist_cm = target.get("distance_cm") if target else None

        # ── Periodic console status ───────────────────────────────────────
        frame_count += 1
        if frame_count % STATUS_EVERY == 0:
            if target is not None:
                t_str = (f"target={target['label']} "
                         f"err=({err_px:+.0f},{err_y:+.0f})px")
                if dist_cm is not None:
                    t_str += f" dist={dist_cm:.1f}cm"
                if target.get("mat_mm") is not None:
                    t_str += (f" mat=({target['mat_mm'][0]:.0f},"
                              f"{target['mat_mm'][1]:.0f})mm")
            else:
                t_str = "no object visible"
            jog = f"JOG-J{arm.jog_axis}" if arm.jogging else "     "
            print(f"[STATUS] {state:8s} {jog} b={arm.base_deg:+6.1f} "
                  f"s={arm.shoulder_deg:+6.1f} e={arm.elbow_deg:+6.1f}  "
                  f"markers={n_markers}/4  {t_str}")

        # ════════════════════ STATE MACHINE ═══════════════════════════════
        if state == STATE_IDLE:
            pass                       # camera + detection live, motors idle

        elif state == STATE_SCAN:
            # confirm a find only with a solid boundary (>= 2 markers)
            valid_find = (target is not None
                          and n_markers >= SCAN_MIN_MARKERS)
            stop_hits = stop_hits + 1 if valid_find else 0
            if stop_hits >= DETECT_HITS:
                arm.jog_stop()
                print(f"[SCAN] Object found: {target['label']} at "
                      f"base={arm.base_deg:+.1f} deg "
                      f"({n_markers}/4 markers) -> centering")
                state = STATE_CENTER
                center_cx, lock_hits, lost_frames = [], 0, 0
                saturated, dry_hint = 0, False
                msg = f"found {target['label']}"
            else:
                # reverse smoothly at the sweep bounds
                if arm.jogging and (
                        (arm.jog_dir > 0 and arm.base_deg >= BASE_MAX) or
                        (arm.jog_dir < 0 and arm.base_deg <= BASE_MIN)):
                    arm.jog_stop(resync=not arm.dry_run)
                    sweep_dir = -sweep_dir
                if not arm.jogging:
                    arm.jog_speed(1, SCAN_SPS)
                    arm.jog_start(1, sweep_dir)

        elif state == STATE_CENTER:
            if target is None:
                if arm.jogging:
                    arm.jog_stop()
                lost_frames += 1
                if lost_frames > LOST_LIMIT:
                    print("[CENTER] Object lost -> rescanning")
                    go_rescan("object lost")
            elif arm.jogging:
                lost_frames = 0
                # real-time: stop the instant we are close to centre
                if abs(err_px) <= CENTER_TOL_PX * STOP_EARLY_FACTOR:
                    arm.jog_stop()
                elif ((arm.jog_dir > 0 and arm.base_deg >= BASE_MAX) or
                      (arm.jog_dir < 0 and arm.base_deg <= BASE_MIN)):
                    arm.jog_stop()
                    saturated += 1
                    if saturated >= 2:
                        print(f"[CENTER] Base at sweep limit, "
                              f"err={err_px:+.0f}px unreachable -> rescan")
                        saturated = 0
                        go_rescan("object beyond base limit")
            else:
                lost_frames = 0
                cx_new = target["center"][0]
                if center_cx and abs(cx_new - float(np.median(center_cx))) > 100:
                    center_cx = [cx_new]
                else:
                    center_cx.append(cx_new)
                if len(center_cx) >= CENTER_SAMPLES:
                    err = float(np.mean(center_cx)) - frame_cx
                    center_cx = []
                    if abs(err) <= CENTER_TOL_PX:
                        lock_hits += 1
                        if lock_hits >= CENTER_LOCK_HITS:
                            print(f"[CENTER] X centred (err={err:+.0f}px) at "
                                  f"base={arm.base_deg:+.1f} deg "
                                  f"-> shoulder (vertical)")
                            state = STATE_SHOULDER
                            sh_cy, sh_lock, sh_lost, sh_sat = [], 0, 0, 0
                            dry_hint = False
                            msg = "X centred - shoulder centering"
                    else:
                        lock_hits = 0
                        if arm.dry_run:
                            if not dry_hint:
                                dry_hint = True
                                print("[CENTER] DRY-RUN: jog skipped -- slide "
                                      "the object to the vertical centre line.")
                            msg = "dry-run: slide object to centre"
                        else:
                            desired = 1 if (BASE_DIRECTION * -err) > 0 else -1
                            at_bound = ((desired > 0 and
                                         arm.base_deg >= BASE_MAX) or
                                        (desired < 0 and
                                         arm.base_deg <= BASE_MIN))
                            if at_bound:
                                saturated += 1
                                if saturated >= 3:
                                    print(f"[CENTER] Base at sweep limit, err="
                                          f"{err:+.0f}px unreachable -> rescan")
                                    saturated = 0
                                    go_rescan("object beyond base limit")
                            else:
                                saturated = 0
                                arm.jog_speed(1, CENTER_SPS)
                                arm.jog_start(1, desired)

        elif state == STATE_SHOULDER:
            if target is None:
                if arm.jogging:
                    arm.jog_stop()
                sh_lost += 1
                if sh_lost > LOST_LIMIT:
                    print("[SHOULDER] Object lost -> rescanning")
                    go_rescan("object lost")
            elif abs(err_px) > RELOCK_PX:
                if arm.jogging:
                    arm.jog_stop()
                print(f"[SHOULDER] X drifted (err={err_px:+.0f}px) "
                      f"-> back to base centering")
                state = STATE_CENTER
                center_cx, lock_hits, lost_frames = [], 0, 0
                saturated, dry_hint = 0, False
                msg = "x drifted - re-centering base"
            elif arm.jogging:
                sh_lost = 0
                # real-time smooth vertical servo (same as the base)
                if abs(err_y) <= CENTER_TOL_PX * STOP_EARLY_FACTOR:
                    arm.jog_stop()
                elif ((arm.jog_dir > 0 and
                       arm.shoulder_deg >= SHOULDER_MAX) or
                      (arm.jog_dir < 0 and
                       arm.shoulder_deg <= SHOULDER_MIN)):
                    arm.jog_stop()
                    sh_sat += 1
                    if sh_sat >= 2:
                        print(f"[SHOULDER] At joint limit "
                              f"({arm.shoulder_deg:+.1f} deg) -> rescan")
                        sh_sat = 0
                        go_rescan("object beyond shoulder limit")
            else:
                sh_lost = 0
                cy_new = target["center"][1]
                if sh_cy and abs(cy_new - float(np.median(sh_cy))) > 100:
                    sh_cy = [cy_new]
                else:
                    sh_cy.append(cy_new)
                if len(sh_cy) >= CENTER_SAMPLES:
                    err = float(np.mean(sh_cy)) - frame_cy
                    sh_cy = []
                    if abs(err) <= CENTER_TOL_PX and \
                            abs(err_px) > CENTER_TOL_PX * 2:
                        print(f"[SHOULDER] Y centred but X off "
                              f"(err={err_px:+.0f}px) -> base re-centering")
                        state = STATE_CENTER
                        center_cx, lock_hits, lost_frames = [], 0, 0
                        saturated, dry_hint = 0, False
                        msg = "x drifted - re-centering base"
                    elif abs(err) <= CENTER_TOL_PX:
                        sh_lock += 1
                        if sh_lock >= CENTER_LOCK_HITS:
                            print("=" * 60)
                            print("[CENTRED] X+Y centred -> approaching with "
                                  "elbow + shoulder")
                            print(f"  base={arm.base_deg:+.1f}  "
                                  f"sh={arm.shoulder_deg:+.1f}  "
                                  f"el={arm.elbow_deg:+.1f} deg  "
                                  f"dist={dist_cm if dist_cm else -1:.1f} cm")
                            print("=" * 60)
                            state = STATE_APPROACH
                            ap_lost = ap_nodist = ap_flips = 0
                            ap_dir = ELBOW_APPROACH_DIRECTION
                            ap_best = dist_cm
                            dry_hint = False
                            msg = "approaching object"
                    else:
                        sh_lock = 0
                        if arm.dry_run:
                            if not dry_hint:
                                dry_hint = True
                                print("[SHOULDER] DRY-RUN: jog skipped -- "
                                      "slide object to horizontal centre line.")
                            msg = "dry-run: slide object to centre"
                        else:
                            desired = (1 if (SHOULDER_DIRECTION * -err) > 0
                                       else -1)
                            at_bound = ((desired > 0 and
                                         arm.shoulder_deg >= SHOULDER_MAX) or
                                        (desired < 0 and
                                         arm.shoulder_deg <= SHOULDER_MIN))
                            if at_bound:
                                sh_sat += 1
                                if sh_sat >= 3:
                                    print(f"[SHOULDER] At joint limit "
                                          f"({arm.shoulder_deg:+.1f} deg) "
                                          f"-> rescan")
                                    sh_sat = 0
                                    go_rescan("object beyond shoulder limit")
                            else:
                                sh_sat = 0
                                arm.jog_speed(2, SHOULDER_SPS)
                                arm.jog_start(2, desired)

        elif state == STATE_APPROACH:
            # Elbow jogs smoothly toward the object; shoulder jogs Y back
            # to centre whenever it drifts; base fixes X drift.
            if target is None:
                if arm.jogging:
                    arm.jog_stop()
                ap_lost += 1
                if ap_lost > LOST_LIMIT:
                    print("[APPROACH] Object lost -> rescanning")
                    go_rescan("object lost")
            elif abs(err_px) > RELOCK_PX:
                if arm.jogging:
                    arm.jog_stop()
                print(f"[APPROACH] X drifted (err={err_px:+.0f}px) "
                      f"-> base re-centering")
                state = STATE_CENTER
                center_cx, lock_hits, lost_frames = [], 0, 0
                saturated, dry_hint = 0, False
                msg = "x drifted - re-centering base"
            else:
                ap_lost = 0
                if dist_cm is None:
                    if arm.jogging:
                        arm.jog_stop()
                    ap_nodist += 1
                    msg = "no distance (unknown object)"
                    if ap_nodist > 45:
                        print("[APPROACH] No distance measurement for this "
                              "object -> holding here (ARRIVED)")
                        state = STATE_ARRIVED
                        locked_off = locked_lost = 0
                        msg = "arrived (no distance available)"
                elif dist_cm <= APPROACH_STOP_CM:
                    arm.jog_stop()
                    print("=" * 60)
                    print("[ARRIVED] ARM STOPPED AT OBJECT")
                    print(f"  object  : {target['label']}")
                    print(f"  base    : {arm.base_deg:+.2f} deg")
                    print(f"  shoulder: {arm.shoulder_deg:+.2f} deg")
                    print(f"  elbow   : {arm.elbow_deg:+.2f} deg")
                    print(f"  distance: {dist_cm:.1f} cm "
                          f"(stop at {APPROACH_STOP_CM:.0f} cm)")
                    if target.get("mat_mm") is not None:
                        m = target["mat_mm"]
                        print(f"  mat pos : ({m[0]:.0f}, {m[1]:.0f}) mm")
                    print("=" * 60)
                    state = STATE_ARRIVED
                    locked_off = locked_lost = 0
                    msg = "ARRIVED -- motors stopped at object"
                elif arm.dry_run:
                    ap_nodist = 0
                    if not dry_hint:
                        dry_hint = True
                        print("[APPROACH] DRY-RUN: jog skipped -- bring the "
                              "object closer to the camera to simulate "
                              f"arrival (< {APPROACH_STOP_CM:.0f} cm).")
                    msg = f"dry-run: bring object < {APPROACH_STOP_CM:.0f} cm"
                elif arm.jogging:
                    ap_nodist = 0
                    if arm.jog_axis == 3:
                        # Elbow advancing -- watch the DISTANCE TREND and
                        # auto-reverse if the object is getting farther.
                        if ap_best is None or \
                                dist_cm < ap_best - ELBOW_PROGRESS_CM:
                            ap_best = dist_cm     # real progress
                            ap_flips = 0
                        if err_y is not None and abs(err_y) > APPROACH_Y_TOL:
                            arm.jog_stop()        # pause: fix Y first
                        elif dist_cm > ap_best + ELBOW_TREND_CM:
                            arm.jog_stop()
                            ap_dir = -ap_dir
                            ap_flips += 1
                            ap_best = dist_cm
                            print(f"[APPROACH] Distance growing "
                                  f"({dist_cm:.1f} cm) -> elbow reversed "
                                  f"({'+' if ap_dir > 0 else '-'}) "
                                  f"[{ap_flips}/3]")
                            if ap_flips >= 3:
                                print("[APPROACH] Cannot reduce distance "
                                      "-> holding here (ARRIVED)")
                                state = STATE_ARRIVED
                                locked_off = locked_lost = 0
                                msg = "arrived (distance not reducing)"
                        elif ((arm.jog_dir > 0 and
                               arm.elbow_deg >= ELBOW_MAX) or
                              (arm.jog_dir < 0 and
                               arm.elbow_deg <= ELBOW_MIN)):
                            arm.jog_stop()
                            print(f"[APPROACH] Elbow at limit "
                                  f"({arm.elbow_deg:+.1f} deg) -> holding "
                                  f"here (ARRIVED) at {dist_cm:.1f} cm")
                            state = STATE_ARRIVED
                            locked_off = locked_lost = 0
                            msg = "arrived (elbow limit)"
                    else:
                        # shoulder fixing Y during the approach
                        if err_y is None or \
                                abs(err_y) <= CENTER_TOL_PX * STOP_EARLY_FACTOR:
                            arm.jog_stop()
                        elif ((arm.jog_dir > 0 and
                               arm.shoulder_deg >= SHOULDER_MAX) or
                              (arm.jog_dir < 0 and
                               arm.shoulder_deg <= SHOULDER_MIN)):
                            arm.jog_stop()
                            print("[APPROACH] Shoulder at limit during "
                                  "approach -> holding here (ARRIVED)")
                            state = STATE_ARRIVED
                            locked_off = locked_lost = 0
                            msg = "arrived (shoulder limit)"
                else:
                    ap_nodist = 0
                    if err_y is not None and abs(err_y) > APPROACH_Y_TOL:
                        desired = (1 if (SHOULDER_DIRECTION * -err_y) > 0
                                   else -1)
                        arm.jog_speed(2, SHOULDER_SPS)
                        arm.jog_start(2, desired)
                    else:
                        if ap_best is None:
                            ap_best = dist_cm
                        arm.jog_speed(3, ELBOW_SPS)
                        arm.jog_start(3, ap_dir)

        elif state == STATE_ARRIVED:
            # Motors stopped at the object.  Watchdog keeps tracking.
            if target is None:
                locked_lost += 1
                locked_off = 0
                if locked_lost > LOCKED_LOST_LIMIT:
                    print("[ARRIVED] Object gone -> rescanning")
                    go_rescan("object gone - rescanning")
            else:
                locked_lost = 0
                if abs(err_px) > RELOCK_PX:
                    locked_off += 1
                    if locked_off >= RELOCK_HITS:
                        print(f"[ARRIVED] Object moved (x={err_px:+.0f}px) "
                              f"-> re-centering base")
                        state = STATE_CENTER
                        center_cx, lock_hits, lost_frames = [], 0, 0
                        saturated, dry_hint = 0, False
                        msg = "object moved - re-centering"
                elif err_y is not None and abs(err_y) > RELOCK_PX:
                    locked_off += 1
                    if locked_off >= RELOCK_HITS:
                        print(f"[ARRIVED] Object moved (y={err_y:+.0f}px) "
                              f"-> shoulder re-centering")
                        state = STATE_SHOULDER
                        sh_cy, sh_lock, sh_lost, sh_sat = [], 0, 0, 0
                        dry_hint = False
                        msg = "object moved - shoulder re-centering"
                elif dist_cm is not None and \
                        dist_cm > APPROACH_STOP_CM + 10.0:
                    locked_off += 1
                    if locked_off >= RELOCK_HITS:
                        print(f"[ARRIVED] Object moved away "
                              f"({dist_cm:.1f} cm) -> re-approaching")
                        state = STATE_APPROACH
                        ap_lost = ap_nodist = ap_flips = 0
                        ap_dir = ELBOW_APPROACH_DIRECTION
                        ap_best = dist_cm
                        dry_hint = False
                        msg = "object moved away - approaching"
                else:
                    locked_off = 0

        # ── HUD + keys ────────────────────────────────────────────────────
        draw_hud(frame, state, arm, sweep_dir, err_px, err_y, msg)
        cv.imshow(WIN, frame)
        key = cv.waitKey(1) & 0xFF

        if key in (27, ord('q')):
            break
        elif key == ord('s'):
            print("[KEY] Scan started")
            if arm.jogging:
                arm.jog_stop()
            pose = load_view_pose()
            if pose is not None:
                sh, el = pose
                print(f"[ARM] Moving to scan pose from view_pose.json "
                      f"(sh={sh:+.1f}, el={el:+.1f} deg) ...")
                arm.move_joint(2, sh)
                arm.move_joint(3, el)
                flush_camera(cap)
            go_rescan("scanning")
        elif key == ord('r'):
            print("[KEY] Rescan")
            go_rescan("manual rescan")
        elif key == ord('c'):
            print("[KEY] Calibrate")
            if arm.jogging:
                arm.jog_stop()
            state = STATE_IDLE
            track_pt = None
            if arm.calibrate():
                msg = "calibrated (joints at 0) -- press S to scan"
            else:
                msg = "calibration FAILED -- see console"
        elif key == ord(' '):
            if state == STATE_PAUSED:
                state = prev_state
                print("[KEY] Resumed")
            else:
                if arm.jogging:
                    arm.jog_stop()
                prev_state = state
                state = STATE_PAUSED
                print("[KEY] Paused (motors idle)")

    if arm.jogging:
        arm.jog_stop(resync=False)
    arm.close()
    cap.release()
    cv.destroyAllWindows()
    print(f"[EXIT] Final pose: base={arm.base_deg:+.1f}  "
          f"shoulder={arm.shoulder_deg:+.1f}  "
          f"elbow={arm.elbow_deg:+.1f} deg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
