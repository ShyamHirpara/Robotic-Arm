"""
vision.py  --  Object-detector version of the original ArUco vision.py
=======================================================================
IDENTICAL logic to the original vision.py.
Only change: ArUco ID:23 (arm) and ID:19 (target) replaced by:
    arm_pos    = (0, 0, 0)            -- arm assumed at camera centre
    target_pos = (x_mm, y_mm, z_mm)  -- nearest detected object (solvePnP)

4-corner ArUco mat boundary (IDs 0-3) added as visual ROI indicator.

dx = (tx - 10) - ax      (same formula as original)
dy = (ty - 110) - ay     (same formula as original) -- COMMENTED OUT
dz = (tz + 20)  - az     (same formula as original) -- COMMENTED OUT

d_j1/d_j2/d_j3 logic identical to original.
"""

import cv2
import numpy as np
import time
import os, sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

# Object detection + XYZ measurement (from calculate_distance.py)
from object_detector import detect_objects
from distance_estimator import (
    load_calibration,
    load_plane_calibration,
    make_aruco_detector,
    detect_mat_quad,
    draw_mat_overlay,
    point_in_quad,
    build_object_points,
    get_image_corners,
    triangle_similarity_distance,
)

# ===================== CALIBRATION (same constants as original) =====================
stepper1 = 6
# stepper2 = 9    # COMMENTED OUT -- only testing base
# stepper3 = 6    # COMMENTED OUT -- only testing base

DEAD_MM   = 30
# Y_DEAD_MM = 7   # COMMENTED OUT
# Z_DEAD_MM = 20  # COMMENTED OUT

MIN_STEP       = 2
MAX_STEP_SMALL = 4
MAX_STEP_MED   = 10
MAX_STEP_LARGE = 18

# ARM_MARKER_ID    = 23  -- REMOVED (no sticker needed)
# TARGET_MARKER_ID = 19  -- REMOVED (no sticker needed)


def adaptive_limit(d):
    d = abs(d)
    if d > 80:
        return MAX_STEP_LARGE
    if d > 40:
        return MAX_STEP_MED
    return MAX_STEP_SMALL


# ── solvePnP XYZ for detected object (from calculate_distance.py) ─────────────
def get_xyz_mm(det, K, dist_coeffs):
    """Returns (x_mm, y_mm, z_mm). None if solvePnP fails."""
    if det.get("w_cm") is None:
        return None, None, None
    x, y   = det["pt1"]
    w, h   = det["pixel_w"], det["pixel_h"]
    cx, cy = det["center"]
    z_init = triangle_similarity_distance(w, h, det["w_cm"], det["h_cm"], K, cx, cy)
    if not (0 < z_init < 500):
        z_init = 30.0
    obj_pts = build_object_points(det["w_cm"], det["h_cm"])
    img_pts = get_image_corners(x, y, w, h)
    ok, _, tvec = cv2.solvePnP(
        obj_pts, img_pts, K, dist_coeffs,
        rvec=np.zeros((3, 1), np.float64),
        tvec=np.array([[0.], [0.], [z_init]]),
        useExtrinsicGuess=True,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        return None, None, None
    return float(tvec[0][0]) * 10.0, float(tvec[1][0]) * 10.0, float(tvec[2][0]) * 10.0


# ===================== VISION THREAD =====================
def vision_loop(command_queue, shared_state=None):
    print("[Vision] Thread started")

    # Camera calibration (for solvePnP + mat boundary)
    K, dist_coeffs = load_calibration()
    plane          = load_plane_calibration()
    mat_detect     = make_aruco_detector()
    mat_quad       = None

    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("[Vision] ERROR: Camera not opened")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ── 4-corner mat boundary (visual ROI indicator) ──────────────────────
        mat_quad, n_markers = detect_mat_quad(gray, mat_detect, mat_quad, plane=plane)
        draw_mat_overlay(frame, mat_quad, n_markers)

        # ── Object detection (replaces ArUco ID:19 target) ────────────────────
        all_dets   = detect_objects(frame)
        detections = []
        if mat_quad is not None:
            detections = [d for d in all_dets
                          if point_in_quad(d["center"][0], d["center"][1], mat_quad)]

        # Pick nearest object as target (smallest Z depth)
        target_pos = None
        for det in sorted(detections, key=lambda d: d["center"][1]):
            x_mm, y_mm, z_mm = get_xyz_mm(det, K, dist_coeffs)
            if x_mm is not None:
                target_pos = (x_mm, y_mm, z_mm)

                # Show XYZ on camera (same as original putText)
                cx, cy = det["center"]
                cv2.rectangle(frame, det["pt1"], det["pt2"], det["color_bgr"], 2)
                cv2.putText(
                    frame,
                    f"{det['label']} X:{int(x_mm)} Y:{int(y_mm)} Z:{int(z_mm)}",
                    (det["pt1"][0], det["pt1"][1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, det["color_bgr"], 2
                )
                break  # use only nearest object

        # ── arm_pos: arm assumed at camera centre (no physical marker) ────────
        arm_pos = (0, 0, 0)   # replaces ArUco ID:23

        # ── EXACT original logic below (unchanged) ────────────────────────────
        if arm_pos and target_pos:
            ax, ay, az = arm_pos
            tx, ty, tz = target_pos

            dx = (tx) - ax
            dy = (ty - 110) - ay
            dz = (tz + 20) - az

            d_j1 = d_j2 = d_j3 = 0

            # Get current degree from shared state (populated by sender thread)
            current_degree = 0.0
            command_pending = False
            if shared_state is not None:
                current_degree = shared_state.get("current_degree", 0.0)
                command_pending = shared_state.get("command_pending", False)

            # target position (in mm) along X axis
            target_position_mm = tx - 10
            # current position (in mm) along X axis
            current_position_mm = -current_degree * stepper1
            # move position = target position - current position
            move_position_mm = target_position_mm - current_position_mm
            # target degree = target position in degrees
            target_degree = -target_position_mm / stepper1
            # move degree = target degree - current degree
            move_degree = target_degree - current_degree

            if abs(move_position_mm) > DEAD_MM:
                limit = adaptive_limit(move_position_mm)
                raw_step = int(round(move_degree))
                d_j1 = max(-limit, min(limit, raw_step))

            # if abs(dy) > Y_DEAD_MM:                                     # COMMENTED OUT
            #     d_j2 = max(-adaptive_limit(dy),                          # COMMENTED OUT
            #                min(adaptive_limit(dy), int(dy / stepper2)))  # COMMENTED OUT

            # if abs(dz) > Z_DEAD_MM:                                      # COMMENTED OUT
            #     d_j3 = max(-adaptive_limit(dz),                          # COMMENTED OUT
            #                min(adaptive_limit(dz), int(-dz / stepper3))) # COMMENTED OUT

            if abs(d_j1) < MIN_STEP: d_j1 = 0
            # if abs(d_j2) < MIN_STEP: d_j2 = 0   # COMMENTED OUT
            # if abs(d_j3) < MIN_STEP: d_j3 = 0   # COMMENTED OUT

            # ── Only send command if arm needs to move AND no command pending ──
            if d_j1 != 0:
                if command_pending:
                    # Previous command still being processed — wait
                    pass
                elif not command_queue.full():
                    shared_state["command_pending"] = True
                    command_queue.put((d_j1, d_j2, d_j3))

                # ── Detailed terminal print (only when moving) ─────────────
                print(
                    f"[BASE] target=({int(tx)},{int(ty)},{int(tz)}) mm  "
                    f"current position={int(round(current_position_mm))} mm "
                    f"move position= {int(round(move_position_mm))} mm  \n"
                    f"current degree={int(round(current_degree)):+d} deg  "
                    f"target degree = {int(round(target_degree))} "
                    f"and move degree =  {int(round(move_degree)):+d} deg"
                    f"{'  [PENDING]' if command_pending else ''}"
                )
            else:
                # Arm is within dead zone — object arrived at base
                print(
                    f"[BASE] ✅ ARRIVED  move_pos={int(round(move_position_mm))} mm "
                    f"(within dead zone {DEAD_MM} mm)  "
                    f"current degree={int(round(current_degree)):+d} deg"
                )
                time.sleep(0.5)  # throttle prints when idle

        cv2.imshow("Vision", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
