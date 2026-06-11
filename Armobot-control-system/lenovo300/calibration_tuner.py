"""
calibration_tuner.py  —  Real-time Camera Matrix Fine-Tuning with Trackbars
=============================================================================
Loads camera_calibration.npz and lets you drag live OpenCV trackbars to
adjust fx, fy, cx, cy until the distance readings are accurate.

WHY YOU MIGHT NEED THIS:
------------------------
Even with a good calibration (reprojection error < 1.0 px), the computed
focal length can be slightly off — especially with the Lenovo 300's wide
angle lens. This tool lets you fine-tune without re-running a checkerboard
session.

HOW TO USE:
-----------
1. Place an object at an EXACT known distance (e.g. 30 cm).
2. Run this script. Two windows open:
     - "Tuner Controls"   : sliders for fx, fy, cx, cy
     - "Tuned Live View"  : camera feed with distance overlay
3. Drag fx/fy until the distance badge reads the correct value.
4. cx/cy are usually fine from calibration (leave them unless image looks off).
5. Press [s] to save the tuned matrix  →  overwrites camera_calibration.npz.
6. Run distance_estimator.py — it will pick up the new values automatically.

Keys:
  q → quit (without saving)
  s → save tuned matrix to camera_calibration.npz
  z → toggle distortion  (LOADED vs ALL-ZEROS — useful if lens is distorting)
  r → reset all sliders to the originally loaded values
  c → print current matrix to terminal

Trackbar scale:
  fx / fy  : slider value / 10  →  e.g. slider 7000 = fx 700.0 px
  cx / cy  : slider value       →  direct pixel value

Author: Shyam Hirpara  |  Date: 2026-06-09
"""

import cv2 as cv
import numpy as np
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config
from object_detector import detect_objects, preprocess
from distance_estimator import (
    load_calibration,
    build_object_points,
    get_image_corners,
    draw_3d_axes,
    rodrigues_to_euler,
)

# ── TRACKBAR SETTINGS ─────────────────────────────────────────────────────────
FX_FY_SCALE = 10          # slider ticks are ×10 so we get 1 decimal place
FX_FY_MAX   = 30_000      # max slider = 3000.0 px  (covers all webcams)
CX_MAX      = config.FRAME_W
CY_MAX      = config.FRAME_H

WIN_CTRL  = "Tuner Controls  [s]=save [z]=zero-dist [r]=reset [q]=quit"
WIN_VIDEO = "Tuned Live View"
# ──────────────────────────────────────────────────────────────────────────────


def make_K(fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    """Build a 3×3 camera intrinsic matrix from four scalars."""
    K = np.eye(3, dtype=np.float64)
    K[0, 0], K[1, 1] = fx, fy
    K[0, 2], K[1, 2] = cx, cy
    return K


def render_frame(frame: np.ndarray,
                 K: np.ndarray, dist: np.ndarray,
                 show_axes: bool, show_pose: bool) -> np.ndarray:
    """
    Detect objects, run solvePnP with the given K, and return annotated frame.

    Args:
        frame     : Raw BGR webcam frame.
        K         : Current (possibly tuned) 3×3 camera matrix.
        dist      : Distortion coefficients (may be zeroed).
        show_axes : Draw 3D axes overlay.
        show_pose : Draw pitch/yaw/roll text.

    Returns:
        np.ndarray: Annotated BGR frame.
    """
    frame_ud   = cv.undistort(frame, K, dist)
    output     = frame_ud.copy()
    detections = detect_objects(frame_ud)

    for det in detections:
        x, y   = det["pt1"]
        x2, y2 = det["pt2"]
        w, h   = det["pixel_w"], det["pixel_h"]
        cx, cy = det["center"]
        color  = det["color_bgr"]
        label  = det["label"]

        distance_cm = None
        rvec = tvec = None

        if det["w_cm"] is not None:
            obj_pts = build_object_points(det["w_cm"], det["h_cm"])
            img_pts = get_image_corners(x, y, w, h)
            ok, rvec, tvec = cv.solvePnP(obj_pts, img_pts, K, dist,
                                          flags=cv.SOLVEPNP_IPPE)
            if ok:
                distance_cm = float(tvec[2, 0])
                if not (0 < distance_cm < 500):
                    distance_cm = None

        cv.rectangle(output, det["pt1"], det["pt2"], color, 2)
        cv.circle(output, (cx, cy), 4, color, -1)

        if distance_cm is not None:
            prox_col  = (0, 140, 255) if distance_cm < config.CLOSE_DIST_CM else (0, 255, 255)
            dist_text = f"{distance_cm:.1f} cm"
        else:
            prox_col, dist_text = (128, 128, 128), "n/a"

        (tw, th), _ = cv.getTextSize(dist_text, cv.FONT_HERSHEY_SIMPLEX, 0.65, 2)
        lx, ly = cx - tw // 2, y - 14
        cv.rectangle(output, (lx-4, ly-th-4), (lx+tw+4, ly+4), (0, 0, 0), -1)
        cv.putText(output, dist_text, (lx, ly),
                   cv.FONT_HERSHEY_SIMPLEX, 0.65, prox_col, 2, cv.LINE_AA)

        cv.putText(output, label, (x, y2 + 18),
                   cv.FONT_HERSHEY_SIMPLEX, 0.46, color, 2, cv.LINE_AA)

        if show_pose and rvec is not None:
            p, ya, r = rodrigues_to_euler(rvec)
            cv.putText(output, f"P:{p:+.1f} Y:{ya:+.1f} R:{r:+.1f}",
                       (x, y2 + 34), cv.FONT_HERSHEY_SIMPLEX, 0.36,
                       (180, 180, 255), 1, cv.LINE_AA)

        if show_axes and rvec is not None and distance_cm is not None:
            draw_3d_axes(output, rvec, tvec, K, dist, cx, cy)

    fx, fy = K[0, 0], K[1, 1]
    cv.putText(output, f"fx={fx:.1f}  fy={fy:.1f}  objs={len(detections)}",
               (10, 26), cv.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 0), 2, cv.LINE_AA)

    return output


def run_tuner(cam_index: int = config.CAMERA_INDEX):
    """
    Open the camera and run the interactive fine-tuning session.

    Args:
        cam_index (int): Camera device index.
    """
    # ── Load base calibration ─────────────────────────────────────────────────
    base_K, base_dist = load_calibration()
    base_fx = float(base_K[0, 0])
    base_fy = float(base_K[1, 1])
    base_cx = float(base_K[0, 2])
    base_cy = float(base_K[1, 2])

    # ── Camera ────────────────────────────────────────────────────────────────
    cap = cv.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {cam_index}")
        return

    cap.set(cv.CAP_PROP_FRAME_WIDTH,  config.FRAME_W)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, config.FRAME_H)
    cap.set(cv.CAP_PROP_FPS,          config.FPS)

    ret, test = cap.read()
    fh, fw = test.shape[:2] if ret else (config.FRAME_H, config.FRAME_W)

    # ── Clamp / validate initial slider positions ─────────────────────────────
    def clamp(v: float, lo: float, hi: float) -> int:
        return int(max(lo, min(hi, v)))

    init_fx = clamp(base_fx * FX_FY_SCALE, 0, FX_FY_MAX)
    init_fy = clamp(base_fy * FX_FY_SCALE, 0, FX_FY_MAX)
    init_cx = clamp(base_cx, 0, CX_MAX)
    init_cy = clamp(base_cy, 0, CY_MAX)

    warned = False
    if base_fx > config.FX_MAX_SANE or base_fx < config.FX_MIN_SANE:
        init_fx = int(680 * FX_FY_SCALE)   # reasonable Lenovo 300 default
        print(f"[WARN] Loaded fx={base_fx:.0f} is outside sane range — "
              f"slider reset to 680.0 px")
        warned = True
    if base_fy > config.FX_MAX_SANE or base_fy < config.FX_MIN_SANE:
        init_fy = int(680 * FX_FY_SCALE)
        if not warned:
            print(f"[WARN] Loaded fy={base_fy:.0f} is outside sane range — "
                  f"slider reset to 680.0 px")

    # ── Trackbar window ───────────────────────────────────────────────────────
    cv.namedWindow(WIN_CTRL, cv.WINDOW_NORMAL)
    cv.resizeWindow(WIN_CTRL, 720, 260)

    cv.createTrackbar("fx  (/10)", WIN_CTRL, init_fx, FX_FY_MAX, lambda v: None)
    cv.createTrackbar("fy  (/10)", WIN_CTRL, init_fy, FX_FY_MAX, lambda v: None)
    cv.createTrackbar("cx  (px)",  WIN_CTRL, init_cx, CX_MAX,    lambda v: None)
    cv.createTrackbar("cy  (px)",  WIN_CTRL, init_cy, CY_MAX,    lambda v: None)

    cv.namedWindow(WIN_VIDEO, cv.WINDOW_NORMAL)

    zero_dist = False
    show_axes = True
    show_pose = True
    prev_t    = time.time()

    print("\n[INFO] Calibration tuner started.")
    print("       Keys: [q]=quit [s]=save [z]=zero-dist [r]=reset [c]=print matrix")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # ── Read sliders ──────────────────────────────────────────────────
        fx = cv.getTrackbarPos("fx  (/10)", WIN_CTRL) / FX_FY_SCALE
        fy = cv.getTrackbarPos("fy  (/10)", WIN_CTRL) / FX_FY_SCALE
        cx = float(cv.getTrackbarPos("cx  (px)", WIN_CTRL))
        cy = float(cv.getTrackbarPos("cy  (px)", WIN_CTRL))

        K_now  = make_K(fx, fy, cx, cy)
        d_now  = np.zeros((5, 1), dtype=np.float64) if zero_dist else base_dist

        # ── Info panel ────────────────────────────────────────────────────
        now = time.time()
        fps = 1.0 / max(now - prev_t, 1e-6)
        prev_t = now

        info = np.zeros((90, 720, 3), dtype=np.uint8)
        cv.putText(info, f"fx={fx:.1f}  fy={fy:.1f}  cx={cx:.0f}  cy={cy:.0f}",
                   (10, 26), cv.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv.LINE_AA)
        dist_tag = "ZEROED" if zero_dist else "LOADED"
        cv.putText(info, f"distortion: {dist_tag}  |  FPS: {fps:.1f}  |  {fw}x{fh}",
                   (10, 54), cv.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1, cv.LINE_AA)
        cv.putText(info, "[z]=dist  [r]=reset  [s]=save  [a]=axes  [p]=pose  [q]=quit",
                   (10, 78), cv.FONT_HERSHEY_SIMPLEX, 0.40, (140, 140, 140), 1, cv.LINE_AA)
        cv.imshow(WIN_CTRL, info)

        # ── Video frame ───────────────────────────────────────────────────
        output = render_frame(frame, K_now, d_now, show_axes, show_pose)
        cv.putText(output, f"FPS:{fps:.1f}", (output.shape[1]-100, 26),
                   cv.FONT_HERSHEY_SIMPLEX, 0.48, (0, 200, 255), 2, cv.LINE_AA)
        cv.imshow(WIN_VIDEO, output)

        key = cv.waitKey(1) & 0xFF

        if key == ord('q'):
            print("[INFO] Quit — no changes saved.")
            break

        elif key == ord('s'):
            np.savez(config.CALIB_FILE,
                     camera_matrix=K_now,
                     dist_coeffs=d_now,
                     reprojection_error=np.float64(-1),
                     frame_size=np.array([fw, fh]),
                     tuned_manually=True)
            print(f"\n[SAVED] Tuned matrix written to: {config.CALIB_FILE}")
            print(f"  fx={fx:.1f}  fy={fy:.1f}  cx={cx:.0f}  cy={cy:.0f}")
            print(f"  distortion: {dist_tag}")

        elif key == ord('z'):
            zero_dist = not zero_dist
            print(f"[INFO] Distortion -> {'ZEROED' if zero_dist else 'LOADED'}")

        elif key == ord('r'):
            cv.setTrackbarPos("fx  (/10)", WIN_CTRL, init_fx)
            cv.setTrackbarPos("fy  (/10)", WIN_CTRL, init_fy)
            cv.setTrackbarPos("cx  (px)",  WIN_CTRL, init_cx)
            cv.setTrackbarPos("cy  (px)",  WIN_CTRL, init_cy)
            print("[INFO] Sliders reset to loaded calibration values.")

        elif key == ord('a'):
            show_axes = not show_axes

        elif key == ord('p'):
            show_pose = not show_pose

        elif key == ord('c'):
            print(f"\n[CURRENT MATRIX]\n{K_now}")

    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    run_tuner()
