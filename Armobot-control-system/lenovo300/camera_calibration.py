"""
camera_calibration.py  —  Lenovo 300 FHD Webcam Calibration Tool
==================================================================
Captures checkerboard frames from the Lenovo 300 webcam and computes the
camera intrinsic matrix (K) and distortion coefficients using OpenCV's
cv.calibrateCamera().

Output: saves  camera_calibration.npz  (loaded by distance_estimator.py)

HOW TO USE:
-----------
1. Run generate_checkerboard.py and print the board on A4 landscape.
2. Measure one printed square with a ruler — update SQUARE_SIZE_MM in config.py.
3. Run this script. A live 720p window opens.
4. Hold the board and press [SPACE] to capture.

   CAPTURE TIPS FOR LENOVO 300 (95 degree wide FOV):
   - The wide angle means you can tilt the board much more than usual.
   - Capture from these angles for best results:
       a. Straight on — 3 captures at different distances (30, 50, 70 cm)
       b. Tilt left  30 deg, 45 deg
       c. Tilt right 30 deg, 45 deg
       d. Tilt up    30 deg, 45 deg
       e. Tilt down  30 deg, 45 deg
       f. Board in top-left corner of frame
       g. Board in top-right corner of frame
       h. Board in bottom-left corner of frame
       i. Board in bottom-right corner of frame
   - 15-25 captures is enough; diminishing returns after 30.

5. Press [c] to run calibration. Results are printed + saved.
6. Verify: fx and fy should be roughly 580-700 px for the Lenovo 300 at 720p.
   If values are far outside this range, recapture with more tilted angles.

Keys:
  SPACE → capture current frame
  c     → calibrate (need >= MIN_CALIB_FRAMES captures)
  d     → delete last capture
  u     → toggle undistort preview (after calibration)
  q     → quit

Author: Shyam Hirpara  |  Date: 2026-06-09
"""

import cv2 as cv
import numpy as np
import os
import sys

# ── ensure lenovo300/ is on the path ─────────────────────────────────────────
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config

# ── SUBPIXEL REFINEMENT CRITERIA ──────────────────────────────────────────────
SUBPIX_CRITERIA = (
    cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER,
    30, 0.001
)

WIN = "Lenovo 300 Calibration  [SPACE]=capture [c]=calibrate [q]=quit"


def build_object_points() -> np.ndarray:
    """
    Generate the 3D world coordinates of checkerboard inner corners.

    The board lies flat at Z=0. Corners are spaced SQUARE_SIZE_MM apart.

    Returns:
        np.ndarray: Shape (N, 3) float32.
    """
    cols, rows = config.CHESSBOARD_SIZE
    pts = np.zeros((cols * rows, 3), np.float32)
    pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    pts *= config.SQUARE_SIZE_MM
    return pts


def overlay_hud(frame: np.ndarray, n: int, found: bool,
                undistort_on: bool, calibrated: bool) -> None:
    """Draw capture count, board-found indicator, and key hints on the frame."""
    h, w = frame.shape[:2]
    bar = frame.copy()
    cv.rectangle(bar, (0, 0), (w, 55), (0, 0, 0), -1)
    cv.addWeighted(bar, 0.5, frame, 0.5, 0, frame)

    ready_color = (0, 220, 0) if n >= config.MIN_CALIB_FRAMES else (0, 180, 255)
    cv.putText(frame, f"Captures: {n}/{config.MIN_CALIB_FRAMES}+",
               (10, 22), cv.FONT_HERSHEY_SIMPLEX, 0.65, ready_color, 2, cv.LINE_AA)

    board_txt = "BOARD DETECTED" if found else "searching for board..."
    board_col = (0, 255, 0) if found else (0, 80, 255)
    cv.putText(frame, board_txt,
               (10, 46), cv.FONT_HERSHEY_SIMPLEX, 0.52, board_col, 1, cv.LINE_AA)

    hint = "[SPACE]=capture  [c]=calibrate  [d]=del last  [u]=undistort  [q]=quit"
    cv.putText(frame, hint,
               (w - 570, 22), cv.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1, cv.LINE_AA)

    if calibrated and undistort_on:
        cv.putText(frame, "UNDISTORT ON",
                   (w - 160, 46), cv.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv.LINE_AA)


def run_calibration(cam_index: int = config.CAMERA_INDEX):
    """
    Main interactive calibration session.

    Opens the Lenovo 300 webcam at 720p, lets the user capture checkerboard
    frames, then computes and saves the camera intrinsics.

    Args:
        cam_index (int): Camera device index.
    """
    obj_pts_template = build_object_points()
    all_obj_pts: list = []
    all_img_pts: list = []
    frame_size = None

    cap = cv.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {cam_index}.")
        return

    # Force 720p
    cap.set(cv.CAP_PROP_FRAME_WIDTH,  config.FRAME_W)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, config.FRAME_H)
    cap.set(cv.CAP_PROP_FPS,          config.FPS)

    actual_w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
    print(f"[INFO] Camera opened at {actual_w}x{actual_h}")
    print(f"       Checkerboard: {config.CHESSBOARD_SIZE[0]}x{config.CHESSBOARD_SIZE[1]} inner corners")
    print(f"       Square size : {config.SQUARE_SIZE_MM} mm")
    print(f"       Min captures: {config.MIN_CALIB_FRAMES}")
    print()

    camera_matrix  = None
    dist_coeffs    = None
    undistort_on   = False
    calibrated     = False

    cv.namedWindow(WIN, cv.WINDOW_NORMAL)
    cv.resizeWindow(WIN, 960, 540)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Cannot read frame.")
            break

        if frame_size is None:
            frame_size = (frame.shape[1], frame.shape[0])

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # Detect checkerboard corners
        found, corners = cv.findChessboardCorners(
            gray, config.CHESSBOARD_SIZE,
            cv.CALIB_CB_ADAPTIVE_THRESH | cv.CALIB_CB_NORMALIZE_IMAGE
        )

        display = frame.copy()

        if found:
            corners_refined = cv.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA
            )
            cv.drawChessboardCorners(display, config.CHESSBOARD_SIZE,
                                     corners_refined, found)
        else:
            corners_refined = None

        if undistort_on and calibrated:
            display = cv.undistort(display, camera_matrix, dist_coeffs)
            cv.putText(display, "UNDISTORTED VIEW",
                       (10, frame_size[1] - 12),
                       cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv.LINE_AA)

        overlay_hud(display, len(all_obj_pts), found, undistort_on, calibrated)
        cv.imshow(WIN, display)

        key = cv.waitKey(1) & 0xFF

        # ── SPACE: capture ─────────────────────────────────────────────────
        if key == ord(' '):
            if found and corners_refined is not None:
                all_obj_pts.append(obj_pts_template)
                all_img_pts.append(corners_refined)
                n = len(all_obj_pts)
                print(f"[CAPTURE {n:>2}] OK")
            else:
                print("[WARN] Board not visible — reposition and try again.")

        # ── c: calibrate ───────────────────────────────────────────────────
        elif key == ord('c'):
            n = len(all_obj_pts)
            if n < config.MIN_CALIB_FRAMES:
                print(f"[WARN] Need {config.MIN_CALIB_FRAMES} captures, have {n}.")
                continue

            print(f"\n[INFO] Running calibration with {n} frames...")

            ret_val, camera_matrix, dist_coeffs, rvecs, tvecs = cv.calibrateCamera(
                all_obj_pts, all_img_pts, frame_size, None, None,
                flags=cv.CALIB_FIX_ASPECT_RATIO   # square pixels on Lenovo 300
            )

            # ── Mean reprojection error ────────────────────────────────────
            total_err = 0.0
            for i in range(n):
                proj, _ = cv.projectPoints(
                    all_obj_pts[i], rvecs[i], tvecs[i],
                    camera_matrix, dist_coeffs
                )
                total_err += cv.norm(all_img_pts[i], proj, cv.NORM_L2) / len(proj)
            mean_err = total_err / n

            fx = camera_matrix[0, 0]
            fy = camera_matrix[1, 1]
            cx = camera_matrix[0, 2]
            cy = camera_matrix[1, 2]

            print("\n" + "=" * 60)
            print("  CALIBRATION RESULTS")
            print("=" * 60)
            print(f"  Frames used      : {n}")
            print(f"  Reprojection err : {mean_err:.4f} px  "
                  f"({'GOOD' if mean_err < 0.5 else 'ACCEPTABLE' if mean_err < 1.0 else 'POOR'})")
            print(f"  fx = {fx:.2f} px   fy = {fy:.2f} px")
            print(f"  cx = {cx:.2f} px   cy = {cy:.2f} px")

            # ── Sanity check ───────────────────────────────────────────────
            if fx < config.FX_MIN_SANE or fx > config.FX_MAX_SANE:
                print(f"\n  [WARNING] fx={fx:.0f} is outside expected range "
                      f"[{config.FX_MIN_SANE}, {config.FX_MAX_SANE}]!")
                print("  This usually means the checkerboard was never tilted.")
                print("  Please recapture with more angled positions.")
            else:
                print(f"\n  [OK] fx is within expected range "
                      f"[{config.FX_MIN_SANE}, {config.FX_MAX_SANE}] for Lenovo 300.")

            print(f"\n  Distortion: {dist_coeffs.ravel()}")
            print("=" * 60)

            # ── Save ───────────────────────────────────────────────────────
            np.savez(config.CALIB_FILE,
                     camera_matrix=camera_matrix,
                     dist_coeffs=dist_coeffs,
                     reprojection_error=np.float64(mean_err),
                     frame_size=np.array(frame_size))
            print(f"\n[SAVED] -> '{config.CALIB_FILE}'")
            calibrated = True

        # ── d: delete last ─────────────────────────────────────────────────
        elif key == ord('d'):
            if all_obj_pts:
                all_obj_pts.pop()
                all_img_pts.pop()
                print(f"[INFO] Deleted last. Remaining: {len(all_obj_pts)}")
            else:
                print("[INFO] Nothing to delete.")

        # ── u: toggle undistort ────────────────────────────────────────────
        elif key == ord('u'):
            if calibrated:
                undistort_on = not undistort_on
            else:
                print("[INFO] Calibrate first before toggling undistort.")

        # ── q: quit ────────────────────────────────────────────────────────
        elif key == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    run_calibration()
