"""
aruco_plane_calibrator.py  —  Workspace Plane Calibration via 4 ArUco Corners
==============================================================================
Uses the 4 ArUco markers placed at the corners of the black working mat
(IDs 0–3 from aruco_checkerboard_A4.png) to calibrate the workspace plane.

What this does:
  1. Detects the 4 corner ArUco markers in the live camera feed.
  2. Computes a perspective homography:
         pixel (u, v)  →  real-world (X_mm, Y_mm) on the mat plane
  3. Verifies the homography by mapping pixel centres back to known mm positions.
  4. Saves the calibration to  aruco_plane_calibration.npz  for use by the
     distance estimator and pick-and-place planning code.

Marker ID layout (matching aruco_checkerboard_A4.png):
  ID 0 = Top-Left      ID 1 = Top-Right
  ID 3 = Bottom-Left   ID 2 = Bottom-Right

How to use:
  1. Place the printed aruco_checkerboard_A4.png sheet flat on the black mat
     OR place the 4 individual ArUco markers at the 4 corners of the mat.
  2. Measure the real-world dimensions of your black mat (in mm).
     Update MAT_W_MM and MAT_H_MM below.
  3. Run this script. A live camera window opens.
  4. Ensure ALL 4 markers are clearly visible in the frame.
  5. Press [SPACE] to lock the calibration when the green overlay looks correct.
  6. Press [q] to quit.

Keys:
  SPACE → capture & save calibration (when all 4 markers visible)
  v     → verify: click points in the window to see their mm coordinates
  q     → quit

Output file: aruco_plane_calibration.npz
  Keys: H (3x3 homography), H_inv (inverse), mat_w_mm, mat_h_mm,
        img_corners (4x2 pixel coords), world_corners (4x2 mm coords),
        reprojection_error_px

Author: Shyam Hirpara  |  Date: 2026-06-10
"""

import cv2 as cv
import numpy as np
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config

# ── SETTINGS — update to match your actual black mat dimensions ────────────────
# Measure your physical black mat with a tape measure.
MAT_W_MM = 480.0   # mat width  in mm  (left edge to right edge)
MAT_H_MM = 495.0   # mat height in mm  (top edge to bottom edge)

# ArUco dictionary — must match the one used to generate the markers
ARUCO_DICT_ID = cv.aruco.DICT_4X4_50

# Output file
OUT_FILE = os.path.join(os.path.dirname(__file__), "aruco_plane_calibration.npz")

# Marker IDs and their corresponding real-world corners on the mat (mm):
#   ID 0 = Top-Left     → (    0,      0)
#   ID 1 = Top-Right    → (W_mm,      0)
#   ID 2 = Bottom-Right → (W_mm,  H_mm)
#   ID 3 = Bottom-Left  → (    0,  H_mm)
# ──────────────────────────────────────────────────────────────────────────────


def get_aruco_detector():
    """Return (dictionary, parameters) compatible with OpenCV 4.x and 4.8+."""
    aruco_dict = cv.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
    try:
        # OpenCV 4.7+ — DetectorParameters object
        params = cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(aruco_dict, params)
        use_new_api = True
    except AttributeError:
        # Older OpenCV 4.x
        params = cv.aruco.DetectorParameters_create()
        detector = None
        use_new_api = False
    return aruco_dict, params, detector, use_new_api


def detect_markers(frame_gray, aruco_dict, params, detector, use_new_api):
    """
    Detect all ArUco markers in the grayscale frame.

    Returns:
        corners (list), ids (np.ndarray|None)
    """
    if use_new_api:
        corners, ids, _ = detector.detectMarkers(frame_gray)
    else:
        corners, ids, _ = cv.aruco.detectMarkers(frame_gray, aruco_dict, parameters=params)
    return corners, ids


def get_marker_centre(corners_one_marker: np.ndarray) -> np.ndarray:
    """Return the pixel centre of a single marker (mean of 4 corners)."""
    return corners_one_marker[0].mean(axis=0)   # shape (2,)


def build_world_corners(w_mm: float, h_mm: float) -> dict:
    """
    Build the mapping: marker_id → real-world position (mm) on the mat.

    Convention (Y increases downward, matching image coords):
      ID 0 → Top-Left     (0,     0)
      ID 1 → Top-Right    (w_mm,  0)
      ID 2 → Bottom-Right (w_mm,  h_mm)
      ID 3 → Bottom-Left  (0,     h_mm)
    """
    return {
        0: np.array([0.0,    0.0],    dtype=np.float64),
        1: np.array([w_mm,   0.0],    dtype=np.float64),
        2: np.array([w_mm,   h_mm],   dtype=np.float64),
        3: np.array([0.0,    h_mm],   dtype=np.float64),
    }


def compute_homography(img_pts_dict: dict, world_pts_dict: dict
                       ) -> tuple[np.ndarray, float]:
    """
    Compute the homography H such that:
        [X_mm, Y_mm, 1]^T  ∝  H · [u, v, 1]^T

    Args:
        img_pts_dict   : {marker_id: pixel_centre (2,)}
        world_pts_dict : {marker_id: real_world_mm (2,)}

    Returns:
        H              : 3x3 homography matrix (pixel → mm)
        reproj_err_px  : Mean reprojection error in pixels (H_inv used)
    """
    ids_present = sorted(img_pts_dict.keys())
    if len(ids_present) < 4:
        raise ValueError(f"Need all 4 markers, got {ids_present}")

    # Build ordered Nx2 arrays (TL, TR, BR, BL order = IDs 0,1,2,3)
    src = np.array([img_pts_dict[i]   for i in [0, 1, 2, 3]], dtype=np.float64)
    dst = np.array([world_pts_dict[i] for i in [0, 1, 2, 3]], dtype=np.float64)

    H, mask = cv.findHomography(src, dst, method=0)   # exact fit with 4 pts

    # Reprojection: map mm back to pixels using H_inv, measure pixel distance
    H_inv = np.linalg.inv(H)
    errors = []
    for i in [0, 1, 2, 3]:
        px_orig  = img_pts_dict[i]
        mm_pt    = np.append(world_pts_dict[i], 1.0)
        px_back  = H_inv @ mm_pt
        px_back /= px_back[2]
        errors.append(np.linalg.norm(px_back[:2] - px_orig))
    reproj_err = float(np.mean(errors))

    return H, reproj_err


def pixel_to_mm(u: float, v: float, H: np.ndarray) -> tuple[float, float]:
    """Map a pixel coordinate to mat mm using homography H."""
    pt = H @ np.array([u, v, 1.0], dtype=np.float64)
    return float(pt[0] / pt[2]), float(pt[1] / pt[2])


def draw_overlay(frame: np.ndarray,
                 detected: dict,
                 world_pts: dict,
                 H: np.ndarray | None,
                 all_found: bool) -> None:
    """
    Draw real-time overlay:
      - Each detected marker: coloured dot + ID + pixel centre
      - Lines connecting the 4 corners (when all found)
      - mm coordinate read-out at each corner
      - Semi-transparent green quad showing the calibrated mat area
    """
    h_img, w_img = frame.shape[:2]

    # ── Colour scheme per corner ───────────────────────────────────────────────
    ID_COLORS = {0: (0, 255, 100),    # green  — TL
                 1: (0, 200, 255),    # cyan   — TR
                 2: (255, 100, 0),    # blue   — BR
                 3: (255, 0, 180)}    # magenta— BL

    for mid, pixel_centre in detected.items():
        pu, pv = int(pixel_centre[0]), int(pixel_centre[1])
        col = ID_COLORS.get(mid, (255, 255, 255))

        cv.circle(frame, (pu, pv), 8, col, -1)
        cv.circle(frame, (pu, pv), 10, (255, 255, 255), 1)

        if H is not None:
            x_mm, y_mm = pixel_to_mm(pixel_centre[0], pixel_centre[1], H)
            label = f"ID:{mid}  ({x_mm:.0f},{y_mm:.0f}) mm"
        else:
            label = f"ID:{mid}"

        label_pos = (pu + 12, pv + 5)
        cv.putText(frame, label, label_pos,
                   cv.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 0),    2, cv.LINE_AA)
        cv.putText(frame, label, label_pos,
                   cv.FONT_HERSHEY_SIMPLEX, 0.50, col,           1, cv.LINE_AA)

    # ── Draw quad outline when all 4 found ────────────────────────────────────
    if all_found and len(detected) == 4:
        order = [0, 1, 2, 3, 0]   # close the polygon
        for i in range(4):
            a = detected[order[i]].astype(int)
            b = detected[order[i + 1]].astype(int)
            cv.line(frame, tuple(a), tuple(b), (0, 255, 0), 2, cv.LINE_AA)

        # Semi-transparent fill
        pts = np.array([detected[i].astype(int) for i in [0, 1, 2, 3]])
        overlay = frame.copy()
        cv.fillConvexPoly(overlay, pts, (0, 255, 0))
        cv.addWeighted(overlay, 0.12, frame, 0.88, 0, frame)


def draw_hud(frame: np.ndarray, n_found: int, calibrated: bool,
             reproj_err: float | None) -> None:
    """Draw top HUD strip with status info."""
    h, w = frame.shape[:2]
    bar = frame.copy()
    cv.rectangle(bar, (0, 0), (w, 60), (10, 10, 10), -1)
    cv.addWeighted(bar, 0.65, frame, 0.35, 0, frame)

    # Marker count
    count_col = (0, 220, 0) if n_found == 4 else (0, 140, 255)
    cv.putText(frame, f"Markers detected: {n_found}/4",
               (10, 22), cv.FONT_HERSHEY_SIMPLEX, 0.60, count_col, 2, cv.LINE_AA)

    # Status
    if calibrated and reproj_err is not None:
        status = f"CALIBRATED  reproj={reproj_err:.2f} px  [SPACE]=save  [q]=quit"
        status_col = (0, 255, 100)
    elif n_found == 4:
        status = "All markers found!  Press [SPACE] to calibrate & save"
        status_col = (0, 220, 255)
    else:
        status = "Show all 4 ArUco corner markers to the camera  [q]=quit"
        status_col = (80, 80, 255)

    cv.putText(frame, status,
               (10, 48), cv.FONT_HERSHEY_SIMPLEX, 0.46, status_col, 1, cv.LINE_AA)

    # Mat dimensions (top-right)
    dim_txt = f"Mat: {MAT_W_MM:.0f} x {MAT_H_MM:.0f} mm"
    (tw, _), _ = cv.getTextSize(dim_txt, cv.FONT_HERSHEY_SIMPLEX, 0.42, 1)
    cv.putText(frame, dim_txt, (w - tw - 10, 22),
               cv.FONT_HERSHEY_SIMPLEX, 0.42, (180, 180, 180), 1, cv.LINE_AA)


def run_calibrator(cam_index: int = config.CAMERA_INDEX):
    """
    Interactive live calibration session.
    Opens camera, detects the 4 ArUco markers, computes and saves homography.
    """
    aruco_dict, params, detector, use_new_api = get_aruco_detector()
    world_pts = build_world_corners(MAT_W_MM, MAT_H_MM)

    cap = cv.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {cam_index}")
        sys.exit(1)

    cap.set(cv.CAP_PROP_FRAME_WIDTH,  config.FRAME_W)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, config.FRAME_H)
    cap.set(cv.CAP_PROP_FPS,          config.FPS)

    actual_w = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

    print("=" * 62)
    print("  ARMOBOT Workspace Plane Calibrator")
    print("=" * 62)
    print(f"  Camera       : {cam_index} @ {actual_w}x{actual_h}")
    print(f"  Mat size     : {MAT_W_MM:.0f} x {MAT_H_MM:.0f} mm")
    print(f"  ArUco dict   : DICT_4X4_50")
    print(f"  Marker IDs   : 0=TL  1=TR  2=BR  3=BL")
    print(f"  Output file  : {OUT_FILE}")
    print()
    print("  STEPS:")
    print("  1. Place all 4 ArUco markers at the corners of the black mat.")
    print("  2. Move the camera until all 4 green circles appear.")
    print("  3. Press [SPACE] to lock & save the calibration.")
    print("=" * 62)

    WIN = "ARMOBOT Plane Calibrator  [SPACE]=capture  [q]=quit"
    cv.namedWindow(WIN, cv.WINDOW_NORMAL)
    cv.resizeWindow(WIN, 1000, 580)

    # Mouse-click → mm readout (active after calibration)
    H_saved = [None]

    def on_mouse(event, ux, vy, flags, param):
        if event == cv.EVENT_LBUTTONDOWN and H_saved[0] is not None:
            xmm, ymm = pixel_to_mm(ux, vy, H_saved[0])
            print(f"  [CLICK] pixel ({ux}, {vy})  →  mat ({xmm:.1f} mm, {ymm:.1f} mm)")

    cv.setMouseCallback(WIN, on_mouse)

    H_live      = None
    reproj_err  = None
    calibrated  = False

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Cannot read frame.")
            break

        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        corners, ids = detect_markers(gray, aruco_dict, params, detector, use_new_api)

        # Build dict of detected marker centres
        detected = {}
        if ids is not None:
            ids_flat = ids.flatten()
            for idx, mid in enumerate(ids_flat):
                if mid in world_pts:
                    detected[int(mid)] = get_marker_centre(corners[idx])

        all_found = len(detected) == 4

        # Compute live homography preview whenever all 4 are visible
        if all_found:
            try:
                H_live, reproj_err = compute_homography(detected, world_pts)
            except Exception as e:
                H_live = None
                reproj_err = None
        else:
            H_live = None

        # Draw overlay
        display = frame.copy()
        draw_overlay(display, detected, world_pts, H_live, all_found)
        draw_hud(display, len(detected), calibrated, reproj_err)

        cv.imshow(WIN, display)
        key = cv.waitKey(1) & 0xFF

        # ── SPACE: save calibration ────────────────────────────────────────────
        if key == ord(' '):
            if not all_found:
                print(f"[WARN] Only {len(detected)}/4 markers visible. "
                      "Show all 4 and try again.")
                continue

            H, err = compute_homography(detected, world_pts)
            H_inv  = np.linalg.inv(H)

            img_corners   = np.array([detected[i]   for i in [0,1,2,3]], dtype=np.float64)
            world_corners = np.array([world_pts[i]  for i in [0,1,2,3]], dtype=np.float64)

            np.savez(OUT_FILE,
                     H=H,
                     H_inv=H_inv,
                     mat_w_mm=np.float64(MAT_W_MM),
                     mat_h_mm=np.float64(MAT_H_MM),
                     img_corners=img_corners,
                     world_corners=world_corners,
                     reprojection_error_px=np.float64(err))

            H_saved[0] = H
            calibrated  = True

            print()
            print("=" * 62)
            print("  CALIBRATION SAVED")
            print("=" * 62)
            print(f"  File            : {OUT_FILE}")
            print(f"  Mat size        : {MAT_W_MM:.0f} x {MAT_H_MM:.0f} mm")
            print(f"  Reproj error    : {err:.3f} px  "
                  f"({'EXCELLENT' if err < 1 else 'GOOD' if err < 3 else 'POOR'})")
            print()
            print("  Homography H (pixel → mm):")
            for row in H:
                print(f"    {row}")
            print()
            print("  Corner mapping:")
            for mid in [0, 1, 2, 3]:
                pu, pv   = detected[mid]
                xmm, ymm = world_pts[mid]
                print(f"    ID {mid}: pixel ({pu:.1f}, {pv:.1f})  →  "
                      f"({xmm:.0f} mm, {ymm:.0f} mm)")
            print()
            print("  [TIP] Click anywhere in the window to read mm coordinates.")
            print("  [TIP] Press [q] to quit.")
            print("=" * 62)

        # ── q: quit ────────────────────────────────────────────────────────────
        elif key == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()

    if not calibrated:
        print("\n[INFO] No calibration was saved.")
    else:
        print(f"\n[INFO] Done. Load calibration with:")
        print(f"         data = np.load('{OUT_FILE}')")
        print(f"         H    = data['H']          # pixel → mat mm")
        print(f"         H_inv= data['H_inv']      # mat mm → pixel")


if __name__ == "__main__":
    run_calibrator()
