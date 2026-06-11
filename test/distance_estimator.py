"""
distance_estimator.py  —  Live Multi-Object solvePnP Distance Estimation
==========================================================================
Detects and classifies white objects (Small Cube, Large Cube, Cylinder)
using object_detector.py, then estimates each object's distance from the
Lenovo 300 webcam using the Perspective-n-Point (solvePnP) algorithm.

MATH (Pinhole Camera Model):
-----------------------------
    s × [u, v, 1]T  =  K × [R | t] × [X, Y, Z, 1]T

Where K (intrinsic matrix) is loaded from camera_calibration.npz.
cv.solvePnP() solves for t (translation vector). t[2] = Z = depth (cm).

Per-object solvePnP:
  Each detected + classified object uses its own real-world front-face
  dimensions (from config.OBJECTS) to build 4 object points, then matches
  them to the 4 bounding-box pixel corners as image points.

MODES:
------
  camera  — live webcam stream (default)
  image   — single saved image

Keys:
  q → quit
  s → save snapshot
  m → toggle mask view
  a → toggle 3D axes
  p → toggle pose (pitch/yaw/roll) text
  c → print current detections to terminal

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


# ── LOAD CALIBRATION ──────────────────────────────────────────────────────────

def load_calibration() -> tuple[np.ndarray, np.ndarray]:
    """
    Load camera matrix K and distortion coefficients from config.CALIB_FILE.

    Returns:
        (camera_matrix, dist_coeffs): Both are np.ndarray.

    Raises:
        SystemExit: If the calibration file does not exist.
    """
    path = config.CALIB_FILE
    if not os.path.exists(path):
        print(f"[ERROR] Calibration file not found: {path}")
        print("  Run camera_calibration.py first.")
        sys.exit(1)

    data = np.load(path)
    K    = data["camera_matrix"]
    dist = data["dist_coeffs"]
    err  = float(data.get("reprojection_error", -1))

    print(f"[INFO] Calibration loaded from '{path}'")
    print(f"  fx={K[0,0]:.2f}  fy={K[1,1]:.2f}  cx={K[0,2]:.2f}  cy={K[1,2]:.2f}")
    if err >= 0:
        quality = "GOOD" if err < 0.5 else ("ACCEPTABLE" if err < 1.0 else "POOR")
        print(f"  Reprojection error: {err:.4f} px  ({quality})")

    return K, dist


# ── ARUCO PLANE CALIBRATION ───────────────────────────────────────────

PLANE_CALIB_FILE = os.path.join(os.path.dirname(__file__), "aruco_plane_calibration.npz")


def load_plane_calibration(path: str = PLANE_CALIB_FILE):
    """
    Load the ArUco workspace-plane calibration produced by aruco_plane_calibrator.py.

    Returns:
        dict with keys H, H_inv, mat_w_mm, mat_h_mm  — or None if file missing.
    """
    if not os.path.exists(path):
        return None
    data = np.load(path)
    plane = {
        "H":        data["H"],          # 3x3 pixel → mat mm
        "H_inv":    data["H_inv"],      # 3x3 mat mm → pixel
        "mat_w_mm": float(data["mat_w_mm"]),
        "mat_h_mm": float(data["mat_h_mm"]),
    }
    err = float(data.get("reprojection_error_px", -1))
    quality = "EXCELLENT" if err < 1 else ("GOOD" if err < 3 else "POOR")
    print(f"[INFO] Plane calibration loaded from '{path}'")
    print(f"  Mat: {plane['mat_w_mm']:.0f} x {plane['mat_h_mm']:.0f} mm  "
          f"reproj_err={err:.2f} px ({quality})")
    return plane


# ── GEOMETRY HELPERS ──────────────────────────────────────────────────────────

def build_object_points(w_cm: float, h_cm: float) -> np.ndarray:
    """
    Build 4 coplanar 3D front-face corners of an object (Z=0 plane).

    The origin is at the face centre (top-left clockwise order):
        (-w/2, -h/2, 0)  →  (w/2, -h/2, 0)
              ↓                      ↓
        (-w/2,  h/2, 0)  →  (w/2,  h/2, 0)

    Args:
        w_cm: Front face width in centimetres.
        h_cm: Front face height in centimetres.

    Returns:
        np.ndarray: Shape (4, 3) float32.
    """
    hw, hh = w_cm / 2.0, h_cm / 2.0
    return np.array([
        [-hw, -hh, 0.0],
        [ hw, -hh, 0.0],
        [ hw,  hh, 0.0],
        [-hw,  hh, 0.0],
    ], dtype=np.float32)


def get_image_corners(x: int, y: int, w: int, h: int) -> np.ndarray:
    """
    Extract the 4 bounding-box corners as solvePnP image points (clockwise).

    Returns:
        np.ndarray: Shape (4, 1, 2) float32.
    """
    return np.array([
        [[float(x),     float(y)    ]],
        [[float(x + w), float(y)    ]],
        [[float(x + w), float(y + h)]],
        [[float(x),     float(y + h)]],
    ], dtype=np.float32)


def triangle_similarity_distance(pixel_w: int, pixel_h: int,
                                 real_w_cm: float, real_h_cm: float,
                                 K: np.ndarray,
                                 obj_cx: int, obj_cy: int) -> float:
    """
    Estimate object distance using the Triangle Similarity (focal length) formula.

    This method is robust across the entire camera frame, including off-center
    positions where solvePnP with axis-aligned bounding box corners is inaccurate.

    Formula:
        d_w = (real_width_cm  x focal_length_x) / effective_pixel_width
        d_h = (real_height_cm x focal_length_y) / effective_pixel_height
        d   = (d_w + d_h) / 2

    Tilt compensation (for near-square objects):
        When a cube is viewed at an angle, its bounding box width is inflated by
        the visible side face.  For objects whose real w ≈ h (square face), we
        substitute min(pixel_w, pixel_h) for both axes, because the shorter bbox
        dimension is far less distorted by horizontal tilt than the wider one.

    Ray-angle → Euclidean correction:
        Triangle Similarity gives Z-depth (distance along the optical axis).
        We convert to the true physical (Euclidean) distance by multiplying by
        the secant of the ray angle:
            d_euclidean = d_z × √(1 + (Δx/fx)² + (Δy/fy)²)
        where Δx, Δy are pixel offsets from the principal point.

    Args:
        pixel_w   : Bounding box width in pixels.
        pixel_h   : Bounding box height in pixels.
        real_w_cm : Known real-world object width in cm.
        real_h_cm : Known real-world object height in cm.
        K         : 3x3 camera intrinsic matrix.
        obj_cx    : Bounding box centre x in pixels.
        obj_cy    : Bounding box centre y in pixels.

    Returns:
        float: Estimated Euclidean distance in centimetres.
    """
    fx     = K[0, 0]
    fy     = K[1, 1]
    cx_cam = K[0, 2]
    cy_cam = K[1, 2]

    # ── Tilt compensation ────────────────────────────────────────────────────
    # For near-square objects, use min(pixel_w, pixel_h) as the effective
    # dimension so that the wider side-face (visible when tilted) does not
    # shrink the distance estimate.
    if abs(real_w_cm - real_h_cm) < 1.0:   # roughly square face (e.g. cube)
        eff_px_w = min(pixel_w, pixel_h)
        eff_px_h = min(pixel_w, pixel_h)
    else:
        eff_px_w = pixel_w
        eff_px_h = pixel_h

    d_w = (real_w_cm * fx) / max(eff_px_w, 1)
    d_h = (real_h_cm * fy) / max(eff_px_h, 1)

    # Z-depth along the optical axis
    z_depth = (d_w + d_h) / 2.0

    # ── Ray-angle → Euclidean correction ────────────────────────────────────
    # Off-centre objects have a longer Euclidean distance than their Z-depth.
    ray_x = (obj_cx - cx_cam) / fx
    ray_y = (obj_cy - cy_cam) / fy
    euclidean_dist = z_depth * np.sqrt(1.0 + ray_x ** 2 + ray_y ** 2)

    return euclidean_dist


def rodrigues_to_euler(rvec) -> tuple[float, float, float]:
    """
    Convert a solvePnP rotation vector to Euler angles (pitch, yaw, roll) in degrees.

    Args:
        rvec: Rotation vector (3×1 array).

    Returns:
        (pitch, yaw, roll) in degrees.
    """
    R, _ = cv.Rodrigues(rvec)
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        pitch = np.degrees(np.arctan2( R[2, 1], R[2, 2]))
        yaw   = np.degrees(np.arctan2(-R[2, 0], sy))
        roll  = np.degrees(np.arctan2( R[1, 0], R[0, 0]))
    else:
        pitch = np.degrees(np.arctan2(-R[1, 2], R[1, 1]))
        yaw   = np.degrees(np.arctan2(-R[2, 0], sy))
        roll  = 0.0
    return pitch, yaw, roll


# ── ARUCO MAT DETECTION HELPERS ───────────────────────────────────────────

def make_aruco_detector():
    """
    Build a callable ArUco detector for DICT_4X4_50.
    Compatible with both OpenCV 4.x and 4.7+ (new API).

    Returns a function:  detect(gray) → (corners, ids)
    """
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
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


# Known mat corner world positions (mm), indexed by marker ID:
#   0 = TL  1 = TR  2 = BR  3 = BL
# These are computed from mat_w_mm / mat_h_mm stored in plane calibration.


def _world_mm_corners(plane: dict) -> dict:
    """Return {marker_id: np.array([x_mm, y_mm])} for all 4 corners."""
    W, H = plane["mat_w_mm"], plane["mat_h_mm"]
    return {
        0: np.array([0.0, 0.0]),
        1: np.array([W,   0.0]),
        2: np.array([W,   H  ]),
        3: np.array([0.0, H  ]),
    }


def _hybrid_quad(detected_centres: dict, plane: dict):
    """
    Reconstruct the full 4-corner mat quad in current pixel space from
    any number of visible ArUco markers (1–4) plus the saved plane calibration.

    Strategy:
      • Visible markers  → use their fresh detected pixel centre.
      • Invisible markers → project their known mm position to pixel using
        the calibration H_inv (the mm→pixel homography saved at calibration).
      • Compute an updated homography from these 4 mixed correspondences.
      • Project ALL 4 mat corners (mm) → pixel using the updated H_inv.

    This gives a geometrically correct ROI quad even when only 1–3 markers
    are visible (e.g. camera zoomed in on part of the mat).

    Args:
        detected_centres : {marker_id: np.array([u, v])}  (only visible ones)
        plane            : dict from load_plane_calibration()

    Returns:
        quad (np.ndarray (4,2) float32) or None on failure.
    """
    world_mm   = _world_mm_corners(plane)
    H_inv_calib = plane["H_inv"]     # mm → pixel at calibration time

    # Build 4 pixel ⇔ mm correspondences
    pixel_pts = np.zeros((4, 2), dtype=np.float64)
    world_pts = np.zeros((4, 2), dtype=np.float64)

    for mid in [0, 1, 2, 3]:
        mm_pos = world_mm[mid]
        if mid in detected_centres:
            # Fresh detection — high quality
            pixel_pts[mid] = detected_centres[mid]
        else:
            # Project mm → pixel using calibration H_inv
            pt = H_inv_calib @ np.array([mm_pos[0], mm_pos[1], 1.0])
            pixel_pts[mid] = pt[:2] / pt[2]
        world_pts[mid] = mm_pos

    # Compute updated H (pixel → mm) from the combined correspondences
    H_updated, _ = cv.findHomography(pixel_pts, world_pts, method=0)
    if H_updated is None:
        return None

    H_updated_inv = np.linalg.inv(H_updated)

    # Build quad corners:
    #   • Detected markers  → use their measured pixel position DIRECTLY.
    #     This guarantees zero re-projection error for visible corners so
    #     the coloured dot and the quad vertex always coincide.
    #   • Missing markers   → project from mm through the updated H_inv.
    #     The updated H was constrained by the fresh detections, so this
    #     estimate is much better than using the stale calibration H_inv alone.
    W, H = plane["mat_w_mm"], plane["mat_h_mm"]
    corners_mm = [
        np.array([0.0, 0.0, 1.0]),   # TL  (ID 0)
        np.array([W,   0.0, 1.0]),   # TR  (ID 1)
        np.array([W,   H,   1.0]),   # BR  (ID 2)
        np.array([0.0, H,   1.0]),   # BL  (ID 3)
    ]
    corner_ids = [0, 1, 2, 3]

    quad = []
    for mid, pt_mm in zip(corner_ids, corners_mm):
        if mid in detected_centres:
            # Use the measured pixel position — no projection error
            quad.append(detected_centres[mid].astype(np.float64))
        else:
            # Estimate via updated homography (constrained by fresh detections)
            pt_px  = H_updated_inv @ pt_mm
            pt_px /= pt_px[2]
            quad.append(pt_px[:2])

    return np.array(quad, dtype=np.float32)


def detect_mat_quad(gray: np.ndarray, detect_fn, last_quad, plane=None):
    """
    Detect ArUco corner markers (IDs 0–3) and return the mat ROI quad.

    Behaviour by number of visible markers:
      • N=0 : no markers — return cached last_quad (may be None).
      • N=1–3 : hybrid mode (requires plane calibration) — reconstruct the
                full quad from fresh + calibration-projected corner estimates.
                Falls back to last_quad if no plane calibration available.
      • N=4 : all markers visible — exact homography, best accuracy.

    Args:
        gray       : Grayscale frame.
        detect_fn  : ArUco detect callable from make_aruco_detector().
        last_quad  : Previously computed quad (for temporal caching).
        plane      : Dict from load_plane_calibration(), or None.

    Returns:
        (quad, n_found) where quad is (4,2) float32 or None.
    """
    corners, ids = detect_fn(gray)

    detected = {}
    if ids is not None:
        for idx, mid in enumerate(ids.flatten()):
            if int(mid) in [0, 1, 2, 3]:
                detected[int(mid)] = corners[idx][0].mean(axis=0)

    n = len(detected)

    if n == 0:
        # No markers at all — return cached (may be None)
        return last_quad, 0

    if n == 4 or (plane is not None):
        # Exact (N=4) or hybrid (N<4 with plane calibration)
        quad = _hybrid_quad(detected, plane) if plane is not None else \
               np.array([detected[i] for i in [0, 1, 2, 3]], dtype=np.float32)
        return quad, n

    # N=1–3 without plane calibration — fall back to cached
    return last_quad, n


def point_in_quad(px: float, py: float, quad) -> bool:
    """Return True if pixel (px, py) is inside the quad polygon (or no quad given)."""
    if quad is None:
        return True
    result = cv.pointPolygonTest(quad, (float(px), float(py)), measureDist=False)
    return result >= 0


def pixel_to_mat_mm(u: float, v: float, H: np.ndarray):
    """Map pixel (u, v) → mat real-world (x_mm, y_mm) using homography H."""
    pt = H @ np.array([u, v, 1.0], dtype=np.float64)
    return float(pt[0] / pt[2]), float(pt[1] / pt[2])


def draw_mat_overlay(output: np.ndarray, quad, n_markers: int) -> None:
    """
    Draw the detected mat boundary:
      • Semi-transparent green fill inside the quad.
      • Green border lines.
      • Corner dots + ID labels.
      • Marker count badge.
    """
    CORNER_LABELS = ["ID:0 TL", "ID:1 TR", "ID:2 BR", "ID:3 BL"]
    CORNER_COLORS = [
        (0, 230, 80),    # TL green
        (0, 210, 255),   # TR cyan
        (255, 120, 0),   # BR blue
        (255, 30, 200),  # BL magenta
    ]

    if quad is not None:
        pts = quad.astype(np.int32)
        # Semi-transparent fill
        overlay = output.copy()
        cv.fillConvexPoly(overlay, pts, (0, 200, 60))
        cv.addWeighted(overlay, 0.12, output, 0.88, 0, output)
        # Border
        for i in range(4):
            cv.line(output, tuple(pts[i]), tuple(pts[(i + 1) % 4]),
                    (0, 220, 60), 2, cv.LINE_AA)
        # Corner dots + labels
        for i, (pt, lbl, col) in enumerate(zip(pts, CORNER_LABELS, CORNER_COLORS)):
            cv.circle(output, tuple(pt), 7, col, -1)
            cv.circle(output, tuple(pt), 9, (255, 255, 255), 1)
            offset = (pt[0] + 10, pt[1] + 5)
            cv.putText(output, lbl, offset, cv.FONT_HERSHEY_SIMPLEX,
                       0.40, (0, 0, 0), 2, cv.LINE_AA)
            cv.putText(output, lbl, offset, cv.FONT_HERSHEY_SIMPLEX,
                       0.40, col,       1, cv.LINE_AA)

    # Marker count badge (bottom-right)
    h_out, w_out = output.shape[:2]
    if n_markers == 4:
        col  = (0, 220, 60)
        text = f"Mat markers: {n_markers}/4"
    elif n_markers >= 1:
        col  = (0, 200, 255)
        text = f"Mat markers: {n_markers}/4  (hybrid ROI)"
    else:
        col  = (0, 100, 255)
        text = f"Mat markers: 0/4"
    cv.putText(output, text,
               (w_out - 260, h_out - 42),
               cv.FONT_HERSHEY_SIMPLEX, 0.46, col, 1, cv.LINE_AA)


def draw_3d_axes(frame, rvec, tvec, K, dist, cx: int, cy: int) -> None:
    """
    Draw XYZ coordinate axes on the detected object face.

    X = red arrow, Y = green arrow, Z = blue arrow (toward camera).

    Args:
        frame     : Image to draw on (modified in-place).
        rvec, tvec: Pose from solvePnP.
        K, dist   : Camera intrinsics.
        cx, cy    : Pixel centre of the object.
    """
    L = config.AXIS_LENGTH_CM
    pts3d = np.float32([
        [0, 0,  0],   # origin
        [L, 0,  0],   # X
        [0, L,  0],   # Y
        [0, 0, -L],   # Z (toward camera)
    ])
    proj, _ = cv.projectPoints(pts3d, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)

    o  = (int(proj[0][0]), int(proj[0][1]))
    px = (int(proj[1][0]), int(proj[1][1]))
    py = (int(proj[2][0]), int(proj[2][1]))
    pz = (int(proj[3][0]), int(proj[3][1]))

    cv.arrowedLine(frame, o, px, (0,   0, 255), 2, tipLength=0.3)   # X red
    cv.arrowedLine(frame, o, py, (0, 255,   0), 2, tipLength=0.3)   # Y green
    cv.arrowedLine(frame, o, pz, (255, 0,   0), 2, tipLength=0.3)   # Z blue
    cv.putText(frame, "X", (px[0]+4, px[1]), cv.FONT_HERSHEY_SIMPLEX, 0.38,
               (0,   0, 255), 1, cv.LINE_AA)
    cv.putText(frame, "Y", (py[0]+4, py[1]), cv.FONT_HERSHEY_SIMPLEX, 0.38,
               (0, 255,   0), 1, cv.LINE_AA)
    cv.putText(frame, "Z", (pz[0]+4, pz[1]), cv.FONT_HERSHEY_SIMPLEX, 0.38,
               (255,   0,   0), 1, cv.LINE_AA)


# ── HUD DRAWING HELPERS ──────────────────────────────────────────────────────────

def _draw_corner_bracket(img, x1, y1, x2, y2, color, thickness=2, arm=18):
    """
    Draw corner-bracket style bounding box (4 L-shaped corners only).
    Much cleaner than a full rectangle at high object densities.
    """
    for px, py, sx, sy in [
        (x1, y1, +1, +1), (x2, y1, -1, +1),
        (x1, y2, +1, -1), (x2, y2, -1, -1),
    ]:
        cv.line(img, (px, py), (px + sx * arm, py),            color, thickness, cv.LINE_AA)
        cv.line(img, (px, py), (px,            py + sy * arm), color, thickness, cv.LINE_AA)


def _draw_pill_badge(img, cx, top_y, text, bg_color, text_color=(0, 0, 0),
                     font_scale=0.58, thickness=2):
    """
    Draw a rounded-rectangle (pill) badge centred above top_y.
    Returns the y-coordinate of the badge top edge.
    """
    font = cv.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv.getTextSize(text, font, font_scale, thickness)
    pad_x, pad_y = 10, 5
    bx1 = cx - tw // 2 - pad_x
    bx2 = cx + tw // 2 + pad_x
    by1 = top_y - th - pad_y * 2 - 6
    by2 = top_y - 6
    # filled rounded rect via two overlapping rectangles + circles
    cv.rectangle(img, (bx1 + 8, by1), (bx2 - 8, by2), bg_color, -1)
    cv.rectangle(img, (bx1, by1 + 8), (bx2, by2 - 8), bg_color, -1)
    for cx_, cy_ in [(bx1+8, by1+8), (bx2-8, by1+8), (bx1+8, by2-8), (bx2-8, by2-8)]:
        cv.circle(img, (cx_, cy_), 8, bg_color, -1)
    tx = cx - tw // 2
    ty = by2 - pad_y
    cv.putText(img, text, (tx, ty), font, font_scale, text_color, thickness, cv.LINE_AA)
    return by1


def _draw_id_chip(img, x, y, obj_id, color):
    """
    Draw a small numbered chip in the top-left corner of the bounding box.
    """
    text  = str(obj_id)
    font  = cv.FONT_HERSHEY_SIMPLEX
    (tw, th), _ = cv.getTextSize(text, font, 0.45, 2)
    px, py = x + 4, y + th + 4
    cv.rectangle(img, (x + 2, y + 2), (x + tw + 8, y + th + 8), color, -1)
    cv.putText(img, text, (px, py), font, 0.45, (0, 0, 0), 2, cv.LINE_AA)


def draw_object_hud(output, det, obj_id: int,
                    show_axes: bool, show_pose: bool,
                    K, dist_coeffs) -> None:
    """
    Render the full per-object HUD overlay onto *output* (in-place).

    Elements drawn:
      • Corner-bracket bounding box (coloured, 2 px)
      • Pill-shaped distance badge above the box (green/orange by proximity)
      • Numbered object ID chip in top-left corner
      • Object label below the box
      • Pixel size text inside the box (small, grey)
      • Pose P/Y/R text inside the box (below centre) — only when show_pose
      • 3D XYZ axes — only when show_axes

    Args:
        output      : Annotated BGR frame (modified in-place).
        det         : Detection dict from detect_objects() + distance/rvec/tvec.
        obj_id      : 1-based object index for the ID chip.
        show_axes   : Whether to draw 3D axes.
        show_pose   : Whether to overlay pitch/yaw/roll text.
        K           : Camera matrix.
        dist_coeffs : Distortion coefficients.
    """
    x, y   = det["pt1"]
    x2, y2 = det["pt2"]
    w      = det["pixel_w"]
    h      = det["pixel_h"]
    cx, cy = det["center"]
    label  = det["label"]
    color  = det["color_bgr"]
    dist   = det.get("distance_cm")
    rvec   = det.get("rvec")
    tvec   = det.get("tvec")

    # ── Corner-bracket bounding box ──────────────────────────────────────
    _draw_corner_bracket(output, x, y, x2, y2, color, thickness=2, arm=20)

    # ── Distance pill badge ────────────────────────────────────────────
    if dist is not None:
        dist_m   = dist / 100.0
        badge_bg = (0, 100, 255) if dist < config.CLOSE_DIST_CM else (30, 200, 30)
        badge_tx = (255, 255, 255)
        badge_str = f"{dist:.1f} cm  |  {dist_m:.2f} m"
    else:
        badge_bg  = (60, 60, 60)
        badge_tx  = (200, 200, 200)
        badge_str = "no distance"
    _draw_pill_badge(output, cx, y, badge_str, badge_bg, badge_tx)

    # ── Object ID chip (top-left corner) ─────────────────────────────
    _draw_id_chip(output, x, y, obj_id, color)

    # ── Object label below the box ──────────────────────────────────
    cv.putText(output, label, (x + 2, y2 + 18),
               cv.FONT_HERSHEY_SIMPLEX, 0.52, color, 2, cv.LINE_AA)

    # ── Pixel size (small grey, centre of box) ───────────────────────
    cv.putText(output, f"{w}×{h}px", (cx - 22, cy - 6),
               cv.FONT_HERSHEY_SIMPLEX, 0.34, (160, 160, 160), 1, cv.LINE_AA)

    # ── Pose text inside box (below centre) ────────────────────────
    if show_pose and rvec is not None:
        pitch, yaw, roll = rodrigues_to_euler(rvec)
        pose_str = f"P{pitch:+.0f}° Y{yaw:+.0f}° R{roll:+.0f}°"
        cv.putText(output, pose_str, (cx - 30, cy + 14),
                   cv.FONT_HERSHEY_SIMPLEX, 0.34, (170, 170, 255), 1, cv.LINE_AA)

    # ── 3D axes ───────────────────────────────────────────────────
    if show_axes and rvec is not None and dist is not None:
        draw_3d_axes(output, rvec, tvec, K, dist_coeffs, cx, cy)


# ── MAIN PROCESSING ───────────────────────────────────────────────────────────

def process_frame(frame: np.ndarray,
                  K: np.ndarray,
                  dist: np.ndarray,
                  show_axes: bool = True,
                  show_pose: bool = True,
                  detect_aruco=None,
                  plane: dict = None,
                  mat_quad=None):
    """
    Run ArUco mat detection, object detection (ROI-filtered), and distance
    estimation on one frame.

    ArUco ROI:
      If detect_aruco is provided, the 4 corner markers (IDs 0-3) are located
      every frame to define the mat polygon.  Only objects whose bounding-box
      centre falls inside that polygon are processed and displayed.
      Without detect_aruco the whole frame is used (legacy behaviour).

    Distance:
      Triangle Similarity formula (robust off-centre) for distance_cm.
      solvePnP ITERATIVE used only for 3D axes / orientation.

    Args:
        frame        : Input BGR frame.
        K            : 3x3 camera matrix.
        dist         : Distortion coefficients.
        show_axes    : Draw 3D XYZ axes on each object.
        show_pose    : Overlay pitch/yaw/roll text.
        detect_aruco : Callable detect(gray)→(corners,ids) or None.
        plane        : Dict from load_plane_calibration() or None.
        mat_quad     : Last known quad (4,2) float32 for temporal caching.

    Returns:
        output     (np.ndarray)  : Annotated frame.
        mask       (np.ndarray)  : Binary detection mask.
        detections (list[dict])  : ROI-filtered per-object results.
        quad_state (tuple)       : (quad_pts, n_markers) for caching.
    """
    frame_ud = cv.undistort(frame, K, dist)
    output   = frame_ud.copy()
    gray     = cv.cvtColor(frame_ud, cv.COLOR_BGR2GRAY)

    # ── ArUco mat boundary ────────────────────────────────────────────────
    quad      = None
    n_markers = 0
    if detect_aruco is not None:
        quad, n_markers = detect_mat_quad(gray, detect_aruco, mat_quad, plane=plane)
        draw_mat_overlay(output, quad, n_markers)

    # Use quad for ROI when at least 1 marker visible (hybrid fills in the rest).
    # Only skip ROI filtering when 0 markers are detected AND no cached quad.
    roi_quad = quad  # hybrid_quad or exact quad or cached (may be None if 0 detected)

    # ── Detect all objects, then filter to mat ROI ─────────────────────────
    all_dets   = detect_objects(frame_ud)
    mask       = preprocess(frame_ud)
    detections = [d for d in all_dets
                  if point_in_quad(d["center"][0], d["center"][1], roi_quad)]

    for i, det in enumerate(detections):
        x, y   = det["pt1"]
        x2, y2 = det["pt2"]
        w      = det["pixel_w"]
        h      = det["pixel_h"]
        cx, cy = det["center"]

        # ── Distance: Triangle Similarity ───────────────────────────────
        distance_cm = None
        rvec = tvec = None

        if det["w_cm"] is not None and det["h_cm"] is not None:
            distance_cm = triangle_similarity_distance(
                w, h, det["w_cm"], det["h_cm"], K, cx, cy
            )
            if not (0 < distance_cm < 500):
                distance_cm = None

            # solvePnP — pose only (axes + rotation)
            obj_pts   = build_object_points(det["w_cm"], det["h_cm"])
            img_pts   = get_image_corners(x, y, w, h)
            init_rvec = np.zeros((3, 1), dtype=np.float64)
            init_tvec = np.array([[0.0], [0.0],
                                  [distance_cm if distance_cm else 30.0]],
                                 dtype=np.float64)
            ok, rvec, tvec = cv.solvePnP(
                obj_pts, img_pts, K, dist,
                rvec=init_rvec, tvec=init_tvec,
                useExtrinsicGuess=(distance_cm is not None),
                flags=cv.SOLVEPNP_ITERATIVE
            )
            if not ok:
                rvec = tvec = None

        det["distance_cm"] = distance_cm
        det["rvec"]        = rvec
        det["tvec"]        = tvec

        # ── Real-world mat position (mm) via homography ──────────────────
        if plane is not None and quad is not None and n_markers >= 1:
            xmm, ymm = pixel_to_mat_mm(cx, cy, plane["H"])
            det["mat_x_mm"] = xmm
            det["mat_y_mm"] = ymm
        else:
            det["mat_x_mm"] = None
            det["mat_y_mm"] = None

        # ── Draw full HUD for this object ─────────────────────────────────
        draw_object_hud(output, det, i + 1, show_axes, show_pose, K, dist)

        # Mat mm badge below the object label
        if det["mat_x_mm"] is not None:
            mm_str = f"mat ({det['mat_x_mm']:.0f}, {det['mat_y_mm']:.0f}) mm"
            cv.putText(output, mm_str, (x + 2, y2 + 36),
                       cv.FONT_HERSHEY_SIMPLEX, 0.36, (0, 220, 60), 1, cv.LINE_AA)

    # ── Status bar ────────────────────────────────────────────────────────────
    n_total = len(all_dets)
    n_roi   = len(detections)
    n_known = sum(1 for d in detections if d.get("distance_cm") is not None)
    fh = output.shape[0]
    cv.rectangle(output, (0, fh - 32), (output.shape[1], fh), (20, 20, 20), -1)
    roi_str = "in ROI" if detect_aruco else "total"
    status  = (f"  Objects: {n_roi} {roi_str} / {n_total} detected  "
               f"({n_known} measured)   "
               f"[m] mask  [a] axes  [p] pose  [s] save  [q] quit")
    cv.putText(output, status, (6, fh - 10),
               cv.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1, cv.LINE_AA)

    return output, mask, detections, (quad, n_markers)


# ── RUN MODES ─────────────────────────────────────────────────────────────────

def run_camera(cam_index: int = config.CAMERA_INDEX):
    """
    Live webcam distance estimation with ArUco mat ROI.

    Loads plane calibration (aruco_plane_calibration.npz) if available.
    Runs ArUco marker detection every frame to define the mat boundary.
    Only objects inside the mat polygon are measured.

    Args:
        cam_index (int): Camera device index.
    """
    K, dist = load_calibration()

    # ── Optional plane calibration ──────────────────────────────────────
    plane        = load_plane_calibration()
    detect_aruco = make_aruco_detector()
    mat_quad     = None    # cache last-known good mat quad

    if plane is None:
        print("[INFO] No plane calibration found. Run aruco_plane_calibrator.py first.")
        print("       ArUco markers will still define the ROI boundary visually.")
    else:
        print(f"       Object detection restricted to mat ROI "
              f"({plane['mat_w_mm']:.0f}×{plane['mat_h_mm']:.0f} mm).")

    cap = cv.VideoCapture(cam_index)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open camera {cam_index}")
        return

    cap.set(cv.CAP_PROP_FRAME_WIDTH,  config.FRAME_W)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, config.FRAME_H)
    cap.set(cv.CAP_PROP_FPS,          config.FPS)

    print(f"\n[INFO] Distance estimator started. Camera {cam_index} @ "
          f"{int(cap.get(cv.CAP_PROP_FRAME_WIDTH))}x"
          f"{int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))}")
    print("       Keys: [q]=quit  [s]=save  [m]=mask  [a]=axes  [p]=pose  [c]=print")

    show_axes = True
    show_pose = True
    show_mask = False
    prev_time = time.time()

    WIN = "ARMOBOT Distance Estimator  (ArUco ROI)"
    cv.namedWindow(WIN, cv.WINDOW_NORMAL)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Cannot read frame.")
            break

        result, mask, dets, (mat_quad, _) = process_frame(
            frame, K, dist, show_axes, show_pose,
            detect_aruco=detect_aruco,
            plane=plane,
            mat_quad=mat_quad,
        )

        # FPS chip
        now      = time.time()
        fps      = 1.0 / max(now - prev_time, 1e-6)
        prev_time = now
        fps_text = f"FPS {fps:.0f}"
        (fw_, fh_), _ = cv.getTextSize(fps_text, cv.FONT_HERSHEY_SIMPLEX, 0.50, 1)
        fw2 = result.shape[1]
        cv.rectangle(result, (fw2 - fw_ - 16, 6), (fw2 - 4, fh_ + 14), (20, 20, 20), -1)
        cv.putText(result, fps_text, (fw2 - fw_ - 8, fh_ + 8),
                   cv.FONT_HERSHEY_SIMPLEX, 0.50, (0, 220, 255), 1, cv.LINE_AA)

        display = cv.cvtColor(mask, cv.COLOR_GRAY2BGR) if show_mask else result
        cv.imshow(WIN, display)

        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            ts = time.strftime("%Y%m%d_%H%M%S")
            fname = f"snapshot_{ts}.jpg"
            cv.imwrite(fname, result)
            print(f"[INFO] Saved {fname}")
        elif key == ord('m'):
            show_mask = not show_mask
        elif key == ord('a'):
            show_axes = not show_axes
        elif key == ord('p'):
            show_pose = not show_pose
        elif key == ord('c'):
            print(f"\n[DETECTIONS]  {len(dets)} object(s) in ROI:")
            for i, d in enumerate(dets):
                dist_str = (f"{d['distance_cm']:.2f} cm"
                            if d.get("distance_cm") is not None else "n/a")
                mm_str   = (f"  mat=({d['mat_x_mm']:.0f},{d['mat_y_mm']:.0f}) mm"
                            if d.get("mat_x_mm") is not None else "")
                print(f"  {i+1}. {d['label']:12s}  "
                      f"dist={dist_str:>10}{mm_str}  "
                      f"aspect={d['aspect']:.2f}  fill={d['fill']:.2f}")

    cap.release()
    cv.destroyAllWindows()


def run_image(path: str):
    """
    Run distance estimation on a saved image file.

    Args:
        path (str): Path to input image.
    """
    K, dist = load_calibration()

    img = cv.imread(path)
    if img is None:
        print(f"[ERROR] Cannot read image: {path}")
        return

    result, mask, dets, _ = process_frame(
        img, K, dist,
        detect_aruco=make_aruco_detector(),
        plane=load_plane_calibration(),
    )

    print(f"\n[INFO] Detected {len(dets)} object(s):")
    for i, d in enumerate(dets):
        dist_str = f"{d['distance_cm']:.2f} cm" if d.get("distance_cm") is not None else "n/a"
        print(f"  {i+1}. {d['label']:12s}  dist={dist_str}  "
              f"aspect={d['aspect']:.2f}  fill={d['fill']:.2f}")

    WIN = "Distance Estimation Result  [q]=quit [s]=save"
    cv.namedWindow(WIN, cv.WINDOW_NORMAL)
    cv.imshow(WIN, result)
    if cv.waitKey(0) == ord('s'):
        cv.imwrite("result.jpg", result)
        print("[INFO] Saved result.jpg")
    cv.destroyAllWindows()


# ── ENTRY POINT ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    MODE = "camera"   # "camera" | "image"

    if MODE == "camera":
        run_camera()
    elif MODE == "image":
        run_image("test.jpg")
