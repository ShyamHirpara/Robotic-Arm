"""
generate_aruco_checkerboard.py  —  Checkerboard with ArUco Corner Markers
==========================================================================
Generates a calibration / workspace-boundary sheet:

  • A standard black-and-white checkerboard in the centre (for camera calibration).
  • One unique ArUco marker (DICT_4X4_50) at each of the 4 corners (IDs 0-3).

The ArUco corner markers let the camera:
  1. Precisely locate the 4 corners of the black working plane.
  2. Compute a homography to map image pixels → real-world plane coordinates.
  3. Define the arm's workspace boundary automatically.

Layout (landscape A4):
  ┌────────────────────────────────────────────────────────────┐
  │ [ArUco 0]   ·  ·  checkerboard  ·  ·   [ArUco 1]         │
  │              ██░░██░░██░░██░░██░░                          │
  │              ░░██░░██░░██░░██░░██                          │
  │              (9×6 inner corners @ 27mm)                    │
  │ [ArUco 3]                          [ArUco 2]              │
  └────────────────────────────────────────────────────────────┘

Corner marker IDs:
  0 = Top-Left      1 = Top-Right
  3 = Bottom-Left   2 = Bottom-Right

Usage:
    python generate_aruco_checkerboard.py
    → saves aruco_checkerboard_A4.png
    → print LANDSCAPE on A4, 100% scale (no scaling / fit-to-page).
    → measure one checkerboard square with a ruler (~27 mm).
    → laminate or tape flat to your black working mat.

Author: Shyam Hirpara  |  Date: 2026-06-10
"""

import cv2 as cv
import numpy as np
import os
import sys

import config

# ── SETTINGS ──────────────────────────────────────────────────────────────────
# Checkerboard (same as existing calibration board)
CB_COLS      = config.CHESSBOARD_SIZE[0] + 1   # 10 squares per row
CB_ROWS      = config.CHESSBOARD_SIZE[1] + 1   # 7  squares per col
SQUARE_PX    = 100                              # pixels per square (100 px ≈ 27 mm when printed at ~94 DPI)

# ArUco markers
ARUCO_DICT   = cv.aruco.DICT_4X4_50            # small, robust dictionary
ARUCO_IDS    = [0, 1, 2, 3]                    # TL, TR, BR, BL
# Marker size in pixels — roughly 2.5× a checkerboard square
MARKER_PX    = int(SQUARE_PX * 2.5)

# Overall page border (white margin around everything)
PAGE_BORDER  = 60                              # px — keep at least this much white

# Padding between ArUco marker and the checkerboard
INNER_PAD    = 30                              # px

# Output
OUT_FILE = os.path.join(os.path.dirname(__file__), "aruco_checkerboard_A4.png")
# ──────────────────────────────────────────────────────────────────────────────


def get_aruco_dictionary():
    """Return the ArUco dictionary, compatible with OpenCV 4.x and 4.8+."""
    try:
        # OpenCV 4.7+ — predefined dict object
        return cv.aruco.getPredefinedDictionary(ARUCO_DICT)
    except AttributeError:
        # Older OpenCV 4.x
        return cv.aruco.Dictionary_get(ARUCO_DICT)


def generate_marker(aruco_dict, marker_id: int, size_px: int) -> np.ndarray:
    """Generate a single ArUco marker as a grayscale numpy array."""
    try:
        # OpenCV 4.7+
        marker = np.zeros((size_px, size_px), dtype=np.uint8)
        cv.aruco.generateImageMarker(aruco_dict, marker_id, size_px, marker, 1)
    except AttributeError:
        # Older API
        marker = cv.aruco.drawMarker(aruco_dict, marker_id, size_px)
    return marker


def generate_checkerboard(cols: int, rows: int, square_px: int) -> np.ndarray:
    """Return a grayscale checkerboard (no border) — pure pattern only."""
    w = cols * square_px
    h = rows * square_px
    board = np.full((h, w), 255, dtype=np.uint8)
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                y0, x0 = r * square_px, c * square_px
                board[y0:y0 + square_px, x0:x0 + square_px] = 0
    return board


def add_label(canvas, text, x, y, font_scale=0.45, thickness=1):
    """Add a small black label below a marker."""
    cv.putText(canvas, text, (x, y),
               cv.FONT_HERSHEY_SIMPLEX, font_scale, 0, thickness, cv.LINE_AA)


def build_sheet() -> np.ndarray:
    aruco_dict = get_aruco_dictionary()

    # Generate the 4 ArUco markers
    m = {mid: generate_marker(aruco_dict, mid, MARKER_PX) for mid in ARUCO_IDS}

    # Generate the checkerboard block
    cb = generate_checkerboard(CB_COLS, CB_ROWS, SQUARE_PX)
    cb_h, cb_w = cb.shape

    # ── Compute total canvas size ──────────────────────────────────────────────
    # Width: left_border + marker + inner_pad + checkerboard + inner_pad + marker + right_border
    total_w = PAGE_BORDER + MARKER_PX + INNER_PAD + cb_w + INNER_PAD + MARKER_PX + PAGE_BORDER
    # Height: top_border + marker + inner_pad + checkerboard + inner_pad + marker + bottom_border
    total_h = PAGE_BORDER + MARKER_PX + INNER_PAD + cb_h + INNER_PAD + MARKER_PX + PAGE_BORDER

    canvas = np.full((total_h, total_w), 255, dtype=np.uint8)

    # ── Anchor positions ───────────────────────────────────────────────────────
    # Checkerboard top-left origin
    cb_x = PAGE_BORDER + MARKER_PX + INNER_PAD
    cb_y = PAGE_BORDER + MARKER_PX + INNER_PAD

    # Marker origins (top-left corner of each marker square)
    # ID 0 — Top-Left
    m0_x, m0_y = PAGE_BORDER, PAGE_BORDER
    # ID 1 — Top-Right
    m1_x = cb_x + cb_w + INNER_PAD
    m1_y = PAGE_BORDER
    # ID 2 — Bottom-Right
    m2_x = m1_x
    m2_y = cb_y + cb_h + INNER_PAD
    # ID 3 — Bottom-Left
    m3_x, m3_y = PAGE_BORDER, m2_y

    # ── Paste elements ─────────────────────────────────────────────────────────
    # Checkerboard
    canvas[cb_y:cb_y + cb_h, cb_x:cb_x + cb_w] = cb

    # ArUco markers
    for (mid, mx, my) in [(0, m0_x, m0_y),
                          (1, m1_x, m1_y),
                          (2, m2_x, m2_y),
                          (3, m3_x, m3_y)]:
        canvas[my:my + MARKER_PX, mx:mx + MARKER_PX] = m[mid]

    # ── Draw thin guide lines connecting marker centres to checkerboard ────────
    # (light gray, for visual reference only — won't affect detection)
    line_color = 200  # gray
    # TL marker bottom-right → CB top-left
    cv.line(canvas,
            (m0_x + MARKER_PX, m0_y + MARKER_PX),
            (cb_x, cb_y),
            line_color, 1, cv.LINE_AA)
    # TR marker bottom-left → CB top-right
    cv.line(canvas,
            (m1_x, m1_y + MARKER_PX),
            (cb_x + cb_w, cb_y),
            line_color, 1, cv.LINE_AA)
    # BR marker top-left → CB bottom-right
    cv.line(canvas,
            (m2_x, m2_y),
            (cb_x + cb_w, cb_y + cb_h),
            line_color, 1, cv.LINE_AA)
    # BL marker top-right → CB bottom-left
    cv.line(canvas,
            (m3_x + MARKER_PX, m3_y),
            (cb_x, cb_y + cb_h),
            line_color, 1, cv.LINE_AA)

    # ── Draw a dashed rectangle outline around the entire working area ─────────
    # corners of the working area (outer edges of all 4 markers)
    wa_x1, wa_y1 = PAGE_BORDER - 8, PAGE_BORDER - 8
    wa_x2 = m2_x + MARKER_PX + 8
    wa_y2 = m2_y + MARKER_PX + 8
    # Dashed rectangle
    dash = 14
    for x in range(wa_x1, wa_x2, dash * 2):
        cv.line(canvas, (x, wa_y1), (min(x + dash, wa_x2), wa_y1), 180, 1)
        cv.line(canvas, (x, wa_y2), (min(x + dash, wa_x2), wa_y2), 180, 1)
    for y in range(wa_y1, wa_y2, dash * 2):
        cv.line(canvas, (wa_x1, y), (wa_x1, min(y + dash, wa_y2)), 180, 1)
        cv.line(canvas, (wa_x2, y), (wa_x2, min(y + dash, wa_y2)), 180, 1)

    # ── ID labels (tiny text below / beside each marker) ──────────────────────
    label_offset = 14
    add_label(canvas, "ID:0  (TL)", m0_x, m0_y + MARKER_PX + label_offset)
    add_label(canvas, "ID:1  (TR)", m1_x, m1_y + MARKER_PX + label_offset)
    add_label(canvas, "ID:2  (BR)", m2_x, m2_y + MARKER_PX + label_offset)
    add_label(canvas, "ID:3  (BL)", m3_x, m3_y + MARKER_PX + label_offset)

    # ── Title / print instruction at bottom ────────────────────────────────────
    title = "ARMOBOT Workspace | ArUco DICT_4X4_50 | Print LANDSCAPE A4 @ 100% | ~27mm/square"
    cv.putText(canvas, title,
               (PAGE_BORDER, total_h - 18),
               cv.FONT_HERSHEY_SIMPLEX, 0.40, 100, 1, cv.LINE_AA)

    return canvas


if __name__ == "__main__":
    print("=" * 62)
    print("  ARMOBOT ArUco Checkerboard Generator")
    print("=" * 62)

    sheet = build_sheet()
    cv.imwrite(OUT_FILE, sheet)

    h, w = sheet.shape
    print(f"  File       : {OUT_FILE}")
    print(f"  Image size : {w} × {h} px")
    print(f"  Checkerboard: {CB_COLS} × {CB_ROWS} squares  ({CB_COLS-1}×{CB_ROWS-1} inner corners)")
    print(f"  Square size : {config.SQUARE_SIZE_MM} mm (verify with ruler after printing)")
    print(f"  ArUco IDs  : 0=Top-Left  1=Top-Right  2=Bottom-Right  3=Bottom-Left")
    print(f"  Dictionary : DICT_4X4_50")
    print()
    print("  PRINT INSTRUCTIONS:")
    print("  1. Open aruco_checkerboard_A4.png")
    print("  2. Print in LANDSCAPE on A4  —  select 'Actual size' (NOT 'Fit to page')")
    print("  3. Measure one checkerboard square with a ruler.")
    print(f"     It should be ~{config.SQUARE_SIZE_MM} mm. If not, update SQUARE_SIZE_MM in config.py")
    print("  4. Tape / laminate flat onto your black working mat.")
    print("     Place markers at the physical corners of the mat.")
    print()
    print("  DETECTION NOTES:")
    print("  - Use cv2.aruco.detectMarkers() with DICT_4X4_50 to find corners.")
    print("  - IDs: 0=TL  1=TR  2=BR  3=BL  (matches standard homography order)")
    print("=" * 62)

    # Quick preview
    scale = min(1.0, 1400 / max(w, h))
    preview = cv.resize(sheet, (int(w * scale), int(h * scale)),
                        interpolation=cv.INTER_AREA)
    cv.imshow("ArUco Checkerboard A4 — press any key to close", preview)
    cv.waitKey(0)
    cv.destroyAllWindows()
