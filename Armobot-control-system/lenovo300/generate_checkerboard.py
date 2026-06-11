"""
generate_checkerboard.py  —  A4 Calibration Checkerboard Generator
====================================================================
Generates a pixel-perfect black-and-white checkerboard for camera calibration.

A4 paper in landscape = 297 mm × 210 mm usable.
Board: 10 cols × 7 rows of squares at 27 mm each = 270 mm × 189 mm → fits A4.
Inner corners (what OpenCV detects): 9 × 6

Usage:
    python generate_checkerboard.py
    → saves  checkerboard_9x6_A4.png
    → print in landscape on A4, measure one square with a ruler,
      confirm it is ~27 mm, then set SQUARE_SIZE_MM in config.py.

Author: Shyam Hirpara  |  Date: 2026-06-09
"""

import cv2 as cv
import numpy as np
import os

import config

# ── SETTINGS ──────────────────────────────────────────────────────────────────
# Derived from config; override here if you need a different size.
COLS       = config.CHESSBOARD_SIZE[0] + 1   # squares per row  (= inner corners + 1)
ROWS       = config.CHESSBOARD_SIZE[1] + 1   # squares per col
SQUARE_PX  = 100                              # pixel size of one square
BORDER_PX  = 40                              # white border around the board
OUT_FILE   = os.path.join(os.path.dirname(__file__), "checkerboard_9x6_A4.png")
# ──────────────────────────────────────────────────────────────────────────────


def generate_checkerboard(cols: int, rows: int,
                          square_px: int, border_px: int) -> np.ndarray:
    """
    Generate a pixel-perfect checkerboard image with a white border.

    Args:
        cols      : Number of squares per row.
        rows      : Number of squares per column.
        square_px : Pixel size of each square.
        border_px : White border width around the entire pattern.

    Returns:
        np.ndarray: Grayscale uint8 image.
    """
    total_w = cols * square_px + 2 * border_px
    total_h = rows * square_px + 2 * border_px

    board = np.full((total_h, total_w), 255, dtype=np.uint8)  # all white

    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                y0 = border_px + r * square_px
                x0 = border_px + c * square_px
                board[y0 : y0 + square_px, x0 : x0 + square_px] = 0   # black

    return board


if __name__ == "__main__":
    board = generate_checkerboard(COLS, ROWS, SQUARE_PX, BORDER_PX)
    cv.imwrite(OUT_FILE, board)

    inner_c = COLS - 1
    inner_r = ROWS - 1
    w_mm    = COLS * config.SQUARE_SIZE_MM
    h_mm    = ROWS * config.SQUARE_SIZE_MM

    print("=" * 58)
    print("  Checkerboard Generated")
    print("=" * 58)
    print(f"  File         : {OUT_FILE}")
    print(f"  Squares      : {COLS} cols x {ROWS} rows")
    print(f"  Inner corners: {inner_c} x {inner_r}  <-- use in camera_calibration.py")
    print(f"  Board size   : {w_mm:.0f} x {h_mm:.0f} mm  (fits A4 landscape)")
    print(f"  Square size  : {config.SQUARE_SIZE_MM} mm  (verify with ruler after printing!)")
    print()
    print("  PRINT INSTRUCTIONS:")
    print("  1. Open the PNG and print in LANDSCAPE on A4 paper.")
    print("  2. Make sure 'Fit to page' or 'Actual size' is selected.")
    print("  3. Measure one square with a ruler after printing.")
    print(f"  4. If it is NOT {config.SQUARE_SIZE_MM} mm, update SQUARE_SIZE_MM in config.py.")
    print("=" * 58)

    # Quick preview
    h, w = board.shape
    preview = cv.resize(board, (w // 2, h // 2))
    cv.imshow("Checkerboard A4  — press any key to close", preview)
    cv.waitKey(0)
    cv.destroyAllWindows()
