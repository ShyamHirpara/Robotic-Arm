"""
config.py  —  Central Configuration for lenovo300 Distance Estimation System
==============================================================================
Single source of truth for all parameters. Import this in every other script.
Edit values here; changes propagate to all scripts automatically.

Lenovo 300 FHD Webcam specs used in this design:
  - Sensor  : 2 MP CMOS, fixed focus
  - Max res : 1920x1080 @ 30fps  (we use 1280x720 for USB 2.0 reliability)
  - FOV     : 95 degrees horizontal (ultra-wide)
  - Expected fx/fy at 720p: ~580-700 px  (wide FOV = lower focal length)

Author: Shyam Hirpara  |  Date: 2026-06-09
"""

import numpy as np
import os

# ── CAMERA ────────────────────────────────────────────────────────────────────
CAMERA_INDEX  = 0             # USB webcam index (try 1 if 0 doesn't work)
FRAME_W       = 1280          # capture width  (pixels)
FRAME_H       = 720           # capture height (pixels)
FPS           = 30            # target frame rate

# ── CALIBRATION CHECKERBOARD (A4 paper) ───────────────────────────────────────
# 10×7 squares, each 27 mm → board is 270×189 mm → fits A4 (297×210 mm) landscape
CHESSBOARD_SIZE  = (9, 6)     # inner corners (cols, rows)  NOT square count
SQUARE_SIZE_MM   = 27.0       # real-world size of one square in millimetres
MIN_CALIB_FRAMES = 15         # minimum captures before calibration is allowed
CALIB_FILE       = os.path.join(os.path.dirname(__file__), "camera_calibration.npz")

# ── OBJECT CATALOGUE ──────────────────────────────────────────────────────────
# Each dict entry defines one object type.
# w_cm / h_cm = the FRONT FACE dimensions used in solvePnP (width × height).
# For cubes    : front face = L cm wide × H cm tall (as it stands upright).
# For cylinder : front face = diameter cm wide × height cm tall.
#
# aspect_lo / aspect_hi  : expected bounding-box w/h range at any distance.
# fill_lo   / fill_hi    : expected contour_area / bbox_area range.
#
OBJECTS = [
    {
        "label"      : "Small Cube",
        "w_cm"       : 2.5,          # front face width  (L = 2.5 cm)
        "h_cm"       : 5.0,          # front face height (H = 5.0 cm)
        "shape"      : "rect",
        "aspect_lo"  : 0.30,         # w/h lower bound
        "aspect_hi"  : 0.70,         # w/h upper bound
        "fill_lo"    : 0.72,         # rectangular → high fill
        "fill_hi"    : 1.00,
        "color_bgr"  : (255, 100,  0),   # blue label
    },
    {
        "label"      : "Large Cube",
        "w_cm"       : 4.5,          # front face width  (L = 4.5 cm)
        "h_cm"       : 4.5,          # front face height (H = 4.5 cm)
        "shape"      : "rect",
        "aspect_lo"  : 0.50,         # wider range to handle angled/tilted views
        "aspect_hi"  : 2.00,
        "fill_lo"    : 0.72,
        "fill_hi"    : 1.00,
        "color_bgr"  : (0, 0, 255),      # red label
    },
    {
        "label"      : "Cylinder",
        "w_cm"       : 2.0,          # diameter
        "h_cm"       : 4.0,          # height
        "shape"      : "cylinder",
        "aspect_lo"  : 0.30,         # similar aspect to Small Cube
        "aspect_hi"  : 0.70,
        "fill_lo"    : 0.55,         # rounded top/bottom → lower fill than cube
        "fill_hi"    : 0.78,
        "color_bgr"  : (0, 200,  0),     # green label
    },
]

# ── LAB BRIGHT-OBJECT DETECTION ─────────────────────────────────────────────
# Detection now uses the L (luminance) channel of LAB colour space with
# Otsu's automatic threshold, so no manual HSV tuning is needed.
#
# LAB_MIN_L  : Minimum absolute L value (0-255) a pixel must have to be
#              considered bright. This acts as a floor guard alongside Otsu,
#              so medium-bright backgrounds (grey mats, light wood) are
#              rejected even when Otsu splits below them.
#              Raise if too many background blobs appear.
#              Lower if the white objects are being missed in dim lighting.
LAB_MIN_L = 160

# Legacy HSV params (kept for reference — no longer used):
# HSV_LOWER_WHITE = np.array([40,   0, 150], dtype=np.uint8)
# HSV_UPPER_WHITE = np.array([180, 85, 255], dtype=np.uint8)

# ── DETECTION GEOMETRY FILTERS ────────────────────────────────────────────────
# Blob size limits in pixels (scaled for 720p)
MIN_AREA      = 1800      # minimum blob area  — raised to reject ruler/thin objects
MAX_AREA      = 60_000    # maximum blob area  — reject merged blobs / frame edge
MAX_ASPECT    = 5.0       # reject blobs wider than 5× their height (e.g. ruler)

# Contour polygon sides (after approxPolyDP)
MIN_SIDES     = 3
MAX_SIDES     = 10        # relaxed upper limit to catch cylinders (more vertices)

# Minimum fill ratio across ALL objects (pre-classification filter)
GLOBAL_FILL_MIN = 0.50

# Ignore detections whose centre is within this many pixels of the frame edge
ROI_MARGIN_X  = 40
ROI_MARGIN_Y  = 25

# ── MORPHOLOGICAL PREPROCESSING ───────────────────────────────────────────────
BLUR_KERNEL   = 5         # Gaussian blur (must be odd)
MORPH_KERNEL  = 7         # Open/Close kernel
SEVER_KERNEL  = 5         # Cable-severing erode/dilate kernel
SEVER_ERODE   = 5         # erode iterations
SEVER_DILATE  = 7         # dilate iterations

# ── DISPLAY ───────────────────────────────────────────────────────────────────
FONT           = 1        # cv2.FONT_HERSHEY_SIMPLEX  (avoid importing cv2 here)
CLOSE_DIST_CM  = 20.0     # proximity warning threshold (cm)
FONT_SCALE_LG  = 0.65
FONT_SCALE_SM  = 0.40
AXIS_LENGTH_CM = 1.5      # length of 3D axes drawn on each object

# Sanity bounds for focal length after calibration
FX_MIN_SANE = 200
FX_MAX_SANE = 2000
