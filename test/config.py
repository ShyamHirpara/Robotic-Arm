"""
config.py  --  test/ folder configuration
Combines arm control constants (original test/config.py)
with object detector constants (from lenovo300/config.py).
"""

import os as _os
import numpy as _np

PICO_IP   = "192.168.4.1"
PICO_PORT = 81

STEP_INTERVAL = 0.15  # seconds between TCP commands

# ── Arm control constants (original test/vision.py) ──────────────────────────
stepper1 = 6    # kept as lower-case for direct vision.py compatibility
STEPPER1 = 6    # mm per degree  (base joint 1)   -- ACTIVE
# STEPPER2 = 9  # shoulder joint 2 -- COMMENTED OUT
# STEPPER3 = 6  # elbow    joint 3 -- COMMENTED OUT

DEAD_MM        = 15
MIN_STEP       = 2
MAX_STEP_SMALL = 4
MAX_STEP_MED   = 10
MAX_STEP_LARGE = 18

HOME_SHOULDER = -20.0
HOME_ELBOW    = -65.0

# ── Camera settings (lenovo300 webcam) ───────────────────────────────────────
CAMERA_INDEX = 1      # change to 0 if camera not found on index 1
FRAME_W      = 1280
FRAME_H      = 720
FPS          = 30

_DIR = _os.path.dirname(_os.path.abspath(__file__))   # test/ folder
CALIB_FILE = _os.path.join(_DIR, "camera_calibration.npz")
PLANE_FILE = _os.path.join(_DIR, "aruco_plane_calibration.npz")

# ── Object catalogue (from lenovo300/config.py) ───────────────────────────────
OBJECTS = [
    {
        "label"      : "Small Cube",
        "w_cm"       : 2.5,
        "h_cm"       : 5.0,
        "shape"      : "rect",
        "aspect_lo"  : 0.30,
        "aspect_hi"  : 0.70,
        "fill_lo"    : 0.72,
        "fill_hi"    : 1.00,
        "color_bgr"  : (255, 100,  0),
    },
    {
        "label"      : "Large Cube",
        "w_cm"       : 4.5,
        "h_cm"       : 4.5,
        "shape"      : "rect",
        "aspect_lo"  : 0.50,
        "aspect_hi"  : 2.00,
        "fill_lo"    : 0.72,
        "fill_hi"    : 1.00,
        "color_bgr"  : (0, 0, 255),
    },
    {
        "label"      : "Cylinder",
        "w_cm"       : 2.0,
        "h_cm"       : 4.0,
        "shape"      : "cylinder",
        "aspect_lo"  : 0.30,
        "aspect_hi"  : 0.70,
        "fill_lo"    : 0.55,
        "fill_hi"    : 0.78,
        "color_bgr"  : (0, 200,  0),
    },
]

# ── LAB detection thresholds (from lenovo300/config.py) ──────────────────────
LAB_MIN_L     = 160

# ── Detection geometry filters ────────────────────────────────────────────────
MIN_AREA      = 1800
MAX_AREA      = 60_000
MAX_ASPECT    = 5.0
MIN_SIDES     = 3
MAX_SIDES     = 10
GLOBAL_FILL_MIN = 0.50
ROI_MARGIN_X  = 40
ROI_MARGIN_Y  = 25

# ── Morphological preprocessing ───────────────────────────────────────────────
BLUR_KERNEL   = 5
MORPH_KERNEL  = 7
SEVER_KERNEL  = 5
SEVER_ERODE   = 5
SEVER_DILATE  = 7
