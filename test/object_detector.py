"""
object_detector.py  —  Multi-Object Detection and Classification
=================================================================
Detects bright/white objects (cubes and cylinder) in a BGR frame using
LAB-L channel + Otsu adaptive thresholding + morphological cleanup,
then classifies each valid contour into one of the three object
types defined in config.OBJECTS.

Classification uses two scale-invariant shape descriptors:
  - aspect_ratio = bounding_box_width / bounding_box_height
  - fill_ratio   = contour_area       / bounding_box_area

Each object type in config.py defines the expected ranges for
these descriptors. The classifier picks the first match.
If no object matches, the contour is labelled "Unknown".

Public API:
    preprocess(frame)                   -> mask
    classify_object(cnt, x, y, w, h)   -> dict or None
    detect_objects(frame)               -> list[dict]

Author: Shyam Hirpara  |  Date: 2026-06-09
"""

import cv2 as cv
import numpy as np
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config


# ── PREPROCESSING ─────────────────────────────────────────────────────────────

def preprocess(frame: np.ndarray) -> np.ndarray:
    """
    Convert a BGR frame into a binary mask isolating bright/white object regions.

    Pipeline:
      1. Gaussian blur              — reduce pixel noise
      2. BGR → LAB                  — perceptually uniform colour space
      3. Extract L (luminance) ch.  — brightness, independent of hue
      4. Otsu auto-threshold        — finds optimal bright/dark split per frame
      5. Min-L absolute guard       — AND with a fixed floor to reject mid-tones
      6. Morphological OPEN         — remove small noise blobs
      7. Morphological CLOSE        — fill small holes inside objects
      8. Erode + dilate pass        — sever thin cable/wire connections

    Why LAB + Otsu instead of HSV inRange?
      HSV white detection needs manually tuned bounds that drift with ambient
      light.  The L channel in LAB isolates perceived brightness independently
      of colour, and Otsu's method automatically picks the optimal threshold
      for each frame — so the detector self-calibrates to the current lighting
      with zero manual intervention.

      A minimum absolute-L guard (config.LAB_MIN_L) is AND-ed with the Otsu
      mask so that medium-bright backgrounds (e.g. a grey mat) never pass even
      when Otsu places its split below them.

    Args:
        frame (np.ndarray): Input BGR image.

    Returns:
        np.ndarray: Binary (0/255) mask, same H×W as frame.
    """
    blurred = cv.GaussianBlur(
        frame,
        (config.BLUR_KERNEL, config.BLUR_KERNEL), 0
    )

    # ── LAB L-channel extraction ──────────────────────────────────────────
    lab = cv.cvtColor(blurred, cv.COLOR_BGR2LAB)
    L, _, _ = cv.split(lab)

    # ── Otsu adaptive threshold (auto-adapts to scene brightness) ─────────
    _, mask_otsu = cv.threshold(L, 0, 255, cv.THRESH_BINARY + cv.THRESH_OTSU)

    # ── Minimum absolute luminance guard (rejects medium-bright bg areas) ─
    _, mask_abs  = cv.threshold(L, config.LAB_MIN_L, 255, cv.THRESH_BINARY)

    # Both conditions must be true: auto-split AND truly bright
    mask = cv.bitwise_and(mask_otsu, mask_abs)

    # ── Morphological cleanup ─────────────────────────────────────────────
    k_main  = np.ones((config.MORPH_KERNEL,  config.MORPH_KERNEL),  np.uint8)
    k_sever = np.ones((config.SEVER_KERNEL, config.SEVER_KERNEL), np.uint8)

    mask = cv.morphologyEx(mask, cv.MORPH_OPEN,  k_main)
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, k_main)
    mask = cv.erode (mask, k_sever, iterations=config.SEVER_ERODE)
    mask = cv.dilate(mask, k_sever, iterations=config.SEVER_DILATE)

    return mask


# ── CLASSIFICATION ────────────────────────────────────────────────────────────

def classify_object(cnt, x: int, y: int, w: int, h: int,
                    frame_shape: tuple) -> dict | None:
    """
    Classify a contour into one of the objects defined in config.OBJECTS,
    or return None if the contour fails basic geometry filters.

    Filters applied (in order):
      1. Area within [MIN_AREA, MAX_AREA]
      2. Polygon side count within [MIN_SIDES, MAX_SIDES]
      3. Global fill ratio >= GLOBAL_FILL_MIN
      4. Global aspect ratio <= MAX_ASPECT  (rejects rulers, cables, etc.)
      5. Blob centre inside ROI (not too close to frame edges)

    Classification (first matching object wins):
      - aspect_ratio = w / h  must be within object's [aspect_lo, aspect_hi]
      - fill_ratio   must be within object's [fill_lo,   fill_hi]

    Args:
        cnt         : OpenCV contour array.
        x, y, w, h  : Bounding rectangle from cv.boundingRect().
        frame_shape : Shape of the frame (height, width, ...).

    Returns:
        dict: Matching entry from config.OBJECTS (with added 'area' key), or
        None: if the contour should be rejected.
    """
    area     = cv.contourArea(cnt)
    bbox_area = w * h
    if bbox_area == 0:
        return None

    # ── Basic size filter ─────────────────────────────────────────────────
    if not (config.MIN_AREA < area < config.MAX_AREA):
        return None

    # ── Polygon sides filter ──────────────────────────────────────────────
    epsilon = 0.05 * cv.arcLength(cnt, True)
    approx  = cv.approxPolyDP(cnt, epsilon, True)
    sides   = len(approx)
    if not (config.MIN_SIDES <= sides <= config.MAX_SIDES):
        return None

    # ── Global fill ratio filter ──────────────────────────────────────────
    fill_ratio = area / float(bbox_area)
    if fill_ratio < config.GLOBAL_FILL_MIN:
        return None

    # ── ROI filter (centre must be away from frame edges) ─────────────────
    fh, fw = frame_shape[:2]
    cx, cy = x + w // 2, y + h // 2
    in_roi = (
        cx > config.ROI_MARGIN_X        and
        cy > config.ROI_MARGIN_Y        and
        cx < (fw - config.ROI_MARGIN_X) and
        cy < (fh - config.ROI_MARGIN_Y)
    )
    if not in_roi:
        return None

    # ── Global aspect ratio filter (rejects ruler, cables, elongated noise) ──
    raw_aspect = w / float(h) if h > 0 else 999
    if raw_aspect > config.MAX_ASPECT or raw_aspect < (1.0 / config.MAX_ASPECT):
        return None

    # ── Object classification ─────────────────────────────────────────────
    aspect = raw_aspect

    for obj in config.OBJECTS:
        aspect_ok = obj["aspect_lo"] <= aspect <= obj["aspect_hi"]
        fill_ok   = obj["fill_lo"]   <= fill_ratio <= obj["fill_hi"]
        if aspect_ok and fill_ok:
            return {**obj, "area": area, "fill": fill_ratio, "aspect": aspect}

    # Contour passes geometry tests but matches no object → Unknown
    return {
        "label"    : "Unknown",
        "w_cm"     : None,
        "h_cm"     : None,
        "shape"    : "unknown",
        "color_bgr": (128, 128, 128),
        "area"     : area,
        "fill"     : fill_ratio,
        "aspect"   : aspect,
    }


# ── DETECTION ─────────────────────────────────────────────────────────────────

def detect_objects(frame: np.ndarray) -> list[dict]:
    """
    Run the full detection pipeline on one frame.

    Returns a list of detection dicts, one per valid contour. Each dict has:
        'label'     : str   — object name from config.OBJECTS (or "Unknown")
        'pt1'       : (x, y)         — top-left bounding box corner
        'pt2'       : (x+w, y+h)     — bottom-right bounding box corner
        'center'    : (cx, cy)        — bounding box centre
        'pixel_w'   : int             — bounding box width in pixels
        'pixel_h'   : int             — bounding box height in pixels
        'w_cm'      : float | None    — real front-face width (cm)
        'h_cm'      : float | None    — real front-face height (cm)
        'shape'     : str             — 'rect' | 'cylinder' | 'unknown'
        'color_bgr' : (B, G, R)       — display colour for this object
        'area'      : float           — contour area in pixels
        'fill'      : float           — fill ratio
        'aspect'    : float           — w/h ratio
        'contour'   : np.ndarray      — raw contour (for solvePnP corners)
        'mask'      : np.ndarray      — binary mask (useful for debug)

    Args:
        frame (np.ndarray): Input BGR image.

    Returns:
        list[dict]: One entry per validated detection.
    """
    mask     = preprocess(frame)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    results  = []

    for cnt in contours:
        x, y, w, h = cv.boundingRect(cnt)
        obj = classify_object(cnt, x, y, w, h, frame.shape)
        if obj is None:
            continue

        results.append({
            **obj,
            "pt1"    : (x, y),
            "pt2"    : (x + w, y + h),
            "center" : (x + w // 2, y + h // 2),
            "pixel_w": w,
            "pixel_h": h,
            "contour": cnt,
            "mask"   : mask,
        })

    return results
