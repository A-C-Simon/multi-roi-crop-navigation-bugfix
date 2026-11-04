"""Synthetic corridor images and pipeline helpers for regression tests."""
import math

import cv2
import numpy as np

from lookahead_corridor_map import LookaheadCorridorMap, LookaheadParams
from mr_vs import MultiROIVS, MRVSParams
from run_mr_navigation import process_image
from temporal_filter import TemporalNavigationFilter, TemporalFilterParams
from test_multi_roi import MultiROIDetector

H, W = 480, 640
SOIL = (60, 65, 120)
CROP = (30, 200, 30)


def corridor(center=320.0, half=70.0, lean_deg=0.0, h=H, w=W, gaps=()):
    """Two green rows around `center` (x at the image bottom).

    lean_deg > 0 leans the corridor right toward the top of the image
    (mr_vs convention: Theta > 0). gaps: (row_index, y0, y1) erased spans.
    """
    img = np.full((h, w, 3), SOIL, np.uint8)
    d = h * math.tan(math.radians(lean_deg))
    for idx, xb in enumerate((center - half, center + half)):
        xt = xb + d
        poly = np.array([[xb - 5, h - 1], [xb + 5, h - 1],
                         [xt + 5, 0], [xt - 5, 0]], np.int32)
        cv2.fillPoly(img, [poly], CROP)
        for r, y0, y1 in gaps:
            if r == idx:
                x0 = int(min(xb, xt)) - 8
                x1 = int(max(xb, xt)) + 8
                img[int(y0):int(y1), x0:x1] = SOIL
    return img


def blank(h=H, w=W):
    return np.full((h, w, 3), SOIL, np.uint8)


class Stack:
    """detector + optional map + temporal filter + controller, fixed dt."""

    def __init__(self, use_map=False, n_strips=10, ignore_initial=0,
                 nav_curve=True, dt=0.05, temporal=True, **vs_kw):
        self.det = MultiROIDetector(morph="struct", n_strips=n_strips,
                                    ignore_initial=ignore_initial,
                                    nav_curve=nav_curve)
        self.vs = MultiROIVS(MRVSParams(**vs_kw))
        self.tf = TemporalNavigationFilter(TemporalFilterParams(
            n_strips=n_strips, ignore_initial=ignore_initial)) if temporal else None
        self.map = LookaheadCorridorMap(LookaheadParams(
            n_bins=n_strips, ignore_initial=ignore_initial)) if use_map else None
        self.dt = dt
        self.last_w = 0.0

    def step(self, img, draw=False):
        out = process_image(img, self.det, self.vs, draw=draw, t_filter=self.tf,
                            dt=self.dt, last_w=self.last_w,
                            lookahead_map=self.map)
        self.last_w = float(out["w"])
        return out
