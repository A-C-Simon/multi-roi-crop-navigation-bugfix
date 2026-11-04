"""
LookaheadCorridorMap — visible-future corridor memory
=====================================================
Spatiotemporal memory that stores the corridor observed ahead and
motion-compensates it as the robot moves forward. Bottom-strip
observations that conflict with a high-confidence remembered future
corridor are rejected/held instead of replanning the navigation line.

Design: image-space first, no camera calibration required.
Bins correspond to MultiROI strips (default 10). Each bin stores
center_x, width, left_x, right_x, confidence, age.

Motion compensation: constant forward_px_per_frame shift downward.
Optionally, could be estimated by profile matching — kept simple.

Gating: per-strip center/width within gate fractions of predicted width.
Spatial support: a bottom conflict is only accepted if upper bins also
support the new geometry.

Output is a corrected raw-like measurement for the scalar temporal filter.

Coordinate frames: bins, predictions and the detector prior live in the
DETECTOR frame (the border-cropped binary mask, strip rows from the
detector's strip_profile). Values returned for control and drawing
(corrected_bottom_x, corrected_X, left/right, pred_bottom_x, raw_bottom_x)
are in FULL-IMAGE coordinates: the crop offset is added and the bottom x
is referred to the image bottom row like mr_vs.nav_line_to_feature's P.
Headings use the mr_vs convention: Theta > 0 when the corridor leans
right toward the top of the image.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional, Dict, List, Tuple

import numpy as np

def wrapToPi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

@dataclass
class LookaheadParams:
    image_width: int = 640
    image_height: int = 480
    n_bins: int = 10
    # motion: how many image pixels the ground moves down per frame
    shift_px: float = 14.0
    # vertical coverage must match detector (0.75 → ROIs cover bottom 3/4)
    vertical_coverage: float = 0.75
    # number of bottom strips ignored for nav (approach phase)
    ignore_initial: int = 0
    # if True, try to estimate shift by profile matching (small range)
    estimate_shift: bool = False
    estimate_range: Tuple[float, float] = (-2.0, 20.0)
    estimate_step: float = 4.0

    # gating fractions (relative to corridor width)
    max_center_gate_frac: float = 0.35
    max_width_gate_frac: float = 0.35
    max_flank_gate_frac: float = 0.40  # for left/right individually

    # confidence dynamics
    conf_decay: float = 0.97          # per-frame decay when not updated
    conf_alpha: float = 0.35          # EMA when updated
    map_alpha: float = 0.30           # EMA for center/width
    min_conf_for_hold: float = 0.40   # need at least this to hold/reject
    min_conf_to_update: float = 0.15  # below this map is considered no_map

    # acceptance of true changes
    accept_frames: int = 4            # consecutive frames with spatial support
    accept_bins: int = 3              # upper bins supporting new geometry
    bottom_bins: int = 2              # mu <= bottom_bins considered local
    upper_start: int = 4              # mu >= upper_start considered future (1-indexed)

    # minimal width sanity
    min_width_px: float = 20.0
    # minimal center gate px
    min_center_gate_px: float = 12.0

@dataclass
class MapBin:
    center_x: float = 0.0
    width: float = 0.0
    left_x: float = 0.0
    right_x: float = 0.0
    confidence: float = 0.0
    age: int = 0
    last_update: int = 0
    two_sided: bool = False

class LookaheadCorridorMap:
    """
    Causal, lightweight lookahead map.
    Call update(strip_profile, res, image_shape, dt, last_w) each frame.
    Returns dict with corrected measurement + diagnostics.
    """

    def __init__(self, params: Optional[LookaheadParams] = None):
        self.params = params or LookaheadParams()
        self.frame_idx: int = 0
        self.bins: List[MapBin] = []
        self._init_bins()
        # persistence counters for bottom conflict -> switch
        self._conflict_frames: int = 0
        self._switch_frames: int = 0
        self._last_map_status: str = "init"
        # for shift estimation history
        self._last_raw_centers: Optional[List[float]] = None
        # detector-frame geometry, known after the first update()
        self._frame_known: bool = False
        self._strip_height: Optional[float] = None

    def _init_bins(self):
        p = self.params
        self.bins = [
            MapBin(center_x=p.image_width/2.0, width=120.0,
                   left_x=p.image_width/2.0-60, right_x=p.image_width/2.0+60,
                   confidence=0.0, age=999)
            for _ in range(p.n_bins)
        ]

    def reset(self):
        self.frame_idx = 0
        self._init_bins()
        self._conflict_frames = 0
        self._switch_frames = 0
        self._last_map_status = "init"
        self._last_raw_centers = None
        self._frame_known = False
        self._strip_height = None

    def get_prediction(self, image_shape: Optional[Tuple[int,int]] = None) -> List[MapBin]:
        """Predicted bins for current frame (motion-compensated) without updating state.
        Used as prior for detector ROI gating. Returned list length n_bins, each MapBin with confidence.
        """
        # The map works in the detector frame learned in update(); the shape
        # given here (usually the full image) only seeds the geometry before
        # the first update, when every bin still has zero confidence.
        if image_shape is not None and not self._frame_known:
            h, w = image_shape[:2]
            self.params.image_height = h
            self.params.image_width = w
        return self._predict_bins(self.params.shift_px)

    def _strip_dh(self) -> float:
        """Strip height in detector rows (the detector's own grid if known)."""
        if self._strip_height:
            return float(self._strip_height)
        p = self.params
        return (p.image_height * float(getattr(p, "vertical_coverage", 1.0))) / max(1, p.n_bins)

    # --------------------------------------------------------------
    def _predict_bins(self, shift_px: float) -> List[MapBin]:
        """Motion-compensated prediction: evidence seen higher up moves down."""
        p = self.params
        n = p.n_bins
        dh = self._strip_dh()
        shift_strips = shift_px / max(1.0, dh)
        pred: List[MapBin] = []
        for i in range(n):  # i=0 bottom mu=1
            # predicted comes from old bin at i + shift_strips (higher up)
            src_f = i + shift_strips
            lo = int(math.floor(src_f))
            hi = int(math.ceil(src_f))
            if lo < 0 or lo >= n:
                # no future evidence for this low strip -> empty prediction
                pred.append(MapBin(center_x=p.image_width/2.0, width=120.0,
                                   left_x=p.image_width/2.0-60, right_x=p.image_width/2.0+60,
                                   confidence=0.0, age=999, two_sided=False))
                continue
            if lo == hi:
                src = self.bins[lo]
                pred.append(MapBin(
                    center_x=src.center_x, width=src.width,
                    left_x=src.left_x, right_x=src.right_x,
                    confidence=max(0.0, src.confidence * p.conf_decay),
                    age=src.age+1, last_update=src.last_update, two_sided=src.two_sided))
            else:
                # interpolate
                if hi >= n:
                    hi = lo
                a = self.bins[lo]
                b = self.bins[hi]
                frac = src_f - lo
                # only interpolate if both have reasonable confidence; else take higher
                if a.confidence < 0.05 and b.confidence < 0.05:
                    pred.append(MapBin(center_x=p.image_width/2.0, width=120.0,
                                       left_x=p.image_width/2.0-60, right_x=p.image_width/2.0+60,
                                       confidence=0.0, age=999, two_sided=False))
                    continue
                weak = p.min_conf_to_update
                if a.confidence < weak or b.confidence < weak:
                    # one side is unobserved/decayed memory (its centre is the
                    # placeholder image centre): take the more confident bin
                    # alone instead of pulling it toward the placeholder
                    src = a if a.confidence >= b.confidence else b
                    pred.append(MapBin(
                        center_x=src.center_x, width=src.width,
                        left_x=src.left_x, right_x=src.right_x,
                        confidence=max(0.0, src.confidence * p.conf_decay),
                        age=src.age+1, last_update=src.last_update, two_sided=src.two_sided))
                    continue
                # weighted average
                cx = (1-frac)*a.center_x + frac*b.center_x
                w = (1-frac)*a.width + frac*b.width if a.width>0 and b.width>0 else max(a.width,b.width)
                lx = (1-frac)*a.left_x + frac*b.left_x
                rx = (1-frac)*a.right_x + frac*b.right_x
                conf = ((1-frac)*a.confidence + frac*b.confidence) * p.conf_decay
                pred.append(MapBin(center_x=cx, width=w, left_x=lx, right_x=rx,
                                   confidence=conf, age=max(a.age,b.age)+1,
                                   last_update=max(a.last_update,b.last_update),
                                   two_sided=a.two_sided or b.two_sided))
        return pred

    # --------------------------------------------------------------
    def _per_strip_gate(self, raw_center: Optional[float], raw_width: Optional[float],
                        pred: MapBin, median_width: float) -> Tuple[bool, float, float]:
        """Check if raw observation at this strip is consistent with prediction.
        Returns (consistent, dx, dw). Missing raw -> not consistent.
        """
        p = self.params
        if raw_center is None or raw_width is None or not math.isfinite(raw_width) or raw_width < 5:
            return False, 9999, 9999
        if pred.confidence < p.min_conf_to_update:
            # no strong prediction -> treat as consistent (no map to compare)
            return True, 0.0, 0.0
        # thresholds relative to predicted width
        ref_w = max(p.min_width_px, pred.width if pred.width > 5 else median_width)
        center_gate = max(p.min_center_gate_px, p.max_center_gate_frac * ref_w)
        width_gate = p.max_width_gate_frac * ref_w
        dx = abs(raw_center - pred.center_x)
        dw = abs(raw_width - pred.width)
        consistent = (dx <= center_gate) and (dw <= width_gate)
        return consistent, dx, dw

    # --------------------------------------------------------------
    def update(self, strip_profile: List[Dict], res: Dict,
               image_shape: Tuple[int,int], dt: Optional[float]=None,
               last_w: Optional[float]=None,
               crop_offset: Optional[Tuple[int, int]] = None) -> Dict:
        """
        Update map with current strip observations and return corrected measurement.

        strip_profile: list length n_bins with keys mu, y_center, center_x, width, left_x, right_x, two_sided, accepted_nav, roi
            (detector frame: border-cropped binary coordinates)
        res: full detector res for median_width, binary shape, crop_offset, strip_height
        image_shape: (h, w) of the FULL input image (output frame)
        crop_offset: (dx, dy) detector-to-full offset; defaults to res["crop_offset"]
        dt, last_w: accepted for interface compatibility; motion compensation
            uses the fixed shift_px (see LookaheadParams).

        Returns dict with:
          corrected_bottom_x, corrected_width, corrected_left_x, corrected_right_x,
          corrected_theta, map_confidence, map_status, track_valid,
          raw_map_innovation_x, raw_map_innovation_width,
          spatial_support_score, support_old, support_new,
          pred_bins, bins (for overlay), conflict_frames etc.
        """
        p = self.params
        if len(strip_profile) != p.n_bins:
            raise ValueError(
                f"lookahead map has n_bins={p.n_bins} but the detector strip "
                f"profile has {len(strip_profile)} strips; configure both with "
                "the same strip count")
        self.frame_idx += 1
        h_full, w_full = image_shape[:2]
        res = res if isinstance(res, dict) else {}
        if crop_offset is None:
            crop_offset = res.get("crop_offset", (0, 0)) or (0, 0)
        ox, oy = float(crop_offset[0]), float(crop_offset[1])
        binary = res.get("binary")
        if binary is not None and hasattr(binary, "shape"):
            h, w = binary.shape[:2]
        else:
            h, w = int(h_full - 2 * oy), int(w_full - 2 * ox)
        # keep params in the detector frame (bins/predictions/prior)
        p.image_width = w
        p.image_height = h
        sh = res.get("strip_height")
        if sh:
            self._strip_height = float(sh)
        self._frame_known = True

        # optionally estimate shift by profile matching
        shift_px = p.shift_px
        if p.estimate_shift and len(strip_profile) == p.n_bins and self._last_raw_centers is not None:
            # try candidate shifts and pick one with minimal robust residual where both raw and map have evidence
            best_shift = shift_px
            best_err = float("inf")
            for cand in np.arange(p.estimate_range[0], p.estimate_range[1]+1e-6, p.estimate_step):
                pred_cand = self._predict_bins(cand)
                err = 0.0
                cnt = 0
                for mu_idx, prof in enumerate(strip_profile):
                    if not prof.get("two_sided"):
                        continue
                    raw_c = prof["center_x"]
                    if pred_cand[mu_idx].confidence < 0.15:
                        continue
                    err += abs(raw_c - pred_cand[mu_idx].center_x)
                    cnt += 1
                if cnt >= 3:
                    err = err / cnt
                    if err < best_err:
                        best_err = err
                        best_shift = float(cand)
            # only adopt if alignment confidence good (err small)
            if best_err < 0.25 * (res.get("median_width",120) or 120):
                shift_px = best_shift

        pred_bins = self._predict_bins(shift_px)
        median_w = float(res.get("median_width", 120) or 120)
        # effective bottom for approach-phase ignore
        eff_bottom_idx = int(max(0, min(p.n_bins-1, getattr(p, "ignore_initial", 0))))

        # per-strip classification
        per_strip_status: List[str] = []
        per_strip_dx: List[float] = []
        per_strip_dw: List[float] = []
        consistent_flags: List[bool] = []

        for mu_idx, prof in enumerate(strip_profile):
            pred = pred_bins[mu_idx]
            if not prof.get("two_sided"):
                # missing observation -> occlusion
                per_strip_status.append("missing")
                consistent_flags.append(False)
                per_strip_dx.append(9999)
                per_strip_dw.append(9999)
                continue
            cons, dx, dw = self._per_strip_gate(prof["center_x"], prof["width"], pred, median_w)
            consistent_flags.append(cons)
            per_strip_dx.append(dx)
            per_strip_dw.append(dw)
            per_strip_status.append("map_accept" if cons else "map_reject")

        # --- bottom reference strip ---
        # The effective bottom (after ignore_initial) is the reference. When
        # it has no two-sided observation, the next one or two strips up act
        # as a proxy. Everything below (consistency, prediction, support,
        # holds) uses the SAME reference strip and its own row prediction.
        ref_idx = eff_bottom_idx
        bottom_missing = not strip_profile[eff_bottom_idx].get("two_sided")
        if bottom_missing:
            for k in range(eff_bottom_idx + 1, min(p.n_bins, eff_bottom_idx + 3)):
                if strip_profile[k].get("two_sided"):
                    ref_idx = k
                    bottom_missing = False
                    break
        ref_prof = strip_profile[ref_idx]
        bottom_raw_c = float(ref_prof["center_x"]) if not bottom_missing else None
        bottom_raw_w = float(ref_prof["width"]) if not bottom_missing else None
        bottom_consistent = consistent_flags[ref_idx] if not bottom_missing else False
        y_ref = float(ref_prof["y_center"])
        pred_bottom = pred_bins[ref_idx]
        pred_conf = pred_bottom.confidence

        # spatial support evaluation for future vs local
        # upper bins: mu >= upper_start (1-indexed) and above the reference
        upper_indices = [i for i in range(p.n_bins)
                         if (i+1) >= p.upper_start and i > ref_idx]
        support_old = 0
        support_new = 0
        for idx in upper_indices:
            prof = strip_profile[idx]
            if not prof.get("two_sided"):
                continue
            # check vs predicted old map
            pred = pred_bins[idx]
            if pred.confidence > p.min_conf_for_hold:
                cons_old, _, _ = self._per_strip_gate(prof["center_x"], prof["width"], pred, median_w)
                if cons_old:
                    support_old += 1
            # check vs the reference bottom's new value (if valid)
            if bottom_raw_c is not None and bottom_raw_w is not None:
                ref_w = max(p.min_width_px, bottom_raw_w)
                c_gate = max(p.min_center_gate_px, p.max_center_gate_frac * ref_w)
                w_gate = p.max_width_gate_frac * ref_w
                dx_n = abs(prof["center_x"] - bottom_raw_c)
                dw_n = abs(prof["width"] - bottom_raw_w)
                if dx_n <= c_gate and dw_n <= w_gate:
                    support_new += 1

        # determine map_status for bottom and corrected values
        # defaults: if no map, use raw
        raw_bottom_x = bottom_raw_c if bottom_raw_c is not None else w/2.0
        raw_width = bottom_raw_w if bottom_raw_w is not None else median_w
        # raw heading from two-sided strips at/above the effective bottom
        # (fit x = a*y + b in image rows; mr_vs convention Theta = -atan(a))
        raw_theta = 0.0
        try:
            pts = [(prof["center_x"], prof["y_center"]) for idx, prof in enumerate(strip_profile) if prof.get("two_sided") and idx >= eff_bottom_idx]
            if len(pts) >= 2:
                xs = np.array([q[0] for q in pts], dtype=float)
                ys = np.array([q[1] for q in pts], dtype=float)
                A = np.vstack([ys, np.ones(len(ys))]).T
                a, b = np.linalg.lstsq(A, xs, rcond=None)[0]
                raw_theta = -math.atan(a)
        except Exception:
            raw_theta = 0.0

        # decide corrected (values at the reference row, detector frame)
        map_status = "no_map"
        corrected_bottom_x = raw_bottom_x
        corrected_width = raw_width
        if ref_prof.get("left_x") is not None and not bottom_missing:
            corrected_left_x = float(ref_prof.get("left_x"))
            corrected_right_x = float(ref_prof.get("right_x"))
        else:
            corrected_left_x = float(raw_bottom_x - raw_width/2)
            corrected_right_x = float(raw_bottom_x + raw_width/2)
        innovation_x = 0.0
        innovation_width = 0.0
        if pred_bottom.confidence > 0:
            innovation_x = (raw_bottom_x - pred_bottom.center_x) if bottom_raw_c is not None else 0.0
            innovation_width = (raw_width - pred_bottom.width) if bottom_raw_w is not None else 0.0

        # Case handling
        if pred_conf < p.min_conf_to_update:
            # no strong map -> accept raw (initialization)
            map_status = "no_map"
            corrected_bottom_x = raw_bottom_x
            corrected_width = raw_width
            self._conflict_frames = 0
            self._switch_frames = 0
        elif bottom_missing:
            # occlusion: bottom has no observation, use predicted if confident
            if pred_conf >= p.min_conf_for_hold:
                map_status = "occlusion_hold"
                corrected_bottom_x = pred_bottom.center_x
                corrected_width = pred_bottom.width
                corrected_left_x = pred_bottom.left_x
                corrected_right_x = pred_bottom.right_x
                # slight decay already in pred, no extra
                self._conflict_frames = 0
            else:
                # no observation and no confident memory: no track output
                map_status = "held_no_line"
                corrected_bottom_x = raw_bottom_x
        elif bottom_consistent:
            # raw agrees with map -> accept and update map
            map_status = "map_accept"
            corrected_bottom_x = raw_bottom_x
            corrected_width = raw_width
            self._conflict_frames = 0
            self._switch_frames = 0
        else:
            # bottom conflicts with predicted high-conf map
            # check spatial support
            if support_old >= p.accept_bins and support_new < p.accept_bins:
                # future still supports old -> local gap, hold
                map_status = "map_hold"
                corrected_bottom_x = pred_bottom.center_x
                corrected_width = pred_bottom.width
                corrected_left_x = pred_bottom.left_x
                corrected_right_x = pred_bottom.right_x
                self._conflict_frames += 1
                self._switch_frames = 0
            elif support_new >= p.accept_bins:
                # new geometry has spatial support -> may be genuine shift, require temporal persistence
                self._conflict_frames += 1
                self._switch_frames += 1
                if self._switch_frames >= p.accept_frames:
                    map_status = "map_switch"
                    # smoothly blend toward new
                    blend = 0.35
                    corrected_bottom_x = (1-blend)*pred_bottom.center_x + blend*raw_bottom_x
                    corrected_width = (1-p.map_alpha)*pred_bottom.width + p.map_alpha*raw_width
                    # upper bins follow the new geometry below; the bottom
                    # bins keep their prediction until it is consistent again
                    self._conflict_frames = 0
                    self._switch_frames = 0
                else:
                    map_status = "map_pending"
                    # hold but slight nudge
                    corrected_bottom_x = (1-0.08)*pred_bottom.center_x + 0.08*raw_bottom_x
                    corrected_width = (1-0.05)*pred_bottom.width + 0.05*raw_width
            else:
                # ambiguous: neither old nor new has strong spatial support -> cautious hold
                # treat as pending, decay slightly
                map_status = "map_pending"
                corrected_bottom_x = (1-0.08)*pred_bottom.center_x + 0.08*raw_bottom_x
                corrected_width = (1-0.05)*pred_bottom.width + 0.05*raw_width
                self._conflict_frames += 1
                self._switch_frames = 0

        # Track validity: a value the controller may steer on. False when
        # neither a fresh bottom observation nor confident memory exists
        # (the w/2 placeholder must never be passed on as a measurement).
        if map_status == "held_no_line":
            track_valid = False
        elif map_status == "no_map":
            track_valid = bottom_raw_c is not None
        else:
            track_valid = True
        if map_status in ("occlusion_hold", "map_hold"):
            track_source = "predicted"
        elif map_status == "map_pending":
            track_source = "blended"
        elif track_valid:
            track_source = "observed"
        else:
            track_source = "none"

        # predicted heading from confident predicted bins (same convention)
        pred_theta = 0.0
        try:
            pts_pred = []
            for i, b in enumerate(pred_bins):
                if b.confidence > 0.2:
                    pts_pred.append((b.center_x, strip_profile[i]["y_center"]))
            if len(pts_pred) >= 2:
                xs = np.array([q[0] for q in pts_pred]); ys = np.array([q[1] for q in pts_pred])
                A = np.vstack([ys, np.ones(len(ys))]).T
                a, b = np.linalg.lstsq(A, xs, rcond=None)[0]
                pred_theta = -math.atan(a)
        except Exception:
            pred_theta = 0.0

        if map_status in ("map_hold", "occlusion_hold"):
            corrected_theta = pred_theta
        elif map_status == "map_switch":
            corrected_theta = (1-0.35)*pred_theta + 0.35*raw_theta
        elif map_status == "map_pending":
            corrected_theta = (1-0.08)*pred_theta + 0.08*raw_theta
        else:
            corrected_theta = raw_theta

        # now update bins with corrected/observed values
        # For each bin, if raw exists and was consistent (or we are in switch), update towards raw; else keep predicted with decay
        new_bins: List[MapBin] = []
        for i, prof in enumerate(strip_profile):
            pred = pred_bins[i]
            raw_c = prof["center_x"] if prof.get("two_sided") else None
            raw_wid = prof["width"] if prof.get("two_sided") else None
            raw_l = prof.get("left_x")
            raw_r = prof.get("right_x")
            # decide per-bin update
            if not prof.get("two_sided"):
                # missing -> keep predicted with decay (already decayed)
                nb = MapBin(center_x=pred.center_x, width=pred.width,
                            left_x=pred.left_x, right_x=pred.right_x,
                            confidence=max(0.0, pred.confidence*0.97),
                            age=pred.age+1, last_update=pred.last_update,
                            two_sided=False)
                new_bins.append(nb)
                continue
            # raw exists
            cons = per_strip_status[i] == "map_accept"
            # For bins where map_status is hold/occlusion, we should NOT update effective bottom bins with raw conflicting value
            is_eff_bottom = (eff_bottom_idx <= i < eff_bottom_idx + p.bottom_bins)
            if map_status in ("map_hold", "occlusion_hold") and is_eff_bottom:
                # hold predicted
                nb = MapBin(center_x=pred.center_x, width=pred.width,
                            left_x=pred.left_x, right_x=pred.right_x,
                            confidence=max(0.0, pred.confidence*0.98),
                            age=pred.age+1, last_update=pred.last_update, two_sided=True)
                new_bins.append(nb)
            elif map_status == "map_pending" and is_eff_bottom:
                # slight update
                alpha = 0.08
                cx = (1-alpha)*pred.center_x + alpha*raw_c
                wid = (1-0.05)*pred.width + 0.05*raw_wid
                lx = raw_l if raw_l is not None else (cx - wid/2)
                rx = raw_r if raw_r is not None else (cx + wid/2)
                conf = (1-p.conf_alpha)*pred.confidence + p.conf_alpha*0.6
                nb = MapBin(center_x=cx, width=wid, left_x=lx, right_x=rx,
                            confidence=max(0.0, min(1.0, conf)), age=0, last_update=self.frame_idx, two_sided=True)
                new_bins.append(nb)
            else:
                # check if this upper strip supports the new bottom geometry (true shift)
                supports_new_here = False
                if bottom_raw_c is not None and bottom_raw_w is not None:
                    ref_w2 = max(p.min_width_px, bottom_raw_w)
                    c_gate2 = max(p.min_center_gate_px, p.max_center_gate_frac * ref_w2)
                    w_gate2 = p.max_width_gate_frac * ref_w2
                    if abs(raw_c - bottom_raw_c) <= c_gate2 and abs(raw_wid - bottom_raw_w) <= w_gate2:
                        supports_new_here = True
                # for true shift, allow upper to follow new even though it conflicts with old map
                if map_status in ("map_pending", "map_switch") and supports_new_here and (i+1) >= p.upper_start:
                    # update upper toward new raw
                    alpha_up = 0.30 if map_status == "map_switch" else 0.15
                    cx = (1-alpha_up)*(pred.center_x if pred.confidence>0.05 else raw_c) + alpha_up*raw_c
                    wid = (1-alpha_up)*(pred.width if pred.confidence>0.05 else raw_wid) + alpha_up*raw_wid
                    lx = raw_l if raw_l is not None else (cx - wid/2)
                    rx = raw_r if raw_r is not None else (cx + wid/2)
                    conf = (1-p.conf_alpha)*pred.confidence + p.conf_alpha*0.7
                    nb = MapBin(center_x=cx, width=wid, left_x=lx, right_x=rx,
                                confidence=max(0.0, min(1.0, conf)), age=0, last_update=self.frame_idx, two_sided=True)
                    new_bins.append(nb)
                elif cons:
                    # update towards raw
                    cx = (1-p.map_alpha)*pred.center_x + p.map_alpha*raw_c if pred.confidence>0.05 else raw_c
                    wid = (1-p.map_alpha)*pred.width + p.map_alpha*raw_wid if pred.confidence>0.05 else raw_wid
                    lx = raw_l if raw_l is not None else (cx - wid/2)
                    rx = raw_r if raw_r is not None else (cx + wid/2)
                    conf = (1-p.conf_alpha)*pred.confidence + p.conf_alpha*1.0
                    if prof.get("accepted_nav"):
                        conf = min(1.0, conf+0.05)
                    nb = MapBin(center_x=cx, width=wid, left_x=lx, right_x=rx,
                                confidence=max(0.0, min(1.0, conf)), age=0, last_update=self.frame_idx, two_sided=True)
                    new_bins.append(nb)
                else:
                    # inconsistent but not bottom hold -> keep predicted but decay more
                    # This may happen for isolated upper outlier -> keep predicted
                    if pred.confidence > p.min_conf_for_hold:
                        nb = MapBin(center_x=pred.center_x, width=pred.width,
                                    left_x=pred.left_x, right_x=pred.right_x,
                                    confidence=max(0.0, pred.confidence*0.95),
                                    age=pred.age+1, last_update=pred.last_update, two_sided=True)
                        new_bins.append(nb)
                    else:
                        # no strong prediction, accept raw even if per-strip inconsistent but no map
                        cx = raw_c; wid = raw_wid; lx=raw_l if raw_l is not None else cx-wid/2; rx=raw_r if raw_r is not None else cx+wid/2
                        nb = MapBin(center_x=cx, width=wid, left_x=lx, right_x=rx,
                                    confidence=0.5, age=0, last_update=self.frame_idx, two_sided=True)
                        new_bins.append(nb)

        self.bins = new_bins

        # store raw centers for next shift estimation
        self._last_raw_centers = [prof["center_x"] if prof.get("two_sided") else None for prof in strip_profile]

        # overall map confidence: average of bins with decent confidence, weighted
        confs = [b.confidence for b in self.bins if b.confidence>0.05]
        avg_conf = float(np.mean(confs)) if confs else 0.0

        # spatial support score: support_old vs support_new ratio
        spatial_score = float(support_old - support_new)  # positive means old supported

        # --- output frame: full image, bottom row (same reference as mr_vs P) ---
        # Extrapolate the reference-row centre to the detector's bottom row
        # along the corrected heading, then add the crop offset.
        tan_t = math.tan(float(np.clip(corrected_theta, -math.radians(45), math.radians(45))))
        y_bot_det = float(h - 1)
        bottom_x_det = corrected_bottom_x - (y_bot_det - y_ref) * tan_t
        corrected_bottom_full = bottom_x_det + ox
        corrected_X = corrected_bottom_full - w_full/2.0

        return {
            "corrected_bottom_x": float(corrected_bottom_full),
            "corrected_width": float(corrected_width),
            "corrected_left_x": float(corrected_left_x + ox),
            "corrected_right_x": float(corrected_right_x + ox),
            "corrected_ref_y": float(y_ref + oy),
            "corrected_theta": float(corrected_theta),
            "corrected_X": float(corrected_X),
            "map_confidence": float(avg_conf),
            "map_status": map_status,
            "track_valid": bool(track_valid),
            "track_source": track_source,
            "raw_map_innovation_x": float(innovation_x),
            "raw_map_innovation_width": float(innovation_width),
            "spatial_support_old": int(support_old),
            "spatial_support_new": int(support_new),
            "spatial_score": float(spatial_score),
            "pred_bottom_x": float(pred_bottom.center_x + ox),
            "pred_width": float(pred_bottom.width),
            "pred_theta": float(pred_theta),
            "bottom_consistent": bool(bottom_consistent),
            "bottom_missing": bool(bottom_missing),
            "bottom_ref_idx": int(ref_idx),
            "conflict_frames": int(self._conflict_frames),
            "switch_frames": int(self._switch_frames),
            "per_strip_status": per_strip_status,
            "per_strip_dx": per_strip_dx,
            "per_strip_dw": per_strip_dw,
            "pred_bins": pred_bins,  # detector frame, for diagnostics/overlay
            "bins": list(self.bins),  # current map, detector frame
            "crop_offset": (int(ox), int(oy)),
            "shift_px": float(shift_px),
            "raw_bottom_x": float(raw_bottom_x + ox),
            "raw_width": float(raw_width),
            "frame_idx": int(self.frame_idx),
        }

    def get_map_overlay(self, image_shape: Tuple[int,int], res: Optional[Dict] = None,
                        bins: Optional[List[MapBin]] = None) -> List[Tuple[int,int,int,int]]:
        """Map bins as full-image rectangles (x_lo, x_hi, y_top, y_bot).

        Uses the detector's strip rows (res["strip_profile"]) and crop offset
        when given, so boxes line up with the detector ROIs; otherwise the
        map's own strip grid with no offset.
        """
        h, w = image_shape[:2]
        bins = self.bins if bins is None else bins
        prof = (res or {}).get("strip_profile") or []
        ox, oy = (res or {}).get("crop_offset", (0, 0)) or (0, 0)
        dh = self._strip_dh()
        hb = self.params.image_height
        rects = []
        for i, b in enumerate(bins):
            if i < len(prof):
                y_top = int(prof[i]["y_top"]) + int(oy)
                y_bot = int(prof[i]["y_bot"]) + int(oy)
            else:
                y_top = int(max(0, hb - (i+1)*dh)) + int(oy)
                y_bot = int(y_top + dh)
            x_lo = int(max(0, b.center_x - b.width/2 + ox))
            x_hi = int(min(w, b.center_x + b.width/2 + ox))
            rects.append((x_lo, x_hi, max(0, y_top), min(h, y_bot)))
        return rects
