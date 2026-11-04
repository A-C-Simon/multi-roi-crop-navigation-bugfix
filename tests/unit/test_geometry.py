"""Geometry, sign and coordinate-frame regressions (A-01, F-01, F-02, F-15,
F-16, P-09, P-11)."""
import math

import numpy as np
import pytest

from synth import Stack, corridor, H, W
from test_multi_roi import MultiROIDetector, pre_corridor_clean
from mr_vs import MultiROIVS, MRVSParams


def _x_at(line, y):
    w, b = line
    return (y - b) / w


@pytest.mark.parametrize("order", [1, -1])
def test_vertical_tls_line_keeps_x_for_both_point_orders(order):
    """A-01: a vertical TLS fit must encode x = c for either SVD sign."""
    pts = [(300.0, y) for y in (100.0, 200.0, 300.0, 400.0)][::order]
    line, acc, rej = MultiROIDetector._fit_line_tls(pts)
    assert abs(_x_at(line, 250.0) - 300.0) < 1e-3
    assert len(acc) == 4 and not rej


@pytest.mark.parametrize("nav_curve", [False, True])
def test_centred_corridor_gives_centred_feature_in_line_and_spline_mode(nav_curve):
    """A-01 end to end: a centred two-row image (identical midpoint x on
    every strip) must not be mirrored off-image in straight-line mode."""
    st = Stack(nav_curve=nav_curve, temporal=False)
    out = st.step(corridor(center=320.0))
    res = out["res"]
    assert res["n_two_sided"] >= 9
    assert abs(out["F_raw"][0]) < 3.0          # lateral feature, px
    assert abs(out["w"]) < 0.05


def _theta_and_x(lean, use_map, frames=4):
    st = Stack(use_map=use_map)
    for _ in range(frames):
        out = st.step(corridor(center=320.0, lean_deg=lean))
    return out["info"]["filt_err_theta_deg"], out["info"]["filt_err_x"], out["info"]["w_raw"]


@pytest.mark.parametrize("lean", [8.0, -8.0])
def test_heading_sign_and_lateral_frame_agree_with_and_without_memory(lean):
    """F-01 and F-02: the lookahead map must report the controller's heading
    convention and full-image lateral feature."""
    th_raw, x_raw, w_raw = _theta_and_x(lean, False)
    th_map, x_map, w_map = _theta_and_x(lean, True)
    # mr_vs convention: lean right toward the top -> Theta > 0
    assert math.copysign(1, th_raw) == math.copysign(1, lean)
    assert math.copysign(1, th_map) == math.copysign(1, lean)
    assert abs(th_raw - lean) < 1.5 and abs(th_map - lean) < 1.5
    assert abs(x_raw - x_map) < 2.0
    # steering opposes the heading in both paths
    assert w_raw * lean < 0 and w_map * lean < 0


def test_centred_corridor_has_no_crop_offset_bias_with_memory():
    """F-02: a centred corridor reads ~0 px with the map (was -11 px)."""
    _, x_map, _ = _theta_and_x(0.0, True)
    _, x_raw, _ = _theta_and_x(0.0, False)
    assert abs(x_map) < 2.0 and abs(x_raw) < 2.0


def test_midpoints_sit_on_measured_strip_centre_rows():
    """F-15: Q points use the centre row of the strip they were measured
    on, the same row as the strip profile and the detection chains."""
    det = MultiROIDetector(morph="struct")
    res = det.detect(corridor(center=320.0, lean_deg=8.0))
    prof = res["strip_profile"]
    for (qx, qy), sp in zip(res["q"], prof):
        assert abs(qy - sp["y_center"]) < 1e-6
    # accepted_nav matching still works after the row change
    assert sum(1 for sp in prof if sp["accepted_nav"]) == res["n_q_accepted"]


def test_struct_clean_strip_index_is_exact():
    """F-16: a component centred in strip mu is judged against the bands of
    strips mu-1, mu and mu+1. The old off-by-one used mu-2..mu, so a weed
    blob in strip 3 aligned only with a band in strip 4 was erased."""
    h, w, n, dh = 400, 400, 10, 40
    binary = np.zeros((h, w), np.uint8)
    binary[:, 100:110] = 255                  # a crop row in every strip
    binary[240:280, 252:261] = 255            # band in strip 4 only
    binary[300:304, 250:263] = 255            # short blob in strip 3, aligned
    binary[300:304, 350:363] = 255            # short blob in strip 3, unaligned
    cleaned, info = pre_corridor_clean(binary, n_strips=n, l_frac=0.05)
    assert cleaned[301, 255] == 255           # adjacent-strip band keeps it
    assert cleaned[301, 355] == 0             # no band nearby -> removed
    assert cleaned[200, 105] == 255           # row kept


def test_map_strip_grid_and_overlay_follow_detector_rows():
    """P-09/P-11: map bins use the detector strip rows and crop offset."""
    st = Stack(use_map=True)
    out = st.step(corridor(center=320.0))
    res = out["res"]
    dx, dy = res["crop_offset"]
    assert st.map._strip_dh() == pytest.approx(res["strip_height"])
    rects = st.map.get_map_overlay((H, W), res=res)
    for (x_lo, x_hi, y_top, y_bot), sp in zip(rects, res["strip_profile"]):
        assert y_top == sp["y_top"] + dy and y_bot == sp["y_bot"] + dy
    # bin 1 box is centred on the corridor in full-image coordinates
    x_lo, x_hi, _, _ = rects[1]
    assert abs((x_lo + x_hi) / 2.0 - 320.0) < 3.0


def test_mr_vs_feature_matches_documented_convention():
    """Reference for the convention the map now follows."""
    vs = MultiROIVS(MRVSParams())
    # line through (300,400) bottom and (340,200) top, cropped == full
    w_slope = (200 - 400) / (340 - 300)
    b = 400 - w_slope * 300
    F, _ = vs.nav_line_to_feature((w_slope, b), None, (0, 0), (480, 640),
                                  vertical_coverage=1.0)
    assert F[2] > 0
