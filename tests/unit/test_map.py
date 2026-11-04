"""Lookahead map evidence handling (P-07, P-08a/b, F-03) and its use."""
import pytest

from lookahead_corridor_map import LookaheadCorridorMap, LookaheadParams, MapBin
from synth import Stack, corridor
from test_multi_roi import MultiROIDetector


def _profile(centers, width=140.0, y0=430.0, dh=34.0):
    prof = []
    for i, c in enumerate(centers):
        two = c is not None
        prof.append({
            "mu": i + 1, "y_center": y0 - i * dh, "y_top": int(y0 - i * dh - dh / 2),
            "y_bot": int(y0 - i * dh + dh / 2),
            "center_x": float(c) if two else 308.0,
            "width": width if two else None,
            "left_x": c - width / 2 + 5 if two else None,
            "right_x": c + width / 2 - 5 if two else None,
            "two_sided": two, "accepted_nav": two,
        })
    return prof


def _res(width=140.0):
    return {"median_width": width, "crop_offset": (0, 0), "strip_height": 34}


def test_interpolation_ignores_unobserved_neighbour():
    """P-07: an observed bin beside an empty one must not be pulled toward
    the placeholder centre (was 246.7 for an observed 200)."""
    m = LookaheadCorridorMap(LookaheadParams(image_width=616, image_height=462))
    m._frame_known = True
    m._strip_height = 34.0
    m.bins[3] = MapBin(center_x=200.0, width=140.0, left_x=135, right_x=265,
                       confidence=0.8, age=0)
    pred = m.get_prediction()
    # bins 2 and 3 draw on bin 3 (and the empty bin 4 for bin 3): the
    # observed centre is kept instead of being blended toward 308 px
    assert pred[2].center_x == pytest.approx(200.0)
    assert pred[3].center_x == pytest.approx(200.0)
    assert pred[4].confidence < 0.05


def test_top_observed_strip_is_accepted_at_memory_edge():
    m = LookaheadCorridorMap(LookaheadParams(n_bins=10))
    centers = [200.0] * 6 + [None] * 4
    statuses = []
    for _ in range(10):
        out = m.update(_profile(centers), _res(), (480, 640), crop_offset=(0, 0))
        statuses.append(out["per_strip_status"][5])
    assert "map_reject" not in statuses[1:]


def test_proxy_bottom_consistent_with_memory_is_accepted():
    """P-08a: a missing bottom with a consistent proxy strip is an accept,
    not a conflict hold."""
    m = LookaheadCorridorMap(LookaheadParams(n_bins=10))
    full = [300.0] * 10
    for _ in range(5):
        m.update(_profile(full), _res(), (480, 640))
    out = m.update(_profile([None] + [300.0] * 9), _res(), (480, 640))
    assert not out["bottom_missing"]
    assert out["bottom_ref_idx"] == 1
    assert out["bottom_consistent"]
    assert out["map_status"] == "map_accept"


def test_support_new_uses_effective_bottom_after_ignore():
    """P-08b: with ignore_initial=3, support for a new geometry is counted
    against strip 4 (the effective bottom), not the ignored strip 1."""
    m = LookaheadCorridorMap(LookaheadParams(n_bins=10, ignore_initial=3))
    for _ in range(5):
        m.update(_profile([300.0] * 10), _res(), (480, 640))
    shifted = [200.0, 200.0, 200.0] + [360.0] * 7
    out = m.update(_profile(shifted), _res(), (480, 640))
    assert out["bottom_ref_idx"] == 3
    assert out["spatial_support_new"] >= 3


def test_false_local_bottom_displacement_is_held_by_memory():
    """Intended protection: upper strips support the remembered corridor."""
    m = LookaheadCorridorMap(LookaheadParams(n_bins=10))
    for _ in range(6):
        m.update(_profile([300.0] * 10), _res(), (480, 640))
    for _ in range(12):
        out = m.update(_profile([400.0, 400.0] + [300.0] * 8), _res(), (480, 640))
        assert out["map_status"] == "map_hold"
        assert abs(out["corrected_bottom_x"] - 300.0) < 1.0


def test_strip_bin_mismatch_is_rejected_explicitly():
    """F-03: a strip/bin mismatch is a configuration error, not an
    IndexError on the second frame."""
    det = MultiROIDetector(n_strips=8)
    m = LookaheadCorridorMap(LookaheadParams(n_bins=10))
    res = det.detect(corridor())
    with pytest.raises(ValueError, match="n_bins"):
        m.update(res["strip_profile"], res, (480, 640))


def test_cli_rejects_mismatched_bins(monkeypatch, capsys):
    import run_mr_navigation
    monkeypatch.setattr("sys.argv", ["multiroi-nav", "--input", "x", "--n_strips", "8",
                                     "--lookahead-bins", "10"])
    with pytest.raises(SystemExit):
        run_mr_navigation.main()
    assert "must equal --n_strips" in capsys.readouterr().err


def test_cli_rejects_single_strip(monkeypatch, capsys):
    """P-10: one strip cannot produce a navigation fit."""
    import run_mr_navigation
    monkeypatch.setattr("sys.argv", ["multiroi-nav", "--input", "x", "--n_strips", "1"])
    with pytest.raises(SystemExit):
        run_mr_navigation.main()
    assert "n_strips" in capsys.readouterr().err


def test_detector_validates_strip_configuration():
    with pytest.raises(ValueError):
        MultiROIDetector(n_strips=1)
    with pytest.raises(ValueError):
        MultiROIDetector(n_strips=5, ignore_initial=4)
    MultiROIDetector(n_strips=5, ignore_initial=3)


def test_map_and_raw_agree_through_ignored_bottom_strips():
    st = Stack(use_map=True, ignore_initial=3)
    for _ in range(6):
        out = st.step(corridor(center=300.0, lean_deg=5.0))
    st2 = Stack(use_map=False, ignore_initial=3)
    for _ in range(6):
        out2 = st2.step(corridor(center=300.0, lean_deg=5.0))
    assert abs(out["info"]["filt_bottom_x"] - out2["info"]["filt_bottom_x"]) < 3.0
