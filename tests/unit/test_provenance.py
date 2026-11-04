"""Evidence provenance: observation vs remembered track vs no prior (F-04,
F-05)."""
import numpy as np
import pytest

from synth import Stack, blank, corridor
from temporal_filter import TemporalNavigationFilter, TemporalFilterParams
from test_multi_roi import MultiROIDetector


def test_blank_frame_is_not_an_observation():
    res = MultiROIDetector(morph="struct").detect(blank())
    assert res["nav_line"] is None and res["nav_curve"] is None
    assert res["nav_source"] == "none" and not res["nav_observed"]
    assert res["n_q_accepted"] == 0 and res["n_two_sided"] == 0


@pytest.mark.parametrize("use_map", [False, True])
def test_blank_start_does_not_create_an_observed_track(use_map):
    st = Stack(use_map=use_map)
    for _ in range(5):
        out = st.step(blank())
        info = out["info"]
        assert info["status"] == "held_no_line"
        assert not info["has_observation"] and not info["has_track"]
        assert not info["has_line"]
        assert info["track_source"] == "none"
        assert out["w"] == 0.0


@pytest.mark.parametrize("use_map", [False, True])
def test_first_observation_after_blank_start_initialises_the_track(use_map):
    """The placeholder centre must not gate the first real corridor (an
    off-centre corridor 50 px away is >= the jump threshold)."""
    st = Stack(use_map=use_map)
    for _ in range(4):
        st.step(blank())
    out = st.step(corridor(center=250.0))
    info = out["info"]
    assert info["status"] == "accepted"
    assert info["has_observation"] and info["has_track"]
    assert abs(info["filt_bottom_x"] - 251.0) < 3.0


@pytest.mark.parametrize("use_map", [False, True])
def test_remembered_track_survives_full_occlusion(use_map):
    """Established off-centre track, 20 blank frames, then crops return:
    the estimate must not drift toward the image centre (was 281 -> 313 px
    without and 269 -> 286 px with the map)."""
    st = Stack(use_map=use_map)
    for _ in range(10):
        out = st.step(corridor(center=270.0))
    x0 = out["info"]["filt_bottom_x"]
    for i in range(20):
        out = st.step(blank())
        info = out["info"]
        assert abs(info["filt_bottom_x"] - x0) < 1.0
        assert not info["has_observation"]
        assert info["has_track"] and info["track_source"] == "predicted"
        assert info["n_q_accepted"] == 0
        assert out["v"] > 0.0      # deliberate travel on the established track
    for _ in range(3):
        out = st.step(corridor(center=270.0))
    assert out["info"]["status"] == "accepted"
    assert abs(out["info"]["filt_bottom_x"] - x0) < 1.0


def test_one_sided_plant_gap_holds_the_corridor():
    """Missing plants on one row over the lower half: no lateral excursion."""
    st = Stack(use_map=True)
    for _ in range(8):
        out = st.step(corridor(center=320.0))
    x0 = out["info"]["filt_bottom_x"]
    xs = []
    for _ in range(6):
        out = st.step(corridor(center=320.0, gaps=[(0, 240, 480)]))
        xs.append(out["info"]["filt_bottom_x"])
    assert max(abs(x - x0) for x in xs) < 4.0


def test_prior_seeded_fallback_is_labelled_prior():
    st = Stack(use_map=True)
    for _ in range(6):
        st.step(corridor(center=300.0))
    out = st.step(blank())
    assert out["res"]["nav_source"] == "prior"
    assert out["res"]["nav_line"] is None
    assert out["map_out"]["map_status"] == "occlusion_hold"
    assert out["map_out"]["track_valid"]


def test_map_without_confidence_never_emits_placeholder_track():
    st = Stack(use_map=True)
    out = st.step(blank())
    assert not out["map_out"]["track_valid"]
    assert out["map_out"]["track_source"] == "none"


def test_info_reports_observation_for_lock_consumers():
    """F-05: info carries has_observation / has_line for the ROS locks."""
    st = Stack()
    out = st.step(corridor(center=320.0))
    assert out["info"]["has_observation"] is True
    assert out["info"]["has_line"] is True
    out = st.step(blank())
    assert out["info"]["has_observation"] is False
    assert out["info"]["has_line"] is True        # held track


def test_filter_placeholder_state_replaced_by_first_observation():
    tf = TemporalNavigationFilter(TemporalFilterParams())
    tf.update({"has_line": False})
    out = tf.update({"raw_bottom_x": 200.0, "raw_theta": 0.0, "raw_width": 140.0,
                     "has_line": True, "n_two_sided": 9, "n_q_accepted": 9})
    assert out["status"] == "accepted" and out["filt_bottom_x"] == 200.0
    assert out["track_established"]


def test_line_mode_blank_frame_commands_no_turn():
    """A-01 x F-04: line mode on a blank frame previously produced a
    saturated turn through the mirrored vertical line."""
    st = Stack(nav_curve=False, temporal=False)
    out = st.step(blank())
    assert out["F_raw"] is None
    assert out["w"] == 0.0


def test_cold_start_commands_nothing_until_first_observation():
    st = Stack()
    for _ in range(3):
        out = st.step(blank())
        assert out["v"] == 0.0 and out["w"] == 0.0
        assert out["info"].get("no_track_stop")
    out = st.step(corridor(center=320.0))
    assert out["v"] > 0.0 and out["info"]["has_track"]


def test_map_hold_feature_is_not_fresh_filter_evidence():
    """A held map feature is supplied to the filter as remembered, while the
    public frame observation flag stays true."""
    st = Stack(use_map=True)
    for _ in range(6):
        st.step(corridor(center=300.0))
    # bottom strips displaced, upper strips support the remembered corridor
    img = corridor(center=300.0)
    img[330:, :] = (60, 65, 120)
    import cv2
    for xb in (330, 470):
        cv2.rectangle(img, (xb - 5, 330), (xb + 5, 479), (30, 200, 30), -1)
    out = st.step(img)
    # the displaced bottom is suppressed by the prior; fresh upper strips
    # still make the frame an observation, the supplied feature is memory
    assert out["map_out"]["track_source"] == "predicted"
    assert out["info"]["has_observation"]
    assert not out["info"]["filter_input_observed"]
