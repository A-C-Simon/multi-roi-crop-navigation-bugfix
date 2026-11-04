"""Hold, rejection and reachable recovery (F-07, F-09)."""
import math

import pytest

from synth import Stack, corridor
from temporal_filter import TemporalNavigationFilter, TemporalFilterParams

ALLOWED = [(10, 0), (10, 3), (10, 4), (8, 2), (6, 0), (5, 3)]


def _raw(x=320.0, theta_deg=0.0, n_two=10, observed=True, has_line=True,
         usable=None, width=140.0):
    d = {"raw_bottom_x": x, "raw_theta": math.radians(theta_deg),
         "raw_width": width, "has_line": has_line, "has_observation": observed,
         "n_two_sided": n_two, "n_q_accepted": n_two, "n_q_rejected": 0,
         "weed_pressure": 0.0}
    if usable is not None:
        d["n_evidence_strips"] = usable
    return d


@pytest.mark.parametrize("n,ig", ALLOWED)
def test_commit_floor_is_reachable_for_allowed_configurations(n, ig):
    tf = TemporalNavigationFilter(TemporalFilterParams(n_strips=n, ignore_initial=ig))
    usable = n - ig
    floor = tf.commit_floor()
    assert 2 <= floor <= usable
    assert floor == 7 if (n, ig) == (10, 0) else True
    tf.update(_raw(n_two=usable))
    statuses = [tf.update(_raw(theta_deg=20.0, n_two=usable))["status"] for _ in range(10)]
    assert "pending_accepted" in statuses
    assert statuses.index("pending_accepted") == 6     # 4 + 3 coherent frames


@pytest.mark.parametrize("n,ig", ALLOWED)
def test_sparse_evidence_never_commits_a_large_change(n, ig):
    tf = TemporalNavigationFilter(TemporalFilterParams(n_strips=n, ignore_initial=ig))
    usable = n - ig
    tf.update(_raw(n_two=usable))
    sparse = tf.commit_floor() - 1
    for _ in range(100):
        out = tf.update(_raw(theta_deg=20.0, n_two=sparse))
        assert out["status"] == "pending"
    assert out["filt_theta_deg"] == pytest.approx(0.0, abs=1e-9)


def test_unreachable_explicit_floor_is_rejected():
    with pytest.raises(ValueError):
        TemporalNavigationFilter(TemporalFilterParams(
            n_strips=10, ignore_initial=4, commit_min_two_sided=7))
    with pytest.raises(ValueError):
        TemporalNavigationFilter(TemporalFilterParams(n_strips=2, ignore_initial=1))


def test_remembered_prediction_never_commits_itself():
    """A map hold emitting a different stale slope (no fresh observation)
    is held strictly and never becomes a pending candidate."""
    tf = TemporalNavigationFilter(TemporalFilterParams())
    tf.update(_raw())
    for _ in range(60):
        out = tf.update(_raw(theta_deg=-25.0, observed=False))
        assert out["status"] == "held"
        assert out["pending_count"] == 0
    assert out["filt_theta_deg"] == pytest.approx(0.0, abs=1e-9)


def test_contradicting_observations_break_the_pending_streak():
    """F-09: observations near the held state between candidate frames mean
    the candidate is not consecutive; it must not commit."""
    tf = TemporalNavigationFilter(TemporalFilterParams())
    tf.update(_raw())
    statuses = []
    for _ in range(10):
        statuses.append(tf.update(_raw(x=420.0))["status"])
        tf.update(_raw(x=322.0, n_two=0))   # low evidence, agrees with hold
    assert "pending_accepted" not in statuses


def test_candidate_is_retained_across_frames_without_observation():
    """Intentional: missing observations neither confirm nor refute it."""
    tf = TemporalNavigationFilter(TemporalFilterParams())
    tf.update(_raw())
    for _ in range(3):
        tf.update(_raw(x=420.0))
    for _ in range(20):
        out = tf.update(_raw(has_line=False, observed=False))
        assert out["status"] == "held_no_line"
    assert out["pending_count"] == 3


@pytest.mark.parametrize("use_map", [False, True])
def test_single_and_short_false_displacements_leave_no_trace(use_map):
    st = Stack(use_map=use_map)
    for _ in range(6):
        out = st.step(corridor(center=320.0))
    x0, th0 = out["info"]["filt_bottom_x"], out["info"]["filt_err_theta_deg"]
    for k in (1, 3):
        for _ in range(k):
            out = st.step(corridor(center=320.0, lean_deg=-25.0))
            assert abs(out["info"]["filt_err_theta_deg"] - th0) < 0.5
            assert abs(out["info"]["filt_bottom_x"] - x0) < 1.0
        for _ in range(6):
            out = st.step(corridor(center=320.0))
        assert abs(out["info"]["filt_err_theta_deg"] - th0) < 0.5


@pytest.mark.parametrize("n,ig", [(10, 0), (10, 4), (8, 2), (6, 0)])
def test_persistent_full_evidence_heading_change_is_reachable(n, ig):
    """End to end: a coherent 15 degree heading change (above the 12 degree
    gate) seen with full two-sided evidence is committed in every allowed
    configuration (the fixed floor of 7 made this unreachable for 6 usable
    strips). 15 degrees keeps both rows inside the frame for tall strips."""
    st = Stack(n_strips=n, ignore_initial=ig)
    for _ in range(4):
        st.step(corridor(center=320.0))
    statuses = []
    for _ in range(25):
        out = st.step(corridor(center=320.0, lean_deg=15.0))
        statuses.append(out["info"]["status"])
    assert "pending_accepted" in statuses
    assert out["info"]["filt_err_theta_deg"] > 12.0


@pytest.mark.parametrize("use_map", [False, True])
def test_gradual_curvature_is_followed(use_map):
    st = Stack(use_map=use_map)
    for _ in range(4):
        st.step(corridor(center=320.0))
    for i in range(41):
        lean = 10.0 * i / 40.0
        out = st.step(corridor(center=320.0, lean_deg=lean))
        assert out["info"]["status"] in ("accepted",)
    assert abs(out["info"]["filt_err_theta_deg"] - 10.0) < 1.5


def test_genuine_lane_shift_still_reached_with_memory():
    """With map-held features no longer counted as fresh evidence, a real
    60 px corridor shift is still followed through the map switch."""
    st = Stack(use_map=True)
    for _ in range(5):
        st.step(corridor(center=320.0))
    for _ in range(40):
        out = st.step(corridor(center=380.0))
    assert abs(out["info"]["filt_bottom_x"] - 381.0) < 3.0
    assert out["info"]["status"] == "accepted"


def test_held_heading_follows_own_rotation_without_observation():
    """Held-track heading is rotated by the commanded yaw on frames without
    observation, so a stale heading does not command a turn indefinitely."""
    from mr_vs import MultiROIVS, MRVSParams
    for gain, expect_small in ((1.0, True), (0.0, False)):
        tf = TemporalNavigationFilter(TemporalFilterParams(held_yaw_gain=gain))
        vs = MultiROIVS(MRVSParams())
        tf.update(_raw(x=317.0, theta_deg=-11.0, n_two=9))
        w = 0.0
        for _ in range(200):
            o = tf.update(_raw(has_line=False, observed=False), dt=0.05, last_w=w)
            _, w, _ = vs.compute_control(o["filt_F"], dt=0.05, confidence=o["confidence"])
        assert (abs(w) < 0.03) == expect_small
