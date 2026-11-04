"""Command smoothing at different update rates (A-02, F-06)."""
import numpy as np
import pytest

from mr_vs import MultiROIVS, MRVSParams
from temporal_filter import CommandSmoother, CommandSmootherParams

RATES = [20, 60, 100]


def _run(hz, F, seconds=3.0, **kw):
    params = dict(ki=0.0)
    params.update(kw)
    vs = MultiROIVS(MRVSParams(**params))
    dt = 1.0 / hz
    ws = [vs.compute_control(np.array(F, float), dt=dt, confidence=1.0)[1]
          for _ in range(int(seconds * hz))]
    return vs, np.array(ws), dt


@pytest.mark.parametrize("hz", RATES)
def test_steady_turn_passes_smoother_at_every_rate(hz):
    """A-02: a steady demand (-0.3125 rad/s) is issued at 20, 60 and 100 Hz
    (previously zero forever above ~60 Hz)."""
    vs, ws, dt = _run(hz, [100.0, 0.0, 0.0])
    assert ws[-1] == pytest.approx(-0.3125, abs=1e-3)
    p = vs.params
    # the PUBLISHED command obeys the rate limit, including the deadband exit
    steps = np.abs(np.diff(np.concatenate([[0.0], ws])))
    assert np.all(steps <= p.max_w_rate * dt + 1e-12)


@pytest.mark.parametrize("hz", RATES)
def test_physical_response_time_is_rate_independent(hz):
    """Time to reach 90% of the demand is set by physical limits, not by the
    number of updates."""
    _, ws, dt = _run(hz, [100.0, 0.0, 0.0])
    t90 = (np.argmax(np.abs(ws) >= 0.9 * 0.3125) + 1) * dt
    assert 0.2 <= t90 <= 0.6


@pytest.mark.parametrize("hz", RATES)
def test_small_steady_demand_beyond_deadband_is_issued(hz):
    """A 0.1 rad/s demand (below the old from-rest threshold of
    deadband/alpha at high rates) exceeds the 0.02 deadband and is issued."""
    # w_raw = -lambda_x * 16/640 = -0.1
    _, ws, _ = _run(hz, [16.0, 0.0, 0.0], lambda_x=4.0)
    assert ws[-1] == pytest.approx(-0.1, abs=2e-3)


@pytest.mark.parametrize("hz", RATES)
def test_no_lock_after_returning_through_deadband(hz):
    """Turn, settle to zero, then turn the other way: never stuck at zero."""
    vs = MultiROIVS(MRVSParams(ki=0.0))
    dt = 1.0 / hz
    seq = [100.0] * int(2 * hz) + [0.0] * int(2 * hz) + [-100.0] * int(2 * hz)
    ws = [vs.compute_control(np.array([x, 0, 0.0]), dt=dt, confidence=1.0)[1]
          for x in seq]
    assert ws[int(2 * hz) - 1] < -0.3
    assert abs(ws[int(4 * hz) - 1]) < 1e-9
    assert ws[-1] > 0.3


def test_deadband_still_zeroes_tiny_commands():
    _, ws, _ = _run(100, [3.0, 0.0, 0.0])      # w_raw ~ -0.009
    assert np.all(ws == 0.0)


@pytest.mark.parametrize("hz", RATES)
def test_parallel_command_smoother_has_no_rate_lock(hz):
    sm = CommandSmoother(CommandSmootherParams())
    out = [sm.smooth(-0.3, 0.2, dt=1.0 / hz)[0] for _ in range(int(3 * hz))]
    assert out[-1] == pytest.approx(-0.3, abs=1e-3)


def test_explicit_dt_drives_integrator_and_smoother():
    """F-06: with an explicit source dt, results do not depend on host
    timing between calls."""
    import time
    F = np.array([100.0, 0.0, 0.0])
    a = MultiROIVS(MRVSParams())
    b = MultiROIVS(MRVSParams())
    for _ in range(5):
        wa = a.compute_control(F, dt=0.05, confidence=1.0)[1]
        time.sleep(0.02)
        wb = b.compute_control(F, dt=0.05, confidence=1.0)[1]
    assert wa == pytest.approx(wb, abs=1e-12)
    assert b.compute_control(F, dt=0.05, confidence=1.0)[2]["dt"] == pytest.approx(0.05)


@pytest.mark.parametrize("hz", RATES)
def test_published_rate_bound_through_reversals_and_stops(hz):
    """Returned commands (not hidden state) stay within max_w_rate*dt per
    update through turn, stop, reverse turn and noise inside the deadband."""
    vs = MultiROIVS(MRVSParams(ki=0.0))
    dt = 1.0 / hz
    rng = np.random.default_rng(0)
    seq = ([100.0] * int(1.5 * hz) + [0.0] * int(1.5 * hz) + [-100.0] * int(1.5 * hz)
           + [100.0] * int(0.3 * hz) + list(rng.uniform(-3, 3, int(1.5 * hz))))
    prev, out = 0.0, []
    for x in seq:
        w = vs.compute_control(np.array([x, 0.0, 0.0]), dt=dt, confidence=1.0)[1]
        assert abs(w - prev) <= vs.params.max_w_rate * dt + 1e-12
        prev = w
        out.append(w)
    n = int(1.5 * hz)
    assert out[n - 1] < -0.3 and abs(out[2 * n - 1]) < 1e-12 and out[3 * n - 1] > 0.3
    assert abs(out[-1]) < 1e-12           # noise inside the deadband suppressed
    # F=None (no feature) decay path is bounded too
    vs2 = MultiROIVS(MRVSParams(ki=0.0))
    prev = 0.0
    for k in range(int(3 * hz)):
        F = np.array([100.0, 0, 0]) if k < 1.5 * hz else None
        w = vs2.compute_control(F, dt=dt, confidence=1.0)[1]
        assert abs(w - prev) <= vs2.params.max_w_rate * dt + 1e-12
        prev = w


@pytest.mark.parametrize("hz", RATES)
def test_command_smoother_published_rate_bound(hz):
    sm = CommandSmoother(CommandSmootherParams())
    dt = 1.0 / hz
    prev = 0.0
    seq = [-0.3] * int(2 * hz) + [0.0] * int(2 * hz) + [0.3] * int(2 * hz)
    for x in seq:
        w = sm.smooth(x, 0.2, dt=dt)[0]
        assert abs(w - prev) <= sm.params.max_w_rate * dt + 1e-12
        prev = w
    assert prev == pytest.approx(0.3, abs=1e-3)
