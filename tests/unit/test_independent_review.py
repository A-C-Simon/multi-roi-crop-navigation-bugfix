"""Control contract checks independent of the implementation's stored state."""
import numpy as np
import pytest

from mr_vs import MRVSParams, MultiROIVS
from run_mr_navigation import process_image
from temporal_filter import TemporalNavigationFilter
from test_multi_roi import MultiROIDetector


@pytest.mark.parametrize('dt', [0.01, 0.005])
def test_published_steering_obeys_rate_limit_across_deadband(dt):
    controller = MultiROIVS(MRVSParams(ki=0.0))
    previous = 0.0
    for _ in range(round(2.0 / dt)):
        _, command, _ = controller.compute_control(
            np.array([100.0, 0.0, 0.0]), dt=dt, confidence=1.0, smooth=True)
        assert abs(command - previous) <= controller.params.max_w_rate * dt + 1e-12
        previous = command
    assert previous < -0.25


def test_no_forward_command_before_initial_track_acquisition():
    # This is different from travelling through a gap on an established track.
    blank = np.full((480, 640, 3), (60, 65, 120), dtype=np.uint8)
    detector = MultiROIDetector(morph='struct')
    controller = MultiROIVS()
    temporal = TemporalNavigationFilter()
    for _ in range(5):
        result = process_image(blank, detector, controller, draw=False,
                               t_filter=temporal, dt=0.05)
        assert not result['info']['has_track']
        assert result['v'] == 0.0
        assert result['w'] == 0.0


def test_predicted_map_output_is_not_fresh_filter_evidence(monkeypatch):
    import cv2
    from lookahead_corridor_map import LookaheadCorridorMap

    image = np.full((480, 640, 3), (60, 65, 120), dtype=np.uint8)
    for x in (250, 390):
        cv2.rectangle(image, (x - 5, 0), (x + 5, 479), (30, 200, 30), -1)
    memory = LookaheadCorridorMap()
    original = memory.update

    def remembered_heading(*args, **kwargs):
        result = original(*args, **kwargs)
        result.update(track_valid=True, track_source='predicted',
                      map_status='map_hold', corrected_theta=0.4)
        return result

    temporal = TemporalNavigationFilter()
    captured = {}

    def capture_input(raw, **kwargs):
        captured.update(raw)
        raise RuntimeError('filter input captured')

    monkeypatch.setattr(memory, 'update', remembered_heading)
    monkeypatch.setattr(temporal, 'update', capture_input)
    with pytest.raises(RuntimeError, match='filter input captured'):
        process_image(image, MultiROIDetector(morph='struct'), MultiROIVS(),
                      draw=False, t_filter=temporal, lookahead_map=memory, dt=0.05)
    assert captured['has_line']
    assert not captured['has_observation']
