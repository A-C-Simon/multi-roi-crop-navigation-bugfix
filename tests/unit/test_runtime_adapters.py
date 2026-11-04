"""Runner, stack, simulator adapter and ROS node behaviour (F-05, F-06,
F-08, F-13, F-14, F-17). ROS modules are stubbed so these run without ROS."""
import csv
import importlib
import math
import sys
import time
import types
from pathlib import Path

import cv2
import numpy as np
import pytest

from synth import blank, corridor

ROOT = Path(__file__).resolve().parents[2]
MRSIM = ROOT / "tests" / "sim_ros2" / "src" / "mrsim" / "mrsim"


# ---------------------------------------------------------------- runners
def test_video_runner_uses_source_frame_interval(tmp_path):
    """F-06: video control intervals come from the file, not host time."""
    import run_mr_navigation as rmn
    from mr_vs import MultiROIVS, MRVSParams
    from test_multi_roi import MultiROIDetector
    path = tmp_path / "clip.mp4"
    wr = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), 25, (640, 480))
    if not wr.isOpened():
        pytest.skip("mp4 writer unavailable")
    for _ in range(8):
        wr.write(corridor(center=320.0))
    wr.release()
    rmn.run_video(str(path), str(tmp_path), MultiROIDetector(morph="struct"),
                  MultiROIVS(MRVSParams(geometry_from_frame=True)))
    rows = list(csv.DictReader(open(tmp_path / "clip_nav.csv")))
    assert len(rows) == 8
    dts = [float(r["dt_s"]) for r in rows[1:]]
    assert all(abs(d - 0.04) < 2e-3 for d in dts)
    assert all(r["has_observation"] == "1" for r in rows)


def test_controller_geometry_follows_frame_when_requested():
    """F-13: normalization width follows the frame unless fixed."""
    from run_mr_navigation import process_image
    from mr_vs import MultiROIVS, MRVSParams
    from test_multi_roi import MultiROIDetector
    vs = MultiROIVS(MRVSParams(geometry_from_frame=True))
    img = cv2.resize(corridor(center=320.0), (1280, 960))
    process_image(img, MultiROIDetector(morph="struct"), vs, draw=False, dt=0.05)
    assert (vs.params.width, vs.params.height) == (1280, 960)
    fixed = MultiROIVS(MRVSParams(width=640, height=480))
    process_image(img, MultiROIDetector(morph="struct"), fixed, draw=False, dt=0.05)
    assert fixed.params.width == 640


def test_navigation_stack_shares_the_pipeline_and_accepts_dt():
    from mr_navigation_stack import MultiROINavigationStack
    stack = MultiROINavigationStack()
    for _ in range(3):
        cmd, overlay, res = stack.step(corridor(center=320.0, lean_deg=6.0), dt=0.05)
    assert overlay is not None and cmd.has_line
    assert stack.last_map is not None and stack.last_map["map_status"] == "map_accept"
    assert stack.last_filt["filt_theta_deg"] > 4.0      # heading sign preserved
    assert cmd.w < 0


# --------------------------------------------------------- sim adapter
def _pipeline_module(monkeypatch):
    monkeypatch.setenv("MULTIROI_DIR", str(ROOT))
    monkeypatch.syspath_prepend(str(MRSIM))
    sys.modules.pop("pipeline", None)
    return importlib.import_module("pipeline")


def test_sim_pipeline_passes_source_dt_and_defaults_to_no_map(monkeypatch):
    mod = _pipeline_module(monkeypatch)
    monkeypatch.delenv("MRSIM_LOOKAHEAD", raising=False)
    pl = mod.build_pipeline("multiroi")
    assert pl.lookahead_map is None
    out = pl.process(corridor(center=320.0), dt=0.04)
    assert out["info"]["dt"] == pytest.approx(0.04)
    assert out["map_out"] is None


def test_sim_pipeline_lookahead_option_and_reset(monkeypatch):
    mod = _pipeline_module(monkeypatch)
    monkeypatch.setenv("MRSIM_LOOKAHEAD", "1")
    pl = mod.build_pipeline("multiroi")
    assert pl.lookahead_map is not None
    for _ in range(3):
        out = pl.process(corridor(center=300.0), dt=0.05)
    assert out["map_out"]["map_status"] == "map_accept"
    pl.reset()
    assert pl.t_filter.state is None
    assert pl.lookahead_map.frame_idx == 0


# ------------------------------------------------------------ ROS node
class _Pub:
    def __init__(self, topic):
        self.topic, self.msgs = topic, []

    def publish(self, m):
        self.msgs.append(m)

    def get_subscription_count(self):
        return 0


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, s):
        self.lines.append(("info", s))

    warn = warning = error = info


def _install_ros_stubs(monkeypatch):
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        monkeypatch.setitem(sys.modules, name, m)
        return m

    class Node:
        def __init__(self, name):
            self._log, self.pubs, self.timers, self.subs = _Log(), {}, [], []

        def get_parameter(self, name):
            raise KeyError(name)

        def create_subscription(self, typ, topic, cb, qos, **kw):
            self.subs.append((topic, cb))

        def create_publisher(self, typ, topic, qos):
            self.pubs[topic] = _Pub(topic)
            return self.pubs[topic]

        def create_timer(self, period, cb):
            self.timers.append(cb)

        def get_logger(self):
            return self._log

        def destroy_node(self):
            pass

    class _V:
        def __init__(self):
            self.x = self.y = self.z = 0.0

    class Twist:
        def __init__(self):
            self.linear, self.angular = _V(), _V()

    class Bridge:
        def imgmsg_to_cv2(self, msg, desired_encoding="bgr8"):
            return msg.data

    q = types.SimpleNamespace(BEST_EFFORT=1, KEEP_LAST=1, VOLATILE=1)
    mod("rclpy", init=lambda **k: None, spin=lambda n: None, ok=lambda: False,
        shutdown=lambda: None)
    mod("rclpy.node", Node=Node)
    mod("rclpy.qos", QoSProfile=lambda **k: None, ReliabilityPolicy=q,
        HistoryPolicy=q, DurabilityPolicy=q)
    mod("sensor_msgs"); mod("sensor_msgs.msg", Image=object)
    mod("geometry_msgs"); mod("geometry_msgs.msg", Twist=Twist)
    mod("nav_msgs"); mod("nav_msgs.msg", Odometry=object)
    mod("std_msgs"); mod("std_msgs.msg", Empty=object)
    mod("cv_bridge", CvBridge=Bridge)


def _msg(t, img=None, enc="bgr8"):
    st = types.SimpleNamespace(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))
    return types.SimpleNamespace(header=types.SimpleNamespace(stamp=st),
                                 encoding=enc, data=img if img is not None else blank())


@pytest.fixture
def nav(monkeypatch, tmp_path):
    _install_ros_stubs(monkeypatch)
    monkeypatch.setenv("MULTIROI_DIR", str(ROOT))
    monkeypatch.setenv("MRSIM_LOG_DIR", str(tmp_path))
    monkeypatch.setenv("MRSIM_MAX_SECONDS", "1.0")
    monkeypatch.setenv("MRSIM_CAMERA_TIMEOUT", "0.5")
    monkeypatch.setenv("MRSIM_LANE_END_X", "100")
    monkeypatch.syspath_prepend(str(MRSIM))
    for name in ("nav_node", "pipeline"):
        sys.modules.pop(name, None)
    nn = importlib.import_module("nav_node")
    node = nn.MultiROINavNode()
    return nn, node


class _RecordingPipeline:
    name = "stub"

    def __init__(self, fail=False):
        self.fail, self.dts, self.resets = fail, [], 0

    def process(self, bgr, dt=None):
        self.dts.append(dt)
        if self.fail:
            raise RuntimeError("injected")
        return {"v": 0.2, "w": 0.3, "overlay": None,
                "info": {"has_observation": True, "has_line": True,
                         "status": "accepted", "confidence": 0.9, "err_x": 1.0}}

    def reset(self):
        self.resets += 1


def _cmds(node):
    return node.pubs["/cmd_vel"].msgs


def test_node_passes_stamp_dt_to_pipeline(nav):
    nn, node = nav
    node.pipeline = _RecordingPipeline()
    node._on_image(_msg(0.00))
    node._on_image(_msg(0.04))
    assert node.pipeline.dts == [pytest.approx(0.05), pytest.approx(0.04)]
    assert _cmds(node)[-1].angular.z == pytest.approx(0.3)


def test_pipeline_failure_publishes_stop_and_keeps_run_end_checks(nav):
    """F-08: an exception stops the rover for that tick and the max_seconds
    check still runs (previously the early return skipped both)."""
    nn, node = nav
    node.pipeline = _RecordingPipeline()
    node._on_image(_msg(0.0))
    node.pipeline.fail = True
    node._on_image(_msg(0.5))
    assert _cmds(node)[-1].linear.x == 0.0 and _cmds(node)[-1].angular.z == 0.0
    with pytest.raises(nn.NavRunFinished):
        node._on_image(_msg(1.2))
    assert node._stopped and node.csv_f.closed
    n = len(_cmds(node))
    node._stop_robot("shutdown")            # F-17: idempotent
    node._on_image(_msg(1.3))               # ignored after stop
    assert len(_cmds(node)) == n


def test_unsupported_encoding_is_a_perception_failure(nav):
    nn, node = nav
    node.pipeline = _RecordingPipeline()
    node._on_image(_msg(0.0))
    node._on_image(_msg(0.05, enc="mono16"))
    assert _cmds(node)[-1].linear.x == 0.0
    assert node.n_perception_errors == 1


def test_camera_watchdog_commands_stop_after_timeout(nav):
    nn, node = nav
    assert node.timers, "watchdog timer must exist when camera_timeout > 0"
    node.pipeline = _RecordingPipeline()
    node._on_image(_msg(0.0))
    before = len(_cmds(node))
    node._watchdog()
    assert len(_cmds(node)) == before          # fresh frame: no action
    node._last_img_mono = time.monotonic() - 1.0
    node._watchdog()
    assert _cmds(node)[-1].linear.x == 0.0 and node._stall_active


def test_reset_rover_restores_navigation_state(nav):
    """F-14: /reset_rover resets perception, lap and row-change state."""
    nn, node = nav
    node.pipeline = _RecordingPipeline()
    node._lap_angle, node.lanes_done, node.drive_dir, node.phase = 3.0, 2, -1, "spin1"
    node._on_reset(None)
    assert node.pipeline.resets == 1
    assert node._lap_angle == 0.0 and node.lanes_done == 0
    assert node.drive_dir == 1 and node.phase == "follow"
    assert any(t == "/reset_rover" for t, _ in node.subs)


def test_locks_require_fresh_observation(nav):
    """F-05: rear/primary locks read has_observation (previously a key that
    never existed, so locks never engaged)."""
    nn, node = nav
    t = 10.0
    out = {"info": {"has_observation": True, "has_line": True}}
    assert node._pipe_locked(out, t, t) is not None
    held = {"info": {"has_observation": False, "has_line": True}}
    assert node._pipe_locked(held, t, t) is None
    assert node._pipe_locked(out, t - 1.0, t) is None
