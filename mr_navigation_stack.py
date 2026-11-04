"""
MultiROI Navigation Stack - Rover Control
==========================================
Implements the full navigation stack for a rover using the MultiROI
furrow detector. This is the "rest from there as to an actual rover
moving" after the navigation line is available.

The stack consists of:
1. Perception: MultiROI detector (already available, furrow centre line)
2. State Estimation: Visual features (X, Theta) from navigation line
3. Control: Visual servoing controller that outputs v,w
4. Actuation: Rover kinematics (differential drive) update

This is designed to be used both offline (on Photos) and online
(with ROS2 or with a simulated rover in Gazebo).

The rover is modeled as a differential drive with:
  x_dot = v * cos(theta)
  y_dot = v * sin(theta)
  theta_dot = w

Where (x,y,theta) is the rover pose in the world frame.

For the visual servoing, we use the same interaction matrix approach
as in ExG/agribot_vs.cpp but simplified for the furrow.

Author: adapted from ExG navigation stack for MultiROI furrow
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict

import cv2
import numpy as np

from test_multi_roi import MultiROIDetector
from mr_vs import MultiROIVS, MRVSParams, wrapToPi
from temporal_filter import TemporalNavigationFilter, TemporalFilterParams
from lookahead_corridor_map import LookaheadCorridorMap, LookaheadParams
import time as _time


@dataclass
class RoverPose:
    x: float = 0.0  # meters, forward is +x in world? Use standard
    y: float = 0.0
    theta: float = 0.0  # radians, 0 is along +x

    def copy(self):
        return RoverPose(self.x, self.y, self.theta)


@dataclass
class RoverCommand:
    v: float  # linear m/s
    w: float  # angular rad/s
    err_x: float
    err_theta: float
    has_line: bool


class RoverSimulator:
    """Simple differential drive rover simulator."""

    def __init__(self, initial_pose: Optional[RoverPose] = None, dt: float = 0.1):
        self.pose = initial_pose or RoverPose()
        self.dt = dt
        self.path: List[RoverPose] = [self.pose.copy()]
        self.commands: List[RoverCommand] = []

    def update(self, v: float, w: float):
        """Update pose using differential drive kinematics."""
        # Simple Euler integration
        self.pose.x += v * math.cos(self.pose.theta) * self.dt
        self.pose.y += v * math.sin(self.pose.theta) * self.dt
        self.pose.theta = wrapToPi(self.pose.theta + w * self.dt)
        self.path.append(self.pose.copy())
        return self.pose.copy()

    def get_path_array(self):
        """Return path as numpy array for plotting."""
        return np.array([[p.x, p.y, p.theta] for p in self.path])


class MultiROINavigationStack:
    """
    Full navigation stack: perception + control + actuation.

    Usage:
        detector = MultiROIDetector()
        vs = MultiROIVS()
        nav_stack = MultiROINavigationStack(detector, vs)
        # For each image:
        cmd, overlay = nav_stack.step(bgr_image)
        # cmd is RoverCommand with v,w
        # Update simulator or publish to /cmd_vel
        simulator.update(cmd.v, cmd.w)

    Temporal robustness: a TemporalNavigationFilter sits between
    detector and controller.  Raw and filtered states are both
    available via `self.last_raw` / `self.last_filt`.
    """

    def __init__(self, detector: Optional[MultiROIDetector] = None,
                 vs: Optional[MultiROIVS] = None,
                 simulator: Optional[RoverSimulator] = None,
                 use_temporal: bool = True,
                 t_params: Optional[TemporalFilterParams] = None,
                 use_lookahead: bool = True,
                 l_params: Optional[LookaheadParams] = None,
                 ignore_initial: int = 0):
        # handle ignore_initial passed via detector/l_params as well
        self.ignore_initial = int(ignore_initial)
        if detector is None:
            detector = MultiROIDetector(ignore_initial=self.ignore_initial)
        elif self.ignore_initial:
            detector.ignore_initial = self.ignore_initial
        self.detector = detector
        # the detector is the source of truth for the ignored-strip count
        self.ignore_initial = int(getattr(self.detector, "ignore_initial", 0))
        if vs is None:
            # default controller: normalize with the actual frame geometry
            vs = MultiROIVS()
            vs.params.geometry_from_frame = True
        self.vs = vs
        # keep the drawn nav line clipping consistent with the detector's ROIs
        self.vs.params.vertical_coverage = float(getattr(self.detector, "vertical_coverage", 0.75))
        self.simulator = simulator or RoverSimulator()
        self.step_count = 0
        self.last_cmd = RoverCommand(0, 0, 0, 0, False)
        self.use_temporal = use_temporal
        self.t_filter: Optional[TemporalNavigationFilter] = None
        if use_temporal:
            tp = t_params or TemporalFilterParams(
                image_width=self.vs.params.width,
                image_height=self.vs.params.height,
                n_strips=self.detector.n,
                ignore_initial=int(getattr(self.detector, "ignore_initial", 0)))
            # will be lazily corrected to actual image shape on first step
            self.t_filter = TemporalNavigationFilter(tp)
        self.use_lookahead = use_lookahead
        self.lookahead_map: Optional[LookaheadCorridorMap] = None
        if use_lookahead:
            lp = l_params or LookaheadParams(
                image_width=self.vs.params.width,
                image_height=self.vs.params.height,
                n_bins=self.detector.n,
                ignore_initial=self.ignore_initial,
                vertical_coverage=float(getattr(self.detector, "vertical_coverage", 0.75)))
            # ensure sync
            lp.ignore_initial = self.ignore_initial
            lp.vertical_coverage = float(getattr(self.detector, "vertical_coverage", 0.75))
            self.lookahead_map = LookaheadCorridorMap(lp)
        self.last_raw: Optional[Dict] = None
        self.last_filt: Optional[Dict] = None
        self.last_map: Optional[Dict] = None
        self._prev_time: Optional[float] = None

    def step(self, bgr, draw=True, dt=None):
        """
        One navigation step: detect + lookahead map + temporal filter + control.

        Delegates the perception/control chain to
        run_mr_navigation.process_image so the stack and the runners share
        one implementation (coordinate frames, heading convention, evidence
        provenance, map overlay).

        dt: source frame interval in seconds (camera timestamps, simulation
            time). When None the host clock between calls is used.

        Returns (cmd, overlay, res) where
          cmd is RoverCommand
          overlay is BGR image with debug overlay (filtered red, raw cyan, map green)
          res is the raw detector result dict
        Also updates self.last_raw / self.last_filt / self.last_map for diagnostics.
        """
        from run_mr_navigation import process_image

        if dt is None:
            now = _time.perf_counter()
            if self._prev_time is None:
                dt = 0.05
            else:
                dt = now - self._prev_time
            self._prev_time = now
        dt = max(1e-3, min(0.5, float(dt)))

        out = process_image(bgr, self.detector, self.vs, draw=draw,
                            t_filter=self.t_filter, dt=dt,
                            last_w=self.last_cmd.w,
                            lookahead_map=self.lookahead_map)
        info = out["info"]
        v, w_ang = float(out["v"]), float(out["w"])
        has_line = bool(info.get("has_line", False))
        cmd = RoverCommand(v=v, w=w_ang,
                           err_x=info.get("filt_err_x", info.get("err_x", 0)),
                           err_theta=info.get("filt_err_theta_deg", info.get("err_theta_deg", 0)),
                           has_line=has_line)
        self.last_raw = out.get("raw_dict")
        filt_out = out.get("filt_out")
        map_out = out.get("map_out")
        self.last_filt = filt_out if (self.t_filter is not None and filt_out is not map_out) else None
        self.last_map = map_out

        # Update simulator
        self.simulator.update(cmd.v, cmd.w)
        self.last_cmd = cmd
        self.step_count += 1

        overlay = out.get("overlay")
        if draw and overlay is not None:
            pose = self.simulator.pose
            y_txt = 84 if self.lookahead_map is not None else (66 if self.t_filter is not None else 60)
            txt2 = f"Pose x={pose.x:.2f} y={pose.y:.2f} th={math.degrees(pose.theta):.1f}deg | Step {self.step_count}"
            cv2.putText(overlay, txt2, (10, y_txt), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1, cv2.LINE_AA)

        return cmd, overlay, out["res"]

    def reset(self, pose: Optional[RoverPose] = None):
        """Reset the stack and simulator."""
        self.simulator = RoverSimulator(initial_pose=pose)
        self.step_count = 0
        self.last_cmd = RoverCommand(0, 0, 0, 0, False)
        self.last_raw = None
        self.last_filt = None
        self.last_map = None
        self._prev_time = None
        if self.t_filter is not None:
            self.t_filter.reset()
        if self.lookahead_map is not None:
            self.lookahead_map.reset()
        self.vs.reset_smoother()


def demo_offline(photos_dir="../../Photos", output_dir="./nav_stack_output"):
    """Demo: run navigation stack on all Photos as if they were sequential frames."""
    import glob
    import os
    import csv

    detector = MultiROIDetector()
    vs = MultiROIVS()
    nav_stack = MultiROINavigationStack(detector, vs)

    paths = sorted(glob.glob(os.path.join(photos_dir, "*.png")) +
                   glob.glob(os.path.join(photos_dir, "*.jpg")))
    if not paths:
        print(f"No images in {photos_dir}")
        return

    os.makedirs(output_dir, exist_ok=True)
    csv_path = os.path.join(output_dir, "rover_path.csv")
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["frame", "filename", "v", "w_deg", "err_x", "err_theta_deg",
                         "pose_x", "pose_y", "pose_theta_deg", "has_line"])

        for i, p in enumerate(paths):
            bgr = cv2.imread(p)
            if bgr is None:
                continue
            name = os.path.basename(p)
            cmd, overlay, res = nav_stack.step(bgr, draw=True)
            # Save overlay
            cv2.imwrite(os.path.join(output_dir, f"{os.path.splitext(name)[0]}_nav.png"), overlay)
            pose = nav_stack.simulator.pose
            writer.writerow([i, name, f"{cmd.v:.3f}", f"{math.degrees(cmd.w):.2f}",
                             f"{cmd.err_x:.1f}", f"{cmd.err_theta:.2f}",
                             f"{pose.x:.3f}", f"{pose.y:.3f}", f"{math.degrees(pose.theta):.1f}",
                             int(cmd.has_line)])
            print(f"[{i}] {name}: v={cmd.v:.2f} w={math.degrees(cmd.w):.1f}deg err_x={cmd.err_x:.0f} err_theta={cmd.err_theta:.1f} -> pose x={pose.x:.2f} y={pose.y:.2f} th={math.degrees(pose.theta):.1f}")

    # Save path plot
    try:
        import matplotlib.pyplot as plt
        path_arr = nav_stack.simulator.get_path_array()
        plt.figure(figsize=(10, 6))
        plt.plot(path_arr[:, 0], path_arr[:, 1], "-o", markersize=3, label="Rover path")
        plt.xlabel("X (m) forward")
        plt.ylabel("Y (m) lateral")
        plt.title("Rover Path - MultiROI Furrow Following (Photos as sequence)")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.savefig(os.path.join(output_dir, "rover_path.png"), dpi=150)
        plt.close()
        print(f"Path plot saved to {output_dir}/rover_path.png")
    except Exception as e:
        print(f"Could not plot path: {e}")

    print(f"\nDone. Outputs in {os.path.abspath(output_dir)}")
    print(f"CSV: {csv_path}")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="MultiROI Navigation Stack Demo")
    parser.add_argument("--input", default="../../Photos", help="Photos folder")
    parser.add_argument("--output", default="./nav_stack_output", help="Output folder")
    args = parser.parse_args()
    demo_offline(args.input, args.output)
