#!/usr/bin/env python3
"""
Stereo -> Depth -> Point Cloud -> 2D Occupancy Grid (log-odds)

Inputs:
  - left image
  - right image
Outputs:
  - occupancy grid as a numpy array in [-1, 0..100] like ROS nav_msgs/OccupancyGrid
    (-1 unknown, 0 free, 100 occupied)
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Tuple

import cv2
import numpy as np


# -----------------------------
# 0) Parameters (ALL variables)
# -----------------------------
@dataclass
class StereoParams:
    # Camera intrinsics
    fx: float = 700.0
    fy: float = 700.0
    cx: float = 640.0
    cy: float = 360.0

    # Stereo baseline (meters)
    baseline_m: float = 0.12

    # Disparity algorithm (StereoSGBM)
    min_disparity: int = 0
    num_disparities: int = 128          # MUST be divisible by 16
    block_size: int = 7                 # odd: 3..11 typical
    p1: int = 8 * 3 * 7 * 7
    p2: int = 32 * 3 * 7 * 7
    disp12_max_diff: int = 1
    pre_filter_cap: int = 63
    uniqueness_ratio: int = 10
    speckle_window_size: int = 100
    speckle_range: int = 2
    mode: int = cv2.STEREO_SGBM_MODE_SGBM_3WAY

    # Depth filtering
    depth_min_m: float = 0.3
    depth_max_m: float = 15.0
    disparity_min_valid: float = 1.0    # pixels; avoid divide by ~0

    # Point subsampling (speed)
    stride_u: int = 2
    stride_v: int = 2

    # Ground removal (optional simple heuristic)
    enable_ground_filter: bool = True
    # Keep points with height in [z_min, z_max] where z is camera-up axis after transform.
    # We'll define camera frame: X right, Y down, Z forward (OpenCV default).
    # We convert to "robot-ish": x forward, y left, z up.
    z_up_min_m: float = -0.2   # below this considered ground-ish
    z_up_max_m: float = 1.5


@dataclass
class GridParams:
    # Grid geometry
    resolution_m: float = 0.10           # cell size (m)
    width_m: float = 20.0                # grid width (meters)
    height_m: float = 20.0               # grid height (meters)

    # Grid origin in robot frame (x forward, y left)
    # This defines where (0,0) cell is in the robot frame.
    origin_x_m: float = -5.0             # meters
    origin_y_m: float = -10.0            # meters

    # Occupancy update (log odds)
    l0: float = 0.0                      # prior log-odds
    l_occ: float = 0.85                  # add when occupied evidence
    l_free: float = -0.40                # add when free evidence
    l_min: float = -5.0                  # clamp
    l_max: float = 5.0

    # Raycasting / free-space marking
    enable_ray_free_space: bool = True
    # How far along the ray to mark free (stop a bit before obstacle)
    free_stop_margin_m: float = 0.20

    # Converting log-odds -> occupancy probability thresholds
    # Unknown if never updated; else map prob to [0..100]
    prob_free_thresh: float = 0.35
    prob_occ_thresh: float = 0.65


# -----------------------------
# 1) Helpers
# -----------------------------
def clamp(x: np.ndarray, lo: float, hi: float) -> np.ndarray:
    return np.minimum(np.maximum(x, lo), hi)


def logodds_to_prob(l: np.ndarray) -> np.ndarray:
    # p = 1 / (1 + exp(-l))
    return 1.0 / (1.0 + np.exp(-l))


def bresenham(x0: int, y0: int, x1: int, y1: int):
    """Yield grid cells along a line from (x0,y0) to (x1,y1)."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


# -----------------------------
# 2) Disparity + Depth
# -----------------------------
def compute_disparity(left_bgr: np.ndarray, right_bgr: np.ndarray, sp: StereoParams) -> np.ndarray:
    left = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
    right = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)

    # Ensure valid num_disparities
    num_disp = int(math.ceil(sp.num_disparities / 16) * 16)
    block_size = sp.block_size if sp.block_size % 2 == 1 else sp.block_size + 1

    matcher = cv2.StereoSGBM_create(
        minDisparity=sp.min_disparity,
        numDisparities=num_disp,
        blockSize=block_size,
        P1=sp.p1,
        P2=sp.p2,
        disp12MaxDiff=sp.disp12_max_diff,
        preFilterCap=sp.pre_filter_cap,
        uniquenessRatio=sp.uniqueness_ratio,
        speckleWindowSize=sp.speckle_window_size,
        speckleRange=sp.speckle_range,
        mode=sp.mode,
    )

    disp = matcher.compute(left, right).astype(np.float32) / 16.0  # SGBM returns fixed-point
    return disp


def disparity_to_depth(disp: np.ndarray, sp: StereoParams) -> np.ndarray:
    # depth = fx * baseline / disparity
    depth = np.full_like(disp, np.nan, dtype=np.float32)

    valid = np.isfinite(disp) & (disp >= sp.disparity_min_valid)
    depth[valid] = (sp.fx * sp.baseline_m) / disp[valid]

    # clamp by depth min/max
    depth[(depth < sp.depth_min_m) | (depth > sp.depth_max_m)] = np.nan
    return depth


# -----------------------------
# 3) Depth -> 3D points in robot frame
# -----------------------------
def depth_to_points_robot(depth: np.ndarray, sp: StereoParams) -> np.ndarray:
    """
    OpenCV pinhole projection assumes:
      - image u right, v down
      - camera frame: X right, Y down, Z forward

    We'll convert to robot-ish frame:
      x forward, y left, z up
    mapping:
      x = Z
      y = -X
      z = -Y
    Returns Nx3 points (x,y,z) in meters.
    """
    h, w = depth.shape

    us = np.arange(0, w, sp.stride_u, dtype=np.float32)
    vs = np.arange(0, h, sp.stride_v, dtype=np.float32)
    uu, vv = np.meshgrid(us, vs)

    dd = depth[vv.astype(np.int32), uu.astype(np.int32)]
    valid = np.isfinite(dd)

    uu = uu[valid]
    vv = vv[valid]
    dd = dd[valid]

    # camera frame
    X = (uu - sp.cx) * dd / sp.fx
    Y = (vv - sp.cy) * dd / sp.fy
    Z = dd

    # robot frame
    x = Z
    y = -X
    z = -Y

    pts = np.stack([x, y, z], axis=1)

    if sp.enable_ground_filter:
        z_ok = (pts[:, 2] >= sp.z_up_min_m) & (pts[:, 2] <= sp.z_up_max_m)
        pts = pts[z_ok]

    return pts


# -----------------------------
# 4) Occupancy grid mapping
# -----------------------------
class OccupancyGrid2D:
    def __init__(self, gp: GridParams):
        self.gp = gp
        self.width_cells = int(round(gp.width_m / gp.resolution_m))
        self.height_cells = int(round(gp.height_m / gp.resolution_m))

        # log-odds grid + touched mask for unknown handling
        self.L = np.full((self.height_cells, self.width_cells), gp.l0, dtype=np.float32)
        self.touched = np.zeros((self.height_cells, self.width_cells), dtype=bool)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Robot frame world coords (x forward, y left) -> grid indices (ix, iy)
        We store grid as [row=iy, col=ix], with iy increasing upward in y? We'll define:
          - ix increases with +x
          - iy increases with +y
        But numpy rows increase downward, so we flip when writing final image.
        We'll keep internal as (iy, ix) with iy increasing with +y (mathematically),
        and map to array row = (height-1 - iy).
        """
        ix = int(math.floor((x - self.gp.origin_x_m) / self.gp.resolution_m))
        iy = int(math.floor((y - self.gp.origin_y_m) / self.gp.resolution_m))
        return ix, iy

    def in_bounds(self, ix: int, iy: int) -> bool:
        return (0 <= ix < self.width_cells) and (0 <= iy < self.height_cells)

    def _set_logodds(self, ix: int, iy: int, delta: float):
        if not self.in_bounds(ix, iy):
            return
        row = self.height_cells - 1 - iy
        col = ix
        self.L[row, col] = float(np.clip(self.L[row, col] + delta, self.gp.l_min, self.gp.l_max))
        self.touched[row, col] = True

    def update_with_points(self, points_xyz: np.ndarray):
        """
        points_xyz: Nx3 in robot frame (x forward, y left, z up)
        We do:
          - mark occupied at endpoint cell
          - optionally mark free along the ray from (0,0) to (x,y)
        """
        if points_xyz.size == 0:
            return

        robot_ix, robot_iy = self.world_to_grid(0.0, 0.0)

        for x, y, _z in points_xyz:
            end_ix, end_iy = self.world_to_grid(float(x), float(y))
            if not self.in_bounds(end_ix, end_iy):
                continue

            if self.gp.enable_ray_free_space:
                # stop a bit before obstacle
                dist = math.hypot(x, y)
                stop_dist = max(0.0, dist - self.gp.free_stop_margin_m)
                if dist > 1e-6 and stop_dist > 0.0:
                    xs = (stop_dist / dist) * x
                    ys = (stop_dist / dist) * y
                    stop_ix, stop_iy = self.world_to_grid(xs, ys)
                else:
                    stop_ix, stop_iy = end_ix, end_iy

                # free cells along line, excluding endpoint cell
                for ix, iy in bresenham(robot_ix, robot_iy, stop_ix, stop_iy):
                    if (ix, iy) == (end_ix, end_iy):
                        break
                    self._set_logodds(ix, iy, self.gp.l_free)

            # occupied endpoint
            self._set_logodds(end_ix, end_iy, self.gp.l_occ)

    def to_occupancy_msg_array(self) -> np.ndarray:
        """
        Returns grid in ROS-style integer values:
          -1 unknown, 0..100 probability
        """
        probs = logodds_to_prob(self.L)
        occ = np.full_like(self.L, -1, dtype=np.int16)

        # If touched, assign 0..100
        touched = self.touched
        occ[touched] = (probs[touched] * 100.0).astype(np.int16)

        # Optionally you can hard-threshold:
        # occ[touched & (probs < self.gp.prob_free_thresh)] = 0
        # occ[touched & (probs > self.gp.prob_occ_thresh)] = 100

        return occ


# -----------------------------
# 5) Main
# -----------------------------
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left", required=True, help="path to left image")
    parser.add_argument("--right", required=True, help="path to right image")
    parser.add_argument("--show", action="store_true", help="visualize disparity + grid")
    args = parser.parse_args()

    # ====== PARAMETERS HERE ======
    stereo_params = StereoParams(
        fx=700.0, fy=700.0, cx=640.0, cy=360.0,
        baseline_m=0.12,
        num_disparities=128,
        block_size=7,
        depth_min_m=0.3,
        depth_max_m=15.0,
        stride_u=2,
        stride_v=2,
        enable_ground_filter=True,
        z_up_min_m=-0.2,
        z_up_max_m=1.5,
    )

    grid_params = GridParams(
        resolution_m=0.10,
        width_m=20.0,
        height_m=20.0,
        origin_x_m=-5.0,
        origin_y_m=-10.0,
        enable_ray_free_space=True,
        free_stop_margin_m=0.20,
        l_occ=0.85,
        l_free=-0.40,
        l_min=-5.0,
        l_max=5.0,
    )
    # =============================

    left = cv2.imread(args.left, cv2.IMREAD_COLOR)
    right = cv2.imread(args.right, cv2.IMREAD_COLOR)
    if left is None or right is None:
        raise FileNotFoundError("Could not read left/right images.")

    disp = compute_disparity(left, right, stereo_params)
    depth = disparity_to_depth(disp, stereo_params)
    pts = depth_to_points_robot(depth, stereo_params)

    grid = OccupancyGrid2D(grid_params)
    grid.update_with_points(pts)
    occ = grid.to_occupancy_msg_array()

    print("Occupancy grid shape:", occ.shape)
    print("Values: -1 unknown, 0..100 occupied probability")

    if args.show:
        # disparity visualization
        disp_vis = disp.copy()
        disp_vis[~np.isfinite(disp_vis)] = 0
        disp_vis = cv2.normalize(disp_vis, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        # occupancy visualization (unknown=127 gray, free black-ish, occupied white-ish)
        occ_vis = np.zeros((occ.shape[0], occ.shape[1]), dtype=np.uint8)
        unknown_mask = (occ < 0)
        occ_vis[unknown_mask] = 127
        known = ~unknown_mask
        occ_vis[known] = np.clip(occ[known], 0, 100).astype(np.uint8) * 255 // 100

        cv2.imshow("left", left)
        cv2.imshow("disparity (normalized)", disp_vis)
        cv2.imshow("occupancy grid (gray)", cv2.resize(occ_vis, (600, 600), interpolation=cv2.INTER_NEAREST))
        cv2.waitKey(0)


if __name__ == "__main__":
    main()
