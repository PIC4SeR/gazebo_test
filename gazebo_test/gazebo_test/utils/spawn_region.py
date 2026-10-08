"""Start poses sampled around a set centroid: the goals/poses YAML ``spawn_region``.

An alternative to hand-written ``poses:`` for an episode. Every robot is placed
inside a circle of ``radius`` around ``centroid`` with at least ``min_gap``
between any two bodies, and the whole set is then translated so its mean is
EXACTLY ``centroid`` -- which is what the perturbation task holds (its default
hold point is the mean of the start poses) and what a formation run starts from::

    spawn_region:                  # applies to every episode without `poses`
      centroid: [0.0, 0.0]         # mean of the start positions, exactly
      radius: 2.0                  # every robot body lies inside this circle [m]
      min_gap: 0.9                 # body-to-body clearance between robots [m]
      heading: random              # random | inward | outward | <angle in rad>
      seed: 0                      # episode k (1-based in `episodes`) uses seed + k

    spawn_region:                  # or one block per episode
      episode_1: {centroid: [0.0, 0.0], radius: 2.0, min_gap: 0.9, seed: 7}

A body's radius is the robot's own ``radius:`` in the ``robots:`` list, else the
model's from MODEL_RADIUS. Pure numpy (no ROS), so offline ports can import it
and reproduce the exact poses Gazebo spawns from the same seed.
"""
import math
from typing import Dict, List, Sequence

import numpy as np

# Footprint radii (smallest circle around the body), as measured for the tuning
# sim's HuNav port (swarm_control tuning/scenarios.py TB2_BODY / JACKAL_BODY).
MODEL_RADIUS = {"turtlebot2": 0.178, "jackal": 0.326}

# Random sequential placement rarely fills more than about half of the disk.
_MAX_DENSITY = 0.5


def sample_poses(robot_radii: Sequence[float], centroid: Sequence[float], radius: float,
                 min_gap: float = 0.0, heading="random", seed: int = 0,
                 max_tries: int = 2000) -> np.ndarray:
    """(n, 3) start poses [x, y, theta] for robots with the given body radii.

    Positions are drawn uniformly in the disk (each body fully inside it) and
    accepted only if they keep ``min_gap`` to every robot already placed; the set
    is then shifted so its mean is exactly ``centroid`` and redrawn if the shift
    pushed a body outside the circle. Same seed, same poses.
    """
    radii = np.asarray(robot_radii, dtype=float)
    c = np.asarray(centroid, dtype=float).reshape(2)
    if np.any(radii >= radius):
        raise ValueError(f"spawn_region radius {radius} m is smaller than a robot body")
    need = float(np.sum(math.pi * (radii + min_gap / 2.0) ** 2))
    if need > _MAX_DENSITY * math.pi * radius ** 2:
        raise ValueError(
            f"spawn_region: {len(radii)} robots with min_gap {min_gap} m do not fit in "
            f"radius {radius} m (need about {math.sqrt(need / (_MAX_DENSITY * math.pi)):.2f} m)")
    rng = np.random.default_rng(seed)
    for _ in range(max_tries):
        pts: List[np.ndarray] = []
        for r_i in radii:
            for _ in range(200):
                rho = (radius - r_i) * math.sqrt(rng.random())
                ang = 2.0 * math.pi * rng.random()
                p = c + rho * np.array([math.cos(ang), math.sin(ang)])
                if all(np.linalg.norm(p - q) >= r_i + radii[j] + min_gap
                       for j, q in enumerate(pts)):
                    pts.append(p)
                    break
            else:
                break
        if len(pts) < len(radii):
            continue
        xy = np.array(pts)
        xy += c - xy.mean(axis=0)
        if np.all(np.linalg.norm(xy - c, axis=1) + radii <= radius + 1e-9):
            return np.column_stack([xy, _headings(xy, c, heading, rng)])
    raise ValueError(f"spawn_region: could not place {len(radii)} robots in radius "
                     f"{radius} m with min_gap {min_gap} m after {max_tries} tries")


def _headings(xy: np.ndarray, c: np.ndarray, heading, rng) -> np.ndarray:
    if heading == "random":
        return rng.uniform(-math.pi, math.pi, len(xy))
    if heading == "inward":
        return np.arctan2(c[1] - xy[:, 1], c[0] - xy[:, 0])
    if heading == "outward":
        return np.arctan2(xy[:, 1] - c[1], xy[:, 0] - c[0])
    return np.full(len(xy), float(heading))


def episode_config(spawn_cfg: dict, episode: str, index: int) -> dict:
    """The spawn_region block for ``episode`` (index 1-based), seed resolved."""
    if episode in spawn_cfg:                       # per-episode form
        cfg = dict(spawn_cfg[episode])
        cfg.setdefault("seed", index)
        return cfg
    cfg = dict(spawn_cfg)
    cfg["seed"] = int(cfg.get("seed", 0)) + index
    return cfg


def robot_radius(robot: dict) -> float:
    if "radius" in robot:
        return float(robot["radius"])
    model = robot.get("model")
    if model not in MODEL_RADIUS:
        raise ValueError(f"robot '{robot.get('name')}': no 'radius:' and no known "
                         f"radius for model '{model}' (known: {sorted(MODEL_RADIUS)})")
    return MODEL_RADIUS[model]


def poses_from_spawn_region(spawn_cfg: dict, episode: str, index: int,
                            fleet: List[dict]) -> Dict[str, List[float]]:
    """{robot name: [x, y, theta]} for one episode of a goals/poses YAML."""
    cfg = episode_config(spawn_cfg, episode, index)
    missing = {"centroid", "radius"} - set(cfg)
    if missing:
        raise ValueError(f"spawn_region for '{episode}' needs {sorted(missing)}")
    poses = sample_poses([robot_radius(r) for r in fleet], cfg["centroid"], float(cfg["radius"]),
                         float(cfg.get("min_gap", 0.0)), cfg.get("heading", "random"),
                         int(cfg["seed"]))
    return {r["name"]: [float(v) for v in p] for r, p in zip(fleet, poses)}
