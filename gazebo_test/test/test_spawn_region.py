"""spawn_region: start poses sampled around a set centroid (utils/spawn_region.py).

Pure numpy, no ROS needed: ``python3 -m pytest test/test_spawn_region.py``.
"""
import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "gazebo_test" / "utils"))
import spawn_region as SR  # noqa: E402

FLEET = [{"name": "turtlebot2", "model": "turtlebot2"}, {"name": "turtlebot2_1", "model": "turtlebot2"},
         {"name": "turtlebot2_2", "model": "turtlebot2"}, {"name": "jackal", "model": "jackal"}]
RADII = [0.178, 0.178, 0.178, 0.326]


@pytest.mark.parametrize("seed", range(20))
def test_centroid_is_exact_and_bodies_fit_with_the_gap(seed):
    c, R, gap = np.array([1.5, -2.0]), 2.2, 0.9
    p = SR.sample_poses(RADII, c, R, gap, seed=seed)
    assert np.allclose(p[:, :2].mean(axis=0), c, atol=1e-12)
    assert np.all(np.linalg.norm(p[:, :2] - c, axis=1) + RADII <= R + 1e-9)
    for i in range(4):
        for j in range(i + 1, 4):
            assert np.linalg.norm(p[i, :2] - p[j, :2]) >= RADII[i] + RADII[j] + gap - 1e-12


def test_same_seed_same_poses_and_seeds_differ():
    a = SR.sample_poses(RADII, (0, 0), 2.0, 0.6, seed=3)
    assert np.array_equal(a, SR.sample_poses(RADII, (0, 0), 2.0, 0.6, seed=3))
    assert not np.allclose(a, SR.sample_poses(RADII, (0, 0), 2.0, 0.6, seed=4))


def test_heading_modes():
    c = np.zeros(2)
    p = SR.sample_poses(RADII, c, 2.0, 0.5, heading="inward", seed=1)
    to_c = np.arctan2(-p[:, 1], -p[:, 0])
    assert np.allclose(np.cos(p[:, 2] - to_c), 1.0)
    p = SR.sample_poses(RADII, c, 2.0, 0.5, heading=1.57, seed=1)
    assert np.allclose(p[:, 2], 1.57)
    p = SR.sample_poses(RADII, c, 2.0, 0.5, heading="random", seed=1)
    assert np.all(np.abs(p[:, 2]) <= math.pi)


def test_an_impossible_region_is_refused():
    with pytest.raises(ValueError, match="do not fit"):
        SR.sample_poses(RADII, (0, 0), 0.8, 0.9)
    with pytest.raises(ValueError, match="smaller than a robot"):
        SR.sample_poses(RADII, (0, 0), 0.3)


def test_episode_seeds_and_the_per_episode_form():
    shared = {"centroid": [0, 0], "radius": 2.0, "min_gap": 0.6, "seed": 10}
    assert SR.episode_config(shared, "episode_2", 2)["seed"] == 12
    a = SR.poses_from_spawn_region(shared, "episode_1", 1, FLEET)
    b = SR.poses_from_spawn_region(shared, "episode_2", 2, FLEET)
    assert set(a) == {r["name"] for r in FLEET} and a != b
    per = {"episode_1": {"centroid": [3, 3], "radius": 2.0, "seed": 5}}
    pa = SR.poses_from_spawn_region(per, "episode_1", 1, FLEET)
    assert np.allclose(np.mean([v[:2] for v in pa.values()], axis=0), [3, 3])


def test_radius_comes_from_the_robot_or_its_model():
    assert SR.robot_radius({"name": "x", "model": "jackal"}) == 0.326
    assert SR.robot_radius({"name": "x", "model": "unknown", "radius": 0.2}) == 0.2
    with pytest.raises(ValueError, match="no known radius"):
        SR.robot_radius({"name": "x", "model": "unknown"})
