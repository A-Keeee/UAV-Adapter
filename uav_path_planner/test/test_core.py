"""Geometric and derivative checks independent of ROS and Gazebo."""
from pathlib import Path

import numpy as np
import pytest
from uav_path_planner.core import Box, Spline, World, plan


WORLD = Path(__file__).resolve().parents[2]/'assets/gazebo_world/walls_demo.sdf'


def walls():
    return World.from_sdf(WORLD, [[-15, -16, .8], [25, 18, 4]])


@pytest.mark.parametrize('start,goal', [([0, 0, 2.5], [0, 8, 2.5]),
                                      ([0, 8, 2.5], [8, 8, 2.5]),
                                      ([8, 8, 2.5], [8, 0, 2.5])])
def test_demo_routes_are_certified_and_dynamically_feasible(start, goal):
    world = walls()
    raw, spline, info = plan(world, start, goal)
    assert all(world.segment_free(a, b) for a, b in zip(raw, raw[1:]))
    assert spline.certified(world)
    assert all(world.free(spline.sample(t)) for t in np.linspace(0, spline.duration, 400))
    np.testing.assert_allclose(spline.sample(0), start, atol=1e-10)
    np.testing.assert_allclose(spline.sample(spline.duration), goal, atol=1e-10)
    for order in (1, 2, 3):
        np.testing.assert_allclose(spline.sample(0, order), 0, atol=1e-9)
        np.testing.assert_allclose(spline.sample(spline.duration, order), 0, atol=1e-9)
    assert all(b <= limit for b, limit in zip(spline.bounds(), (1, .7, 1.5)))
    if start[0] == 0:
        assert not world.segment_free(start, goal)  # a direct route really hits a wall
        assert info['spline_length_m'] > np.linalg.norm(np.array(start)-goal)+1


def test_derivatives_match_finite_differences():
    _, spline, _ = plan(walls(), [0, 0, 2.5], [0, 8, 2.5])
    for t in np.linspace(.1, spline.duration-.1, 30):
        for order in (1, 2, 3):
            h = 1e-4
            numeric = (spline.sample(t+h, order-1)-spline.sample(t-h, order-1))/(2*h)
            np.testing.assert_allclose(spline.sample(t, order), numeric, atol=1e-6)


@pytest.mark.parametrize('goal', [[5, 0, 2.5], [50, 0, 2.5], [0, 0, float('nan')]])
def test_bad_waypoints_rejected(goal):
    with pytest.raises(ValueError):
        plan(walls(), [0, 0, 2.5], goal)


def test_unreachable_goal_rejected():
    world = World([Box('barrier', np.array([-.5, -4, 0]), np.array([.5, 4, 5]))],
                  [[-3, -2, 1], [3, 2, 3]], radius=.3, resolution=.5)
    with pytest.raises(ValueError, match='no collision-free route'):
        world.astar([-2, 0, 2], [2, 0, 2])


def test_collision_between_sparse_samples_is_rejected():
    world = World([Box('thin', np.array([-.01, -1, 0]), np.array([.01, 1, 3]))],
                  [[-3, -3, .5], [3, 3, 4]], radius=.1)
    points = np.array([[-2., 0, 2]]*4+[[2., 0, 2]]*4)
    spline = Spline(points, [0.]*8+[1.]*8)
    assert world.free(spline.sample(0)) and world.free(spline.sample(1))
    assert not spline.certified(world)


def test_rotated_sdf_box_is_not_silently_mapped_wrong(tmp_path):
    sdf = tmp_path/'rotated.sdf'
    sdf.write_text('<sdf><world><model name="wall"><static>true</static>'
                   '<pose>0 0 1 0 0 0.5</pose><link><collision><geometry>'
                   '<box><size>1 2 3</size></box></geometry></collision></link></model></world></sdf>')
    with pytest.raises(ValueError, match='axis-aligned'):
        World.from_sdf(sdf, [[-3, -3, .5], [3, 3, 4]])
