import sys
import os
import pytest

vamp = pytest.importorskip("vamp")

from ompl import base as ob
from ompl import geometric as og

# Add demos/vamp to path for VampStateSpace imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'demos', 'vamp'))
from vamp_state_space import VampMotionValidator, VampStateValidityChecker, VampStateSpace

# Panda 7-DOF configurations
START = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]
GOAL = [2.35, 1., 0., -0.8, 0, 2.5, 0.785]

SPHERES = [
    [0.55, 0, 0.25],
    [0.35, 0.35, 0.25],
    [0, 0.55, 0.25],
    [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25],
    [0, -0.55, 0.25],
    [0.35, -0.35, 0.25],
]


def create_vamp_setup():
    """Create a VAMP SimpleSetup for Panda planning with sphere obstacles."""
    robot = vamp.panda
    dimension = robot.dimension()

    env = vamp.Environment()
    for center in SPHERES:
        env.add_sphere(vamp.Sphere(center, 0.2))

    space = VampStateSpace(robot=robot)
    si = ob.SpaceInformation(space)
    si.setMotionValidator(VampMotionValidator(si=si, env=env, robot=robot))
    si.setStateValidityChecker(VampStateValidityChecker(si=si, env=env, robot=robot))

    ss = og.SimpleSetup(si)

    start = si.allocState()
    start[0:dimension] = START
    goal = si.allocState()
    goal[0:dimension] = GOAL
    ss.setStartAndGoalStates(start, goal)

    return ss, si


def test_vamp_rrt_connect():
    ss, si = create_vamp_setup()
    planner = og.RRTConnect(si)
    ss.setPlanner(planner)

    result = ss.solve(5.0)
    assert result, "RRTConnect failed to find a solution"

    path = ss.getSolutionPath()
    assert path.getStateCount() >= 2


def test_vamp_rrt():
    ss, si = create_vamp_setup()
    planner = og.RRT(si)
    ss.setPlanner(planner)

    result = ss.solve(5.0)
    assert result, "RRT failed to find a solution"

    path = ss.getSolutionPath()
    assert path.getStateCount() >= 2


def test_vamp_state_space_bounds():
    robot = vamp.panda
    space = VampStateSpace(robot=robot)
    bounds = space.getBounds()
    dimension = robot.dimension()

    assert len(bounds.low) >= dimension
    assert len(bounds.high) >= dimension

    for i in range(dimension):
        assert bounds.low[i] < bounds.high[i], f"Invalid bounds at dim {i}"


if __name__ == "__main__":
    test_vamp_state_space_bounds()
    test_vamp_rrt_connect()
    test_vamp_rrt()
