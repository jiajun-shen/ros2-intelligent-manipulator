"""Tests for planar two-link forward and inverse kinematics."""

from dataclasses import replace
from math import pi

import pytest

from manipulator_2d.kinematics import (
    forward_kinematics,
    inverse_kinematics,
    normalize_angle,
)
from manipulator_2d.model import JointLimits, TwoLinkManipulator


def create_unit_link_manipulator(
    joint_1_angle: float = 0.0,
    joint_2_angle: float = 0.0,
) -> TwoLinkManipulator:
    """Create a full-rotation manipulator with two one-metre links."""
    joint_limits = JointLimits(minimum=-pi, maximum=pi)
    return TwoLinkManipulator(
        link_1_length=1.0,
        link_2_length=1.0,
        joint_1_angle=joint_1_angle,
        joint_2_angle=joint_2_angle,
        joint_1_limits=joint_limits,
        joint_2_limits=joint_limits,
    )


@pytest.mark.parametrize(
    (
        "joint_1_angle",
        "joint_2_angle",
        "expected_joint_2_position",
        "expected_end_effector_position",
    ),
    [
        (0.0, 0.0, (2.0, 0.0), (3.0, 0.0)),
        (pi / 2, 0.0, (0.0, 2.0), (0.0, 3.0)),
        (0.0, pi / 2, (2.0, 0.0), (2.0, 1.0)),
        (pi / 2, -pi / 2, (0.0, 2.0), (1.0, 2.0)),
        (pi, 0.0, (-2.0, 0.0), (-3.0, 0.0)),
    ],
)
def test_forward_kinematics_for_known_configurations(
    joint_1_angle: float,
    joint_2_angle: float,
    expected_joint_2_position: tuple[float, float],
    expected_end_effector_position: tuple[float, float],
) -> None:
    """Known joint configurations should produce known Cartesian positions."""
    joint_limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=2.0,
        link_2_length=1.0,
        joint_1_angle=joint_1_angle,
        joint_2_angle=joint_2_angle,
        joint_1_limits=joint_limits,
        joint_2_limits=joint_limits,
    )

    joint_2_position, end_effector_position = forward_kinematics(manipulator)

    assert joint_2_position == pytest.approx(expected_joint_2_position)
    assert end_effector_position == pytest.approx(expected_end_effector_position)


def test_inverse_kinematics_returns_two_named_solutions() -> None:
    """The target (1, 1) should have known elbow-down and elbow-up solutions."""
    manipulator = create_unit_link_manipulator()

    elbow_down, elbow_up = inverse_kinematics(manipulator, (1.0, 1.0))

    assert elbow_down.branch == "elbow_down"
    assert not elbow_down.is_singular
    assert (elbow_down.joint_1_angle, elbow_down.joint_2_angle) == pytest.approx((0.0, pi / 2))
    assert elbow_up.branch == "elbow_up"
    assert not elbow_up.is_singular
    assert (elbow_up.joint_1_angle, elbow_up.joint_2_angle) == pytest.approx((pi / 2, -pi / 2))


@pytest.mark.parametrize(
    "target",
    [
        (1.0, 1.0),
        (-1.0, 1.0),
        (-1.0, -1.0),
        (1.0, -1.0),
    ],
)
def test_inverse_kinematics_solutions_reach_targets_in_every_quadrant(
    target: tuple[float, float],
) -> None:
    """Both analytic branches should return to the target through forward kinematics."""
    manipulator = create_unit_link_manipulator()

    solutions = inverse_kinematics(manipulator, target)

    for solution in solutions:
        solved_manipulator = replace(
            manipulator,
            joint_1_angle=solution.joint_1_angle,
            joint_2_angle=solution.joint_2_angle,
        )
        _, solved_target = forward_kinematics(solved_manipulator)
        assert solved_target == pytest.approx(target)


def test_inverse_kinematics_rejects_an_unreachable_target() -> None:
    """A target outside the outer workspace boundary has no real solution."""
    manipulator = create_unit_link_manipulator()

    with pytest.raises(ValueError, match="outside the ideal geometric workspace"):
        inverse_kinematics(manipulator, (3.0, 0.0))


def test_inverse_kinematics_does_not_modify_the_manipulator() -> None:
    """Solving a target should leave the input configuration unchanged."""
    manipulator = create_unit_link_manipulator(
        joint_1_angle=pi / 4,
        joint_2_angle=-pi / 4,
    )

    inverse_kinematics(manipulator, (1.0, 1.0))

    assert manipulator.joint_1_angle == pi / 4
    assert manipulator.joint_2_angle == -pi / 4


@pytest.mark.parametrize(
    ("angle", "expected_angle"),
    [
        (0.0, 0.0),
        (2 * pi, 0.0),
        (-2 * pi, 0.0),
        (3 * pi / 2, -pi / 2),
        (-3 * pi / 2, pi / 2),
        (pi, -pi),
    ],
)
def test_normalize_angle_uses_the_standard_interval(
    angle: float,
    expected_angle: float,
) -> None:
    """Equivalent angles should be represented inside [-pi, pi)."""
    assert normalize_angle(angle) == pytest.approx(expected_angle)


def test_inverse_kinematics_merges_a_singular_configuration() -> None:
    """The two branches coincide when the arm is completely extended."""
    manipulator = create_unit_link_manipulator()

    solutions = inverse_kinematics(manipulator, (2.0, 0.0))

    assert len(solutions) == 1
    singular_solution = solutions[0]
    assert singular_solution.branch == "singular"
    assert singular_solution.is_singular
    assert (
        singular_solution.joint_1_angle,
        singular_solution.joint_2_angle,
    ) == pytest.approx((0.0, 0.0))


@pytest.mark.parametrize(
    (
        "joint_1_limit_values",
        "joint_2_limit_values",
        "initial_angles",
        "expected_branches",
    ),
    [
        ((-0.1, 0.1), (-pi, pi), (0.0, 0.0), ("elbow_down",)),
        ((1.0, 2.0), (-2.0, -1.0), (1.5, -1.5), ("elbow_up",)),
        ((-0.1, 0.1), (-0.1, 0.1), (0.0, 0.0), ()),
    ],
)
def test_inverse_kinematics_filters_solutions_by_joint_limits(
    joint_1_limit_values: tuple[float, float],
    joint_2_limit_values: tuple[float, float],
    initial_angles: tuple[float, float],
    expected_branches: tuple[str, ...],
) -> None:
    """Only solutions satisfying both joint limits should be returned."""
    joint_1_limits = JointLimits(*joint_1_limit_values)
    joint_2_limits = JointLimits(*joint_2_limit_values)
    manipulator = TwoLinkManipulator(
        link_1_length=1.0,
        link_2_length=1.0,
        joint_1_angle=initial_angles[0],
        joint_2_angle=initial_angles[1],
        joint_1_limits=joint_1_limits,
        joint_2_limits=joint_2_limits,
    )

    solutions = inverse_kinematics(manipulator, (1.0, 1.0))

    assert tuple(solution.branch for solution in solutions) == expected_branches
    for solution in solutions:
        assert joint_1_limits.contains(solution.joint_1_angle)
        assert joint_2_limits.contains(solution.joint_2_angle)
