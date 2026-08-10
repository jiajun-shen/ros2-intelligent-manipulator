"""Tests for planar two-link forward kinematics."""

from math import pi

import pytest

from manipulator_2d.kinematics import forward_kinematics
from manipulator_2d.model import JointLimits, TwoLinkManipulator


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
