"""Tests for the planar two-link manipulator data model."""

from math import pi

import pytest

from manipulator_2d.model import JointLimits, TwoLinkManipulator


def test_joint_limits_include_both_boundaries() -> None:
    """Angles equal to either limit should be accepted."""
    limits = JointLimits(minimum=-pi / 2, maximum=pi / 2)

    assert limits.contains(-pi / 2)
    assert limits.contains(0.0)
    assert limits.contains(pi / 2)
    assert not limits.contains(pi)


def test_joint_limits_reject_reversed_range() -> None:
    """The minimum limit cannot be greater than the maximum limit."""
    with pytest.raises(ValueError, match="minimum joint limit"):
        JointLimits(minimum=pi / 2, maximum=-pi / 2)


def test_two_link_manipulator_can_be_created() -> None:
    """A valid geometry and configuration should create a model."""
    joint_1_limits = JointLimits(minimum=-pi, maximum=pi)
    joint_2_limits = JointLimits(minimum=-pi / 2, maximum=pi / 2)

    manipulator = TwoLinkManipulator(
        link_1_length=1.0,
        link_2_length=0.8,
        joint_1_angle=pi / 4,
        joint_2_angle=-pi / 6,
        joint_1_limits=joint_1_limits,
        joint_2_limits=joint_2_limits,
    )

    assert manipulator.link_1_length == 1.0
    assert manipulator.link_2_length == 0.8
    assert manipulator.joint_1_angle == pi / 4
    assert manipulator.joint_2_angle == -pi / 6


@pytest.mark.parametrize(
    ("link_1_length", "link_2_length"),
    [
        (0.0, 1.0),
        (-1.0, 1.0),
        (1.0, 0.0),
        (1.0, -1.0),
    ],
)
def test_two_link_manipulator_rejects_non_positive_link_lengths(
    link_1_length: float,
    link_2_length: float,
) -> None:
    """Both physical link lengths must be greater than zero."""
    limits = JointLimits(minimum=-pi, maximum=pi)

    with pytest.raises(ValueError, match="must be greater than zero"):
        TwoLinkManipulator(
            link_1_length=link_1_length,
            link_2_length=link_2_length,
            joint_1_angle=0.0,
            joint_2_angle=0.0,
            joint_1_limits=limits,
            joint_2_limits=limits,
        )


@pytest.mark.parametrize(
    ("joint_1_angle", "joint_2_angle", "error_message"),
    [
        (-pi - 0.1, 0.0, "joint_1_angle"),
        (pi + 0.1, 0.0, "joint_1_angle"),
        (0.0, -pi / 2 - 0.1, "joint_2_angle"),
        (0.0, pi / 2 + 0.1, "joint_2_angle"),
    ],
)
def test_two_link_manipulator_rejects_angles_outside_limits(
    joint_1_angle: float,
    joint_2_angle: float,
    error_message: str,
) -> None:
    """An initial joint angle outside its limits should be rejected."""
    with pytest.raises(ValueError, match=error_message):
        TwoLinkManipulator(
            link_1_length=1.0,
            link_2_length=0.8,
            joint_1_angle=joint_1_angle,
            joint_2_angle=joint_2_angle,
            joint_1_limits=JointLimits(minimum=-pi, maximum=pi),
            joint_2_limits=JointLimits(minimum=-pi / 2, maximum=pi / 2),
        )


@pytest.mark.parametrize(
    ("joint_1_angle", "joint_2_angle"),
    [
        (-pi, -pi / 2),
        (pi, pi / 2),
    ],
)
def test_two_link_manipulator_accepts_angles_on_boundaries(
    joint_1_angle: float,
    joint_2_angle: float,
) -> None:
    """Initial angles exactly on the joint limits should be accepted."""
    manipulator = TwoLinkManipulator(
        link_1_length=1.0,
        link_2_length=0.8,
        joint_1_angle=joint_1_angle,
        joint_2_angle=joint_2_angle,
        joint_1_limits=JointLimits(minimum=-pi, maximum=pi),
        joint_2_limits=JointLimits(minimum=-pi / 2, maximum=pi / 2),
    )

    assert manipulator.joint_1_angle == joint_1_angle
    assert manipulator.joint_2_angle == joint_2_angle
