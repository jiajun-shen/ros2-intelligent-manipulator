"""Tests for the ideal geometric workspace of a planar 2R manipulator."""

from math import pi

import pytest

from manipulator_2d.kinematics import is_target_reachable, workspace_radius_limits
from manipulator_2d.model import JointLimits, TwoLinkManipulator


def create_manipulator(
    link_1_length: float = 2.0, link_2_length: float = 1.0
) -> TwoLinkManipulator:
    """Create a full-rotation manipulator for workspace tests."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    return TwoLinkManipulator(
        link_1_length=link_1_length,
        link_2_length=link_2_length,
        joint_1_angle=0.0,
        joint_2_angle=0.0,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )


def test_workspace_radius_limits_follow_link_lengths() -> None:
    """The annular workspace should use the folded and extended lengths."""
    manipulator = create_manipulator()

    assert workspace_radius_limits(manipulator) == pytest.approx((1.0, 3.0))


@pytest.mark.parametrize(
    ("target", "expected_reachable"),
    [
        ((1.0, 0.0), True),
        ((3.0, 0.0), True),
        ((0.0, 2.0), True),
        ((-2.0, 0.0), True),
        ((0.0, 0.0), False),
        ((0.9, 0.0), False),
        ((3.1, 0.0), False),
    ],
)
def test_target_reachability_in_annular_workspace(
    target: tuple[float, float],
    expected_reachable: bool,
) -> None:
    """Targets on both boundaries are valid; targets outside are invalid."""
    manipulator = create_manipulator()

    assert is_target_reachable(manipulator, target) is expected_reachable


def test_origin_is_reachable_when_link_lengths_are_equal() -> None:
    """Equal links can fold completely, so the workspace has no inner hole."""
    manipulator = create_manipulator(link_1_length=1.0, link_2_length=1.0)

    assert workspace_radius_limits(manipulator) == pytest.approx((0.0, 2.0))
    assert is_target_reachable(manipulator, (0.0, 0.0))
