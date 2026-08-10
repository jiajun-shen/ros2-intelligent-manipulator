"""Tests for the static planar manipulator visualization."""

from math import pi

import pytest
from matplotlib import pyplot as plt

from manipulator_2d.model import JointLimits, TwoLinkManipulator
from manipulator_2d.visualization import draw_manipulator


def test_draw_manipulator_uses_forward_kinematics_positions() -> None:
    """The plotted link coordinates should match a known configuration."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=2.0,
        link_2_length=1.0,
        joint_1_angle=0.0,
        joint_2_angle=pi / 2,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )

    figure, axes = draw_manipulator(manipulator)

    try:
        link_line = axes.lines[0]
        assert tuple(link_line.get_xdata()) == pytest.approx((0.0, 2.0, 2.0))
        assert tuple(link_line.get_ydata()) == pytest.approx((0.0, 0.0, 1.0))
        assert len(axes.collections) == 3
        assert axes.get_xlabel() == "x (m)"
        assert axes.get_ylabel() == "y (m)"
        assert axes.get_title() == "Planar 2R Manipulator"
    finally:
        plt.close(figure)
