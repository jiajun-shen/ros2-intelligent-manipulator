"""Tests for the interactive planar manipulator application."""

from math import pi

import pytest
from matplotlib import pyplot as plt
from matplotlib.backend_bases import MouseButton, MouseEvent

from manipulator_2d.app import ManipulatorApp, create_manipulator_app
from manipulator_2d.model import JointLimits, TwoLinkManipulator


def click_main_axes(app: ManipulatorApp, target: tuple[float, float]) -> None:
    """Send a left-click event to a target expressed in data coordinates."""
    app.figure.canvas.draw()
    pixel_x, pixel_y = app.axes.transData.transform(target)
    event = MouseEvent(
        "button_press_event",
        app.figure.canvas,
        pixel_x,
        pixel_y,
        button=MouseButton.LEFT,
    )
    app.figure.canvas.callbacks.process("button_press_event", event)


def test_joint_sliders_update_the_manipulator_drawing() -> None:
    """Changing either slider should redraw the links at the new positions."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=2.0,
        link_2_length=1.0,
        joint_1_angle=0.0,
        joint_2_angle=0.0,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )
    app = create_manipulator_app(manipulator)

    try:
        app.joint_2_slider.set_val(pi / 2)
        link_line = app.axes.lines[0]
        assert tuple(link_line.get_xdata()) == pytest.approx((0.0, 2.0, 2.0))
        assert tuple(link_line.get_ydata()) == pytest.approx((0.0, 0.0, 1.0))

        app.joint_1_slider.set_val(pi / 2)
        link_line = app.axes.lines[0]
        assert tuple(link_line.get_xdata()) == pytest.approx((0.0, 0.0, -1.0))
        assert tuple(link_line.get_ydata()) == pytest.approx((0.0, 2.0, 2.0))

        assert manipulator.joint_1_angle == 0.0
        assert manipulator.joint_2_angle == 0.0
    finally:
        plt.close(app.figure)


def test_joint_sliders_record_end_effector_trajectory() -> None:
    """Each new slider pose should add its endpoint to the displayed path."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=2.0,
        link_2_length=1.0,
        joint_1_angle=0.0,
        joint_2_angle=0.0,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )
    app = create_manipulator_app(manipulator)

    try:
        app.joint_2_slider.set_val(pi / 2)
        app.joint_1_slider.set_val(pi / 2)

        trajectory_line = app.axes.lines[1]
        assert trajectory_line.get_label() == "End-effector path"
        assert tuple(trajectory_line.get_xdata()) == pytest.approx((3.0, 2.0, -1.0))
        assert tuple(trajectory_line.get_ydata()) == pytest.approx((0.0, 1.0, 2.0))
    finally:
        plt.close(app.figure)


def test_repeated_slider_value_does_not_duplicate_trajectory_point() -> None:
    """A redraw at the same pose should not add a zero-length path section."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=2.0,
        link_2_length=1.0,
        joint_1_angle=0.0,
        joint_2_angle=0.0,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )
    app = create_manipulator_app(manipulator)

    try:
        app.joint_1_slider.set_val(0.0)

        trajectory_line = app.axes.lines[1]
        assert len(trajectory_line.get_xdata()) == 1
        assert len(trajectory_line.get_ydata()) == 1
    finally:
        plt.close(app.figure)


def test_app_rejects_a_joint_without_slider_range() -> None:
    """A locked joint cannot be represented by a movable Slider."""
    locked_limits = JointLimits(minimum=0.0, maximum=0.0)
    movable_limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=2.0,
        link_2_length=1.0,
        joint_1_angle=0.0,
        joint_2_angle=0.0,
        joint_1_limits=locked_limits,
        joint_2_limits=movable_limits,
    )

    with pytest.raises(ValueError, match="joint 1 slider"):
        create_manipulator_app(manipulator)


@pytest.mark.parametrize(
    ("target", "expected_status"),
    [
        ((2.0, 0.0), "Target (2.00, 0.00): reachable"),
        ((0.0, 0.0), "Target (0.00, 0.00): unreachable"),
    ],
)
def test_clicking_target_displays_reachability(
    target: tuple[float, float],
    expected_status: str,
) -> None:
    """A main-axes click should display the target and its status."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=2.0,
        link_2_length=1.0,
        joint_1_angle=0.0,
        joint_2_angle=0.0,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )
    app = create_manipulator_app(manipulator)

    try:
        click_main_axes(app, target)

        target_marker = app.axes.collections[-1]
        trajectory_line = app.axes.lines[1]
        assert tuple(target_marker.get_offsets()[0]) == pytest.approx(target)
        assert app.axes.texts[-1].get_text() == expected_status
        assert len(trajectory_line.get_xdata()) == 1

        app.joint_1_slider.set_val(pi / 2)
        assert tuple(app.axes.collections[-1].get_offsets()[0]) == pytest.approx(target)
        assert app.axes.texts[-1].get_text() == expected_status
        assert len(app.axes.lines[1].get_xdata()) == 2
    finally:
        plt.close(app.figure)


def test_click_outside_main_axes_is_ignored() -> None:
    """Clicks outside the main plot should not create a target marker."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=2.0,
        link_2_length=1.0,
        joint_1_angle=0.0,
        joint_2_angle=0.0,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )
    app = create_manipulator_app(manipulator)

    try:
        event = MouseEvent(
            "button_press_event",
            app.figure.canvas,
            -10,
            -10,
            button=MouseButton.LEFT,
        )
        app.figure.canvas.callbacks.process("button_press_event", event)

        assert len(app.axes.collections) == 3
        assert not app.axes.texts
    finally:
        plt.close(app.figure)
