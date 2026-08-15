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


def test_clicking_target_applies_default_elbow_down_solution() -> None:
    """A reachable target should instantly apply the default elbow-down pose."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=1.0,
        link_2_length=1.0,
        joint_1_angle=0.0,
        joint_2_angle=0.0,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )
    app = create_manipulator_app(manipulator)

    try:
        click_main_axes(app, (1.0, 1.0))

        target_marker = app.axes.collections[-1]
        link_line = app.axes.lines[0]
        trajectory_line = app.axes.lines[1]
        assert app.ik_branch_selector.value_selected == "Elbow down"
        assert app.joint_1_slider.val == pytest.approx(0.0)
        assert app.joint_2_slider.val == pytest.approx(pi / 2)
        assert tuple(link_line.get_xdata())[-1] == pytest.approx(1.0)
        assert tuple(link_line.get_ydata())[-1] == pytest.approx(1.0)
        assert tuple(target_marker.get_offsets()[0]) == pytest.approx((1.0, 1.0))
        assert app.axes.texts[-1].get_text() == ("Target (1.00, 1.00)\nreached with elbow down")
        assert len(trajectory_line.get_xdata()) == 1
    finally:
        plt.close(app.figure)


def test_selecting_elbow_up_re_solves_the_current_target() -> None:
    """Changing the radio selection should apply the other IK solution."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=1.0,
        link_2_length=1.0,
        joint_1_angle=0.0,
        joint_2_angle=0.0,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )
    app = create_manipulator_app(manipulator)

    try:
        click_main_axes(app, (1.0, 1.0))
        app.ik_branch_selector.set_active(1)

        link_line = app.axes.lines[0]
        trajectory_line = app.axes.lines[1]
        assert app.ik_branch_selector.value_selected == "Elbow up"
        assert app.joint_1_slider.val == pytest.approx(pi / 2)
        assert app.joint_2_slider.val == pytest.approx(-pi / 2)
        assert tuple(link_line.get_xdata())[-1] == pytest.approx(1.0)
        assert tuple(link_line.get_ydata())[-1] == pytest.approx(1.0)
        assert app.axes.texts[-1].get_text() == ("Target (1.00, 1.00)\nreached with elbow up")
        assert len(trajectory_line.get_xdata()) == 1
    finally:
        plt.close(app.figure)


def test_unreachable_target_keeps_the_current_pose() -> None:
    """A geometrically unreachable click should not change either joint."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=2.0,
        link_2_length=1.0,
        joint_1_angle=pi / 4,
        joint_2_angle=-pi / 4,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )
    app = create_manipulator_app(manipulator)

    try:
        click_main_axes(app, (0.0, 0.0))

        assert app.joint_1_slider.val == pytest.approx(pi / 4)
        assert app.joint_2_slider.val == pytest.approx(-pi / 4)
        assert app.axes.texts[-1].get_text() == "Target (0.00, 0.00)\nunreachable"
        assert len(app.axes.lines[1].get_xdata()) == 1
    finally:
        plt.close(app.figure)


def test_target_blocked_by_joint_limits_keeps_the_current_pose() -> None:
    """A geometric target with no limit-valid solution should be reported."""
    narrow_limits = JointLimits(minimum=-0.1, maximum=0.1)
    manipulator = TwoLinkManipulator(
        link_1_length=1.0,
        link_2_length=1.0,
        joint_1_angle=0.0,
        joint_2_angle=0.0,
        joint_1_limits=narrow_limits,
        joint_2_limits=narrow_limits,
    )
    app = create_manipulator_app(manipulator)

    try:
        click_main_axes(app, (1.0, 1.0))

        assert app.joint_1_slider.val == pytest.approx(0.0)
        assert app.joint_2_slider.val == pytest.approx(0.0)
        assert app.axes.texts[-1].get_text() == ("Target (1.00, 1.00)\nblocked by joint limits")
    finally:
        plt.close(app.figure)


def test_unavailable_branch_can_be_replaced_by_a_valid_branch() -> None:
    """The app should distinguish one missing branch from no valid solution."""
    joint_1_limits = JointLimits(minimum=-0.1, maximum=0.1)
    joint_2_limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=1.0,
        link_2_length=1.0,
        joint_1_angle=0.0,
        joint_2_angle=0.0,
        joint_1_limits=joint_1_limits,
        joint_2_limits=joint_2_limits,
    )
    app = create_manipulator_app(manipulator)

    try:
        app.ik_branch_selector.set_active(1)
        click_main_axes(app, (1.0, 1.0))

        assert app.joint_1_slider.val == pytest.approx(0.0)
        assert app.joint_2_slider.val == pytest.approx(0.0)
        assert app.axes.texts[-1].get_text() == ("Target (1.00, 1.00)\nelbow up unavailable")

        app.ik_branch_selector.set_active(0)
        assert app.joint_1_slider.val == pytest.approx(0.0)
        assert app.joint_2_slider.val == pytest.approx(pi / 2)
        assert app.axes.texts[-1].get_text() == ("Target (1.00, 1.00)\nreached with elbow down")
    finally:
        plt.close(app.figure)


def test_singular_target_uses_its_unique_solution() -> None:
    """At a singular target the selector should not invent a second posture."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=1.0,
        link_2_length=1.0,
        joint_1_angle=pi / 4,
        joint_2_angle=-pi / 4,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )
    app = create_manipulator_app(manipulator)

    try:
        app.ik_branch_selector.set_active(1)
        click_main_axes(app, (2.0, 0.0))

        assert app.joint_1_slider.val == pytest.approx(0.0)
        assert app.joint_2_slider.val == pytest.approx(0.0)
        assert app.axes.texts[-1].get_text() == ("Target (2.00, 0.00)\nreached at singular pose")
        assert len(app.axes.lines[1].get_xdata()) == 1
    finally:
        plt.close(app.figure)


def test_manual_slider_change_keeps_target_but_clears_reached_status() -> None:
    """A later manual move must not claim that the old target is still reached."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=1.0,
        link_2_length=1.0,
        joint_1_angle=0.0,
        joint_2_angle=0.0,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )
    app = create_manipulator_app(manipulator)

    try:
        click_main_axes(app, (1.0, 1.0))
        app.joint_1_slider.set_val(pi / 4)

        assert tuple(app.axes.collections[-1].get_offsets()[0]) == pytest.approx((1.0, 1.0))
        assert app.axes.texts[-1].get_text() == ("Target (1.00, 1.00)\nselected; manual pose")
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
