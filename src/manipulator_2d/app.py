"""Interactive Matplotlib application for the planar 2R manipulator."""

from dataclasses import dataclass, replace

from matplotlib import pyplot as plt
from matplotlib.axes import Axes
from matplotlib.backend_bases import MouseButton, MouseEvent
from matplotlib.figure import Figure
from matplotlib.widgets import Slider

from manipulator_2d.kinematics import (
    Point2D,
    forward_kinematics,
    is_target_reachable,
)
from manipulator_2d.model import JointLimits, TwoLinkManipulator
from manipulator_2d.visualization import draw_manipulator


@dataclass(frozen=True)
class ManipulatorApp:
    """References that keep an interactive manipulator window responsive.

    Attributes:
        figure: Complete Matplotlib window.
        axes: Main coordinate system containing the manipulator drawing.
        joint_1_slider: Slider controlling q1 in radians.
        joint_2_slider: Slider controlling q2 in radians.
        target_click_connection_id: Matplotlib callback connection identifier.
    """

    figure: Figure
    axes: Axes
    joint_1_slider: Slider
    joint_2_slider: Slider
    target_click_connection_id: int


def create_manipulator_app(manipulator: TwoLinkManipulator) -> ManipulatorApp:
    """Create a manipulator figure with sliders for both joint angles."""
    _validate_slider_range("joint 1", manipulator.joint_1_limits)
    _validate_slider_range("joint 2", manipulator.joint_2_limits)

    figure, axes = plt.subplots(figsize=(7, 8))
    figure.subplots_adjust(bottom=0.22)
    draw_manipulator(manipulator, axes=axes)

    joint_1_slider_axes = figure.add_axes((0.20, 0.11, 0.65, 0.035))
    joint_2_slider_axes = figure.add_axes((0.20, 0.05, 0.65, 0.035))

    joint_1_slider = Slider(
        ax=joint_1_slider_axes,
        label="q1 (rad)",
        valmin=manipulator.joint_1_limits.minimum,
        valmax=manipulator.joint_1_limits.maximum,
        valinit=manipulator.joint_1_angle,
        valfmt="%1.2f rad",
        color="#137C8B",
    )
    joint_2_slider = Slider(
        ax=joint_2_slider_axes,
        label="q2 (rad)",
        valmin=manipulator.joint_2_limits.minimum,
        valmax=manipulator.joint_2_limits.maximum,
        valinit=manipulator.joint_2_angle,
        valfmt="%1.2f rad",
        color="#D1495B",
    )

    selected_target: Point2D | None = None
    trajectory: list[Point2D] = [forward_kinematics(manipulator)[1]]
    _draw_trajectory(axes, trajectory)

    def redraw(*, record_endpoint: bool) -> None:
        updated_manipulator = replace(
            manipulator,
            joint_1_angle=joint_1_slider.val,
            joint_2_angle=joint_2_slider.val,
        )
        if record_endpoint:
            endpoint = forward_kinematics(updated_manipulator)[1]
            _append_trajectory_point(trajectory, endpoint)

        axes.clear()
        draw_manipulator(updated_manipulator, axes=axes)
        _draw_trajectory(axes, trajectory)
        if selected_target is not None:
            _draw_target_status(axes, updated_manipulator, selected_target)
        figure.canvas.draw_idle()

    def update_manipulator(_value: float) -> None:
        redraw(record_endpoint=True)

    def select_target(event: MouseEvent) -> None:
        nonlocal selected_target
        if (
            event.button != MouseButton.LEFT
            or event.inaxes is not axes
            or event.xdata is None
            or event.ydata is None
        ):
            return

        selected_target = (
            _normalize_click_coordinate(float(event.xdata)),
            _normalize_click_coordinate(float(event.ydata)),
        )
        redraw(record_endpoint=False)

    joint_1_slider.on_changed(update_manipulator)
    joint_2_slider.on_changed(update_manipulator)
    target_click_connection_id = figure.canvas.mpl_connect("button_press_event", select_target)

    return ManipulatorApp(
        figure=figure,
        axes=axes,
        joint_1_slider=joint_1_slider,
        joint_2_slider=joint_2_slider,
        target_click_connection_id=target_click_connection_id,
    )


def _append_trajectory_point(
    trajectory: list[Point2D],
    endpoint: Point2D,
) -> None:
    """Append an endpoint unless it duplicates the latest recorded position."""
    if not trajectory or endpoint != trajectory[-1]:
        trajectory.append(endpoint)


def _draw_trajectory(axes: Axes, trajectory: list[Point2D]) -> None:
    """Draw the recorded end-effector path in world coordinates."""
    x_coordinates = [point[0] for point in trajectory]
    y_coordinates = [point[1] for point in trajectory]
    axes.plot(
        x_coordinates,
        y_coordinates,
        color="#3A86FF",
        linewidth=2,
        alpha=0.8,
        label="End-effector path",
        zorder=1,
    )
    axes.legend(loc="upper right")


def _draw_target_status(
    axes: Axes,
    manipulator: TwoLinkManipulator,
    target: Point2D,
) -> None:
    """Draw one selected target and its geometric reachability status."""
    target_x, target_y = target
    reachable = is_target_reachable(manipulator, target)
    status = "reachable" if reachable else "unreachable"
    color = "#2E8B57" if reachable else "#C0392B"
    marker = "o" if reachable else "x"

    axes.scatter(target_x, target_y, color=color, marker=marker, s=130, zorder=5)
    axes.text(
        0.02,
        0.98,
        f"Target ({target_x:.2f}, {target_y:.2f}): {status}",
        transform=axes.transAxes,
        horizontalalignment="left",
        verticalalignment="top",
        color=color,
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "edgecolor": color},
        zorder=6,
    )


def _validate_slider_range(joint_name: str, limits: JointLimits) -> None:
    """Reject a locked joint because a Slider needs a non-empty range."""
    if limits.minimum == limits.maximum:
        raise ValueError(f"{joint_name} slider requires minimum to be less than maximum")


def _normalize_click_coordinate(value: float) -> float:
    """Convert tiny coordinate-transform noise around zero to exactly zero."""
    return 0.0 if abs(value) < 1e-12 else value
