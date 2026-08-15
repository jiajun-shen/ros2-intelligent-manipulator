"""Interactive Matplotlib application for the planar 2R manipulator."""

from dataclasses import dataclass, replace

from matplotlib import pyplot as plt
from matplotlib.axes import Axes
from matplotlib.backend_bases import MouseButton, MouseEvent
from matplotlib.figure import Figure
from matplotlib.widgets import RadioButtons, Slider

from manipulator_2d.kinematics import (
    InverseKinematicsSolution,
    Point2D,
    SolutionBranch,
    forward_kinematics,
    inverse_kinematics,
)
from manipulator_2d.model import JointLimits, TwoLinkManipulator
from manipulator_2d.visualization import draw_manipulator

_BRANCH_BY_LABEL: dict[str, SolutionBranch] = {
    "Elbow down": "elbow_down",
    "Elbow up": "elbow_up",
}


@dataclass(frozen=True)
class _TargetStatus:
    """Visual result of trying to move the manipulator to one target."""

    message: str
    color: str
    marker: str


@dataclass(frozen=True)
class ManipulatorApp:
    """References that keep an interactive manipulator window responsive.

    Attributes:
        figure: Complete Matplotlib window.
        axes: Main coordinate system containing the manipulator drawing.
        joint_1_slider: Slider controlling q1 in radians.
        joint_2_slider: Slider controlling q2 in radians.
        ik_branch_selector: Radio buttons selecting elbow-down or elbow-up IK.
        target_click_connection_id: Matplotlib callback connection identifier.
    """

    figure: Figure
    axes: Axes
    joint_1_slider: Slider
    joint_2_slider: Slider
    ik_branch_selector: RadioButtons
    target_click_connection_id: int


def create_manipulator_app(manipulator: TwoLinkManipulator) -> ManipulatorApp:
    """Create a manipulator figure with sliders for both joint angles."""
    _validate_slider_range("joint 1", manipulator.joint_1_limits)
    _validate_slider_range("joint 2", manipulator.joint_2_limits)

    figure, axes = plt.subplots(figsize=(7, 8))
    figure.subplots_adjust(bottom=0.27)
    draw_manipulator(manipulator, axes=axes)

    branch_selector_axes = figure.add_axes((0.04, 0.04, 0.20, 0.15))
    branch_selector_axes.set_title("IK branch", fontsize=10)
    joint_1_slider_axes = figure.add_axes((0.34, 0.13, 0.50, 0.035))
    joint_2_slider_axes = figure.add_axes((0.34, 0.07, 0.50, 0.035))

    ik_branch_selector = RadioButtons(
        ax=branch_selector_axes,
        labels=tuple(_BRANCH_BY_LABEL),
        active=0,
        activecolor="#3A86FF",
    )

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
    selected_target_status: _TargetStatus | None = None
    selected_branch: SolutionBranch = "elbow_down"
    trajectory: list[Point2D] = [forward_kinematics(manipulator)[1]]
    _draw_trajectory(axes, trajectory)

    def current_manipulator() -> TwoLinkManipulator:
        """Build the configuration currently displayed by the two sliders."""
        return replace(
            manipulator,
            joint_1_angle=joint_1_slider.val,
            joint_2_angle=joint_2_slider.val,
        )

    def redraw(*, record_endpoint: bool) -> None:
        updated_manipulator = current_manipulator()
        if record_endpoint:
            endpoint = forward_kinematics(updated_manipulator)[1]
            _append_trajectory_point(trajectory, endpoint)

        axes.clear()
        draw_manipulator(updated_manipulator, axes=axes)
        _draw_trajectory(axes, trajectory)
        if selected_target is not None and selected_target_status is not None:
            _draw_target_status(axes, selected_target, selected_target_status)
        figure.canvas.draw_idle()

    def update_manipulator(_value: float) -> None:
        nonlocal selected_target_status
        if selected_target is not None:
            selected_target_status = _TargetStatus(
                message="selected; manual pose",
                color="#5D6D7E",
                marker="+",
            )
        redraw(record_endpoint=True)

    def move_to_selected_target() -> None:
        """Solve the selected target and instantly apply the chosen branch."""
        nonlocal selected_target_status
        if selected_target is None:
            return

        try:
            solutions = inverse_kinematics(current_manipulator(), selected_target)
        except ValueError:
            selected_target_status = _TargetStatus(
                message="unreachable",
                color="#C0392B",
                marker="x",
            )
            redraw(record_endpoint=False)
            return

        if not solutions:
            selected_target_status = _TargetStatus(
                message="blocked by joint limits",
                color="#B26A00",
                marker="x",
            )
            redraw(record_endpoint=False)
            return

        solution = _select_inverse_kinematics_solution(solutions, selected_branch)
        if solution is None:
            branch_label = selected_branch.replace("_", " ")
            selected_target_status = _TargetStatus(
                message=f"{branch_label} unavailable",
                color="#B26A00",
                marker="x",
            )
            redraw(record_endpoint=False)
            return

        _set_slider_values_without_callbacks(
            joint_1_slider,
            joint_2_slider,
            solution,
        )
        if solution.branch == "singular":
            status_message = "reached at singular pose"
        else:
            branch_label = solution.branch.replace("_", " ")
            status_message = f"reached with {branch_label}"
        selected_target_status = _TargetStatus(
            message=status_message,
            color="#2E8B57",
            marker="o",
        )
        redraw(record_endpoint=False)

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
        move_to_selected_target()

    def select_branch(label: str) -> None:
        nonlocal selected_branch
        selected_branch = _BRANCH_BY_LABEL[label]
        move_to_selected_target()

    joint_1_slider.on_changed(update_manipulator)
    joint_2_slider.on_changed(update_manipulator)
    ik_branch_selector.on_clicked(select_branch)
    target_click_connection_id = figure.canvas.mpl_connect("button_press_event", select_target)

    return ManipulatorApp(
        figure=figure,
        axes=axes,
        joint_1_slider=joint_1_slider,
        joint_2_slider=joint_2_slider,
        ik_branch_selector=ik_branch_selector,
        target_click_connection_id=target_click_connection_id,
    )


def _select_inverse_kinematics_solution(
    solutions: tuple[InverseKinematicsSolution, ...],
    selected_branch: SolutionBranch,
) -> InverseKinematicsSolution | None:
    """Choose the requested regular branch, or the unique singular solution."""
    for solution in solutions:
        if solution.branch == "singular":
            return solution
        if solution.branch == selected_branch:
            return solution
    return None


def _set_slider_values_without_callbacks(
    joint_1_slider: Slider,
    joint_2_slider: Slider,
    solution: InverseKinematicsSolution,
) -> None:
    """Apply an IK solution without treating the instant jump as a trajectory."""
    sliders = (joint_1_slider, joint_2_slider)
    previous_event_states = tuple(slider.eventson for slider in sliders)
    previous_draw_states = tuple(slider.drawon for slider in sliders)

    try:
        for slider in sliders:
            slider.eventson = False
            slider.drawon = False
        joint_1_slider.set_val(solution.joint_1_angle)
        joint_2_slider.set_val(solution.joint_2_angle)
    finally:
        for slider, eventson, drawon in zip(
            sliders,
            previous_event_states,
            previous_draw_states,
            strict=True,
        ):
            slider.eventson = eventson
            slider.drawon = drawon


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
    target: Point2D,
    status: _TargetStatus,
) -> None:
    """Draw one selected target and the result of the latest IK attempt."""
    target_x, target_y = target

    axes.scatter(
        target_x,
        target_y,
        color=status.color,
        marker=status.marker,
        s=130,
        zorder=5,
    )
    axes.text(
        0.02,
        0.98,
        f"Target ({target_x:.2f}, {target_y:.2f})\n{status.message}",
        transform=axes.transAxes,
        horizontalalignment="left",
        verticalalignment="top",
        color=status.color,
        bbox={
            "boxstyle": "round,pad=0.3",
            "facecolor": "white",
            "edgecolor": status.color,
        },
        zorder=6,
    )


def _validate_slider_range(joint_name: str, limits: JointLimits) -> None:
    """Reject a locked joint because a Slider needs a non-empty range."""
    if limits.minimum == limits.maximum:
        raise ValueError(f"{joint_name} slider requires minimum to be less than maximum")


def _normalize_click_coordinate(value: float) -> float:
    """Convert tiny coordinate-transform noise around zero to exactly zero."""
    return 0.0 if abs(value) < 1e-12 else value
