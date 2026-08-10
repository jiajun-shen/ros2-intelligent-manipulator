"""Static Matplotlib visualization for a planar two-link manipulator."""

from matplotlib import pyplot as plt
from matplotlib.axes import Axes
from matplotlib.figure import Figure

from manipulator_2d.kinematics import forward_kinematics
from manipulator_2d.model import TwoLinkManipulator


def draw_manipulator(
    manipulator: TwoLinkManipulator,
    axes: Axes | None = None,
) -> tuple[Figure, Axes]:
    """Draw a planar 2R manipulator in the fixed base coordinate frame.

    Args:
        manipulator: A validated planar 2R manipulator configuration.
        axes: Optional existing Matplotlib axes on which to draw.

    Returns:
        The Matplotlib figure and axes containing the static drawing.
    """
    created_axes = axes is None
    if created_axes:
        figure, axes = plt.subplots(figsize=(7, 7))
    else:
        figure = axes.figure

    joint_2_position, end_effector_position = forward_kinematics(manipulator)
    joint_2_x, joint_2_y = joint_2_position
    end_effector_x, end_effector_y = end_effector_position

    axes.plot(
        [0.0, joint_2_x, end_effector_x],
        [0.0, joint_2_y, end_effector_y],
        color="#273043",
        linewidth=4,
        zorder=2,
    )
    axes.scatter(
        0.0,
        0.0,
        color="#137C8B",
        marker="s",
        s=110,
        label="Base / Joint 1",
        zorder=3,
    )
    axes.scatter(
        joint_2_x,
        joint_2_y,
        color="#D1495B",
        s=100,
        label="Joint 2",
        zorder=3,
    )
    axes.scatter(
        end_effector_x,
        end_effector_y,
        color="#F4B942",
        edgecolor="#273043",
        marker="*",
        s=190,
        label="End effector",
        zorder=4,
    )

    maximum_reach = manipulator.link_1_length + manipulator.link_2_length
    plot_limit = maximum_reach * 1.15
    axes.set_xlim(-plot_limit, plot_limit)
    axes.set_ylim(-plot_limit, plot_limit)
    axes.set_aspect("equal", adjustable="box")
    axes.set_xlabel("x (m)")
    axes.set_ylabel("y (m)")
    axes.set_title("Planar 2R Manipulator")
    axes.grid(visible=True, linestyle="--", alpha=0.35)
    axes.legend(loc="upper right")
    if created_axes:
        figure.tight_layout()

    return figure, axes
