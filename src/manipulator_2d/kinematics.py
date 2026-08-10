"""Kinematics and geometric workspace for a planar two-link manipulator."""

from math import cos, hypot, sin

from manipulator_2d.model import TwoLinkManipulator

Point2D = tuple[float, float]


def forward_kinematics(
    manipulator: TwoLinkManipulator,
) -> tuple[Point2D, Point2D]:
    """Calculate joint 2 and end-effector positions in the base frame.

    Args:
        manipulator: A validated planar 2R manipulator configuration.

    Returns:
        The joint 2 position followed by the end-effector position. Each
        position is an ``(x, y)`` tuple measured in metres.
    """
    q1 = manipulator.joint_1_angle
    link_2_world_angle = q1 + manipulator.joint_2_angle

    joint_2_x = manipulator.link_1_length * cos(q1)
    joint_2_y = manipulator.link_1_length * sin(q1)

    end_effector_x = joint_2_x + manipulator.link_2_length * cos(link_2_world_angle)
    end_effector_y = joint_2_y + manipulator.link_2_length * sin(link_2_world_angle)

    joint_2_position = (joint_2_x, joint_2_y)
    end_effector_position = (end_effector_x, end_effector_y)
    return joint_2_position, end_effector_position


def workspace_radius_limits(manipulator: TwoLinkManipulator) -> tuple[float, float]:
    """Return the inner and outer radii of the ideal geometric workspace.

    This length-based workspace assumes that both revolute joints can rotate
    far enough to form every geometric configuration.
    """
    minimum_reach = abs(manipulator.link_1_length - manipulator.link_2_length)
    maximum_reach = manipulator.link_1_length + manipulator.link_2_length
    return minimum_reach, maximum_reach


def is_target_reachable(
    manipulator: TwoLinkManipulator,
    target: Point2D,
) -> bool:
    """Return whether a target is inside the ideal geometric workspace.

    Joint-limit restrictions are intentionally not included at this stage.
    """
    target_x, target_y = target
    target_distance = hypot(target_x, target_y)
    minimum_reach, maximum_reach = workspace_radius_limits(manipulator)
    return minimum_reach <= target_distance <= maximum_reach
