"""Kinematics and geometric workspace for a planar two-link manipulator."""

from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin, sqrt, tau
from typing import Literal

from manipulator_2d.model import TwoLinkManipulator

Point2D = tuple[float, float]
SolutionBranch = Literal["elbow_up", "elbow_down", "singular"]

_INVERSE_KINEMATICS_TOLERANCE = 1e-12


@dataclass(frozen=True)
class InverseKinematicsSolution:
    """One named joint-angle solution for a Cartesian target.

    Angles are measured in radians. With the coordinate convention used by
    this project, a positive q2 is the elbow-down branch and a negative q2 is
    the elbow-up branch. At singular boundaries, the two branches can describe
    the same physical posture; singularity classification is deferred to the
    next learning step.

    Attributes:
        joint_1_angle: Solved q1 angle in radians.
        joint_2_angle: Solved q2 angle in radians.
        branch: Human-readable name of the geometric solution branch.
    """

    joint_1_angle: float
    joint_2_angle: float
    branch: SolutionBranch

    @property
    def is_singular(self) -> bool:
        """Return whether this posture loses one Cartesian motion direction."""
        return abs(sin(self.joint_2_angle)) <= _INVERSE_KINEMATICS_TOLERANCE


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


def inverse_kinematics(
    manipulator: TwoLinkManipulator,
    target: Point2D,
) -> tuple[InverseKinematicsSolution, ...]:
    """Calculate the two analytic joint-angle solutions for a target point.

    The calculation uses the ideal two-link geometry, normalizes angles, and
    removes solutions that violate either joint limit. Geometrically identical
    singular branches are merged into one solution.

    Args:
        manipulator: A validated planar 2R manipulator. Only its link lengths
            are used by this calculation.
        target: Desired end-effector ``(x, y)`` position in metres, expressed
            in the base frame.

    Returns:
        Zero, one, or two valid solutions. For a regular target, elbow-down is
        returned before elbow-up. A singular target returns at most one
        solution with the branch name ``singular``.

    Raises:
        ValueError: If the target is outside the ideal geometric workspace.
    """
    target_x, target_y = target
    link_1_length = manipulator.link_1_length
    link_2_length = manipulator.link_2_length

    cosine_q2 = (target_x**2 + target_y**2 - link_1_length**2 - link_2_length**2) / (
        2.0 * link_1_length * link_2_length
    )

    if (
        cosine_q2 < -1.0 - _INVERSE_KINEMATICS_TOLERANCE
        or cosine_q2 > 1.0 + _INVERSE_KINEMATICS_TOLERANCE
    ):
        raise ValueError("target is outside the ideal geometric workspace")

    cosine_q2 = max(-1.0, min(1.0, cosine_q2))
    positive_sine_q2 = sqrt(max(0.0, 1.0 - cosine_q2**2))

    elbow_down = _solve_inverse_kinematics_branch(
        link_1_length=link_1_length,
        link_2_length=link_2_length,
        target=target,
        cosine_q2=cosine_q2,
        sine_q2=positive_sine_q2,
        branch="elbow_down",
    )
    elbow_up = _solve_inverse_kinematics_branch(
        link_1_length=link_1_length,
        link_2_length=link_2_length,
        target=target,
        cosine_q2=cosine_q2,
        sine_q2=-positive_sine_q2,
        branch="elbow_up",
    )
    return _normalize_filter_and_deduplicate_solutions(
        manipulator,
        (elbow_down, elbow_up),
    )


def normalize_angle(angle: float) -> float:
    """Normalize an angle in radians to the interval ``[-pi, pi)``."""
    normalized_angle = (angle + pi) % tau - pi
    if abs(normalized_angle) <= _INVERSE_KINEMATICS_TOLERANCE:
        return 0.0
    return normalized_angle


def _normalize_filter_and_deduplicate_solutions(
    manipulator: TwoLinkManipulator,
    solutions: tuple[InverseKinematicsSolution, InverseKinematicsSolution],
) -> tuple[InverseKinematicsSolution, ...]:
    """Normalize solutions, enforce limits, and merge equivalent postures."""
    valid_solutions: list[InverseKinematicsSolution] = []

    for solution in solutions:
        normalized_solution = InverseKinematicsSolution(
            joint_1_angle=normalize_angle(solution.joint_1_angle),
            joint_2_angle=normalize_angle(solution.joint_2_angle),
            branch=solution.branch,
        )
        if not manipulator.joint_1_limits.contains(normalized_solution.joint_1_angle):
            continue
        if not manipulator.joint_2_limits.contains(normalized_solution.joint_2_angle):
            continue

        if normalized_solution.is_singular:
            normalized_solution = InverseKinematicsSolution(
                joint_1_angle=normalized_solution.joint_1_angle,
                joint_2_angle=normalized_solution.joint_2_angle,
                branch="singular",
            )

        if not any(
            _solutions_are_equivalent(normalized_solution, existing_solution)
            for existing_solution in valid_solutions
        ):
            valid_solutions.append(normalized_solution)

    return tuple(valid_solutions)


def _solutions_are_equivalent(
    first: InverseKinematicsSolution,
    second: InverseKinematicsSolution,
) -> bool:
    """Return whether two angle pairs describe the same physical posture."""
    joint_1_difference = normalize_angle(first.joint_1_angle - second.joint_1_angle)
    joint_2_difference = normalize_angle(first.joint_2_angle - second.joint_2_angle)
    return (
        abs(joint_1_difference) <= _INVERSE_KINEMATICS_TOLERANCE
        and abs(joint_2_difference) <= _INVERSE_KINEMATICS_TOLERANCE
    )


def _solve_inverse_kinematics_branch(
    *,
    link_1_length: float,
    link_2_length: float,
    target: Point2D,
    cosine_q2: float,
    sine_q2: float,
    branch: SolutionBranch,
) -> InverseKinematicsSolution:
    """Calculate one inverse-kinematics branch from a chosen q2 sine."""
    target_x, target_y = target
    joint_2_angle = atan2(sine_q2, cosine_q2)
    joint_1_angle = atan2(target_y, target_x) - atan2(
        link_2_length * sine_q2,
        link_1_length + link_2_length * cosine_q2,
    )
    return InverseKinematicsSolution(
        joint_1_angle=joint_1_angle,
        joint_2_angle=joint_2_angle,
        branch=branch,
    )


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
