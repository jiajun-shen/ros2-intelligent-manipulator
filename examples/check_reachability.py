"""Print reachability results for several planar target points."""

from math import pi

from manipulator_2d.kinematics import is_target_reachable, workspace_radius_limits
from manipulator_2d.model import JointLimits, TwoLinkManipulator


def main() -> None:
    """Create an ideal full-rotation manipulator and check sample targets."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=2.0,
        link_2_length=1.0,
        joint_1_angle=0.0,
        joint_2_angle=0.0,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )
    targets = [(1.0, 0.0), (2.0, 1.0), (0.0, 0.0), (4.0, 0.0)]

    minimum_reach, maximum_reach = workspace_radius_limits(manipulator)
    print(f"Workspace radius: {minimum_reach:.1f} m to {maximum_reach:.1f} m")
    for target in targets:
        result = "reachable" if is_target_reachable(manipulator, target) else "unreachable"
        print(f"Target {target}: {result}")


if __name__ == "__main__":
    main()
