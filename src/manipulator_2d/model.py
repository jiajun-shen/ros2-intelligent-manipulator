"""Data models for a planar two-link manipulator."""

from dataclasses import dataclass


@dataclass(frozen=True)
class JointLimits:
    """Allowed angular range for one revolute joint, in radians.

    Attributes:
        minimum: Smallest permitted joint angle, in radians.
        maximum: Largest permitted joint angle, in radians.
    """

    minimum: float
    maximum: float

    def __post_init__(self) -> None:
        """Validate that the angular interval is ordered correctly."""
        if self.minimum > self.maximum:
            raise ValueError("minimum joint limit must not be greater than maximum joint limit")

    def contains(self, angle: float) -> bool:
        """Return whether an angle is inside the inclusive limits."""
        return self.minimum <= angle <= self.maximum


@dataclass(frozen=True)
class TwoLinkManipulator:
    """Geometry, joint configuration, and limits of a planar 2R manipulator.

    Link lengths are measured in metres. Joint angles and limits are measured
    in radians.

    Attributes:
        link_1_length: Distance from joint 1 to joint 2.
        link_2_length: Distance from joint 2 to the end effector.
        joint_1_angle: Angle of link 1 relative to the base x-axis.
        joint_2_angle: Angle of link 2 relative to link 1.
        joint_1_limits: Permitted angular range for joint 1.
        joint_2_limits: Permitted angular range for joint 2.
    """

    link_1_length: float
    link_2_length: float
    joint_1_angle: float
    joint_2_angle: float
    joint_1_limits: JointLimits
    joint_2_limits: JointLimits

    def __post_init__(self) -> None:
        """Validate link lengths and the initial joint configuration."""
        if self.link_1_length <= 0:
            raise ValueError("link_1_length must be greater than zero")
        if self.link_2_length <= 0:
            raise ValueError("link_2_length must be greater than zero")
        if not self.joint_1_limits.contains(self.joint_1_angle):
            raise ValueError("joint_1_angle must be within joint_1_limits")
        if not self.joint_2_limits.contains(self.joint_2_angle):
            raise ValueError("joint_2_angle must be within joint_2_limits")
