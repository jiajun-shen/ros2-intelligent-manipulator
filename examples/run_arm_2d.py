"""Run the static planar 2R manipulator visualization."""

from math import pi

from matplotlib import pyplot as plt

from manipulator_2d.model import JointLimits, TwoLinkManipulator
from manipulator_2d.visualization import draw_manipulator


def main() -> None:
    """Create one valid manipulator configuration and display it."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=2.0,
        link_2_length=1.5,
        joint_1_angle=pi / 4,
        joint_2_angle=-pi / 3,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )

    draw_manipulator(manipulator)
    plt.show()


if __name__ == "__main__":
    main()
