"""Run the planar 2R manipulator with sliders, targets, and a motion path."""

from math import pi

from matplotlib import pyplot as plt

from manipulator_2d.app import create_manipulator_app
from manipulator_2d.model import JointLimits, TwoLinkManipulator


def main() -> None:
    """Display an interactive manipulator and record its end-effector path."""
    limits = JointLimits(minimum=-pi, maximum=pi)
    manipulator = TwoLinkManipulator(
        link_1_length=2.0,
        link_2_length=1.5,
        joint_1_angle=pi / 4,
        joint_2_angle=-pi / 3,
        joint_1_limits=limits,
        joint_2_limits=limits,
    )

    app = create_manipulator_app(manipulator)
    plt.show()
    plt.close(app.figure)


if __name__ == "__main__":
    main()
