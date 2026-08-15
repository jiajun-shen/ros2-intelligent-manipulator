"""Run the 2R manipulator with sliders, IK target selection, and a path."""

from math import pi

from matplotlib import pyplot as plt

from manipulator_2d.app import create_manipulator_app
from manipulator_2d.model import JointLimits, TwoLinkManipulator


def main() -> None:
    """Display the interactive manipulator with selectable IK branches."""
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
