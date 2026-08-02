"""Smoke tests for the initial project skeleton."""

import manipulator_2d


def test_package_exposes_version() -> None:
    """The installed package should expose its initial version."""
    assert manipulator_2d.__version__ == "0.1.0"
