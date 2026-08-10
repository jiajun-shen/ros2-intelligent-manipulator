"""Smoke tests for the installable manipulator package."""

import manipulator_2d


def test_package_exposes_version() -> None:
    """The installed package should expose the completed Version 1.0."""
    assert manipulator_2d.__version__ == "1.0.0"
