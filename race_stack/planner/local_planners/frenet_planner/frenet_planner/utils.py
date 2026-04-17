"""Minimal utility functions — port of ROS1 utils.h."""
import math


def norm(x: float, y: float) -> float:
    return math.sqrt(x * x + y * y)
