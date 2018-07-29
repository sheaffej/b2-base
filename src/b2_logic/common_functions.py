import math


def add_radians(a, b):
    """Adds two radian angles, and adjusts sign +/- to be closest to 0
    Parameters:
        :param float a: Angle a
        :param float b: Angle b
    Returns: The resulting angle in radians, with sign adjusted
        :rtype: float
    """
    return (a + b + math.pi) % (2 * math.pi) - math.pi
