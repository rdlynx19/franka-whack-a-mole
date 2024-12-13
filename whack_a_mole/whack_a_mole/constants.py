"""
Constants for Color Detection
=============================

This module contains constants and HSV color ranges for identifying specific colors in images.

Constants
---------
COLORS : dict
    A dictionary mapping color names to their respective integer indices.

COLORS_HSV : dict
    A dictionary mapping color names to their HSV value ranges for color detection.
"""
import numpy as np
COLORS = {"GREEN": 0, "YELLOW": 1, "BLUE": 2, "RED": 3}

COLORS_HSV = {
    "GREEN": [np.array([47, 90, 132]), np.array([94, 255, 191])],
    "YELLOW": [np.array([24, 100, 101]), np.array([52, 255, 230])],
    "BLUE": [np.array((95, 157, 85)), np.array((161, 255, 189))],
    "RED": [np.array((0, 132, 4)), np.array((2, 241, 221))],
}
