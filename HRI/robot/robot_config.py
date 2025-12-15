"""
Robot Configuration Module

Single source of truth for all robot parameters.
Import this module everywhere instead of hardcoding values.
"""

import numpy as np
from typing import Tuple

# Link lengths in meters
LINK1_LENGTH = 0.05  # L1 in meters
LINK2_LENGTH = 0.05  # L2 in meters

# Robot base position in global plane coordinates (x, y)
ROBOT_BASE_POSITION = (0.25, 0.0)

# Joint limits: (min, max) for each joint in radians
JOINT_LIMITS = [
    (-np.pi, np.pi),  # Joint 1 (theta1)
    (-np.pi, np.pi)   # Joint 2 (theta2)
]

# Dummy right arm configuration (for DualArm compatibility)
DUMMY_RIGHT_ARM_BASE = (1000.0, 1000.0)  # Far away, won't interfere
DUMMY_RIGHT_ARM_L1 = 0.01  # Tiny links
DUMMY_RIGHT_ARM_L2 = 0.01

