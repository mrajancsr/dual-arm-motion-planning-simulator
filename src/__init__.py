"""
Dual-Arm Motion Planning Simulator

A Python-based simulator for dual-arm robot motion planning with workspace
and configuration space visualization capabilities.
"""

from .two_link_arm import TwoLinkArm, DualArm
from .workspace_generator import WorkspaceGenerator, DualArmWorkspaceGenerator
from .cspace_generator import CSpaceGenerator, DualArmCSpaceGenerator
from .robot_arm_base import RobotArmBase
from .objects import Object, PointObject, CircularRegion, RectangularRegion
from .problem_generator import Problem, ProblemGenerator

__version__ = "0.1.0"
__author__ = "Rajan Subramanian, Kaya Celebi, Nico Bykhovsky"

__all__ = [
    "TwoLinkArm",
    "DualArm", 
    "WorkspaceGenerator",
    "DualArmWorkspaceGenerator",
    "CSpaceGenerator",
    "DualArmCSpaceGenerator",
    "RobotArmBase",
    "Object",
    "PointObject",
    "CircularRegion",
    "RectangularRegion",
    "Problem",
    "ProblemGenerator"
]
