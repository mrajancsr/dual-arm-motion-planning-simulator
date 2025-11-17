"""
Dual-Arm Motion Planning Simulator

A Python-based simulator for dual-arm robot motion planning with workspace
and configuration space visualization capabilities.
"""

# Import from new modular structure
from .arms.two_link_arm import TwoLinkArm
from .arms.six_link_arm import SixLinkArm
from .dual_arm_system import DualArm
from .robot_arm_base import RobotArmBase
from .workspace_generator import WorkspaceGenerator, DualArmWorkspaceGenerator
from .cspace_generator import CSpaceGenerator, DualArmCSpaceGenerator
from .objects import Object, PointObject, CircularRegion, RectangularRegion
from .problem_generator import Problem, ProblemGenerator
from .rrt_star import RRTStar
from .motion_planner import MotionPlanner
from .simple_problem import PlanningProblem, SimpleProblemGenerator

__version__ = "0.1.0"
__author__ = "Rajan Subramanian, Kaya Celebi, Nico Bykhovsky"

__all__ = [
    "TwoLinkArm",
    "SixLinkArm",
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
    "ProblemGenerator",
    "RRTStar",
    "MotionPlanner",
    "PlanningProblem",
    "SimpleProblemGenerator"
]
