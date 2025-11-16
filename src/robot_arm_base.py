"""
Robot Arm Base Class Module

This module provides an abstract base class for robot arms, enabling
extensibility to different arm types (2-link, 6-link, 7-link, etc.).
"""

from abc import ABC, abstractmethod
from typing import Tuple, List
import numpy as np


class RobotArmBase(ABC):
    """Abstract base class for robot arms."""
    
    @abstractmethod
    def forward_kinematics(self, *joint_angles: float) -> np.ndarray:
        """
        Compute forward kinematics: joint angles -> end-effector position.
        
        Args:
            *joint_angles: Variable number of joint angles (depends on arm type)
            
        Returns:
            End-effector position as numpy array [x, y] (or [x, y, z] for 3D)
        """
        pass
    
    @abstractmethod
    def get_num_joints(self) -> int:
        """
        Get the number of joints in the arm.
        
        Returns:
            Number of joints
        """
        pass
    
    @abstractmethod
    def get_joint_limits(self) -> List[Tuple[float, float]]:
        """
        Get joint angle limits for all joints.
        
        Returns:
            List of (min, max) tuples, one for each joint
        """
        pass

