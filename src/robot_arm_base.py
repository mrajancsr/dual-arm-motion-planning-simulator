"""
Robot Arm Base Class Module

This module provides an abstract base class for robot arms, enabling
extensibility to different arm types (2-link, 6-link, 7-link, etc.).
"""

from abc import ABC, abstractmethod
from typing import Tuple, List, Optional
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
    
    @abstractmethod
    def jacobian(self, *joint_angles: float) -> np.ndarray:
        """
        Compute the Jacobian matrix (joint velocities -> end-effector velocities).
        
        Args:
            *joint_angles: Variable number of joint angles (depends on arm type)
            
        Returns:
            Jacobian matrix as numpy array. Shape varies by arm type:
            - 2x2 for 2-link planar arms
            - 3xN for N-link spatial arms
            - 2xN for N-link planar arms
        """
        pass
    
    @abstractmethod
    def ik_iterative(self, target_position: np.ndarray, **kwargs) -> Optional[Tuple[float, ...]]:
        """
        Solve inverse kinematics numerically: workspace position -> joint angles.
        
        Args:
            target_position: Target end-effector position as numpy array.
                [x, y] for planar arms, [x, y, z] for spatial arms
            **kwargs: Additional parameters for IK solver:
                - theta_init: Initial guess for joint angles (default: zeros)
                - max_iters: Maximum iterations (default: 200)
                - tol: Convergence tolerance (default: 1e-3)
                - alpha: Step size/learning rate (default: 0.6)
                - joint_limits: Optional joint limit constraints
        
        Returns:
            Tuple of joint angles if solution found, None if unreachable or
            convergence failed. Tuple length matches number of joints.
        """
        pass
    
    @abstractmethod
    def is_valid_configuration(self, *joint_angles: float) -> bool:
        """
        Validate a single-arm configuration.
        
        Checks:
        - Joint limits (all angles within min/max bounds)
        - Workspace reachability (end-effector within reachable region)
        - Self-collision (if applicable for multi-link arms)
        
        Does NOT check inter-arm collisions (that's DualArm's responsibility).
        
        Args:
            *joint_angles: Variable number of joint angles to validate
        
        Returns:
            True if configuration is valid, False otherwise
        """
        pass

