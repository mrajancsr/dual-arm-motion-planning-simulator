"""
Offset Arm Wrapper: Wraps TwoLinkArm to apply θ_home offset for FK.

This ensures the offset is applied exactly once at the kinematics boundary.
The planner works in θ_sim, but FK/collision uses θ_physical = θ_sim + θ_home.
"""

import sys
import os
import numpy as np
from typing import Optional, Tuple, List

# Add src/ to path
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
src_path = os.path.join(project_root, 'src')
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from arms.two_link_arm import TwoLinkArm
from robot_arm_base import RobotArmBase

from .theta_home import apply_offset


class OffsetTwoLinkArm(TwoLinkArm):
    """
    Wrapper around TwoLinkArm that applies θ_home offset to FK.
    
    The planner works in θ_sim (zero-centered).
    This wrapper converts to θ_physical = θ_sim + θ_home before calling FK.
    """
    
    def __init__(self, L1=1.0, L2=0.7, name="OffsetArm"):
        """Initialize with same parameters as TwoLinkArm."""
        super().__init__(L1=L1, L2=L2, name=name)
    
    def forward_kinematics(self, theta1: float, theta2: float) -> np.ndarray:
        """
        Compute FK with offset applied.
        
        Args:
            theta1: θ_sim for joint 1 (planner coordinates)
            theta2: θ_sim for joint 2 (planner coordinates)
            
        Returns:
            End-effector position (same as TwoLinkArm, but uses θ_physical)
        """
        # Apply offset: θ_physical = θ_sim + θ_home
        theta1_physical, theta2_physical = apply_offset(theta1, theta2)
        
        # Call parent FK with physical angles
        return super().forward_kinematics(theta1_physical, theta2_physical)
    
    def jacobian(self, t1: float, t2: float) -> np.ndarray:
        """
        Compute Jacobian with offset applied.
        
        Args:
            t1: θ_sim for joint 1
            t2: θ_sim for joint 2
            
        Returns:
            Jacobian matrix (computed at θ_physical)
        """
        # Apply offset
        t1_physical, t2_physical = apply_offset(t1, t2)
        
        # Call parent jacobian
        return super().jacobian(t1_physical, t2_physical)
    
    def is_valid_configuration(self, *joint_angles: float) -> bool:
        """
        Validate configuration (with offset applied internally via FK).
        
        Args:
            *joint_angles: θ_sim angles (planner coordinates)
            
        Returns:
            True if valid
        """
        # Parent's is_valid_configuration calls forward_kinematics,
        # which will apply the offset automatically
        return super().is_valid_configuration(*joint_angles)
    
    def ik_geometric(self, x: float, y: float):
        """
        Solve IK using geometric method (closed-form).
        
        Returns θ_sim angles (for planner), not θ_physical.
        Internally uses FK with offset, but removes offset from result.
        
        Args:
            x: Target x position
            y: Target y position
            
        Returns:
            List of solutions [(theta1_sim, theta2_sim), ...] or None if unreachable
        """
        # Call parent IK (which uses FK internally - FK will apply offset)
        solutions_physical = super().ik_geometric(x, y)
        
        if solutions_physical is None:
            return None
        
        # Remove offset from solutions to get θ_sim
        from .theta_home import remove_offset
        solutions_sim = []
        for theta1_phys, theta2_phys in solutions_physical:
            theta1_sim, theta2_sim = remove_offset(theta1_phys, theta2_phys)
            solutions_sim.append((theta1_sim, theta2_sim))
        
        return solutions_sim
    
    def ik_iterative(self, *args, **kwargs) -> Optional[Tuple[float, ...]]:
        """
        Solve IK iteratively.
        
        Returns θ_sim angles (for planner), not θ_physical.
        
        Args:
            *args: Either (x, y) or ([x, y] array)
            **kwargs: IK solver parameters
            
        Returns:
            Tuple of (theta1_sim, theta2_sim) or None
        """
        # Call parent IK
        solution_physical = super().ik_iterative(*args, **kwargs)
        
        if solution_physical is None:
            return None
        
        # Remove offset to get θ_sim
        from .theta_home import remove_offset
        theta1_sim, theta2_sim = remove_offset(solution_physical[0], solution_physical[1])
        
        return (theta1_sim, theta2_sim)

