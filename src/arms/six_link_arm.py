"""
Six-Link Planar Arm Implementation

A 6R (6 revolute joint) planar robot arm with forward/inverse kinematics,
Jacobian computation, and configuration validation.
"""

from typing import Optional, Tuple, List
import numpy as np

try:
    from ..robot_arm_base import RobotArmBase
except ImportError:
    from robot_arm_base import RobotArmBase


class SixLinkArm(RobotArmBase):
    """6-link planar robot arm with revolute joints."""
    
    def __init__(self, L1=1.0, L2=0.8, L3=0.6, L4=0.5, L5=0.4, L6=0.3, name="SixLinkArm"):
        """
        Initialize 6-link planar arm.
        
        Args:
            L1: Length of first link
            L2: Length of second link
            L3: Length of third link
            L4: Length of fourth link
            L5: Length of fifth link
            L6: Length of sixth link
            name: Name identifier for the arm
        """
        self.link_lengths = np.array([L1, L2, L3, L4, L5, L6], dtype=np.float64)
        self.name = name

    def forward_kinematics(self, *joint_angles: float) -> np.ndarray:
        """
        Compute forward kinematics: joint angles -> end-effector position.
        
        Args:
            *joint_angles: Six joint angles (theta1, theta2, theta3, theta4, theta5, theta6)
            
        Returns:
            End-effector position as numpy array [x, y]
        """
        if len(joint_angles) != 6:
            raise ValueError(f"Expected 6 joint angles, got {len(joint_angles)}")
        
        cumulative_angle = 0.0
        x, y = 0.0, 0.0
        
        for i, (length, angle) in enumerate(zip(self.link_lengths, joint_angles)):
            cumulative_angle += float(angle)
            x += length * np.cos(cumulative_angle)
            y += length * np.sin(cumulative_angle)
        
        return np.array([x, y])

    def jacobian(self, *joint_angles: float) -> np.ndarray:
        """
        Compute the Jacobian matrix (joint velocities -> end-effector velocities).
        
        Args:
            *joint_angles: Six joint angles (theta1, theta2, theta3, theta4, theta5, theta6)
            
        Returns:
            2x6 Jacobian matrix
        """
        if len(joint_angles) != 6:
            raise ValueError(f"Expected 6 joint angles, got {len(joint_angles)}")
        
        J = np.zeros((2, 6), dtype=np.float64)
        cumulative_angle = 0.0
        
        for i in range(6):
            cumulative_angle += float(joint_angles[i])
            
            # Sum of remaining link lengths (from joint i to end)
            remaining_length = np.sum(self.link_lengths[i:])
            
            J[0, i] = -remaining_length * np.sin(cumulative_angle)  # x-velocity
            J[1, i] = remaining_length * np.cos(cumulative_angle)   # y-velocity
        
        return J

    def ik_iterative(self, *args, **kwargs) -> Optional[Tuple[float, ...]]:
        """
        Solve inverse kinematics numerically: workspace position -> joint angles.
        
        Supports both calling patterns:
        - ik_iterative(x_goal, y_goal, **kwargs)  # Backward compatible
        - ik_iterative(target_position, **kwargs)  # New array-based
        
        Args:
            *args: Either (x_goal, y_goal) as two floats, or [target_position] as np.ndarray
            **kwargs: Additional parameters:
                - theta_init: Initial guess (default: zeros)
                - max_iters: Maximum iterations (default: 200)
                - tol: Convergence tolerance (default: 1e-3)
                - alpha: Step size/learning rate (default: 0.6)
                - joint_limits: Optional joint limit constraints
        
        Returns:
            Tuple of 6 joint angles if solution found, None otherwise
        """
        # Extract x, y from args (supports both calling patterns)
        if len(args) == 1 and isinstance(args[0], np.ndarray):
            # New pattern: ik_iterative(np.array([x, y]), ...)
            x_goal, y_goal = float(args[0][0]), float(args[0][1])
        elif len(args) == 2:
            # Old pattern: ik_iterative(x, y, ...) - backward compatible
            x_goal, y_goal = float(args[0]), float(args[1])
        else:
            raise ValueError(
                "ik_iterative expects either (x, y) or ([x, y] array). "
                f"Got {len(args)} arguments."
            )
        
        # Extract kwargs with defaults
        theta_init = kwargs.get('theta_init', (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        max_iters = kwargs.get('max_iters', 200)
        tol = kwargs.get('tol', 1e-3)
        alpha = kwargs.get('alpha', 0.6)
        max_step = kwargs.get('max_step', 0.1)  # Maximum step size per iteration
        
        # Initialize joint angles
        theta = np.array([float(t) for t in theta_init], dtype=np.float64)
        p_goal = np.array([x_goal, y_goal])
        
        # Get joint limits for clipping
        joint_limits = self.get_joint_limits()
        
        for _ in range(max_iters):
            # Current position
            pos = self.forward_kinematics(*theta)
            error = p_goal - pos
            
            # Check convergence
            if np.linalg.norm(error) < tol:
                return tuple(theta)
            
            # Compute Jacobian and update
            J = self.jacobian(*theta)
            delta_theta = alpha * (J.T @ error)
            
            # Limit step size
            if np.linalg.norm(delta_theta) > max_step:
                delta_theta = max_step * delta_theta / np.linalg.norm(delta_theta)
            
            theta += delta_theta
            
            # Clip to joint limits
            for i, (min_limit, max_limit) in enumerate(joint_limits):
                theta[i] = np.clip(theta[i], min_limit, max_limit)
        
        return None  # Failed to converge

    def is_valid_configuration(self, *joint_angles: float) -> bool:
        """
        Validate a single-arm configuration.
        
        Checks:
        - Joint limits (all angles within min/max bounds)
        - Workspace reachability (end-effector within reachable region)
        
        Args:
            *joint_angles: Six joint angles to validate
        
        Returns:
            True if configuration is valid, False otherwise
        """
        if len(joint_angles) != 6:
            return False
        
        # Check joint limits
        joint_limits = self.get_joint_limits()
        for i, angle in enumerate(joint_angles):
            if angle < joint_limits[i][0] or angle > joint_limits[i][1]:
                return False
        
        # Check workspace reachability
        pos = self.forward_kinematics(*joint_angles)
        r = np.sqrt(pos[0]**2 + pos[1]**2)
        
        # Compute min and max reach
        total_length = np.sum(self.link_lengths)
        max_link_length = np.max(self.link_lengths)
        min_reach = abs(total_length - 2 * max_link_length)  # Approximate minimum reach
        max_reach = total_length
        
        if r < min_reach or r > max_reach:
            return False
        
        # Self-collision check could be added here if needed
        # For now, we skip it as it's complex for 6-link arms
        
        return True

    def get_num_joints(self) -> int:
        """Get the number of joints (6 for 6-link arm)."""
        return 6
    
    def get_joint_limits(self) -> List[Tuple[float, float]]:
        """
        Get joint angle limits.
        
        Returns:
            List of (min, max) tuples for each joint.
            Default: (-π, π) for all joints.
        """
        return [(-np.pi, np.pi)] * 6
    
    def __repr__(self):
        lengths_str = ", ".join([f"L{i+1}={L:.2f}" for i, L in enumerate(self.link_lengths)])
        return f"{self.name}({lengths_str})"

