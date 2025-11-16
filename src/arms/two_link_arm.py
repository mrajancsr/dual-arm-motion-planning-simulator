"""
Two-Link Planar Arm Implementation

A 2R (2 revolute joint) planar robot arm with forward/inverse kinematics,
Jacobian computation, and configuration validation.
"""

from typing import Optional, Tuple, List
import numpy as np

try:
    from ..robot_arm_base import RobotArmBase
except ImportError:
    from robot_arm_base import RobotArmBase


class TwoLinkArm(RobotArmBase):
    """2-link planar robot arm with revolute joints."""
    
    def __init__(self, L1=1.0, L2=0.7, name="Arm"):
        """
        Initialize 2-link planar arm.
        
        Args:
            L1: Length of first link
            L2: Length of second link
            name: Name identifier for the arm
        """
        self.L1 = L1
        self.L2 = L2
        self.name = name

    def forward_kinematics(self, theta1: float, theta2: float) -> np.ndarray:
        """
        Compute forward kinematics: joint angles -> end-effector position.
        
        Args:
            theta1: First joint angle (radians)
            theta2: Second joint angle (radians)
            
        Returns:
            End-effector position as numpy array [x, y]
        """
        x = self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2)
        y = self.L1 * np.sin(theta1) + self.L2 * np.sin(theta1 + theta2)
        return np.array([x, y])

    def jacobian(self, t1: float, t2: float) -> np.ndarray:
        """
        Compute the Jacobian matrix (joint velocities -> end-effector velocities).
        
        Args:
            t1: First joint angle (radians)
            t2: Second joint angle (radians)
            
        Returns:
            2x2 Jacobian matrix
        """
        s1, c1 = np.sin(t1), np.cos(t1)
        s12, c12 = np.sin(t1 + t2), np.cos(t1 + t2)
        J = np.array(
            [
                [-self.L1 * s1 - self.L2 * s12, -self.L2 * s12],
                [self.L1 * c1 + self.L2 * c12, self.L2 * c12],
            ],
            dtype=np.float64,
        )
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
                - theta_init: Initial guess (default: (0.0, 0.0))
                - max_iters: Maximum iterations (default: 200)
                - tol: Convergence tolerance (default: 1e-3)
                - alpha: Step size/learning rate (default: 0.6)
                - joint_limits: Optional joint limit constraints
        
        Returns:
            Tuple of (theta1, theta2) if solution found, None otherwise
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
        theta_init = kwargs.get('theta_init', (0.0, 0.0))
        max_iters = kwargs.get('max_iters', 200)
        tol = kwargs.get('tol', 1e-3)
        alpha = kwargs.get('alpha', 0.6)
        
        t1, t2 = float(theta_init[0]), float(theta_init[1])
        p_goal = np.array([x_goal, y_goal])

        for _ in range(max_iters):
            pos = self.forward_kinematics(t1, t2)
            error = p_goal - pos
            J = self.jacobian(t1, t2)

            dtheta = alpha * (J.T @ error)

            if np.linalg.norm(dtheta) > 0.1:
                dtheta = 0.1 * dtheta / np.linalg.norm(dtheta)

            if np.linalg.norm(error) < tol:
                return (t1, t2)

            t1 += dtheta[0]
            t2 += dtheta[1]

        return None

    def is_valid_configuration(self, *joint_angles: float) -> bool:
        """
        Validate a single-arm configuration.
        
        Checks:
        - Joint limits (all angles within min/max bounds)
        - Workspace reachability (end-effector within reachable region)
        
        Args:
            *joint_angles: Joint angles to validate (theta1, theta2)
        
        Returns:
            True if configuration is valid, False otherwise
        """
        if len(joint_angles) != 2:
            return False
        
        theta1, theta2 = joint_angles[0], joint_angles[1]
        
        # Check joint limits
        joint_limits = self.get_joint_limits()
        if theta1 < joint_limits[0][0] or theta1 > joint_limits[0][1]:
            return False
        if theta2 < joint_limits[1][0] or theta2 > joint_limits[1][1]:
            return False
        
        # Check workspace reachability
        pos = self.forward_kinematics(theta1, theta2)
        r = np.sqrt(pos[0]**2 + pos[1]**2)
        
        # Check if within reachable region
        min_reach = abs(self.L1 - self.L2)
        max_reach = self.L1 + self.L2
        
        if r < min_reach or r > max_reach:
            return False
        
        # For 2-link arms, no self-collision to check
        return True

    def get_num_joints(self) -> int:
        """Get the number of joints (2 for 2-link arm)."""
        return 2
    
    def get_joint_limits(self) -> List[Tuple[float, float]]:
        """
        Get joint angle limits.
        
        Returns:
            List of (min, max) tuples for each joint.
            Default: (-π, π) for both joints.
        """
        return [(-np.pi, np.pi), (-np.pi, np.pi)]
    
    def ik_geometric(self, x: float, y: float):
        """
        Solve inverse kinematics using geometric method (closed-form).
        
        Args:
            x: Target x position
            y: Target y position
            
        Returns:
            List of solutions [(theta1, theta2), ...] or None if unreachable
        """
        L1, L2 = self.L1, self.L2
        r2 = x * x + y * y
        r = np.sqrt(r2)

        # reachability condition for 2R case
        if r > (L1 + L2) or r < abs(L2 - L1):
            return None
        c2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
        s2_pos = np.sqrt(max(0.0, 1 - c2 * c2))
        sols = []
        for s2 in (s2_pos, -s2_pos):
            t2 = np.arctan2(s2, c2)
            k1 = L1 + L2 * np.cos(t2)
            k2 = L2 * np.sin(t2)
            t1 = np.arctan2(y, x) - np.arctan2(k2, k1)
            sols.append((t1, t2))
        return sols  # [(01,02)_elbow_up, (01,02)_elbow_down]
    
    def __repr__(self):
        return f"{self.name}(L1={self.L1}, L2={self.L2})"

