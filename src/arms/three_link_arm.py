"""
Three-Link Planar Arm Implementation

A 3R (3 revolute joint) planar robot arm with forward/inverse kinematics,
Jacobian computation, and configuration validation.
"""

from typing import Optional, Tuple, List
import numpy as np

try:
    from ..robot_arm_base import RobotArmBase
except ImportError:
    from robot_arm_base import RobotArmBase


class ThreeLinkArm(RobotArmBase):
    """3-link planar robot arm with revolute joints."""
    
    def __init__(self, L1=1.0, L2=0.8, L3=0.6, name="ThreeLinkArm"):
        """
        Initialize 3-link planar arm.
        
        Args:
            L1: Length of first link
            L2: Length of second link
            L3: Length of third link
            name: Name identifier for the arm
        """
        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.name = name
        self.link_lengths = np.array([L1, L2, L3], dtype=np.float64)

    def forward_kinematics(self, theta1: float, theta2: float, theta3: float) -> np.ndarray:
        """
        Compute forward kinematics: joint angles -> end-effector position.
        
        Args:
            theta1: First joint angle (radians)
            theta2: Second joint angle (radians)
            theta3: Third joint angle (radians)
            
        Returns:
            End-effector position as numpy array [x, y]
        """
        # Cumulative angles
        a1 = theta1
        a2 = theta1 + theta2
        a3 = theta1 + theta2 + theta3
        
        x = self.L1 * np.cos(a1) + self.L2 * np.cos(a2) + self.L3 * np.cos(a3)
        y = self.L1 * np.sin(a1) + self.L2 * np.sin(a2) + self.L3 * np.sin(a3)
        
        return np.array([x, y])

    def jacobian(self, theta1: float, theta2: float, theta3: float) -> np.ndarray:
        """
        Compute the Jacobian matrix (joint velocities -> end-effector velocities).
        
        Args:
            theta1: First joint angle (radians)
            theta2: Second joint angle (radians)
            theta3: Third joint angle (radians)
            
        Returns:
            2x3 Jacobian matrix
        """
        # Cumulative angles
        a1 = theta1
        a2 = theta1 + theta2
        a3 = theta1 + theta2 + theta3
        
        # Compute sines and cosines
        s1 = np.sin(a1)
        c1 = np.cos(a1)
        s2 = np.sin(a2)
        c2 = np.cos(a2)
        s3 = np.sin(a3)
        c3 = np.cos(a3)
        
        # Jacobian matrix
        J = np.array([
            [
                -self.L1 * s1 - self.L2 * s2 - self.L3 * s3,
                -self.L2 * s2 - self.L3 * s3,
                -self.L3 * s3
            ],
            [
                self.L1 * c1 + self.L2 * c2 + self.L3 * c3,
                self.L2 * c2 + self.L3 * c3,
                self.L3 * c3
            ]
        ], dtype=np.float64)
        
        return J

    def ik_iterative(self, *args, **kwargs) -> Optional[Tuple[float, float, float]]:
        """
        Solve inverse kinematics numerically: workspace position -> joint angles.
        
        Supports both calling patterns:
        - ik_iterative(x_goal, y_goal, **kwargs)  # Backward compatible
        - ik_iterative(target_position, **kwargs)  # New array-based
        
        Args:
            *args: Either (x_goal, y_goal) as two floats, or [target_position] as np.ndarray
            **kwargs: Additional parameters:
                - theta_init: Initial guess (default: (0.0, 0.0, 0.0))
                - max_iters: Maximum iterations (default: 200)
                - tol: Convergence tolerance (default: 1e-3)
                - alpha: Step size/learning rate (default: 0.6)
                - joint_limits: Optional joint limit constraints
        
        Returns:
            Tuple of (theta1, theta2, theta3) if solution found, None otherwise
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
        theta_init = kwargs.get('theta_init', (0.0, 0.0, 0.0))
        max_iters = kwargs.get('max_iters', 200)
        tol = kwargs.get('tol', 1e-3)
        alpha = kwargs.get('alpha', 0.6)
        max_step = kwargs.get('max_step', 0.1)  # Maximum step size per iteration
        
        # Initialize joint angles
        theta = np.array([float(theta_init[0]), float(theta_init[1]), float(theta_init[2])], dtype=np.float64)
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
            
            # Compute Jacobian and update using transpose method
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
            *joint_angles: Three joint angles to validate (theta1, theta2, theta3)
        
        Returns:
            True if configuration is valid, False otherwise
        """
        if len(joint_angles) != 3:
            return False
        
        theta1, theta2, theta3 = joint_angles[0], joint_angles[1], joint_angles[2]
        
        # Check joint limits
        joint_limits = self.get_joint_limits()
        for i, angle in enumerate(joint_angles):
            if angle < joint_limits[i][0] or angle > joint_limits[i][1]:
                return False
        
        # Check workspace reachability
        pos = self.forward_kinematics(theta1, theta2, theta3)
        r = np.sqrt(pos[0]**2 + pos[1]**2)
        
        # Compute min and max reach
        total_length = self.L1 + self.L2 + self.L3
        max_link_length = max(self.L1, self.L2, self.L3)
        min_reach = abs(total_length - 2 * max_link_length)  # Approximate minimum reach
        max_reach = total_length
        
        if r < min_reach or r > max_reach:
            return False
        
        # For 3-link arms, self-collision is possible but rare in practice
        # Could add more sophisticated self-collision check if needed
        
        return True

    def get_num_joints(self) -> int:
        """Get the number of joints (3 for 3-link arm)."""
        return 3
    
    def get_joint_limits(self) -> List[Tuple[float, float]]:
        """
        Get joint angle limits.
        
        Returns:
            List of (min, max) tuples for each joint.
            Default: (-π, π) for all joints.
        """
        return [(-np.pi, np.pi), (-np.pi, np.pi), (-np.pi, np.pi)]
    
    def __repr__(self):
        return f"{self.name}(L1={self.L1:.2f}, L2={self.L2:.2f}, L3={self.L3:.2f})"


# Example usage and testing
if __name__ == "__main__":
    # Create a 3-link arm
    arm = ThreeLinkArm(L1=1.0, L2=0.8, L3=0.6, name="TestArm")
    
    print(f"Created: {arm}")
    print(f"Number of joints: {arm.get_num_joints()}")
    print(f"Joint limits: {arm.get_joint_limits()}")
    
    # Test forward kinematics
    test_angles = (np.pi/4, np.pi/6, -np.pi/8)
    pos = arm.forward_kinematics(*test_angles)
    print(f"\nForward kinematics at {test_angles}:")
    print(f"  End-effector position: {pos}")
    
    # Test Jacobian
    J = arm.jacobian(*test_angles)
    print(f"\nJacobian at {test_angles}:")
    print(J)
    
    # Test inverse kinematics
    target = np.array([1.5, 1.0])
    print(f"\nInverse kinematics to reach {target}:")
    solution = arm.ik_iterative(target, theta_init=(0.5, 0.3, 0.2), max_iters=500)
    
    if solution:
        print(f"  Solution found: {solution}")
        achieved_pos = arm.forward_kinematics(*solution)
        error = np.linalg.norm(achieved_pos - target)
        print(f"  Achieved position: {achieved_pos}")
        print(f"  Error: {error:.6f}")
        print(f"  Valid: {arm.is_valid_configuration(*solution)}")
    else:
        print("  No solution found")
    
    # Test configuration validation
    print(f"\nConfiguration validation:")
    valid_config = (0.5, 0.3, -0.2)
    print(f"  {valid_config} is valid: {arm.is_valid_configuration(*valid_config)}")
    
    invalid_config = (4.0, 0.0, 0.0)  # Outside joint limits
    print(f"  {invalid_config} is valid: {arm.is_valid_configuration(*invalid_config)}")

