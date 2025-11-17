"""
Motion Planner Module

Provides a clean interface for motion planning using RRT*.
Converts workspace goals to C-space and plans paths.
"""

from typing import Optional, List, Tuple
import numpy as np

try:
    from .dual_arm_system import DualArm
    from .rrt_star import RRTStar
except ImportError:
    from dual_arm_system import DualArm
    from rrt_star import RRTStar


class MotionPlanner:
    """
    High-level motion planner that integrates workspace goals with RRT*.
    
    Takes workspace positions and plans collision-free paths.
    """
    
    def __init__(self, dual_arm: DualArm, max_iterations: int = 5000,
                 step_size: float = 0.1, goal_threshold: float = 0.1):
        """
        Initialize motion planner.
        
        Args:
            dual_arm: DualArm system instance
            max_iterations: Max iterations for RRT*
            step_size: Step size for RRT*
            goal_threshold: Goal threshold for RRT*
        """
        self.dual_arm = dual_arm
        self.rrt_planner = RRTStar(
            dual_arm, 
            max_iterations=max_iterations,
            step_size=step_size,
            goal_threshold=goal_threshold
        )
    
    def plan_from_workspace_goals(
        self,
        left_start_pos: np.ndarray,
        left_goal_pos: np.ndarray,
        right_start_pos: np.ndarray,
        right_goal_pos: np.ndarray
    ) -> Optional[List[np.ndarray]]:
        """
        Plan path from workspace positions.
        
        Args:
            left_start_pos: Start position for left arm [x, y] (global coordinates)
            left_goal_pos: Goal position for left arm [x, y] (global coordinates)
            right_start_pos: Start position for right arm [x, y] (global coordinates)
            right_goal_pos: Goal position for right arm [x, y] (global coordinates)
            
        Returns:
            List of configurations forming path, or None if planning fails
        """
        # Convert workspace positions to C-space using IK
        start_config = self._workspace_to_config(left_start_pos, right_start_pos)
        goal_config = self._workspace_to_config(left_goal_pos, right_goal_pos)
        
        if start_config is None:
            return None  # Failed to find valid start config
        if goal_config is None:
            return None  # Failed to find valid goal config
        
        # Plan path using RRT*
        return self.rrt_planner.plan(start_config, goal_config)
    
    def plan_from_configs(
        self,
        start_config: np.ndarray,
        goal_config: np.ndarray
    ) -> Optional[List[np.ndarray]]:
        """
        Plan path directly from C-space configurations.
        
        Args:
            start_config: Start configuration [θ1_left, θ2_left, θ1_right, θ2_right, ...]
            goal_config: Goal configuration [θ1_left, θ2_left, θ1_right, θ2_right, ...]
            
        Returns:
            List of configurations forming path, or None if planning fails
        """
        return self.rrt_planner.plan(start_config, goal_config)
    
    def _workspace_to_config(
        self,
        left_pos: np.ndarray,
        right_pos: np.ndarray,
        max_attempts: int = 20
    ) -> Optional[np.ndarray]:
        """
        Convert workspace positions to C-space configuration using IK.
        
        Args:
            left_pos: Left arm position [x, y] (global coordinates)
            right_pos: Right arm position [x, y] (global coordinates)
            max_attempts: Maximum attempts with different initial guesses
            
        Returns:
            Configuration array, or None if IK fails or solution is invalid
        """
        # Convert to local coordinates
        left_local = left_pos - self.dual_arm.left_base
        right_local = right_pos - self.dual_arm.right_base
        
        # Try multiple initial guesses for IK
        for attempt in range(max_attempts):
            # Random initial guess
            left_init = tuple(np.random.uniform(-np.pi, np.pi, 
                                                self.dual_arm.left_arm.get_num_joints()))
            right_init = tuple(np.random.uniform(-np.pi, np.pi,
                                                 self.dual_arm.right_arm.get_num_joints()))
            
            # Solve IK
            left_config = self.dual_arm.left_arm.ik_iterative(
                left_local[0], left_local[1],
                theta_init=left_init
            )
            right_config = self.dual_arm.right_arm.ik_iterative(
                right_local[0], right_local[1],
                theta_init=right_init
            )
            
            if left_config is None or right_config is None:
                continue
            
            # Combine into full configuration
            config = np.concatenate([np.array(left_config), np.array(right_config)])
            
            # Validate configuration
            if self.dual_arm.is_valid_configuration(config):
                return config
        
        return None
    
    def get_end_effector_positions(self, config: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get end-effector positions for a configuration.
        
        Args:
            config: Configuration array
            
        Returns:
            Tuple of (left_position, right_position) in global coordinates
        """
        left_config, right_config = self.dual_arm._split_configuration(config)
        
        left_local = self.dual_arm.left_arm.forward_kinematics(*left_config)
        right_local = self.dual_arm.right_arm.forward_kinematics(*right_config)
        
        left_global = self.dual_arm.left_base + left_local
        right_global = self.dual_arm.right_base + right_local
        
        return left_global, right_global

