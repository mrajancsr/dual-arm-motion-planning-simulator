"""
Simple Problem Generator

Generates motion planning problems with workspace goals.
Simplified version without object manipulation complexity.
"""

from dataclasses import dataclass
from typing import Tuple, Optional
import numpy as np

try:
    from .dual_arm_system import DualArm
except ImportError:
    from dual_arm_system import DualArm


@dataclass
class PlanningProblem:
    """
    Simple motion planning problem.
    
    Defines start and goal positions for both arms in workspace.
    """
    left_start_pos: np.ndarray  # [x, y] global coordinates
    left_goal_pos: np.ndarray   # [x, y] global coordinates
    right_start_pos: np.ndarray # [x, y] global coordinates
    right_goal_pos: np.ndarray  # [x, y] global coordinates
    dual_arm: DualArm
    
    def __post_init__(self):
        """Validate positions are in workspace."""
        # Basic validation - positions should be reasonable
        if not self._is_reachable(self.left_start_pos, self.dual_arm.left_base):
            raise ValueError(f"Left start position {self.left_start_pos} not reachable")
        if not self._is_reachable(self.left_goal_pos, self.dual_arm.left_base):
            raise ValueError(f"Left goal position {self.left_goal_pos} not reachable")
        if not self._is_reachable(self.right_start_pos, self.dual_arm.right_base):
            raise ValueError(f"Right start position {self.right_start_pos} not reachable")
        if not self._is_reachable(self.right_goal_pos, self.dual_arm.right_base):
            raise ValueError(f"Right goal position {self.right_goal_pos} not reachable")
    
    def _is_reachable(self, pos: np.ndarray, base: np.ndarray) -> bool:
        """Check if position is reachable by arm."""
        # Get arm parameters
        if hasattr(self.dual_arm.left_arm, 'L1') and hasattr(self.dual_arm.left_arm, 'L2'):
            L1 = self.dual_arm.left_arm.L1
            L2 = self.dual_arm.left_arm.L2
        else:
            # For 6-link arms, estimate reach
            if hasattr(self.dual_arm.left_arm, 'link_lengths'):
                L1 = np.sum(self.dual_arm.left_arm.link_lengths)
                L2 = 0
            else:
                L1, L2 = 1.0, 0.7
        
        dist = np.linalg.norm(pos - base)
        max_reach = L1 + L2 if L2 > 0 else L1
        min_reach = abs(L1 - L2) if L2 > 0 else 0.1
        
        return min_reach <= dist <= max_reach


class SimpleProblemGenerator:
    """
    Generates simple motion planning problems.
    
    Creates problems with workspace goals for both arms.
    """
    
    def __init__(self, dual_arm: DualArm):
        """
        Initialize problem generator.
        
        Args:
            dual_arm: DualArm system instance
        """
        self.dual_arm = dual_arm
    
    def generate_random_problem(self, max_attempts: int = 50) -> PlanningProblem:
        """
        Generate a random planning problem with valid IK solutions.
        
        Args:
            max_attempts: Maximum attempts to find valid problem
            
        Returns:
            PlanningProblem with random start/goal positions
        """
        # Get arm reachability
        if hasattr(self.dual_arm.left_arm, 'L1') and hasattr(self.dual_arm.left_arm, 'L2'):
            L1 = self.dual_arm.left_arm.L1
            L2 = self.dual_arm.left_arm.L2
            max_reach = L1 + L2
            min_reach = abs(L1 - L2)
        else:
            max_reach = 2.0  # Default estimate
            min_reach = 0.3
        
        # Try to generate a problem with valid IK solutions
        for attempt in range(max_attempts):
            # Sample positions around each base
            left_start = self._sample_near_base(self.dual_arm.left_base, max_reach, min_reach)
            left_goal = self._sample_near_base(self.dual_arm.left_base, max_reach, min_reach)
            right_start = self._sample_near_base(self.dual_arm.right_base, max_reach, min_reach)
            right_goal = self._sample_near_base(self.dual_arm.right_base, max_reach, min_reach)
            
            # Try to create problem (validates reachability)
            try:
                problem = PlanningProblem(
                    left_start_pos=left_start,
                    left_goal_pos=left_goal,
                    right_start_pos=right_start,
                    right_goal_pos=right_goal,
                    dual_arm=self.dual_arm
                )
                return problem
            except ValueError:
                continue
        
        # Fallback: return a simple problem
        return self.create_problem(
            left_start=self.dual_arm.left_base + np.array([0.5, 0.8]),
            left_goal=self.dual_arm.left_base + np.array([0.3, 1.0]),
            right_start=self.dual_arm.right_base + np.array([-0.5, 0.8]),
            right_goal=self.dual_arm.right_base + np.array([-0.3, 1.0])
        )
    
    def create_problem(
        self,
        left_start: np.ndarray,
        left_goal: np.ndarray,
        right_start: np.ndarray,
        right_goal: np.ndarray
    ) -> PlanningProblem:
        """
        Create a problem from specified positions.
        
        Args:
            left_start: Left arm start position [x, y]
            left_goal: Left arm goal position [x, y]
            right_start: Right arm start position [x, y]
            right_goal: Right arm goal position [x, y]
            
        Returns:
            PlanningProblem instance
        """
        return PlanningProblem(
            left_start_pos=np.array(left_start),
            left_goal_pos=np.array(left_goal),
            right_start_pos=np.array(right_start),
            right_goal_pos=np.array(right_goal),
            dual_arm=self.dual_arm
        )
    
    def _sample_near_base(self, base: np.ndarray, max_reach: float, min_reach: float = 0.3) -> np.ndarray:
        """Sample a random position near base."""
        angle = np.random.uniform(0, 2 * np.pi)
        radius = np.random.uniform(min_reach, 0.9 * max_reach)
        pos = base + radius * np.array([np.cos(angle), np.sin(angle)])
        pos[1] = max(0.1, pos[1])  # Keep above ground
        return pos

