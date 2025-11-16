"""
Problem Generator and Simulator Module

This module generates motion planning problems and simulates the execution
of motion planning algorithms. It manages the state of both arms and the object,
and runs a simulation loop that calls the motion planner.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Callable, Any, Dict
from enum import Enum
import numpy as np
import matplotlib.pyplot as plt

from .two_link_arm import DualArm
from .robot_arm_base import RobotArmBase
from .objects import Object, PointObject, CircularRegion


class ObjectState(Enum):
    """State of the object in the workspace."""
    AT_START = "at_start"
    HELD_BY_LEFT = "held_by_left"
    HELD_BY_RIGHT = "held_by_right"
    AT_HANDOFF = "at_handoff"
    AT_GOAL = "at_goal"


@dataclass
class SimulatorState:
    """Current state of the simulation."""
    left_arm_config: Tuple[float, ...]
    right_arm_config: Tuple[float, ...]
    object_position: np.ndarray
    object_state: ObjectState
    step_count: int = 0
    
    def copy(self):
        """Create a copy of the state."""
        return SimulatorState(
            left_arm_config=self.left_arm_config,
            right_arm_config=self.right_arm_config,
            object_position=self.object_position.copy(),
            object_state=self.object_state,
            step_count=self.step_count
        )


@dataclass
class Problem:
    """Represents a motion planning problem for dual-arm systems."""
    
    start_config_left: Tuple[float, ...]
    start_config_right: Tuple[float, ...]
    object_start: Object
    object_goal: Object
    dual_arm: DualArm
    workspace_bounds: Tuple[float, float, float, float]  # (x_min, x_max, y_min, y_max)
    
    def get_initial_state(self) -> SimulatorState:
        """Get the initial simulator state."""
        return SimulatorState(
            left_arm_config=self.start_config_left,
            right_arm_config=self.start_config_right,
            object_position=self.object_start.get_center(),
            object_state=ObjectState.AT_START,
            step_count=0
        )
    
    def is_goal_reached(self, state: SimulatorState) -> bool:
        """Check if goal is reached."""
        goal_center = self.object_goal.get_center()
        if state.object_state == ObjectState.AT_GOAL:
            return True
        # Check if object is at goal position and held by right arm
        if state.object_state == ObjectState.HELD_BY_RIGHT:
            dist = np.linalg.norm(state.object_position - goal_center)
            if dist < 0.1:  # Tolerance
                return True
        return False
    
    def visualize(self, state: Optional[SimulatorState] = None, ax=None, 
                  show_start=True, show_goal=True):
        """
        Visualize the problem setup and optionally current state.
        
        Args:
            state: Current simulator state to visualize
            ax: Matplotlib axes (creates new if None)
            show_start: Whether to show start configuration
            show_goal: Whether to show goal configuration
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 8))
            ax.set_aspect('equal')
        
        # Plot workspace bounds
        x_min, x_max, y_min, y_max = self.workspace_bounds
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(y_min, y_max)
        
        # Plot arm bases
        ax.scatter(self.dual_arm.left_base[0], self.dual_arm.left_base[1],
                  c='blue', marker='s', s=100, label='Left Base', zorder=10)
        ax.scatter(self.dual_arm.right_base[0], self.dual_arm.right_base[1],
                  c='red', marker='s', s=100, label='Right Base', zorder=10)
        
        # Plot start configuration
        if show_start:
            self._plot_config(ax, self.start_config_left, self.start_config_right,
                           'blue', 'red', 'Start', alpha=0.3, linestyle='--')
            self.object_start.visualize(ax, color='green', alpha=0.5)
            ax.text(self.object_start.get_center()[0], 
                   self.object_start.get_center()[1] + 0.2,
                   'Start', ha='center', fontsize=10, color='green')
        
        # Plot goal
        if show_goal:
            self.object_goal.visualize(ax, color='orange', alpha=0.5)
            ax.text(self.object_goal.get_center()[0],
                   self.object_goal.get_center()[1] + 0.2,
                   'Goal', ha='center', fontsize=10, color='orange')
        
        # Plot current state if provided
        if state is not None:
            self._plot_config(ax, state.left_arm_config, state.right_arm_config,
                           'blue', 'red', 'Current', alpha=0.8, linestyle='-')
            
            # Show object at current position
            obj_color = 'purple' if state.object_state in [ObjectState.HELD_BY_LEFT, ObjectState.HELD_BY_RIGHT] else 'green'
            ax.scatter(state.object_position[0], state.object_position[1],
                      c=obj_color, marker='o', s=150, zorder=15,
                      label=f'Object ({state.object_state.value})')
            
            # Show which arm is holding
            if state.object_state == ObjectState.HELD_BY_LEFT:
                # Draw line from left end-effector to object
                left_pos = self.dual_arm.left_arm.forward_kinematics(*state.left_arm_config)
                left_pos_global = self.dual_arm.left_base + left_pos
                ax.plot([left_pos_global[0], state.object_position[0]],
                       [left_pos_global[1], state.object_position[1]],
                       'g--', linewidth=2, alpha=0.5, label='Holding')
            elif state.object_state == ObjectState.HELD_BY_RIGHT:
                # Draw line from right end-effector to object
                right_pos = self.dual_arm.right_arm.forward_kinematics(*state.right_arm_config)
                right_pos_global = self.dual_arm.right_base + right_pos
                ax.plot([right_pos_global[0], state.object_position[0]],
                       [right_pos_global[1], state.object_position[1]],
                       'g--', linewidth=2, alpha=0.5, label='Holding')
        
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_title('Motion Planning Problem' + (f' (Step {state.step_count})' if state else ''))
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        return ax
    
    def _plot_config(self, ax, left_config, right_config, left_color, right_color,
                    label, alpha=1.0, linestyle='-'):
        """Helper to plot arm configuration."""
        # For 2-link arms, assume (theta1, theta2) format
        if len(left_config) == 2 and len(right_config) == 2:
            lb, lj, le = self.dual_arm.compute_fk_points(
                self.dual_arm.left_arm, self.dual_arm.left_base,
                left_config[0], left_config[1])
            rb, rj, re = self.dual_arm.compute_fk_points(
                self.dual_arm.right_arm, self.dual_arm.right_base,
                right_config[0], right_config[1])
            
            ax.plot([lb[0], lj[0], le[0]], [lb[1], lj[1], le[1]],
                   color=left_color, linestyle=linestyle, alpha=alpha,
                   linewidth=2, label=f'{label} Left' if label else None)
            ax.plot([rb[0], rj[0], re[0]], [rb[1], rj[1], re[1]],
                   color=right_color, linestyle=linestyle, alpha=alpha,
                   linewidth=2, label=f'{label} Right' if label else None)


class ProblemGenerator:
    """Generates motion planning problems and simulates execution."""
    
    def __init__(self, dual_arm: DualArm, workspace_margin: float = 0.5):
        """
        Initialize problem generator.
        
        Args:
            dual_arm: DualArm system instance
            workspace_margin: Margin around workspace bounds for sampling
        """
        self.dual_arm = dual_arm
        self.workspace_margin = workspace_margin
        self.workspace_bounds = self._compute_workspace_bounds()
    
    def _compute_workspace_bounds(self) -> Tuple[float, float, float, float]:
        """Compute workspace bounds based on arm reachability."""
        # Get arm parameters (assuming 2-link arms for now)
        if hasattr(self.dual_arm.left_arm, 'L1') and hasattr(self.dual_arm.left_arm, 'L2'):
            L1 = self.dual_arm.left_arm.L1
            L2 = self.dual_arm.left_arm.L2
        else:
            L1, L2 = 1.0, 0.7
        
        max_reach = L1 + L2
        separation = np.linalg.norm(self.dual_arm.right_base - self.dual_arm.left_base)
        
        x_min = -separation/2 - max_reach - self.workspace_margin
        x_max = separation/2 + max_reach + self.workspace_margin
        y_min = -self.workspace_margin
        y_max = max_reach + self.workspace_margin
        
        return (x_min, x_max, y_min, y_max)
    
    def find_closer_arm(self, object_pos: np.ndarray) -> str:
        """
        Determine which arm is closer to the object.
        
        Args:
            object_pos: Object position [x, y]
            
        Returns:
            'left' or 'right'
        """
        dist_left = np.linalg.norm(object_pos - self.dual_arm.left_base)
        dist_right = np.linalg.norm(object_pos - self.dual_arm.right_base)
        
        return 'left' if dist_left < dist_right else 'right'
    
    def _sample_valid_arm_config(self, arm, base_pos: np.ndarray,
                                target_pos: np.ndarray, max_attempts: int = 100) -> Optional[Tuple]:
        """Sample a valid arm configuration to reach target position."""
        target_local = target_pos - base_pos
        
        if hasattr(arm, 'ik_iterative'):
            config = arm.ik_iterative(
                target_local[0], target_local[1],
                theta_init=(np.random.uniform(-np.pi, np.pi),
                          np.random.uniform(-np.pi, np.pi)),
                alpha=0.6
            )
            if config is not None:
                return config
        
        # Fallback: random sampling
        joint_limits = arm.get_joint_limits() if hasattr(arm, 'get_joint_limits') else [(-np.pi, np.pi), (-np.pi, np.pi)]
        for _ in range(max_attempts):
            config = tuple(np.random.uniform(lim[0], lim[1]) for lim in joint_limits)
            pos = arm.forward_kinematics(*config)
            pos_global = base_pos + pos
            
            if np.linalg.norm(pos_global - target_pos) < 0.1:
                return config
        
        return None
    
    def _is_in_collaborative_workspace(self, pos: np.ndarray) -> bool:
        """Check if position is in collaborative workspace."""
        if hasattr(self.dual_arm.left_arm, 'L1') and hasattr(self.dual_arm.left_arm, 'L2'):
            L1 = self.dual_arm.left_arm.L1
            L2 = self.dual_arm.left_arm.L2
        else:
            L1, L2 = 1.0, 0.7
        
        max_reach = L1 + L2
        dist_left = np.linalg.norm(pos - self.dual_arm.left_base)
        dist_right = np.linalg.norm(pos - self.dual_arm.right_base)
        
        return dist_left <= max_reach and dist_right <= max_reach
    
    def generate_problem(self, mode: str = 'random', **kwargs) -> Problem:
        """
        Generate a motion planning problem.
        
        Args:
            mode: 'random' (default) or 'user'
            **kwargs: Additional parameters
        
        Returns:
            Problem instance
        """
        if mode == 'random':
            return self._generate_random_problem(**kwargs)
        elif mode == 'user':
            return self._generate_user_problem(**kwargs)
        else:
            raise ValueError(f"Unknown mode: {mode}. Use 'random' or 'user'")
    
    def _generate_random_problem(self, object_type: str = 'point',
                                object_start_pos: Optional[np.ndarray] = None,
                                object_goal_pos: Optional[np.ndarray] = None) -> Problem:
        """Generate a random problem."""
        # Sample object positions if not provided
        if object_start_pos is None:
            attempts = 0
            while attempts < 100:
                x = np.random.uniform(self.workspace_bounds[0] + 0.5,
                                     self.workspace_bounds[1] - 0.5)
                y = np.random.uniform(0.5, self.workspace_bounds[3] - 0.5)
                pos = np.array([x, y])
                if self._is_in_collaborative_workspace(pos):
                    object_start_pos = pos
                    break
                attempts += 1
            if object_start_pos is None:
                object_start_pos = np.array([0.0, 1.0])
        
        if object_goal_pos is None:
            attempts = 0
            while attempts < 100:
                x = np.random.uniform(self.workspace_bounds[0] + 0.5,
                                     self.workspace_bounds[1] - 0.5)
                y = np.random.uniform(0.5, self.workspace_bounds[3] - 0.5)
                pos = np.array([x, y])
                if self._is_in_collaborative_workspace(pos) and \
                   np.linalg.norm(pos - object_start_pos) > 0.5:
                    object_goal_pos = pos
                    break
                attempts += 1
            if object_goal_pos is None:
                object_goal_pos = np.array([0.5, 1.5])
        
        # Create objects
        if object_type == 'point':
            obj_start = PointObject(object_start_pos[0], object_start_pos[1])
            obj_goal = PointObject(object_goal_pos[0], object_goal_pos[1])
        elif object_type == 'circle':
            radius = 0.1
            obj_start = CircularRegion(object_start_pos[0], object_start_pos[1], radius)
            obj_goal = CircularRegion(object_goal_pos[0], object_goal_pos[1], radius)
        else:
            raise ValueError(f"Unknown object type: {object_type}")
        
        # Sample start configurations (arms not holding object, in safe positions)
        # Sample left arm to a position away from object
        left_start_pos = object_start_pos + np.array([0.5, 0.3])
        start_config_left = self._sample_valid_arm_config(
            self.dual_arm.left_arm, self.dual_arm.left_base, left_start_pos)
        
        # Sample right arm to a position away from object
        right_start_pos = object_start_pos + np.array([-0.5, 0.3])
        start_config_right = self._sample_valid_arm_config(
            self.dual_arm.right_arm, self.dual_arm.right_base, right_start_pos)
        
        # Fallback if sampling fails
        if start_config_left is None:
            start_config_left = (0.0, 0.0)
        if start_config_right is None:
            start_config_right = (0.0, 0.0)
        
        return Problem(
            start_config_left=start_config_left,
            start_config_right=start_config_right,
            object_start=obj_start,
            object_goal=obj_goal,
            dual_arm=self.dual_arm,
            workspace_bounds=self.workspace_bounds
        )
    
    def _generate_user_problem(self, object_start: Object, object_goal: Object,
                              start_config_left: Optional[Tuple] = None,
                              start_config_right: Optional[Tuple] = None) -> Problem:
        """Generate a problem from user-specified parameters."""
        obj_center_start = object_start.get_center()
        obj_center_goal = object_goal.get_center()
        
        if not self._is_in_collaborative_workspace(obj_center_start):
            raise ValueError("Object start position not in collaborative workspace")
        if not self._is_in_collaborative_workspace(obj_center_goal):
            raise ValueError("Object goal position not in collaborative workspace")
        
        # Generate missing configurations
        if start_config_left is None:
            left_start_pos = obj_center_start + np.array([0.5, 0.3])
            start_config_left = self._sample_valid_arm_config(
                self.dual_arm.left_arm, self.dual_arm.left_base, left_start_pos)
        if start_config_right is None:
            right_start_pos = obj_center_start + np.array([-0.5, 0.3])
            start_config_right = self._sample_valid_arm_config(
                self.dual_arm.right_arm, self.dual_arm.right_base, right_start_pos)
        
        if start_config_left is None:
            start_config_left = (0.0, 0.0)
        if start_config_right is None:
            start_config_right = (0.0, 0.0)
        
        return Problem(
            start_config_left=start_config_left,
            start_config_right=start_config_right,
            object_start=object_start,
            object_goal=object_goal,
            dual_arm=self.dual_arm,
            workspace_bounds=self.workspace_bounds
        )
    
    def simulate(self, problem: Problem, motion_planner: Callable,
                 max_steps: int = 1000, visualize: bool = False,
                 verbose: bool = True) -> Tuple[List[SimulatorState], bool]:
        """
        Simulate the execution of a motion planning algorithm.
        
        Args:
            problem: Problem to solve
            motion_planner: Function that takes (problem, current_state) and returns next_state
            max_steps: Maximum number of simulation steps
            visualize: Whether to visualize each step
            verbose: Whether to print progress
            
        Returns:
            Tuple of (list of states, success flag)
        """
        states = []
        current_state = problem.get_initial_state()
        states.append(current_state.copy())
        
        if verbose:
            print(f"Starting simulation (max {max_steps} steps)")
            print(f"Initial state: Object at {current_state.object_position}, "
                  f"state={current_state.object_state.value}")
        
        for step in range(max_steps):
            # Check if goal reached
            if problem.is_goal_reached(current_state):
                if verbose:
                    print(f"Goal reached at step {step}!")
                return states, True
            
            # Get next state from motion planner
            try:
                next_state = motion_planner(problem, current_state)
                
                # Validate next state
                if next_state is None:
                    if verbose:
                        print(f"Planner returned None at step {step}")
                    break
                
                # Update state
                current_state = next_state
                current_state.step_count = step + 1
                states.append(current_state.copy())
                
                if visualize and step % 10 == 0:  # Visualize every 10 steps
                    fig, ax = plt.subplots(figsize=(10, 8))
                    problem.visualize(current_state, ax)
                    plt.show()
                
                if verbose and step % 50 == 0:
                    print(f"Step {step}: Object at {current_state.object_position}, "
                          f"state={current_state.object_state.value}")
                    
            except Exception as e:
                if verbose:
                    print(f"Error at step {step}: {e}")
                break
        
        if verbose:
            print(f"Simulation ended at step {len(states)-1}")
            if not problem.is_goal_reached(current_state):
                print("Goal not reached")
        
        return states, problem.is_goal_reached(current_state)
    
    def run_solver(self, problem: Problem, solver: Callable) -> Any:
        """
        Run a motion planning solver on the problem (alias for simulate).
        
        Args:
            problem: Problem to solve
            solver: Solver function (same as motion_planner)
            
        Returns:
            Tuple of (list of states, success flag)
        """
        return self.simulate(problem, solver, max_steps=1000, visualize=False, verbose=True)
