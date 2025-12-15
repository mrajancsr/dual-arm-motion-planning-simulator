"""
RRT* Planner Adapter for HRI System

Bridges HRI obstacle format and robot config with existing src/ RRT* code.
"""

import sys
import os
import numpy as np
from typing import List, Dict, Optional, Tuple

# Add src/ to path to import existing code
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
src_path = os.path.join(project_root, 'src')
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from arms.two_link_arm import TwoLinkArm
from dual_arm_system import DualArm
from rrt_star import RRTStar

# Import HRI robot config
from robot.robot_config import (
    LINK1_LENGTH, LINK2_LENGTH, ROBOT_BASE_POSITION, JOINT_LIMITS,
    DUMMY_RIGHT_ARM_BASE, DUMMY_RIGHT_ARM_L1, DUMMY_RIGHT_ARM_L2
)

# Import offset wrapper (applies θ_home for FK/visualization)
try:
    from robot.calibration.offset_arm_wrapper import OffsetTwoLinkArm
    USE_OFFSET = True
except ImportError:
    # Fallback if calibration not available
    USE_OFFSET = False
    OffsetTwoLinkArm = TwoLinkArm


def convert_obstacles_2d_to_planner(obstacle_boxes_2d: List[Dict]) -> List[Dict]:
    """
    Convert HRI 2D obstacle boxes to DualArm format.
    
    Args:
        obstacle_boxes_2d: List of HRI obstacle boxes with 'x_min', 'x_max', 'y_min', 'y_max'
        
    Returns:
        List of DualArm obstacle dicts: [{'type': 'rectangle', 'center': [x, y], 'width': w, 'height': h}, ...]
    """
    planner_obstacles = []
    
    for box in obstacle_boxes_2d:
        x_min = box['x_min']
        x_max = box['x_max']
        y_min = box['y_min']
        y_max = box['y_max']
        
        # Compute center and dimensions
        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0
        width = x_max - x_min
        height = y_max - y_min
        
        planner_obstacles.append({
            'type': 'rectangle',
            'center': [center_x, center_y],
            'width': width,
            'height': height
        })
    
    return planner_obstacles


def setup_dual_arm_system(obstacles: Optional[List[Dict]] = None) -> DualArm:
    """
    Create DualArm system with single active arm (left) and dummy right arm.
    
    Args:
        obstacles: Optional list of DualArm-format obstacles
        
    Returns:
        DualArm instance configured for HRI system
    """
    # Create left arm (actual robot) using config - use offset wrapper if available
    if USE_OFFSET:
        left_arm = OffsetTwoLinkArm(L1=LINK1_LENGTH, L2=LINK2_LENGTH, name="HRI_Robot")
    else:
        left_arm = TwoLinkArm(L1=LINK1_LENGTH, L2=LINK2_LENGTH, name="HRI_Robot")
    
    # Create dummy right arm (tiny, far away)
    right_arm = TwoLinkArm(L1=DUMMY_RIGHT_ARM_L1, L2=DUMMY_RIGHT_ARM_L2, name="Dummy")
    
    # Create DualArm (bases will be set manually after creation)
    dual_arm = DualArm(
        left_arm=left_arm,
        right_arm=right_arm,
        obstacles=obstacles if obstacles is not None else []
    )
    
    # Set bases manually (DualArm constructor sets them from separation, override here)
    dual_arm.left_base = np.array(ROBOT_BASE_POSITION)
    dual_arm.right_base = np.array(DUMMY_RIGHT_ARM_BASE)
    
    return dual_arm


def validate_start_config(dual_arm: DualArm, 
                         start_config: Optional[np.ndarray] = None) -> np.ndarray:
    """
    Validate and return a valid start configuration.
    
    Tries [0, 0, 0, 0] first. If invalid, falls back to safe IK point.
    
    Args:
        dual_arm: DualArm instance
        start_config: Optional start config to validate. If None, tries [0, 0, 0, 0]
        
    Returns:
        Valid start configuration as numpy array [θ1, θ2, 0, 0]
    """
    if start_config is None:
        # Try home position
        start_config = np.array([0.0, 0.0, 0.0, 0.0])
    
    # Check if valid
    if dual_arm.is_valid_configuration(start_config):
        return start_config
    
    # Fallback: solve IK for safe position (10cm forward from base)
    print("  ⚠ Home position [0, 0, 0, 0] invalid, solving IK for safe position...")
    base_x, base_y = dual_arm.left_base
    safe_target_global = np.array([base_x + 0.1, base_y])  # 10cm forward
    safe_target_relative = safe_target_global - dual_arm.left_base
    
    # Solve IK
    safe_angles = dual_arm.left_arm.ik_iterative(safe_target_relative)
    
    if safe_angles is None:
        # IK failed, try a different safe position
        print("  ⚠ IK failed for 10cm forward, trying 5cm forward...")
        safe_target_relative = np.array([0.05, 0.0])
        safe_angles = dual_arm.left_arm.ik_iterative(safe_target_relative)
    
    if safe_angles is None:
        raise ValueError("Could not find valid start configuration (IK failed)")
    
    safe_config = np.array([safe_angles[0], safe_angles[1], 0.0, 0.0])
    
    # Validate the safe config
    if not dual_arm.is_valid_configuration(safe_config):
        raise ValueError("Safe IK solution is still invalid")
    
    print(f"  ✓ Using safe start config: [{safe_angles[0]:.3f}, {safe_angles[1]:.3f}, 0, 0]")
    return safe_config


def solve_goal_config_with_branch_selection(dual_arm: DualArm,
                                           handoff_target_global: Tuple[float, float],
                                           start_config: np.ndarray) -> Optional[np.ndarray]:
    """
    Solve IK for goal configuration with branch selection.
    
    Converts global target to base-relative, solves IK using geometric method
    to get all solutions, and picks the one closest to start configuration.
    
    Args:
        dual_arm: DualArm instance
        handoff_target_global: Target position (x, y) in global coordinates
        start_config: Start configuration [θ1, θ2, 0, 0] for distance comparison
        
    Returns:
        Goal configuration [θ1_goal, θ2_goal, 0, 0] or None if unreachable
    """
    # Convert global target to base-relative
    target_global = np.array(handoff_target_global)
    target_relative = target_global - dual_arm.left_base
    
    # Check if target point itself is inside an obstacle (quick sanity check)
    for obs in dual_arm.obstacles:
        if obs['type'] == 'rectangle':
            center = np.array(obs['center'])
            half_w = obs['width'] / 2
            half_h = obs['height'] / 2
            # Simple axis-aligned check (assuming rotation=0 for now)
            dx = target_global[0] - center[0]
            dy = target_global[1] - center[1]
            if abs(dx) <= half_w and abs(dy) <= half_h:
                print(f"  ⚠ Warning: Target point {target_global} is inside obstacle at {center}")
                break
    
    # Solve IK using geometric method (gets all solutions)
    solutions = dual_arm.left_arm.ik_geometric(target_relative[0], target_relative[1])
    
    if solutions is None or len(solutions) == 0:
        print(f"  ✗ IK failed: Target unreachable at {target_relative}")
        return None
    
    # Extract start angles (first 2 elements)
    start_angles = np.array([start_config[0], start_config[1]])
    
    # Try all solutions and pick the best valid one (closest to start if multiple valid)
    valid_solutions = []
    
    for i, sol in enumerate(solutions):
        sol_angles = np.array(sol)
        test_config = np.array([sol_angles[0], sol_angles[1], 0.0, 0.0])
        
        # Debug: Check individual arm validity first
        left_valid = dual_arm.left_arm.is_valid_configuration(sol_angles[0], sol_angles[1])
        if not left_valid:
            ee_pos = dual_arm.left_arm.forward_kinematics(sol_angles[0], sol_angles[1])
            print(f"    Solution {i+1}: θ_sim=({sol_angles[0]:.3f}, {sol_angles[1]:.3f}), EE=({ee_pos[0]:.3f}, {ee_pos[1]:.3f}) - LEFT ARM INVALID")
            continue
        
        # Check if this solution is valid (no collision)
        if dual_arm.is_valid_configuration(test_config):
            # Compute angular distance to start (for ranking)
            diff = sol_angles - start_angles
            diff = np.arctan2(np.sin(diff), np.cos(diff))
            dist = np.linalg.norm(diff)
            valid_solutions.append((sol_angles, dist))
        else:
            ee_pos = dual_arm.left_arm.forward_kinematics(sol_angles[0], sol_angles[1])
            print(f"    Solution {i+1}: θ_sim=({sol_angles[0]:.3f}, {sol_angles[1]:.3f}), EE=({ee_pos[0]:.3f}, {ee_pos[1]:.3f}) - COLLISION")
    
    if len(valid_solutions) == 0:
        print(f"  ✗ All {len(solutions)} IK solutions are in collision")
        return None
    
    # Pick best solution: prefer same elbow orientation as start, then closest to start
    start_elbow_sign = np.sign(start_angles[1]) if start_angles[1] != 0 else 1
    scored_solutions = []
    for sol_angles, dist in valid_solutions:
        sol_elbow_sign = np.sign(sol_angles[1]) if sol_angles[1] != 0 else 1
        # Prefer same elbow orientation (elbow up vs down)
        elbow_match = 1.0 if sol_elbow_sign == start_elbow_sign else 2.0
        # Combined score: elbow match penalty + distance
        score = elbow_match * dist
        scored_solutions.append((sol_angles, dist, score))
    
    # Sort by score (lower is better)
    scored_solutions.sort(key=lambda x: x[2])
    goal_angles = scored_solutions[0][0]
    
    if len(solutions) > 1:
        elbow_info = "same elbow orientation" if np.sign(goal_angles[1]) == start_elbow_sign else "different elbow orientation"
        print(f"  ✓ IK found {len(solutions)} solutions, {len(valid_solutions)} valid, selected {elbow_info} (distance: {scored_solutions[0][1]:.3f} rad)")
    else:
        print(f"  ✓ IK found 1 solution (valid)")
    
    # Create goal config
    goal_config = np.array([goal_angles[0], goal_angles[1], 0.0, 0.0])
    
    # Verify end-effector reaches target
    ee_pos = dual_arm.left_arm.forward_kinematics(goal_angles[0], goal_angles[1])
    ee_global = dual_arm.left_base + ee_pos
    error = np.linalg.norm(ee_global - target_global)
    
    if error > 0.01:  # 1cm tolerance
        print(f"  ⚠ Warning: IK solution error {error:.4f}m (target: {target_global}, achieved: {ee_global})")
    
    return goal_config


def plan_to_target(handoff_target_global: Tuple[float, float],
                  start_config: np.ndarray,
                  dual_arm: DualArm,
                  max_iterations: int = 3000,
                  step_size: float = 0.05,
                  goal_threshold: float = 0.05) -> Tuple[Optional[List[np.ndarray]], Optional[np.ndarray]]:
    """
    Plan path from start config to handoff target.
    
    Args:
        handoff_target_global: Target position (x, y) in global coordinates
        start_config: Start configuration [θ1, θ2, 0, 0]
        dual_arm: DualArm instance with obstacles
        max_iterations: Maximum RRT* iterations
        step_size: Step size in radians (default: 0.05)
        goal_threshold: Goal threshold in radians (default: 0.05)
        
    Returns:
        Tuple of (path, goal_config) where:
        - path: List of configurations forming path, or None if planning fails
        - goal_config: Goal configuration [θ1, θ2, 0, 0], or None if IK failed
    """
    print(f"\n[Planning] Planning to target: {handoff_target_global}")
    
    # Solve goal config with branch selection
    goal_config = solve_goal_config_with_branch_selection(
        dual_arm, handoff_target_global, start_config
    )
    
    if goal_config is None:
        print("  ✗ Planning failed: Could not solve IK for goal")
        return None, None
    
    print(f"  Goal config: [{goal_config[0]:.3f}, {goal_config[1]:.3f}, 0, 0]")
    
    # Create RRT* planner
    planner = RRTStar(
        dual_arm,
        max_iterations=max_iterations,
        step_size=step_size,
        goal_threshold=goal_threshold,
        verbose=True
    )
    
    # Plan path
    print(f"  Starting RRT* planning...")
    path = planner.plan_single_arm('left', start_config, goal_config)
    
    if path is None:
        print("  ✗ Planning failed: No path found")
        return None, goal_config  # Return goal_config even if planning failed
    
    # Final refinement: check end-effector error and refine if needed
    final_config = path[-1]
    final_ee = dual_arm.left_arm.forward_kinematics(final_config[0], final_config[1])
    final_ee_global = dual_arm.left_base + final_ee
    target_global = np.array(handoff_target_global)
    ee_error = np.linalg.norm(final_ee_global - target_global)
    
    if ee_error > 0.01:  # If more than 1cm error, try to refine
        print(f"  ⚠ Final path end-effector error: {ee_error:.4f}m (target: {target_global}, achieved: {final_ee_global})")
        # Try to refine the last waypoint using IK
        target_relative_refined = target_global - dual_arm.left_base
        refined_solutions = dual_arm.left_arm.ik_geometric(target_relative_refined[0], target_relative_refined[1])
        if refined_solutions:
            # Pick solution closest to current final config
            best_refined = None
            best_dist = float('inf')
            for sol in refined_solutions:
                sol_config = np.array([sol[0], sol[1], 0.0, 0.0])
                if dual_arm.is_valid_configuration(sol_config):
                    dist = np.linalg.norm(sol_config[:2] - final_config[:2])
                    if dist < best_dist:
                        best_dist = dist
                        best_refined = sol_config
            if best_refined is not None:
                path[-1] = best_refined
                refined_ee = dual_arm.left_arm.forward_kinematics(best_refined[0], best_refined[1])
                refined_ee_global = dual_arm.left_base + refined_ee
                new_error = np.linalg.norm(refined_ee_global - target_global)
                print(f"  ✓ Refined final waypoint: error reduced to {new_error:.4f}m")
    else:
        print(f"  ✓ Final path end-effector error: {ee_error:.4f}m (within tolerance)")
    
    print(f"  ✓ Planning successful! Path length: {len(path)} configurations")
    return path, goal_config


def test_collision_checking(dual_arm: DualArm):
    """
    Test collision checking with a few sample configurations.
    
    Args:
        dual_arm: DualArm instance
    """
    print("\n[Collision Test] Testing configuration validity...")
    
    # Test 1: Home position
    config1 = np.array([0.0, 0.0, 0.0, 0.0])
    valid1 = dual_arm.is_valid_configuration(config1)
    print(f"  Test 1 - Home [0, 0, 0, 0]: {'✓ PASS' if valid1 else '✗ FAIL'}")
    
    # Test 2: Valid configuration (straight out)
    config2 = np.array([0.0, 0.0, 0.0, 0.0])
    # Get end-effector position
    left_pos = dual_arm.left_arm.forward_kinematics(0.0, 0.0)
    ee_global = dual_arm.left_base + left_pos
    print(f"    End-effector at: ({ee_global[0]:.3f}, {ee_global[1]:.3f})")
    
    # Test 3: Configuration that might be in obstacle (if obstacles exist)
    if dual_arm.obstacles:
        # Try a config that extends toward an obstacle
        config3 = np.array([np.pi / 4, 0.0, 0.0, 0.0])
        valid3 = dual_arm.is_valid_configuration(config3)
        print(f"  Test 3 - Config [π/4, 0, 0, 0]: {'✓ PASS' if valid3 else '✗ FAIL (may be in obstacle)'}")
    
    # Test 4: Invalid configuration (outside joint limits - extreme)
    config4 = np.array([10.0, 10.0, 0.0, 0.0])  # Way outside limits
    valid4 = dual_arm.is_valid_configuration(config4)
    print(f"  Test 4 - Extreme [10, 10, 0, 0]: {'✓ PASS' if valid4 else '✗ FAIL (expected - outside limits)'}")
    
    print("  ✓ Collision checking tests complete\n")

