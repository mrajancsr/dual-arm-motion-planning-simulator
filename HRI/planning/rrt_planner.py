"""
RRT* Planner Adapter for HRI System

Bridges HRI obstacle format and robot config with existing src/ RRT* code.
"""

import sys
import os
import numpy as np
from typing import List, Dict, Optional, Tuple

project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
src_path = os.path.join(project_root, 'src')
if src_path not in sys.path:
    sys.path.insert(0, src_path)

from arms.two_link_arm import TwoLinkArm
from dual_arm_system import DualArm
from rrt_star import RRTStar

from robot.robot_config import (
    LINK1_LENGTH, LINK2_LENGTH, ROBOT_BASE_POSITION, JOINT_LIMITS,
    DUMMY_RIGHT_ARM_BASE, DUMMY_RIGHT_ARM_L1, DUMMY_RIGHT_ARM_L2
)

try:
    from robot.calibration.offset_arm_wrapper import OffsetTwoLinkArm
    USE_OFFSET = True
except ImportError:
    USE_OFFSET = False
    OffsetTwoLinkArm = TwoLinkArm


def convert_obstacles_2d_to_planner(obstacle_boxes_2d: List[Dict]) -> List[Dict]:
    planner_obstacles = []
    for box in obstacle_boxes_2d:
        x_min = box['x_min']
        x_max = box['x_max']
        y_min = box['y_min']
        y_max = box['y_max']
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
    if USE_OFFSET:
        left_arm = OffsetTwoLinkArm(L1=LINK1_LENGTH, L2=LINK2_LENGTH, name="HRI_Robot")
    else:
        left_arm = TwoLinkArm(L1=LINK1_LENGTH, L2=LINK2_LENGTH, name="HRI_Robot")
    right_arm = TwoLinkArm(L1=DUMMY_RIGHT_ARM_L1, L2=DUMMY_RIGHT_ARM_L2, name="Dummy")
    dual_arm = DualArm(
        left_arm=left_arm,
        right_arm=right_arm,
        obstacles=obstacles if obstacles is not None else []
    )
    dual_arm.left_base = np.array(ROBOT_BASE_POSITION)
    dual_arm.right_base = np.array(DUMMY_RIGHT_ARM_BASE)
    return dual_arm


def validate_start_config(dual_arm: DualArm, 
                         start_config: Optional[np.ndarray] = None) -> np.ndarray:
    if start_config is None:
        start_config = np.array([0.0, 0.0, 0.0, 0.0])
    if dual_arm.is_valid_configuration(start_config):
        return start_config
    print("  ⚠ Home position [0, 0, 0, 0] invalid, solving IK for safe position...")
    base_x, base_y = dual_arm.left_base
    safe_target_global = np.array([base_x + 0.1, base_y])
    safe_target_relative = safe_target_global - dual_arm.left_base
    safe_angles = dual_arm.left_arm.ik_iterative(safe_target_relative)
    if safe_angles is None:
        print("  ⚠ IK failed for 10cm forward, trying 5cm forward...")
        safe_target_relative = np.array([0.05, 0.0])
        safe_angles = dual_arm.left_arm.ik_iterative(safe_target_relative)
    if safe_angles is None:
        raise ValueError("Could not find valid start configuration (IK failed)")
    safe_config = np.array([safe_angles[0], safe_angles[1], 0.0, 0.0])
    if not dual_arm.is_valid_configuration(safe_config):
        raise ValueError("Safe IK solution is still invalid")
    print(f"  ✓ Using safe start config: [{safe_angles[0]:.3f}, {safe_angles[1]:.3f}, 0, 0]")
    return safe_config


def solve_goal_config_with_branch_selection(dual_arm: DualArm,
                                           handoff_target_global: Tuple[float, float],
                                           start_config: np.ndarray) -> Optional[np.ndarray]:
    target_global = np.array(handoff_target_global)
    target_relative = target_global - dual_arm.left_base
    for obs in dual_arm.obstacles:
        if obs['type'] == 'rectangle':
            center = np.array(obs['center'])
            half_w = obs['width'] / 2
            half_h = obs['height'] / 2
            dx = target_global[0] - center[0]
            dy = target_global[1] - center[1]
            if abs(dx) <= half_w and abs(dy) <= half_h:
                print(f"  ⚠ Warning: Target point {target_global} is inside obstacle at {center}")
                break
    solutions = dual_arm.left_arm.ik_geometric(target_relative[0], target_relative[1])
    if solutions is None or len(solutions) == 0:
        print(f"  ✗ IK failed: Target unreachable at {target_relative}")
        return None
    start_angles = np.array([start_config[0], start_config[1]])
    valid_solutions = []
    for i, sol in enumerate(solutions):
        sol_angles = np.array(sol)
        test_config = np.array([sol_angles[0], sol_angles[1], 0.0, 0.0])
        left_valid = dual_arm.left_arm.is_valid_configuration(sol_angles[0], sol_angles[1])
        if not left_valid:
            ee_pos = dual_arm.left_arm.forward_kinematics(sol_angles[0], sol_angles[1])
            print(f"    Solution {i+1}: θ_sim=({sol_angles[0]:.3f}, {sol_angles[1]:.3f}), EE=({ee_pos[0]:.3f}, {ee_pos[1]:.3f}) - LEFT ARM INVALID")
            continue
        if dual_arm.is_valid_configuration(test_config):
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
    start_elbow_sign = np.sign(start_angles[1]) if start_angles[1] != 0 else 1
    scored_solutions = []
    for sol_angles, dist in valid_solutions:
        sol_elbow_sign = np.sign(sol_angles[1]) if sol_angles[1] != 0 else 1
        elbow_match = 1.0 if sol_elbow_sign == start_elbow_sign else 2.0
        score = elbow_match * dist
        scored_solutions.append((sol_angles, dist, score))
    scored_solutions.sort(key=lambda x: x[2])
    goal_angles = scored_solutions[0][0]
    if len(solutions) > 1:
        elbow_info = "same elbow orientation" if np.sign(goal_angles[1]) == start_elbow_sign else "different elbow orientation"
        print(f"  ✓ IK found {len(solutions)} solutions, {len(valid_solutions)} valid, selected {elbow_info} (distance: {scored_solutions[0][1]:.3f} rad)")
    else:
        print(f"  ✓ IK found 1 solution (valid)")
    goal_config = np.array([goal_angles[0], goal_angles[1], 0.0, 0.0])
    ee_pos = dual_arm.left_arm.forward_kinematics(goal_angles[0], goal_angles[1])
    ee_global = dual_arm.left_base + ee_pos
    error = np.linalg.norm(ee_global - target_global)
    if error > 0.01:
        print(f"  ⚠ Warning: IK solution error {error:.4f}m (target: {target_global}, achieved: {ee_global})")
    return goal_config


def plan_to_target(handoff_target_global: Tuple[float, float],
                  start_config: np.ndarray,
                  dual_arm: DualArm,
                  max_iterations: int = 3000,
                  step_size: float = 0.05,
                  goal_threshold: float = 0.05) -> Tuple[Optional[List[np.ndarray]], Optional[np.ndarray]]:
    print(f"\n[Planning] Planning to target: {handoff_target_global}")
    goal_config = solve_goal_config_with_branch_selection(
        dual_arm, handoff_target_global, start_config
    )
    if goal_config is None:
        print("  ✗ Planning failed: Could not solve IK for goal")
        return None, None
    print(f"  Goal config: [{goal_config[0]:.3f}, {goal_config[1]:.3f}, 0, 0]")
    planner = RRTStar(
        dual_arm,
        max_iterations=max_iterations,
        step_size=step_size,
        goal_threshold=goal_threshold,
        verbose=True
    )
    print(f"  Starting RRT* planning...")
    path = planner.plan_single_arm('left', start_config, goal_config)
    if path is None:
        print("  ✗ Planning failed: No path found")
        return None, goal_config
    final_config = path[-1]
    final_ee = dual_arm.left_arm.forward_kinematics(final_config[0], final_config[1])
    final_ee_global = dual_arm.left_base + final_ee
    target_global = np.array(handoff_target_global)
    ee_error = np.linalg.norm(final_ee_global - target_global)
    if ee_error > 0.01:
        print(f"  ⚠ Final path end-effector error: {ee_error:.4f}m (target: {target_global}, achieved: {final_ee_global})")
        target_relative_refined = target_global - dual_arm.left_base
        refined_solutions = dual_arm.left_arm.ik_geometric(target_relative_refined[0], target_relative_refined[1])
        if refined_solutions:
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
    print("\n[Collision Test] Testing configuration validity...")
    config1 = np.array([0.0, 0.0, 0.0, 0.0])
    valid1 = dual_arm.is_valid_configuration(config1)
    print(f"  Test 1 - Home [0, 0, 0, 0]: {'✓ PASS' if valid1 else '✗ FAIL'}")
    config2 = np.array([0.0, 0.0, 0.0, 0.0])
    left_pos = dual_arm.left_arm.forward_kinematics(0.0, 0.0)
    ee_global = dual_arm.left_base + left_pos
    print(f"    End-effector at: ({ee_global[0]:.3f}, {ee_global[1]:.3f})")
    if dual_arm.obstacles:
        config3 = np.array([np.pi / 4, 0.0, 0.0, 0.0])
        valid3 = dual_arm.is_valid_configuration(config3)
        print(f"  Test 3 - Config [π/4, 0, 0, 0]: {'✓ PASS' if valid3 else '✗ FAIL (may be in obstacle)'}")
    config4 = np.array([10.0, 10.0, 0.0, 0.0])
    valid4 = dual_arm.is_valid_configuration(config4)
    print(f"  Test 4 - Extreme [10, 10, 0, 0]: {'✓ PASS' if valid4 else '✗ FAIL (expected - outside limits)'}")
    print("  ✓ Collision checking tests complete\n")

