#!/usr/bin/env python3
"""
Test script for RRT* motion planning implementation.

Tests RRT* planner with both 2-link and 6-link dual-arm systems.
"""

import numpy as np
from typing import List
from src import DualArm, TwoLinkArm, SixLinkArm, RRTStar
import matplotlib.pyplot as plt


def test_rrt_star_2link():
    """Test RRT* with 2-link arms (4D config space)."""
    print("=" * 60)
    print("Test 1: RRT* with 2-Link Arms (4D C-space)")
    print("=" * 60)
    
    # Create dual-arm system
    dual_arm = DualArm(L1=1.0, L2=0.7, separation=2.0)
    
    # Define start and goal configurations
    # Format: [θ1_left, θ2_left, θ1_right, θ2_right]
    start_config = np.array([0.5, 0.3, -0.2, 0.4])
    goal_config = np.array([1.0, -0.5, 0.8, -0.3])
    
    # Validate configurations
    print(f"Start config: {start_config}")
    print(f"Goal config: {goal_config}")
    print(f"Start valid: {dual_arm.is_valid_configuration(start_config)}")
    print(f"Goal valid: {dual_arm.is_valid_configuration(goal_config)}")
    
    # Create RRT* planner
    planner = RRTStar(dual_arm, max_iterations=3000, step_size=0.15, goal_threshold=0.15)
    
    print("\nPlanning path...")
    path = planner.plan(start_config, goal_config)
    
    if path is None:
        print("❌ Planning failed - no path found")
        return False
    
    print(f"✓ Path found with {len(path)} configurations")
    
    # Validate all configurations in path
    valid_count = sum(1 for config in path if dual_arm.is_valid_configuration(config))
    print(f"✓ Valid configurations: {valid_count}/{len(path)}")
    
    # Visualize path
    visualize_path(dual_arm, path, "2-Link Dual-Arm RRT* Path")
    
    return True


def test_rrt_star_6link():
    """Test RRT* with 6-link arms (12D config space)."""
    print("\n" + "=" * 60)
    print("Test 2: RRT* with 6-Link Arms (12D C-space)")
    print("=" * 60)
    
    # Create dual-arm system with 6-link arms
    left_arm = SixLinkArm(L1=0.5, L2=0.4, L3=0.3, L4=0.25, L5=0.2, L6=0.15, name="Left6Link")
    right_arm = SixLinkArm(L1=0.5, L2=0.4, L3=0.3, L4=0.25, L5=0.2, L6=0.15, name="Right6Link")
    dual_arm = DualArm(left_arm=left_arm, right_arm=right_arm, separation=2.0)
    
    # Define start and goal configurations
    # Format: [θ1-6_left, θ1-6_right]
    start_config = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1,  # Left arm
                             0.1, 0.1, 0.1, 0.1, 0.1, 0.1])  # Right arm
    goal_config = np.array([0.5, -0.3, 0.2, -0.1, 0.4, 0.1,  # Left arm
                           -0.3, 0.5, -0.2, 0.1, -0.4, -0.1])  # Right arm
    
    print(f"Start config: {start_config}")
    print(f"Goal config: {goal_config}")
    print(f"Start valid: {dual_arm.is_valid_configuration(start_config)}")
    print(f"Goal valid: {dual_arm.is_valid_configuration(goal_config)}")
    
    # Create RRT* planner
    planner = RRTStar(dual_arm, max_iterations=5000, step_size=0.1, goal_threshold=0.2)
    
    print("\nPlanning path...")
    path = planner.plan(start_config, goal_config)
    
    if path is None:
        print("❌ Planning failed - no path found")
        return False
    
    print(f"✓ Path found with {len(path)} configurations")
    
    # Validate all configurations in path
    valid_count = sum(1 for config in path if dual_arm.is_valid_configuration(config))
    print(f"✓ Valid configurations: {valid_count}/{len(path)}")
    
    # Visualize path
    visualize_path(dual_arm, path, "6-Link Dual-Arm RRT* Path")
    
    return True


def test_rrt_star_path_validation():
    """Test that RRT* paths are collision-free."""
    print("\n" + "=" * 60)
    print("Test 3: Path Validation")
    print("=" * 60)
    
    dual_arm = DualArm(L1=1.0, L2=0.7, separation=2.0)
    
    start_config = np.array([0.3, 0.5, -0.3, 0.4])
    goal_config = np.array([0.8, -0.4, 0.6, -0.2])
    
    planner = RRTStar(dual_arm, max_iterations=2000, step_size=0.1)
    path = planner.plan(start_config, goal_config)
    
    if path is None:
        print("❌ Planning failed")
        return False
    
    # Check path continuity
    print(f"Path length: {len(path)} configurations")
    
    # Check each segment
    invalid_segments = 0
    for i in range(len(path) - 1):
        if not planner.is_path_valid(path[i], path[i+1], num_checks=5):
            invalid_segments += 1
    
    if invalid_segments > 0:
        print(f"❌ Found {invalid_segments} invalid path segments")
        return False
    
    print("✓ All path segments are collision-free")
    return True


def visualize_path(dual_arm: DualArm, path: List[np.ndarray], title: str):
    """
    Visualize the planned path.
    
    Args:
        dual_arm: DualArm system
        path: List of configurations
        title: Plot title
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    
    # Plot 1: End-effector trajectories in workspace
    left_trajectory = []
    right_trajectory = []
    
    for config in path:
        left_config, right_config = dual_arm._split_configuration(config)
        
        left_local = dual_arm.left_arm.forward_kinematics(*left_config)
        right_local = dual_arm.right_arm.forward_kinematics(*right_config)
        
        left_global = dual_arm.left_base + left_local
        right_global = dual_arm.right_base + right_local
        
        left_trajectory.append(left_global)
        right_trajectory.append(right_global)
    
    left_traj = np.array(left_trajectory)
    right_traj = np.array(right_trajectory)
    
    ax1.plot(left_traj[:, 0], left_traj[:, 1], 'b-', linewidth=2, alpha=0.7, label='Left Arm Path')
    ax1.plot(left_traj[0, 0], left_traj[0, 1], 'bo', markersize=12, label='Left Start', markeredgecolor='black', markeredgewidth=2)
    ax1.plot(left_traj[-1, 0], left_traj[-1, 1], 'b*', markersize=15, label='Left Goal', markeredgecolor='black', markeredgewidth=1)
    
    ax1.plot(right_traj[:, 0], right_traj[:, 1], 'r-', linewidth=2, alpha=0.7, label='Right Arm Path')
    ax1.plot(right_traj[0, 0], right_traj[0, 1], 'ro', markersize=12, label='Right Start', markeredgecolor='black', markeredgewidth=2)
    ax1.plot(right_traj[-1, 0], right_traj[-1, 1], 'r*', markersize=15, label='Right Goal', markeredgecolor='black', markeredgewidth=1)
    
    ax1.plot(dual_arm.left_base[0], dual_arm.left_base[1], 's', color='darkblue', markersize=15, 
            label='Left Base', markeredgecolor='black', markeredgewidth=2)
    ax1.plot(dual_arm.right_base[0], dual_arm.right_base[1], 's', color='darkred', markersize=15, 
            label='Right Base', markeredgecolor='black', markeredgewidth=2)
    
    ax1.set_aspect('equal')
    ax1.set_xlabel('X Position (meters)', fontsize=12)
    ax1.set_ylabel('Y Position (meters)', fontsize=12)
    ax1.set_title('End-Effector Trajectories in Workspace', fontsize=14, fontweight='bold')
    ax1.legend(loc='best', fontsize=10)
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Start and goal configurations
    start_left, start_right = dual_arm._split_configuration(path[0])
    goal_left, goal_right = dual_arm._split_configuration(path[-1])
    
    # Start configuration
    start_left_points = dual_arm.compute_fk_points(dual_arm.left_arm, dual_arm.left_base, *start_left)
    start_right_points = dual_arm.compute_fk_points(dual_arm.right_arm, dual_arm.right_base, *start_right)
    
    start_left_x = [p[0] for p in start_left_points]
    start_left_y = [p[1] for p in start_left_points]
    start_right_x = [p[0] for p in start_right_points]
    start_right_y = [p[1] for p in start_right_points]
    
    ax2.plot(start_left_x, start_left_y, '-o', color='blue', linewidth=2, markersize=8, 
             label='Start Left', alpha=0.6)
    ax2.plot(start_right_x, start_right_y, '-o', color='red', linewidth=2, markersize=8,
             label='Start Right', alpha=0.6)
    
    # Goal configuration
    goal_left_points = dual_arm.compute_fk_points(dual_arm.left_arm, dual_arm.left_base, *goal_left)
    goal_right_points = dual_arm.compute_fk_points(dual_arm.right_arm, dual_arm.right_base, *goal_right)
    
    goal_left_x = [p[0] for p in goal_left_points]
    goal_left_y = [p[1] for p in goal_left_points]
    goal_right_x = [p[0] for p in goal_right_points]
    goal_right_y = [p[1] for p in goal_right_points]
    
    ax2.plot(goal_left_x, goal_left_y, '--s', color='blue', linewidth=2, markersize=8, 
             label='Goal Left', alpha=0.8)
    ax2.plot(goal_right_x, goal_right_y, '--s', color='red', linewidth=2, markersize=8,
             label='Goal Right', alpha=0.8)
    
    ax2.plot(dual_arm.left_base[0], dual_arm.left_base[1], 's', color='darkblue', markersize=15, 
            markeredgecolor='black', markeredgewidth=2)
    ax2.plot(dual_arm.right_base[0], dual_arm.right_base[1], 's', color='darkred', markersize=15, 
            markeredgecolor='black', markeredgewidth=2)
    
    ax2.set_aspect('equal')
    ax2.set_xlim(-3, 3)
    ax2.set_ylim(-0.5, 3)
    ax2.set_xlabel('X Position (meters)', fontsize=12)
    ax2.set_ylabel('Y Position (meters)', fontsize=12)
    ax2.set_title('Start and Goal Configurations', fontsize=14, fontweight='bold')
    ax2.legend(fontsize=10)
    ax2.grid(True, alpha=0.3)
    
    plt.suptitle(title, fontsize=16, fontweight='bold', y=1.02)
    plt.tight_layout()
    plt.show()


def main():
    """Run all RRT* tests."""
    print("🧪 Testing RRT* Motion Planning Implementation")
    print("=" * 60)
    
    results = []
    
    try:
        # Test 1: 2-link arms
        results.append(("2-Link RRT*", test_rrt_star_2link()))
        
        # Test 2: 6-link arms
        results.append(("6-Link RRT*", test_rrt_star_6link()))
        
        # Test 3: Path validation
        results.append(("Path Validation", test_rrt_star_path_validation()))
        
        # Summary
        print("\n" + "=" * 60)
        print("Test Summary")
        print("=" * 60)
        for test_name, passed in results:
            status = "✅ PASSED" if passed else "❌ FAILED"
            print(f"{test_name}: {status}")
        
        all_passed = all(result[1] for result in results)
        if all_passed:
            print("\n✅ All tests passed!")
        else:
            print("\n❌ Some tests failed")
        
    except Exception as e:
        print(f"\n❌ Error during testing: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

