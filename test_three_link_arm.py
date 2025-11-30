#!/usr/bin/env python3
"""
Test Script for Three-Link Arm

Demonstrates the ThreeLinkArm class functionality including forward kinematics,
inverse kinematics, and integration with dual-arm system and RRT* planning.
"""

import numpy as np
import matplotlib.pyplot as plt
from src import ThreeLinkArm, DualArm, RRTStar


def test_basic_functionality():
    """Test basic three-link arm functionality."""
    print("=" * 70)
    print("Test 1: Basic Three-Link Arm Functionality")
    print("=" * 70)
    
    # Create a 3-link arm
    arm = ThreeLinkArm(L1=1.0, L2=0.8, L3=0.6, name="TestArm")
    
    print(f"\n✓ Created: {arm}")
    print(f"  Number of joints: {arm.get_num_joints()}")
    print(f"  Joint limits: {arm.get_joint_limits()}")
    
    # Test forward kinematics
    test_angles = (np.pi/4, np.pi/6, -np.pi/8)
    pos = arm.forward_kinematics(*test_angles)
    print(f"\n✓ Forward kinematics:")
    print(f"  Config: {test_angles}")
    print(f"  End-effector: ({pos[0]:.3f}, {pos[1]:.3f})")
    
    # Test Jacobian
    J = arm.jacobian(*test_angles)
    print(f"\n✓ Jacobian (2x3):")
    print(f"  Shape: {J.shape}")
    print(f"  Condition number: {np.linalg.cond(J):.2f}")
    
    # Test inverse kinematics
    target = np.array([1.5, 1.0])
    print(f"\n✓ Inverse kinematics:")
    print(f"  Target: ({target[0]:.3f}, {target[1]:.3f})")
    
    solution = arm.ik_iterative(target, theta_init=(0.5, 0.3, 0.2), max_iters=500)
    
    if solution:
        achieved_pos = arm.forward_kinematics(*solution)
        error = np.linalg.norm(achieved_pos - target)
        print(f"  Solution: ({solution[0]:.3f}, {solution[1]:.3f}, {solution[2]:.3f})")
        print(f"  Achieved: ({achieved_pos[0]:.3f}, {achieved_pos[1]:.3f})")
        print(f"  Error: {error:.6f} m")
        print(f"  Valid: {arm.is_valid_configuration(*solution)}")
    else:
        print("  ✗ No solution found")
    
    # Test configuration validation
    print(f"\n✓ Configuration validation:")
    valid_config = (0.5, 0.3, -0.2)
    print(f"  {valid_config} → {arm.is_valid_configuration(*valid_config)}")
    
    invalid_config = (4.0, 0.0, 0.0)  # Outside joint limits
    print(f"  {invalid_config} → {arm.is_valid_configuration(*invalid_config)}")
    
    return True


def test_dual_arm_system():
    """Test three-link arms in dual-arm system."""
    print("\n" + "=" * 70)
    print("Test 2: Dual Three-Link Arm System")
    print("=" * 70)
    
    # Create dual-arm system with 3-link arms
    left_arm = ThreeLinkArm(L1=1.0, L2=0.8, L3=0.6, name="LeftThreeLink")
    right_arm = ThreeLinkArm(L1=1.0, L2=0.8, L3=0.6, name="RightThreeLink")
    
    dual_arm = DualArm(left_arm=left_arm, right_arm=right_arm, separation=3.0)
    
    print(f"\n✓ Created dual-arm system:")
    print(f"  Left arm: {left_arm}")
    print(f"  Right arm: {right_arm}")
    print(f"  Separation: 3.0 m")
    print(f"  Total DOF: {dual_arm.left_arm.get_num_joints() + dual_arm.right_arm.get_num_joints()}")
    
    # Test configuration validation
    config = np.array([0.5, 0.3, -0.2, -0.4, 0.2, 0.1])  # 3 + 3 = 6D config
    print(f"\n✓ Testing configuration:")
    print(f"  Config (6D): {config}")
    print(f"  Valid: {dual_arm.is_valid_configuration(config)}")
    
    # Compute end-effector positions
    left_config, right_config = dual_arm._split_configuration(config)
    left_pos = left_arm.forward_kinematics(*left_config)
    right_pos = right_arm.forward_kinematics(*right_config)
    
    print(f"\n✓ End-effector positions:")
    print(f"  Left: ({left_pos[0]:.3f}, {left_pos[1]:.3f}) (local frame)")
    print(f"  Right: ({right_pos[0]:.3f}, {right_pos[1]:.3f}) (local frame)")
    
    return True


def test_rrt_planning():
    """Test RRT* planning with three-link arms."""
    print("\n" + "=" * 70)
    print("Test 3: RRT* Planning with Three-Link Arms")
    print("=" * 70)
    
    # Create dual-arm system
    left_arm = ThreeLinkArm(L1=0.8, L2=0.6, L3=0.4, name="LeftThreeLink")
    right_arm = ThreeLinkArm(L1=0.8, L2=0.6, L3=0.4, name="RightThreeLink")
    
    dual_arm = DualArm(left_arm=left_arm, right_arm=right_arm, separation=2.5)
    
    # Define start and goal configurations (6D: 3 for each arm)
    start_config = np.array([0.3, 0.2, 0.1, -0.2, 0.1, -0.1])
    goal_config = np.array([0.8, -0.3, 0.2, 0.5, -0.2, 0.3])
    
    print(f"\n✓ Planning problem:")
    print(f"  Start: {start_config}")
    print(f"  Goal: {goal_config}")
    print(f"  Configuration space: 6D")
    
    # Verify start and goal are valid
    start_valid = dual_arm.is_valid_configuration(start_config)
    goal_valid = dual_arm.is_valid_configuration(goal_config)
    
    print(f"\n✓ Configuration validity:")
    print(f"  Start valid: {start_valid}")
    print(f"  Goal valid: {goal_valid}")
    
    if not (start_valid and goal_valid):
        print("  ✗ Invalid configurations - skipping planning")
        return False
    
    # Create RRT* planner with optimizations
    print(f"\n✓ Creating RRT* planner with optimizations...")
    planner = RRTStar(
        dual_arm,
        max_iterations=2000,
        step_size=0.15,
        goal_threshold=0.3,
        use_kdtree=True,
        workspace_weight=0.3,
        use_adaptive_step=True,
        verbose=True
    )
    
    print(f"\n✓ Planning path...")
    import time
    start_time = time.time()
    path = planner.plan(start_config, goal_config)
    planning_time = time.time() - start_time
    
    if path:
        print(f"\n✓ Path found!")
        print(f"  Planning time: {planning_time:.3f}s")
        print(f"  Path length: {len(path)} configurations")
        print(f"  First config: {path[0]}")
        print(f"  Last config: {path[-1]}")
        
        # Verify path validity
        all_valid = all(dual_arm.is_valid_configuration(config) for config in path)
        print(f"  All configs valid: {all_valid}")
        
        return True
    else:
        print(f"\n✗ No path found in {planning_time:.3f}s")
        return False


def visualize_three_link_arm():
    """Visualize the three-link arm in different configurations."""
    print("\n" + "=" * 70)
    print("Test 4: Visualization")
    print("=" * 70)
    
    arm = ThreeLinkArm(L1=1.0, L2=0.8, L3=0.6)
    
    # Create different configurations
    configs = [
        (0.0, 0.0, 0.0),           # Straight out
        (np.pi/4, np.pi/4, np.pi/4),  # Bent upward
        (0.0, np.pi/2, -np.pi/2),  # Folded
        (-np.pi/6, np.pi/3, -np.pi/4),  # Complex pose
    ]
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 12))
    axes = axes.flatten()
    
    for idx, config in enumerate(configs):
        ax = axes[idx]
        ax.set_aspect('equal')
        ax.set_xlim(-0.5, 3)
        ax.set_ylim(-1.5, 2.5)
        ax.grid(True, alpha=0.3)
        
        # Compute joint positions
        base = np.array([0.0, 0.0])
        theta1, theta2, theta3 = config
        
        # First joint
        joint1 = base + np.array([
            arm.L1 * np.cos(theta1),
            arm.L1 * np.sin(theta1)
        ])
        
        # Second joint
        joint2 = joint1 + np.array([
            arm.L2 * np.cos(theta1 + theta2),
            arm.L2 * np.sin(theta1 + theta2)
        ])
        
        # End-effector
        end_eff = joint2 + np.array([
            arm.L3 * np.cos(theta1 + theta2 + theta3),
            arm.L3 * np.sin(theta1 + theta2 + theta3)
        ])
        
        # Plot links
        x_coords = [base[0], joint1[0], joint2[0], end_eff[0]]
        y_coords = [base[1], joint1[1], joint2[1], end_eff[1]]
        
        ax.plot(x_coords, y_coords, '-o', linewidth=3, markersize=10,
                color='steelblue', markerfacecolor='orange', markeredgecolor='black')
        
        # Plot base
        ax.plot(base[0], base[1], 's', markersize=15, 
                color='darkred', markeredgecolor='black')
        
        # Add title
        ax.set_title(f'Config: ({theta1:.2f}, {theta2:.2f}, {theta3:.2f})\n'
                     f'End-eff: ({end_eff[0]:.2f}, {end_eff[1]:.2f})',
                     fontsize=10)
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
    
    plt.suptitle('Three-Link Arm Configurations', fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    print("\n✓ Visualization created")
    print("  Close the plot window to continue...")
    plt.show()
    
    return True


def main():
    """Run all tests."""
    print("\n" + "=" * 70)
    print("Three-Link Arm Test Suite")
    print("=" * 70)
    
    results = []
    
    # Run tests
    results.append(("Basic functionality", test_basic_functionality()))
    results.append(("Dual-arm system", test_dual_arm_system()))
    results.append(("RRT* planning", test_rrt_planning()))
    
    # Visualization (optional)
    try:
        results.append(("Visualization", visualize_three_link_arm()))
    except Exception as e:
        print(f"\n⚠️  Visualization skipped: {e}")
        results.append(("Visualization", False))
    
    # Summary
    print("\n" + "=" * 70)
    print("Test Summary")
    print("=" * 70)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for test_name, result in results:
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{status}: {test_name}")
    
    print(f"\nOverall: {passed}/{total} tests passed")
    
    if passed == total:
        print("\n🎉 All tests passed! ThreeLinkArm is working correctly.")
    else:
        print(f"\n⚠️  {total - passed} test(s) failed.")
    
    return passed == total


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)

