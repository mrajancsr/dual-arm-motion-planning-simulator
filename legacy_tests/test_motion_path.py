#!/usr/bin/env python3
"""Test motion planning path execution without RRT*"""

import numpy as np
from src.dual_arm_system import DualArm
from src.arms.two_link_arm import TwoLinkArm
from src.arms.six_link_arm import SixLinkArm
import matplotlib.pyplot as plt

def generate_fake_path_2link():
    """Generate a hardcoded path for 2-link dual arm (4D configs)"""
    # Simple linear interpolation between start and goal
    start = np.array([0.5, 0.3, -0.2, 0.4])  # [θ1L, θ2L, θ1R, θ2R]
    goal = np.array([1.0, -0.5, 0.8, -0.3])
    
    # Create 20 intermediate points
    path = [start + t * (goal - start) for t in np.linspace(0, 1, 20)]
    return path

def generate_fake_path_6link():
    """Generate a hardcoded path for 6-link dual arm (12D configs)"""
    start = np.array([0.1]*6 + [0.1]*6)  # All joints at 0.1 rad
    goal = np.array([0.5]*6 + [-0.3]*6)
    
    path = [start + t * (goal - start) for t in np.linspace(0, 1, 20)]
    return path

def test_path_execution(dual_arm, path, arm_type="2-link"):
    """Execute and visualize a fake path"""
    print(f"\n=== Testing {arm_type} Path Execution ===")
    print(f"Path length: {len(path)} configurations")
    
    # Validate all configs in path
    valid_count = 0
    for i, config in enumerate(path):
        if dual_arm.is_valid_configuration(config):
            valid_count += 1
    
    print(f"Valid configurations: {valid_count}/{len(path)}")
    
    # Execute path and track end-effector positions
    left_trajectory = []
    right_trajectory = []
    
    for config in path:
        left_config, right_config = dual_arm._split_configuration(config)
        
        # Compute end-effector positions
        left_local = dual_arm.left_arm.forward_kinematics(*left_config)
        right_local = dual_arm.right_arm.forward_kinematics(*right_config)
        
        left_global = dual_arm.left_base + left_local
        right_global = dual_arm.right_base + right_local
        
        left_trajectory.append(left_global)
        right_trajectory.append(right_global)
    
    # Plot end-effector trajectories
    fig, ax = plt.subplots(figsize=(10, 8))
    
    left_traj = np.array(left_trajectory)
    right_traj = np.array(right_trajectory)
    
    # Plot trajectories with start/end markers
    ax.plot(left_traj[:, 0], left_traj[:, 1], 'b-', linewidth=2, alpha=0.6, label='Left Arm Path')
    ax.plot(left_traj[0, 0], left_traj[0, 1], 'bo', markersize=12, label='Left Start', markeredgecolor='black', markeredgewidth=2)
    ax.plot(left_traj[-1, 0], left_traj[-1, 1], 'b*', markersize=15, label='Left Goal', markeredgecolor='black', markeredgewidth=1)
    
    ax.plot(right_traj[:, 0], right_traj[:, 1], 'r-', linewidth=2, alpha=0.6, label='Right Arm Path')
    ax.plot(right_traj[0, 0], right_traj[0, 1], 'ro', markersize=12, label='Right Start', markeredgecolor='black', markeredgewidth=2)
    ax.plot(right_traj[-1, 0], right_traj[-1, 1], 'r*', markersize=15, label='Right Goal', markeredgecolor='black', markeredgewidth=1)
    
    # Plot bases
    ax.plot(dual_arm.left_base[0], dual_arm.left_base[1], 's', color='darkblue', markersize=15, 
            label='Left Base (Fixed)', markeredgecolor='black', markeredgewidth=2)
    ax.plot(dual_arm.right_base[0], dual_arm.right_base[1], 's', color='darkred', markersize=15, 
            label='Right Base (Fixed)', markeredgecolor='black', markeredgewidth=2)
    
    ax.set_aspect('equal')
    ax.set_xlabel('X Position (meters)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y Position (meters)', fontsize=12, fontweight='bold')
    ax.set_title(f'{arm_type.upper()} Dual-Arm System: End-Effector Trajectories in Workspace\n'
                 f'Showing path from start (○) to goal (★) for both arms', 
                 fontsize=14, fontweight='bold', pad=20)
    ax.legend(loc='best', fontsize=10, framealpha=0.9)
    ax.grid(True, alpha=0.3, linestyle='--')
    
    # Add text annotation
    n_left = dual_arm.left_arm.get_num_joints()
    n_right = dual_arm.right_arm.get_num_joints()
    ax.text(0.02, 0.98, f'Configuration Space: {n_left + n_right}D\n'
                        f'Path Points: {len(path)}\n'
                        f'Valid Configs: {valid_count}/{len(path)}',
            transform=ax.transAxes, fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    plt.show()
    
    # Plot start configuration
    start_left, start_right = dual_arm._split_configuration(path[0])
    fig2, ax2 = plt.subplots(figsize=(10, 8))
    
    # Manually plot since we need better control
    left_points = dual_arm.compute_fk_points(dual_arm.left_arm, dual_arm.left_base, *start_left)
    right_points = dual_arm.compute_fk_points(dual_arm.right_arm, dual_arm.right_base, *start_right)
    
    left_x = [p[0] for p in left_points]
    left_y = [p[1] for p in left_points]
    right_x = [p[0] for p in right_points]
    right_y = [p[1] for p in right_points]
    
    ax2.plot(left_x, left_y, '-o', color='blue', linewidth=3, markersize=10, 
             label='Left Arm', markeredgecolor='black', markeredgewidth=1.5)
    ax2.plot(right_x, right_y, '-o', color='red', linewidth=3, markersize=10,
             label='Right Arm', markeredgecolor='black', markeredgewidth=1.5)
    
    ax2.set_aspect('equal')
    ax2.set_xlim(-3, 3)
    ax2.set_ylim(-0.5, 3)
    ax2.set_xlabel('X Position (meters)', fontsize=12, fontweight='bold')
    ax2.set_ylabel('Y Position (meters)', fontsize=12, fontweight='bold')
    ax2.set_title(f'{arm_type.upper()} START Configuration\n'
                  f'Joint Angles (rad): Left={np.round(start_left, 2)}, Right={np.round(start_right, 2)}',
                  fontsize=14, fontweight='bold', pad=20)
    ax2.legend(fontsize=11)
    ax2.grid(True, alpha=0.3, linestyle='--')
    plt.tight_layout()
    plt.show()
    
    # Plot end configuration
    end_left, end_right = dual_arm._split_configuration(path[-1])
    fig3, ax3 = plt.subplots(figsize=(10, 8))
    
    left_points = dual_arm.compute_fk_points(dual_arm.left_arm, dual_arm.left_base, *end_left)
    right_points = dual_arm.compute_fk_points(dual_arm.right_arm, dual_arm.right_base, *end_right)
    
    left_x = [p[0] for p in left_points]
    left_y = [p[1] for p in left_points]
    right_x = [p[0] for p in right_points]
    right_y = [p[1] for p in right_points]
    
    ax3.plot(left_x, left_y, '-o', color='blue', linewidth=3, markersize=10,
             label='Left Arm', markeredgecolor='black', markeredgewidth=1.5)
    ax3.plot(right_x, right_y, '-o', color='red', linewidth=3, markersize=10,
             label='Right Arm', markeredgecolor='black', markeredgewidth=1.5)
    
    ax3.set_aspect('equal')
    ax3.set_xlim(-3, 3)
    ax3.set_ylim(-0.5, 3)
    ax3.set_xlabel('X Position (meters)', fontsize=12, fontweight='bold')
    ax3.set_ylabel('Y Position (meters)', fontsize=12, fontweight='bold')
    ax3.set_title(f'{arm_type.upper()} GOAL Configuration\n'
                  f'Joint Angles (rad): Left={np.round(end_left, 2)}, Right={np.round(end_right, 2)}',
                  fontsize=14, fontweight='bold', pad=20)
    ax3.legend(fontsize=11)
    ax3.grid(True, alpha=0.3, linestyle='--')
    plt.tight_layout()
    plt.show()
    
    print(f"✓ Path executed successfully!")
    print(f"  Start EE positions: Left={np.round(left_trajectory[0], 3)}, Right={np.round(right_trajectory[0], 3)}")
    print(f"  End EE positions: Left={np.round(left_trajectory[-1], 3)}, Right={np.round(right_trajectory[-1], 3)}")

def main():
    print("🧪 Testing Motion Path Execution (No RRT* needed)")
    print("=" * 60)
    
    # Test 1: 2-link arms
    print("\n--- Test 1: 2-Link Arms ---")
    dual_2link = DualArm(L1=1.0, L2=0.7, separation=2.0)
    path_2link = generate_fake_path_2link()
    test_path_execution(dual_2link, path_2link, "2-link")
    
    # Test 2: 6-link arms
    print("\n--- Test 2: 6-Link Arms ---")
    left_6link = SixLinkArm(name="Left6Link")
    right_6link = SixLinkArm(name="Right6Link")
    dual_6link = DualArm(left_arm=left_6link, right_arm=right_6link, separation=2.0)
    path_6link = generate_fake_path_6link()
    test_path_execution(dual_6link, path_6link, "6-link")
    
    print("\n" + "=" * 60)
    print("✅ All path execution tests completed!")

if __name__ == "__main__":
    main()