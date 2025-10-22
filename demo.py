#!/usr/bin/env python3
"""
Demo script for the Dual-Arm Motion Planning Simulator

This script demonstrates the workspace and configuration space
generation capabilities of the simulator.
"""

import sys
import os

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.two_link_arm import TwoLinkArm, DualArm
from src.workspace_generator import WorkspaceGenerator, DualArmWorkspaceGenerator
from src.cspace_generator import CSpaceGenerator, DualArmCSpaceGenerator
import numpy as np
import matplotlib.pyplot as plt


def demo_single_arm():
    """Demonstrate single arm workspace and C-space generation."""
    print("=== Single Arm Demo ===")
    
    # Create a single arm
    arm = TwoLinkArm(L1=1.0, L2=0.7, name="DemoArm")
    print(f"Created arm: {arm}")
    
    # Workspace generation
    print("\n1. Generating workspace...")
    ws_gen = WorkspaceGenerator(arm, resolution=50)
    fig1 = ws_gen.plot_workspace(show_boundary=True, show_points=True)
    fig1.suptitle("Single Arm Workspace")
    plt.show()
    
    # C-space generation
    print("2. Generating configuration space...")
    cspace_gen = CSpaceGenerator(arm, resolution=50)
    cspace_gen.set_joint_limits((-np.pi, np.pi), (-np.pi/2, np.pi/2))
    
    fig2 = cspace_gen.plot_cspace(show_invalid=True)
    fig2.suptitle("Single Arm Configuration Space")
    plt.show()
    
    fig3 = cspace_gen.plot_cspace_heatmap()
    fig3.suptitle("Single Arm C-space Heatmap")
    plt.show()


def demo_dual_arm():
    """Demonstrate dual arm workspace and C-space generation."""
    print("\n=== Dual Arm Demo ===")
    
    # Create dual arm system
    dual_arm = DualArm(L1=1.0, L2=0.7, separation=2.0)
    print(f"Created dual arm system with separation: {dual_arm.left_base[0]} to {dual_arm.right_base[0]}")
    
    # Dual workspace generation
    print("\n1. Generating dual arm workspace...")
    dual_ws_gen = DualArmWorkspaceGenerator(dual_arm, resolution=50)
    fig1 = dual_ws_gen.plot_dual_workspace(show_boundary=True, show_points=True)
    fig1.suptitle("Dual Arm Workspace")
    plt.show()
    
    # Dual C-space generation
    print("2. Generating dual arm configuration space...")
    dual_cspace_gen = DualArmCSpaceGenerator(dual_arm, resolution=20)
    dual_cspace_gen.set_joint_limits(
        (-np.pi, np.pi), (-np.pi/2, np.pi/2),  # Left arm limits
        (-np.pi, np.pi), (-np.pi/2, np.pi/2)   # Right arm limits
    )
    
    fig2 = dual_cspace_gen.plot_dual_cspace_2d()
    fig2.suptitle("Dual Arm Configuration Space (2D Projections)")
    plt.show()


def demo_motion_planning_setup():
    """Demonstrate the setup for motion planning."""
    print("\n=== Motion Planning Setup Demo ===")
    
    # Create dual arm system
    dual_arm = DualArm(L1=1.0, L2=0.7, separation=2.0)
    
    # Define start and goal positions
    start_positions = {
        'left': np.array([-0.8, 0.5]),
        'right': np.array([1.2, 0.5])
    }
    
    goal_positions = {
        'left': np.array([-0.3, 1.2]),
        'right': np.array([1.7, 1.2])
    }
    
    print(f"Start positions - Left: {start_positions['left']}, Right: {start_positions['right']}")
    print(f"Goal positions - Left: {goal_positions['left']}, Right: {goal_positions['right']}")
    
    # Solve inverse kinematics for start and goal
    print("\nSolving inverse kinematics...")
    
    # Start positions
    left_start_local = start_positions['left'] - dual_arm.left_base
    right_start_local = start_positions['right'] - dual_arm.right_base
    
    left_start_angles = dual_arm.left_arm.ik_iterative(
        left_start_local[0], left_start_local[1],
        theta_init=(0, 0), alpha=0.6
    )
    right_start_angles = dual_arm.right_arm.ik_iterative(
        right_start_local[0], right_start_local[1],
        theta_init=(0, 0), alpha=0.6
    )
    
    # Goal positions
    left_goal_local = goal_positions['left'] - dual_arm.left_base
    right_goal_local = goal_positions['right'] - dual_arm.right_base
    
    left_goal_angles = dual_arm.left_arm.ik_iterative(
        left_goal_local[0], left_goal_local[1],
        theta_init=(0, 0), alpha=0.6
    )
    right_goal_angles = dual_arm.right_arm.ik_iterative(
        right_goal_local[0], right_goal_local[1],
        theta_init=(0, 0), alpha=0.6
    )
    
    if all([left_start_angles, right_start_angles, left_goal_angles, right_goal_angles]):
        print("✓ All IK solutions found successfully!")
        print(f"Left start angles: {np.rad2deg(left_start_angles):.1f}°")
        print(f"Right start angles: {np.rad2deg(right_start_angles):.1f}°")
        print(f"Left goal angles: {np.rad2deg(left_goal_angles):.1f}°")
        print(f"Right goal angles: {np.rad2deg(right_goal_angles):.1f}°")
        
        # Visualize start and goal configurations
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
        
        # Start configuration
        dual_arm.plot_arms(left_start_angles, right_start_angles, 
                          start_positions['left'], start_positions['right'])
        ax1.set_title("Start Configuration")
        
        # Goal configuration  
        dual_arm.plot_arms(left_goal_angles, right_goal_angles,
                          goal_positions['left'], goal_positions['right'])
        ax2.set_title("Goal Configuration")
        
        plt.suptitle("Motion Planning: Start vs Goal Configurations")
        plt.tight_layout()
        plt.show()
        
        print("\n✓ Ready for motion planning! The next step would be to implement")
        print("  RRT or other sampling-based motion planning algorithms.")
        
    else:
        print("✗ Failed to find some IK solutions. Check workspace reachability.")


def main():
    """Run all demonstrations."""
    print("🦾 Dual-Arm Motion Planning Simulator Demo")
    print("=" * 50)
    
    try:
        # Run demonstrations
        demo_single_arm()
        demo_dual_arm()
        demo_motion_planning_setup()
        
        print("\n" + "=" * 50)
        print("✅ Demo completed successfully!")
        print("\nNext steps for motion planning:")
        print("1. Implement RRT algorithm")
        print("2. Add collision detection between arms")
        print("3. Add obstacle avoidance")
        print("4. Implement trajectory smoothing")
        print("5. Add animation capabilities")
        
    except Exception as e:
        print(f"❌ Error during demo: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
