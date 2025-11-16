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
from src.problem_generator import ProblemGenerator, Problem
from src.objects import PointObject, CircularRegion
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
        print(f"Left start angles: {np.degrees(left_start_angles)}")
        print(f"Right start angles: {np.degrees(right_start_angles)}")
        print(f"Left goal angles: {np.degrees(left_goal_angles)}")
        print(f"Right goal angles: {np.degrees(right_goal_angles)}")
        
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


def demo_problem_generation():
    """Demonstrate problem generation for motion planning."""
    print("\n=== Problem Generation Demo ===")
    
    # Create dual arm system
    dual_arm = DualArm(L1=1.0, L2=0.7, separation=2.0)
    print("Created dual arm system")
    
    # Create problem generator
    prob_gen = ProblemGenerator(dual_arm, workspace_margin=0.5)
    print(f"Workspace bounds: {prob_gen.workspace_bounds}")
    
    # Generate random problem
    print("\n1. Generating random problem...")
    try:
        random_problem = prob_gen.generate_problem(mode='random', object_type='point')
        print("✓ Random problem generated successfully!")
        print(f"  Start config - Left: {random_problem.start_config_left}")
        print(f"  Start config - Right: {random_problem.start_config_right}")
        print(f"  Object start: {random_problem.object_start.get_center()}")
        print(f"  Object goal: {random_problem.object_goal.get_center()}")
        print(f"  Note: Goal configs will be determined by motion planner")
        
        # Visualize problem
        print("\n  Visualizing random problem...")
        fig, ax = plt.subplots(figsize=(10, 8))
        random_problem.visualize(state=None, ax=ax)
        fig.suptitle("Random Problem Generation")
        plt.tight_layout()
        plt.show()
        
    except Exception as e:
        print(f"✗ Error generating random problem: {e}")
        import traceback
        traceback.print_exc()
    
    # Generate user-specified problem
    print("\n2. Generating user-specified problem...")
    try:
        # User specifies object positions (within collaborative workspace)
        # Collaborative workspace is roughly between the two arm bases
        # Use positions closer to center and lower to ensure reachability
        obj_start = PointObject(0.0, 0.8)  # Center, reachable by both arms
        obj_goal = PointObject(0.2, 1.0)  # Slightly offset, still reachable
        
        user_problem = prob_gen.generate_problem(
            mode='user',
            object_start=obj_start,
            object_goal=obj_goal
        )
        print("✓ User-specified problem generated successfully!")
        print(f"  Start config - Left: {user_problem.start_config_left}")
        print(f"  Start config - Right: {user_problem.start_config_right}")
        print(f"  Note: Goal configs will be determined by motion planner")
        
        # Visualize problem
        print("\n  Visualizing user-specified problem...")
        fig, ax = plt.subplots(figsize=(10, 8))
        user_problem.visualize(state=None, ax=ax)
        fig.suptitle("User-Specified Problem Generation")
        plt.tight_layout()
        plt.show()
        
    except Exception as e:
        print(f"✗ Error generating user-specified problem: {e}")
        import traceback
        traceback.print_exc()
    
    # Demonstrate circular object
    print("\n3. Generating problem with circular object...")
    try:
        circular_problem = prob_gen.generate_problem(
            mode='random',
            object_type='circle'
        )
        print("✓ Circular object problem generated!")
        
        # Visualize
        fig, ax = plt.subplots(figsize=(10, 8))
        circular_problem.visualize(state=None, ax=ax)
        fig.suptitle("Problem with Circular Object")
        plt.tight_layout()
        plt.show()
        
    except Exception as e:
        print(f"✗ Error generating circular object problem: {e}")
    
    # Demonstrate simulator interface (placeholder)
    print("\n4. Demonstrating simulator interface...")
    print("  (Motion planner will be implemented in future milestone)")
    
    def placeholder_motion_planner(problem: Problem, current_state):
        """
        Placeholder motion planner that demonstrates the interface.
        
        The motion planner should:
        1. Determine which arm is closer to object (if at start)
        2. Plan path for that arm to pick up object
        3. Plan path to handoff location
        4. Plan path for second arm to pick up
        5. Plan path for second arm to goal
        
        For now, this just returns the same state (no movement).
        """
        from src.problem_generator import SimulatorState, ObjectState
        
        # This is a placeholder - real planner will compute next state
        # For demo, just return current state (no movement)
        next_state = current_state.copy()
        next_state.step_count += 1
        
        # Placeholder logic: if at start, move to held by closer arm
        if current_state.object_state == ObjectState.AT_START:
            closer = prob_gen.find_closer_arm(current_state.object_position)
            if closer == 'left':
                next_state.object_state = ObjectState.HELD_BY_LEFT
            else:
                next_state.object_state = ObjectState.HELD_BY_RIGHT
        
        return next_state
    
    try:
        test_problem = prob_gen.generate_problem(mode='random')
        print(f"  Generated test problem:")
        print(f"    Object start: {test_problem.object_start.get_center()}")
        print(f"    Object goal: {test_problem.object_goal.get_center()}")
        print(f"    Start configs: Left={test_problem.start_config_left}, Right={test_problem.start_config_right}")
        
        # Run simulation (will stop quickly since planner doesn't actually move)
        states, success = prob_gen.simulate(
            test_problem, 
            placeholder_motion_planner,
            max_steps=5,  # Just a few steps for demo
            visualize=False,
            verbose=True
        )
        print(f"  Simulation completed: {len(states)} states, success={success}")
        print(f"  Final state: Object at {states[-1].object_position}, state={states[-1].object_state.value}")
        
    except Exception as e:
        print(f"  ✗ Error running simulator: {e}")
        import traceback
        traceback.print_exc()


def main():
    """Run all demonstrations."""
    print("🦾 Dual-Arm Motion Planning Simulator Demo")
    print("=" * 50)
    
    try:
        # Run demonstrations
        demo_single_arm()
        demo_dual_arm()
        demo_motion_planning_setup()
        demo_problem_generation()
        
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
