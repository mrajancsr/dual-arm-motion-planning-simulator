#!/usr/bin/env python3
"""
Simple test script for the problem generator and simulator.
"""

from src.problem_generator import ProblemGenerator, SimulatorState, ObjectState
from src.two_link_arm import DualArm
from src.objects import PointObject
import numpy as np
import matplotlib.pyplot as plt


def simple_motion_planner(problem, current_state):
    """
    Simple motion planner for testing.
    
    This planner demonstrates the interface - it doesn't actually plan,
    just moves the object through the states to show the workflow.
    """
    next_state = current_state.copy()
    next_state.step_count += 1
    
    # Simple state machine to demonstrate the workflow
    if current_state.object_state == ObjectState.AT_START:
        # Determine which arm is closer
        dist_left = np.linalg.norm(current_state.object_position - problem.dual_arm.left_base)
        dist_right = np.linalg.norm(current_state.object_position - problem.dual_arm.right_base)
        closer = 'left' if dist_left < dist_right else 'right'
        if closer == 'left':
            next_state.object_state = ObjectState.HELD_BY_LEFT
            # Move object to left arm's end effector position
            left_pos = problem.dual_arm.left_arm.forward_kinematics(*current_state.left_arm_config)
            next_state.object_position = problem.dual_arm.left_base + left_pos
        else:
            next_state.object_state = ObjectState.HELD_BY_RIGHT
            right_pos = problem.dual_arm.right_arm.forward_kinematics(*current_state.right_arm_config)
            next_state.object_position = problem.dual_arm.right_base + right_pos
            
    elif current_state.object_state == ObjectState.HELD_BY_LEFT:
        # Move to handoff location (center between arms)
        handoff_pos = (problem.dual_arm.left_base + problem.dual_arm.right_base) / 2
        handoff_pos[1] += 0.5  # Raise it a bit
        next_state.object_position = handoff_pos
        next_state.object_state = ObjectState.AT_HANDOFF
        
    elif current_state.object_state == ObjectState.AT_HANDOFF:
        # Right arm picks it up
        next_state.object_state = ObjectState.HELD_BY_RIGHT
        right_pos = problem.dual_arm.right_arm.forward_kinematics(*current_state.right_arm_config)
        next_state.object_position = problem.dual_arm.right_base + right_pos
        
    elif current_state.object_state == ObjectState.HELD_BY_RIGHT:
        # Move to goal
        goal_center = problem.object_goal.get_center()
        next_state.object_position = goal_center
        next_state.object_state = ObjectState.AT_GOAL
    
    return next_state


def test_problem_generation():
    """Test problem generation."""
    print("=" * 60)
    print("Test 1: Problem Generation")
    print("=" * 60)
    
    dual = DualArm(L1=1.0, L2=0.7, separation=2.0)
    pg = ProblemGenerator(dual)
    
    # Generate random problem
    problem = pg.generate_problem(mode='random', object_type='point')
    
    print(f"✓ Problem generated!")
    print(f"  Object start: {problem.object_start.get_center()}")
    print(f"  Object goal: {problem.object_goal.get_center()}")
    print(f"  Start config left: {problem.start_config_left}")
    print(f"  Start config right: {problem.start_config_right}")
    
    # Get initial state
    initial_state = problem.get_initial_state()
    print(f"\n✓ Initial state created:")
    print(f"  Object position: {initial_state.object_position}")
    print(f"  Object state: {initial_state.object_state.value}")
    print(f"  Step count: {initial_state.step_count}")
    
    return problem, pg


def test_simulator(problem, pg):
    """Test the simulator loop."""
    print("\n" + "=" * 60)
    print("Test 2: Simulator Loop")
    print("=" * 60)
    
    # Run simulation with simple planner
    states, success = pg.simulate(
        problem,
        simple_motion_planner,
        max_steps=10,
        visualize=False,
        verbose=True
    )
    
    print(f"\n✓ Simulation completed!")
    print(f"  Total states: {len(states)}")
    print(f"  Success: {success}")
    print(f"\n  State progression:")
    for i, state in enumerate(states):
        print(f"    Step {i}: {state.object_state.value} at {state.object_position}")
    
    return states


def test_visualization(problem, states):
    """Test visualization."""
    print("\n" + "=" * 60)
    print("Test 3: Visualization")
    print("=" * 60)
    
    # Visualize initial problem
    print("Visualizing initial problem...")
    fig, ax = plt.subplots(figsize=(10, 8))
    problem.visualize(state=None, ax=ax)
    ax.set_title("Initial Problem Setup")
    plt.tight_layout()
    plt.show()
    
    # Visualize final state
    if states:
        print("Visualizing final state...")
        fig, ax = plt.subplots(figsize=(10, 8))
        problem.visualize(state=states[-1], ax=ax)
        ax.set_title(f"Final State (Step {states[-1].step_count})")
        plt.tight_layout()
        plt.show()


def test_user_specified_problem():
    """Test user-specified problem generation."""
    print("\n" + "=" * 60)
    print("Test 4: User-Specified Problem")
    print("=" * 60)
    
    dual = DualArm(L1=1.0, L2=0.7, separation=2.0)
    pg = ProblemGenerator(dual)
    
    # User specifies object positions (within collaborative workspace)
    obj_start = PointObject(0.0, 0.8)
    obj_goal = PointObject(0.2, 1.0)  # Adjusted to be in workspace
    
    problem = pg.generate_problem(
        mode='user',
        object_start=obj_start,
        object_goal=obj_goal
    )
    
    print(f"✓ User-specified problem generated!")
    print(f"  Object start: {problem.object_start.get_center()}")
    print(f"  Object goal: {problem.object_goal.get_center()}")
    
    return problem


def main():
    """Run all tests."""
    print("🧪 Testing Problem Generator and Simulator")
    print("=" * 60)
    
    try:
        # Test 1: Problem generation
        problem, pg = test_problem_generation()
        
        # Test 2: Simulator loop
        states = test_simulator(problem, pg)
        
        # Test 3: Visualization (commented out to avoid blocking)
        # Uncomment to see visualizations:
        # test_visualization(problem, states)
        
        # Test 4: User-specified problem
        user_problem = test_user_specified_problem()
        
        print("\n" + "=" * 60)
        print("✅ All tests completed successfully!")
        print("=" * 60)
        print("\nTo see visualizations, uncomment test_visualization() in main()")
        print("Or run: python demo.py")
        
    except Exception as e:
        print(f"\n❌ Error during testing: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

