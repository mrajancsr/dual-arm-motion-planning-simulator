"""
Test Step 2: Obstacle Conversion + Collision Validity Checks

Simple test script to verify obstacle conversion and collision checking work.
"""

import numpy as np
from environment.environment_snapshot import EnvironmentSnapshot
from environment.working_plane import WorkingPlane
from environment.environment_3d import Environment3D
from planning.rrt_planner import (
    convert_obstacles_2d_to_planner,
    setup_dual_arm_system,
    validate_start_config,
    test_collision_checking
)


def test_step2():
    """Test Step 2 functionality."""
    print("=" * 70)
    print("Step 2 Test: Obstacle Conversion + Collision Validity")
    print("=" * 70)
    
    # Load environment snapshot
    print("\n[1] Loading environment snapshot...")
    snapshot = EnvironmentSnapshot(snapshot_path="environment_snapshot")
    print("✓ Snapshot loaded")
    
    # Define working plane
    print("\n[2] Defining working plane...")
    point_cloud = snapshot.get_point_cloud(downsample=8)
    working_plane = WorkingPlane.from_point_cloud(point_cloud, percentile=50.0)
    print(f"✓ Working plane: Z = {working_plane.z0:.3f}m")
    
    # Extract 2D obstacles
    print("\n[3] Extracting 2D obstacles...")
    env = snapshot.to_environment_3d()
    obstacle_boxes_2d = env.extract_2d_obstacles(working_plane)
    print(f"✓ Found {len(obstacle_boxes_2d)} 2D obstacle boxes")
    
    # Convert obstacles
    print("\n[4] Converting obstacles to DualArm format...")
    planner_obstacles = convert_obstacles_2d_to_planner(obstacle_boxes_2d)
    print(f"✓ Converted {len(planner_obstacles)} obstacles")
    if len(planner_obstacles) > 0:
        print(f"  Example obstacle: {planner_obstacles[0]}")
    
    # Setup DualArm system
    print("\n[5] Setting up DualArm system...")
    dual_arm = setup_dual_arm_system(obstacles=planner_obstacles)
    print(f"✓ DualArm created")
    print(f"  Left base: {dual_arm.left_base}")
    print(f"  Right base: {dual_arm.right_base}")
    print(f"  Left arm: L1={dual_arm.left_arm.L1:.3f}m, L2={dual_arm.left_arm.L2:.3f}m")
    print(f"  Right arm: L1={dual_arm.right_arm.L1:.3f}m, L2={dual_arm.right_arm.L2:.3f}m")
    print(f"  Obstacles: {len(dual_arm.obstacles)}")
    
    # Test collision checking
    test_collision_checking(dual_arm)
    
    # Validate start config
    print("\n[6] Validating start configuration...")
    start_config = validate_start_config(dual_arm)
    print(f"✓ Start config: [{start_config[0]:.3f}, {start_config[1]:.3f}, {start_config[2]:.3f}, {start_config[3]:.3f}]")
    
    print("\n" + "=" * 70)
    print("Step 2 Test Complete!")
    print("=" * 70)


if __name__ == "__main__":
    test_step2()

