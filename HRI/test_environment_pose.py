"""
Test Environment + Plane + Pose Integration

Tests the full pipeline:
1. Load/create environment snapshot
2. Define working plane
3. Track human pose and project to plane
4. Visualize everything together
"""

import numpy as np
import cv2
import os
import matplotlib.pyplot as plt
from environment.environment_3d import Environment3D
from environment.environment_snapshot import EnvironmentSnapshot
from environment.working_plane import WorkingPlane
from perception.pose_tracker import PoseTracker
from robot.robot_workspace import Planar2LinkArm
import sys
import os

# Add current directory to path for relative imports
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

# Add robot directory to path for path_executor's relative imports
robot_dir = os.path.join(current_dir, 'robot')
if robot_dir not in sys.path:
    sys.path.insert(0, robot_dir)

from visualization.visualize_environment_pose import visualize_2d_environment_pose, create_live_pose_visualization


def create_environment_snapshot():
    """
    Create and save environment snapshot from depth pipeline.
    Run this once to create a reusable environment.
    """
    print("=" * 70)
    print("Creating Environment Snapshot")
    print("=" * 70)
    
    from perception.depth_estimator import DepthEstimator
    
    # Capture depth snapshot
    estimator = DepthEstimator()
    
    intrinsics_file = "calibration/camera_intrinsics.json"
    if not os.path.exists(intrinsics_file):
        intrinsics_file = None
    
    rgb_frame, depth_map, intrinsics = estimator.capture_depth_snapshot(
        camera_id=0,
        save_path="depth_snapshot",
        intrinsics_file=intrinsics_file
    )
    
    if rgb_frame is None or depth_map is None:
        print("ERROR: Failed to capture snapshot")
        return None
    
    # Build environment
    env = Environment3D(
        snapshot_path="depth_snapshot",
        safety_margin=0.1
    )
    
    # Extract obstacle boxes
    workspace_bounds = {
        'x_min': -1.0, 'x_max': 1.0,
        'y_min': -1.0, 'y_max': 1.0,
        'z_min': 0.0, 'z_max': 1.2
    }
    
    env.extract_obstacle_boxes(
        threshold_percentile=50.0,
        grid_cell_size=30,
        min_box_size=0.02,
        workspace_bounds=workspace_bounds,
        desk_height=0.6
    )
    
    # Create snapshot
    snapshot = EnvironmentSnapshot(environment=env)
    snapshot.save("environment_snapshot")
    
    print("\n✓ Environment snapshot created and saved")
    return snapshot


def test_environment_pose():
    """
    Test loading environment, defining plane, and tracking pose.
    """
    print("=" * 70)
    print("Testing Environment + Plane + Pose")
    print("=" * 70)
    
    # Step 1: Load environment snapshot
    print("\n[Step 1] Loading environment snapshot...")
    snapshot_path = "environment_snapshot"
    
    if not os.path.exists(f"{snapshot_path}_snapshot.npz"):
        print("  Snapshot not found. Creating new one...")
        snapshot = create_environment_snapshot()
        if snapshot is None:
            return
    else:
        snapshot = EnvironmentSnapshot(snapshot_path=snapshot_path)
    
    print("✓ Environment snapshot loaded")
    
    # Step 2: Define working plane
    print("\n[Step 2] Defining working plane...")
    
    # Auto-detect plane from point cloud
    print("  Generating point cloud for plane detection...")
    point_cloud = snapshot.get_point_cloud(downsample=4)  # Use smaller downsample for more points
    
    if len(point_cloud) == 0:
        print("ERROR: No point cloud available")
        print("  This might mean the depth snapshot has very few valid points.")
        print("  Try capturing a new snapshot with more objects in the scene.")
        return
    
    print(f"  Point cloud: {len(point_cloud)} points")
    
    working_plane = WorkingPlane.from_point_cloud(
        point_cloud,
        percentile=50.0,  # Use median X
        epsilon=0.05  # 5cm thickness
    )
    
    print(f"✓ Working plane: Z = {working_plane.z0:.3f}m")
    
    # Step 3: Extract 2D obstacles
    print("\n[Step 3] Extracting 2D obstacles...")
    env = snapshot.to_environment_3d()
    obstacle_boxes_2d = env.extract_2d_obstacles(working_plane)
    print(f"✓ Found {len(obstacle_boxes_2d)} 2D obstacle boxes")
    
    # Step 3.5: Connect to robot and align state
    print("\n[Step 3.5] Connecting to robot and aligning state...")
    robot_bus = None
    robot_mapper = None
    robot_executor = None
    robot_start_config = None
    
    try:
        from robot.path_executor import PathExecutor
        from robot.joint_mapper import JointMapper
        from robot.calibration.move_to_home import move_to_home, load_calibration, PORT, MOTORS
        from lerobot.motors.feetech import FeetechMotorsBus
        from lerobot.motors import Motor, MotorNormMode
        from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS
        import json
        from types import SimpleNamespace
        
        # Connect to robot
        print("  Connecting to robot...")
        calibration = load_calibration()
        robot_bus = FeetechMotorsBus(port=PORT, motors=MOTORS, calibration=calibration)
        robot_bus.connect(handshake=False)
        robot_bus.enable_torque(list(MOTORS.keys()))
        
        # Set speed and acceleration
        for name in MOTORS:
            robot_bus.write("Goal_Time", name, 0)
            robot_bus.write("Goal_Velocity", name, 200)
            robot_bus.write("Acceleration", name, 50)
        
        print("  ✓ Robot connected")
        
        # Move to HOME pose first
        print("  Moving robot to HOME pose...")
        move_to_home(robot_bus, verbose=True)
        
        # Create mapper from calibration file (uses HOME as reference)
        print("  Loading mapper from calibration...")
        robot_mapper = JointMapper.from_calibration_file()
        print(f"  ✓ Mapper loaded: HOME → θ_sim = (0, 0)")
        
        # Read current robot state (should be at HOME now)
        u_sh = robot_bus.read("Present_Position", "shoulder_lift", normalize=True)
        u_el = robot_bus.read("Present_Position", "elbow_flex", normalize=True)
        print(f"  Robot at HOME: u_sh={u_sh:.3f}, u_el={u_el:.3f}")
        
        # Convert to sim angles (should be (0, 0) since we're at HOME)
        theta1_start, theta2_start = robot_mapper.robot_to_sim(u_sh, u_el)
        robot_start_config = np.array([theta1_start, theta2_start, 0.0, 0.0])
        print(f"  Aligned sim state: θ1={theta1_start:.4f}, θ2={theta2_start:.4f}")
        print(f"  ✓ State alignment complete (robot at HOME = θ_sim = 0)")
        
        # Create executor (will be used after planning)
        robot_executor = PathExecutor(
            bus=robot_bus,
            mapper=robot_mapper,
            control_rate=30.0,
            max_delta_per_step=1.0,
            goal_tolerance=2.0,
            stall_timeout=2.0
        )
        
    except Exception as e:
        print(f"  ⚠️  Robot unavailable: {e}")
        print("  Continuing in simulation-only mode...")
        robot_bus = None
        robot_mapper = None
        robot_executor = None
        robot_start_config = None
    
    # Step 4: Initialize pose tracker
    print("\n[Step 4] Initializing pose tracker...")
    pose_tracker = PoseTracker(camera_id=0)
    if not pose_tracker.start():
        print("ERROR: Failed to start camera")
        return
    
    print("✓ Pose tracker ready")
    print("\n[Step 5] Tracking pose (press 'q' to quit)...")
    
    # Step 4: Initialize robot workspace
    print("\n[Step 4] Initializing robot workspace...")
    from robot.robot_config import (
        LINK1_LENGTH, LINK2_LENGTH, ROBOT_BASE_POSITION, JOINT_LIMITS
    )
    robot_arm = Planar2LinkArm(
        base_position=ROBOT_BASE_POSITION,
        link1_length=LINK1_LENGTH,
        link2_length=LINK2_LENGTH,
        joint1_limits=JOINT_LIMITS[0],
        joint2_limits=JOINT_LIMITS[1]
    )
    print(f"  Robot base: ({robot_arm.base_x:.3f}, {robot_arm.base_y:.3f})")
    print(f"  Link lengths: L1={robot_arm.L1:.3f}m, L2={robot_arm.L2:.3f}m")
    print(f"  Joint limits: θ1∈[{robot_arm.theta1_min:.2f}, {robot_arm.theta1_max:.2f}], "
          f"θ2∈[{robot_arm.theta2_min:.2f}, {robot_arm.theta2_max:.2f}]")
    
    # Create live visualization window (will be updated with max_reach after estimation)
    fig, ax, update_pose_plot = create_live_pose_visualization(
        environment_snapshot=snapshot,
        working_plane=working_plane,
        obstacle_boxes_2d=obstacle_boxes_2d,
        max_arm_reach=None,  # Will be set after estimation
        robot_arm=robot_arm,
        handoff_target=None  # Will be set after target selection
    )
    
    # Step 5: Estimate arm reach first
    print("\n[Step 5a] Estimating human arm reach...")
    print("  Move your left arm around to full extension for 5 seconds")
    import time  # Import time for timing operations
    arm_lengths = []
    start_time = time.time()
    duration = 5.0
    
    while time.time() - start_time < duration:
        frame = pose_tracker.get_frame()
        if frame is None:
            continue
        
        elapsed = time.time() - start_time
        
        pose_result = pose_tracker.detect_pose(frame)
        if pose_result is not None:
            pose_plane = pose_tracker.project_to_plane(
                pose_result,
                working_plane,
                snapshot.intrinsics
            )
            
            # Estimate arm length from this frame
            arm_length = pose_tracker.estimate_arm_length(pose_plane)
            if arm_length is not None:
                arm_lengths.append(arm_length)
                print(f"  [{elapsed:.1f}s] Arm length: {arm_length:.3f}m (samples: {len(arm_lengths)})")
            
            # Show preview
            display_frame = pose_plane['frame'].copy()
            cv2.putText(display_frame, f"Estimating reach... {elapsed:.1f}s / {duration:.1f}s", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Pose Tracking (Camera View)", display_frame)
        else:
            # Show preview even without pose
            display_frame = frame.copy()
            cv2.putText(display_frame, f"Estimating reach... {elapsed:.1f}s / {duration:.1f}s", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Pose Tracking (Camera View)", display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    
    # Calculate conservative arm reach estimate
    if len(arm_lengths) > 0:
        arm_lengths = np.array(arm_lengths)
        # Filter sudden jumps (>20% change from median)
        median_length = np.median(arm_lengths)
        valid_mask = np.abs(arm_lengths - median_length) < 0.2 * median_length
        filtered_lengths = arm_lengths[valid_mask]
        
        if len(filtered_lengths) > 0:
            # Use 95th percentile for conservative estimate
            max_reach = np.percentile(filtered_lengths, 95.0)
            print(f"\n✓ Estimated max arm reach: {max_reach:.3f}m (from {len(filtered_lengths)} samples)")
        else:
            max_reach = np.max(arm_lengths)
            print(f"\n✓ Estimated max arm reach: {max_reach:.3f}m (using max, {len(arm_lengths)} samples)")
    else:
        max_reach = None
        print("\n⚠ Could not estimate arm reach (no valid measurements)")
    
    # Step 5b: Live pose tracking loop
    print("\n[Step 5b] Tracking pose (press 'q' to quit)...")
    if max_reach is not None:
        print(f"  Human reachable workspace: {max_reach:.3f}m radius around left shoulder")
    frame_count = 0
    last_pose_data = None
    robot_path = None  # Store planned path for visualization
    
    # Update visualization with max_reach
    # We need to recreate the visualization function with max_reach
    plt.close(fig)
    fig, ax, update_pose_plot = create_live_pose_visualization(
        environment_snapshot=snapshot,
        working_plane=working_plane,
        obstacle_boxes_2d=obstacle_boxes_2d,
        max_arm_reach=max_reach,
        robot_arm=robot_arm
    )
    
    # Step 5a: Select handoff target (once, when we have pose data)
    handoff_target = None
    target_selected = False
    
    try:
        while True:
            frame = pose_tracker.get_frame()
            if frame is None:
                continue
            
            # Detect pose
            pose_result = pose_tracker.detect_pose(frame)
            
            if pose_result is not None:
                # Project to working plane
                pose_plane = pose_tracker.project_to_plane(
                    pose_result,
                    working_plane,
                    snapshot.intrinsics
                )
                last_pose_data = pose_plane
                
                # Step 5a: Select handoff target (once, when we have intersection)
                if not target_selected and max_reach is not None:
                    landmarks_2d_plane = pose_plane.get('landmarks_2d_plane', {})
                    if 'left_shoulder' in landmarks_2d_plane and 'left_wrist' in landmarks_2d_plane:
                        shoulder_x, shoulder_y = landmarks_2d_plane['left_shoulder']
                        wrist_x, wrist_y = landmarks_2d_plane['left_wrist']
                        
                        # Compute intersection
                        workspace_points = robot_arm.compute_workspace(grid_resolution=100)
                        intersection_points = robot_arm.compute_intersection_with_circle(
                            workspace_points,
                            (shoulder_x, shoulder_y),
                            max_reach
                        )
                        
                        if len(intersection_points) > 0:
                            # Select target with clearance
                            handoff_target = robot_arm.select_handoff_target(
                                intersection_points,
                                obstacle_boxes_2d,
                                wrist_position=(wrist_x, wrist_y),
                                clearance=0.05  # 5cm clearance
                            )
                            
                            if handoff_target is not None:
                                print(f"\n[Step 5a] Handoff target selected: ({handoff_target[0]:.3f}, {handoff_target[1]:.3f})")
                                print(f"  Target is {np.linalg.norm(np.array(handoff_target) - np.array([wrist_x, wrist_y])):.3f}m from wrist")
                                target_selected = True
                                
                                # Step 5b: Plan robot path to handoff target
                                print(f"\n[Step 5b] Planning robot path to handoff target...")
                                from planning.rrt_planner import (
                                    convert_obstacles_2d_to_planner,
                                    setup_dual_arm_system,
                                    validate_start_config,
                                    plan_to_target
                                )
                                
                                # Toggle obstacle avoidance (set to False to disable)
                                USE_OBSTACLES = False
                                
                                if USE_OBSTACLES:
                                    # Convert obstacles
                                    planner_obstacles = convert_obstacles_2d_to_planner(obstacle_boxes_2d)
                                    print(f"  Using {len(planner_obstacles)} obstacles")
                                else:
                                    planner_obstacles = []
                                    print(f"  Obstacle avoidance DISABLED (testing mode)")
                                
                                # Setup DualArm
                                dual_arm = setup_dual_arm_system(obstacles=planner_obstacles)
                                
                                # Use robot's current state as start config (if available)
                                if robot_start_config is not None:
                                    # Validate the robot's current state
                                    if dual_arm.is_valid_configuration(robot_start_config):
                                        start_config = robot_start_config
                                        print(f"  Using robot's current state as start: [{start_config[0]:.3f}, {start_config[1]:.3f}, 0, 0]")
                                    else:
                                        print("  ✗ ERROR: Robot pose invalid, move to home and rerun")
                                        print(f"    Robot state: θ1={robot_start_config[0]:.3f}, θ2={robot_start_config[1]:.3f}")
                                        robot_path = None
                                        break
                                else:
                                    # Simulation-only mode: use default start
                                    start_config = validate_start_config(dual_arm)
                                
                                # Plan path
                                path, goal_config = plan_to_target(
                                    handoff_target_global=handoff_target,
                                    start_config=start_config,
                                    dual_arm=dual_arm,
                                    max_iterations=3000,
                                    step_size=0.05,
                                    goal_threshold=0.02  # Tighter threshold for better precision
                                )
                                
                                if path is not None:
                                    print(f"  ✓ Path planning complete: {len(path)} waypoints")
                                    robot_path = path
                                    
                                    # Execute path on real robot (if available)
                                    if robot_executor is not None:
                                        print(f"\n[Step 5c] Robot execution ready")
                                        user_input = input("  Press Enter to execute path on robot, or 'q' to skip: ")
                                        
                                        if user_input.strip().lower() != 'q':
                                            # Execute path
                                            print(f"\n  Executing path on robot...")
                                            result = robot_executor.execute_path(path, verbose=True)
                                            
                                            if result['success']:
                                                print(f"  ✓ Path execution complete!")
                                            else:
                                                print(f"  ⚠️  Path execution incomplete: {result['waypoints_executed']}/{result['total_waypoints']} waypoints")
                                        else:
                                            print(f"  Skipping robot execution")
                                    else:
                                        print(f"  (Simulation-only mode: no robot execution)")
                                else:
                                    print(f"  ✗ Path planning failed")
                                    robot_path = None
                                
                                # Update visualization with target, start, goal, and path
                                plt.close(fig)
                                fig, ax, update_pose_plot = create_live_pose_visualization(
                                    environment_snapshot=snapshot,
                                    working_plane=working_plane,
                                    obstacle_boxes_2d=obstacle_boxes_2d,
                                    max_arm_reach=max_reach,
                                    robot_arm=robot_arm,
                                    handoff_target=handoff_target,
                                    start_config=start_config,
                                    goal_config=goal_config,
                                    robot_path=robot_path
                                )
                                
                                # Animate the robot arm moving along the path
                                if robot_path is not None and len(robot_path) > 0:
                                    from visualization.visualize_environment_pose import animate_robot_path
                                    from robot.robot_config import ROBOT_BASE_POSITION
                                    animate_robot_path(ax, robot_path, ROBOT_BASE_POSITION, animation_speed=0.05)
                
                # Update 2D plane visualization
                update_pose_plot(pose_plane)
                
                # Display frame with pose
                display_frame = pose_plane['frame'].copy()
                cv2.putText(display_frame, f"Plane: Z = {working_plane.z0:.2f}m", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if max_reach is not None:
                    cv2.putText(display_frame, f"Max reach: {max_reach:.2f}m", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if handoff_target is not None:
                    cv2.putText(display_frame, f"Target: ({handoff_target[0]:.2f}, {handoff_target[1]:.2f})",
                               (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                if robot_path is not None:
                    cv2.putText(display_frame, f"Path: {len(robot_path)} waypoints",
                               (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_frame, "Press 'q' to quit and save final visualization", 
                           (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                cv2.imshow("Pose Tracking (Camera View)", display_frame)
            else:
                # No pose detected - clear pose from plot
                update_pose_plot(None)
                cv2.imshow("Pose Tracking (Camera View)", frame)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            
            frame_count += 1
    
    finally:
        pose_tracker.stop()
        cv2.destroyAllWindows()
        plt.close(fig)  # Close matplotlib figure
        
        # Return robot to HOME and disconnect
        if robot_bus is not None:
            print("\n[Cleanup] Returning robot to HOME...")
            try:
                from robot.calibration.move_to_home import move_to_home
                move_to_home(robot_bus, verbose=True)
                
                # Disable torque on all motors
                motor_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
                for name in motor_names:
                    try:
                        robot_bus.write("Torque_Enable", name, 0)
                    except:
                        pass
                time.sleep(0.5)
                robot_bus.disconnect(disable_torque=False)
                print("  ✓ Robot returned to HOME and disconnected")
            except Exception as e:
                print(f"  ⚠️  Error during cleanup: {e}")
                try:
                    robot_bus.disconnect(disable_torque=False)
                except:
                    pass
    
    # Step 6: Visualize
    print("\n[Step 6] Creating visualization...")
    visualize_2d_environment_pose(
        environment_snapshot=snapshot,
        working_plane=working_plane,
        pose_data=last_pose_data,
        obstacle_boxes_2d=obstacle_boxes_2d,
        max_arm_reach=max_reach,
        robot_arm=robot_arm,
        save_path="environment_pose_visualization.png"
    )
    
    print("\n" + "=" * 70)
    print("Test complete!")
    print("=" * 70)


if __name__ == "__main__":
    test_environment_pose()

