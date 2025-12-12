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
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

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
    # TODO: Get actual SO-ARM101 link lengths and joint limits
    # For now, using reasonable defaults for a small desktop arm
    robot_arm = Planar2LinkArm(
        base_position=(0.25, 0.0),  # Robot base shifted left (negative X)
        link1_length=0.05,
        link2_length=0.05,
        joint1_limits=(-np.pi, np.pi),  # Full rotation for joint 1
        joint2_limits=(-np.pi, np.pi)  # Full rotation for joint 2
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
    import time
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
                                
                                # Update visualization with target
                                plt.close(fig)
                                fig, ax, update_pose_plot = create_live_pose_visualization(
                                    environment_snapshot=snapshot,
                                    working_plane=working_plane,
                                    obstacle_boxes_2d=obstacle_boxes_2d,
                                    max_arm_reach=max_reach,
                                    robot_arm=robot_arm,
                                    handoff_target=handoff_target
                                )
                
                # Update 2D plane visualization
                update_pose_plot(pose_plane)
                
                # Display frame with pose
                display_frame = pose_plane['frame'].copy()
                cv2.putText(display_frame, f"Plane: Z = {working_plane.z0:.2f}m", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if max_reach is not None:
                    cv2.putText(display_frame, f"Max reach: {max_reach:.2f}m", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_frame, "Press 'q' to quit and save final visualization", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
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

