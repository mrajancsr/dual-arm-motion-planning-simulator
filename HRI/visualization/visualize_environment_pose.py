"""
Visualization Module

Visualizes environment, working plane, and human pose together.
"""

import numpy as np
import matplotlib
try:
    matplotlib.use('MacOSX')
except:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mediapipe as mp
from typing import Dict, List, Optional, Tuple
from environment.environment_snapshot import EnvironmentSnapshot
from environment.working_plane import WorkingPlane
try:
    from robot.robot_workspace import Planar2LinkArm
except ImportError:
    Planar2LinkArm = None  # Type: ignore


def visualize_2d_environment_pose(environment_snapshot: EnvironmentSnapshot,
                                   working_plane: WorkingPlane,
                                   pose_data: Optional[Dict] = None,
                                   obstacle_boxes_2d: Optional[List[Dict]] = None,
                                   max_arm_reach: Optional[float] = None,
                                   robot_arm: Optional[Planar2LinkArm] = None,
                                   handoff_target: Optional[Tuple[float, float]] = None,
                                   save_path: Optional[str] = None):
    """
    Visualize 2D environment with obstacles, plane, human pose, and robot workspace.
    
    Args:
        environment_snapshot: EnvironmentSnapshot instance
        working_plane: WorkingPlane instance
        pose_data: Optional pose data from PoseTracker.project_to_plane()
        obstacle_boxes_2d: Optional list of 2D obstacle boxes
        max_arm_reach: Optional maximum arm reach in meters (draws red circle around left shoulder)
        robot_arm: Optional Planar2LinkArm instance (draws robot workspace)
        save_path: Optional path to save figure
    """
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Get point cloud and filter to plane
    point_cloud = environment_snapshot.get_point_cloud(downsample=8)
    
    if len(point_cloud) > 0:
        # Filter points in plane
        in_plane_points = []
        for point in point_cloud:
            if working_plane.in_plane(point):
                in_plane_points.append(point)
        
        if len(in_plane_points) > 0:
            in_plane_points = np.array(in_plane_points)
            points_2d = np.array([working_plane.project_to_2d(p) for p in in_plane_points])
            
            # Plot point cloud in 2D
            ax.scatter(points_2d[:, 0], points_2d[:, 1], 
                      c=in_plane_points[:, 2], cmap='viridis', 
                      alpha=0.3, s=1, label='Environment Points')
    
    # Plot 2D obstacle boxes
    if obstacle_boxes_2d is not None and len(obstacle_boxes_2d) > 0:
        for i, box in enumerate(obstacle_boxes_2d):
            x_min, x_max = box['x_min'], box['x_max']
            y_min, y_max = box['y_min'], box['y_max']
            
            # Draw rectangle
            rect = plt.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                               fill=True, alpha=0.3, edgecolor='red', 
                               facecolor='red', label='Obstacle' if i == 0 else '')
            ax.add_patch(rect)
    
    # Plot human pose skeleton
    if pose_data is not None:
        landmarks_2d_plane = pose_data.get('landmarks_2d_plane', {})
        
        if len(landmarks_2d_plane) > 0:
            # Map MediaPipe landmark names to indices for POSE_CONNECTIONS
            landmark_name_to_idx = {
                'nose': 0, 'left_eye_inner': 1, 'left_eye': 2, 'left_eye_outer': 3,
                'right_eye_inner': 4, 'right_eye': 5, 'right_eye_outer': 6,
                'left_ear': 7, 'right_ear': 8, 'mouth_left': 9, 'mouth_right': 10,
                'left_shoulder': 11, 'right_shoulder': 12, 'left_elbow': 13, 'right_elbow': 14,
                'left_wrist': 15, 'right_wrist': 16, 'left_pinky': 17, 'right_pinky': 18,
                'left_index': 19, 'right_index': 20, 'left_thumb': 21, 'right_thumb': 22,
                'left_hip': 23, 'right_hip': 24, 'left_knee': 25, 'right_knee': 26,
                'left_ankle': 27, 'right_ankle': 28, 'left_heel': 29, 'right_heel': 30,
                'left_foot_index': 31, 'right_foot_index': 32
            }
            
            # Use MediaPipe's POSE_CONNECTIONS
            mp_pose = mp.solutions.pose
            connections = mp_pose.POSE_CONNECTIONS
            
            # Draw skeleton connections
            for connection in connections:
                start_idx, end_idx = connection
                
                # Find landmark names for these indices
                start_name = None
                end_name = None
                for name, idx in landmark_name_to_idx.items():
                    if idx == start_idx:
                        start_name = name
                    if idx == end_idx:
                        end_name = name
                
                # Draw if both landmarks are available
                if start_name and end_name and start_name in landmarks_2d_plane and end_name in landmarks_2d_plane:
                    x1, y1 = landmarks_2d_plane[start_name]
                    x2, y2 = landmarks_2d_plane[end_name]
                    
                    # Skip connections that are too long (likely projection errors)
                    dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    if dist > 2.0:  # Skip connections longer than 2 meters (likely wrong)
                        continue
                    
                    ax.plot([x1, x2], [y1, y2], 'b-', linewidth=2, alpha=0.7, zorder=5)
            
            # Draw all keypoints
            for name, (x, y) in landmarks_2d_plane.items():
                # Different colors for different body parts
                if 'wrist' in name:
                    color = 'green'
                    size = 150
                    marker = 'o'
                elif 'elbow' in name or 'shoulder' in name:
                    color = 'blue'
                    size = 100
                    marker = 'o'
                elif 'hip' in name or 'knee' in name or 'ankle' in name:
                    color = 'purple'
                    size = 80
                    marker = 'o'
                else:
                    color = 'gray'
                    size = 50
                    marker = '.'
                
                ax.scatter([x], [y], c=color, s=size, marker=marker, 
                         edgecolors='black', linewidths=1, zorder=10, alpha=0.8)
            
            # Highlight wrists (handoff points) with star
            if 'left_wrist' in landmarks_2d_plane:
                x, y = landmarks_2d_plane['left_wrist']
                ax.scatter([x], [y], c='lime', s=300, marker='*', 
                         edgecolors='black', linewidths=2, zorder=12, label='Left Hand')
            
            if 'right_wrist' in landmarks_2d_plane:
                x, y = landmarks_2d_plane['right_wrist']
                ax.scatter([x], [y], c='lime', s=300, marker='*', 
                         edgecolors='black', linewidths=2, zorder=12, label='Right Hand')
            
            # Draw human reachable workspace (disk around left shoulder)
            if max_arm_reach is not None and 'left_shoulder' in landmarks_2d_plane:
                shoulder_x, shoulder_y = landmarks_2d_plane['left_shoulder']
                circle = plt.Circle((shoulder_x, shoulder_y), max_arm_reach,
                                   fill=False, edgecolor='red', linewidth=2,
                                   linestyle='--', alpha=0.7, zorder=3,
                                   label='Human Reachable Workspace')
                ax.add_patch(circle)
        
        # Draw robot workspace
        if robot_arm is not None:
            workspace_points = robot_arm.compute_workspace(grid_resolution=100)
            
            # Plot workspace points
            ax.scatter(workspace_points[:, 0], workspace_points[:, 1],
                      c='blue', s=1, alpha=0.3, zorder=2, label='Robot Workspace')
            
            # Plot robot base
            ax.scatter([robot_arm.base_x], [robot_arm.base_y],
                      c='darkblue', s=200, marker='s', zorder=11,
                      edgecolors='black', linewidths=2, label='Robot Base')
            
            # Compute and visualize intersection with human reach
            if max_arm_reach is not None and 'left_shoulder' in landmarks_2d_plane:
                shoulder_x, shoulder_y = landmarks_2d_plane['left_shoulder']
                intersection_points = robot_arm.compute_intersection_with_circle(
                    workspace_points,
                    (shoulder_x, shoulder_y),
                    max_arm_reach
                )
                
                if len(intersection_points) > 0:
                    # Plot intersection points in purple with high opacity
                    ax.scatter(intersection_points[:, 0], intersection_points[:, 1],
                              c='purple', s=8, alpha=0.7, zorder=4,
                              label='Handoff Region', edgecolors='darkviolet', linewidths=0.5)
            
            # Try to draw convex hull if possible
            try:
                from scipy.spatial import ConvexHull
                if len(workspace_points) >= 3:
                    hull = ConvexHull(workspace_points)
                    for simplex in hull.simplices:
                        ax.plot(workspace_points[simplex, 0], workspace_points[simplex, 1],
                               'b-', linewidth=1.5, alpha=0.6, zorder=1)
            except:
                pass  # Skip hull if scipy not available or insufficient points
        
        # Draw handoff target if specified
        if handoff_target is not None:
            target_x, target_y = handoff_target
            ax.scatter([target_x], [target_y],
                      c='orange', s=400, marker='*', zorder=15,
                      edgecolors='darkorange', linewidths=2, label='Handoff Target')
    
    ax.set_xlabel('X (m) - Left/Right')
    ax.set_ylabel('Y (m) - Up/Down')
    ax.set_title(f'2D Working Plane (Z = {working_plane.z0:.3f}m)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Visualization saved to: {save_path}")
    
    try:
        plt.show()
    except:
        pass


def create_live_pose_visualization(environment_snapshot: EnvironmentSnapshot,
                                   working_plane: WorkingPlane,
                                   obstacle_boxes_2d: Optional[List[Dict]] = None,
                                   max_arm_reach: Optional[float] = None,
                                   robot_arm: Optional[Planar2LinkArm] = None,
                                   handoff_target: Optional[Tuple[float, float]] = None,
                                   start_config: Optional[np.ndarray] = None,
                                   goal_config: Optional[np.ndarray] = None,
                                   robot_path: Optional[List[np.ndarray]] = None):
    """
    Create a live-updating matplotlib figure for real-time pose visualization.
    
    Args:
        environment_snapshot: EnvironmentSnapshot instance
        working_plane: WorkingPlane instance
        obstacle_boxes_2d: Optional list of 2D obstacle boxes
        max_arm_reach: Optional maximum arm reach in meters (draws red circle around left shoulder)
        robot_arm: Optional Planar2LinkArm instance (draws robot workspace)
        handoff_target: Optional (x, y) target point for handoff (highlighted)
        start_config: Optional start configuration [θ1, θ2, 0, 0] (draws arm at start)
        goal_config: Optional goal configuration [θ1, θ2, 0, 0] (draws arm at goal)
        robot_path: Optional list of configurations (draws end-effector path trace)
    
    Returns:
        Tuple of (fig, ax, update_function) where update_function(pose_data) updates the plot
    """
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Get point cloud and filter to plane (static background)
    point_cloud = environment_snapshot.get_point_cloud(downsample=8)
    
    if len(point_cloud) > 0:
        # Filter points in plane
        in_plane_points = []
        for point in point_cloud:
            if working_plane.in_plane(point):
                in_plane_points.append(point)
        
        if len(in_plane_points) > 0:
            in_plane_points = np.array(in_plane_points)
            points_2d = np.array([working_plane.project_to_2d(p) for p in in_plane_points])
            
            # Plot point cloud in 2D (static)
            ax.scatter(points_2d[:, 0], points_2d[:, 1], 
                      c=in_plane_points[:, 2], cmap='viridis', 
                      alpha=0.3, s=1, label='Environment Points')
    
    # Plot 2D obstacle boxes (static)
    if obstacle_boxes_2d is not None and len(obstacle_boxes_2d) > 0:
        for i, box in enumerate(obstacle_boxes_2d):
            x_min, x_max = box['x_min'], box['x_max']
            y_min, y_max = box['y_min'], box['y_max']
            
            rect = plt.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                               fill=True, alpha=0.3, edgecolor='red', 
                               facecolor='red', label='Obstacle' if i == 0 else '')
            ax.add_patch(rect)
    
    # Draw robot workspace (static background)
    if robot_arm is not None:
        workspace_points = robot_arm.compute_workspace(grid_resolution=100)
        
        # Plot workspace points
        ax.scatter(workspace_points[:, 0], workspace_points[:, 1],
                  c='blue', s=2, alpha=0.4, zorder=2, label='Robot Workspace')
        
        # Plot robot base
        ax.scatter([robot_arm.base_x], [robot_arm.base_y],
                  c='darkblue', s=200, marker='s', zorder=11,
                  edgecolors='black', linewidths=2, label='Robot Base')
        
        # Note: Intersection with human reach will be computed in update_pose() 
        # when pose_data is available
        
        # Try to draw convex hull if possible
        try:
            from scipy.spatial import ConvexHull
            if len(workspace_points) >= 3:
                hull = ConvexHull(workspace_points)
                for simplex in hull.simplices:
                    ax.plot(workspace_points[simplex, 0], workspace_points[simplex, 1],
                           'b-', linewidth=1.5, alpha=0.6, zorder=1)
        except:
            pass  # Skip hull if scipy not available or insufficient points
        
        # Draw handoff target if specified (static background)
        if handoff_target is not None:
            target_x, target_y = handoff_target
            ax.scatter([target_x], [target_y],
                      c='orange', s=400, marker='*', zorder=15,
                      edgecolors='darkorange', linewidths=2, label='Handoff Target')
    
    # Draw robot arm at start and goal configurations (Step 4)
    from robot.robot_config import ROBOT_BASE_POSITION, LINK1_LENGTH, LINK2_LENGTH
    
    def draw_robot_arm(ax, theta1, theta2, base_pos, color, label, linestyle='-', linewidth=3, alpha=1.0):
        """Draw a 2-link robot arm given joint angles (θ_sim)."""
        # Apply θ_home offset for visualization
        try:
            from robot.calibration.theta_home import apply_offset
            theta1_physical, theta2_physical = apply_offset(theta1, theta2)
        except ImportError:
            # Fallback if calibration not available
            theta1_physical, theta2_physical = theta1, theta2
        
        base_x, base_y = base_pos
        
        # Compute joint positions using forward kinematics (with offset applied)
        # Joint 1 (elbow)
        elbow_x = base_x + LINK1_LENGTH * np.cos(theta1_physical)
        elbow_y = base_y + LINK1_LENGTH * np.sin(theta1_physical)
        
        # End-effector
        ee_x = base_x + LINK1_LENGTH * np.cos(theta1_physical) + LINK2_LENGTH * np.cos(theta1_physical + theta2_physical)
        ee_y = base_y + LINK1_LENGTH * np.sin(theta1_physical) + LINK2_LENGTH * np.sin(theta1_physical + theta2_physical)
        
        # Draw arm segments
        ax.plot([base_x, elbow_x], [base_y, elbow_y], 
               color=color, linestyle=linestyle, linewidth=linewidth, 
               alpha=alpha, zorder=8, label=label)
        ax.plot([elbow_x, ee_x], [elbow_y, ee_y], 
               color=color, linestyle=linestyle, linewidth=linewidth, 
               alpha=alpha, zorder=8)
        
        # Draw joints
        ax.scatter([base_x], [base_y], c=color, s=100, marker='s', 
                  zorder=9, edgecolors='black', linewidths=1, alpha=alpha)
        ax.scatter([elbow_x], [elbow_y], c=color, s=80, marker='o', 
                  zorder=9, edgecolors='black', linewidths=1, alpha=alpha)
        ax.scatter([ee_x], [ee_y], c=color, s=120, marker='D', 
                  zorder=9, edgecolors='black', linewidths=1, alpha=alpha)
        
        return (ee_x, ee_y)
    
    # Draw start configuration
    if start_config is not None:
        theta1_start = start_config[0]
        theta2_start = start_config[1]
        draw_robot_arm(ax, theta1_start, theta2_start, ROBOT_BASE_POSITION,
                      color='green', label='Start Config', linestyle='-', linewidth=3, alpha=0.8)
    
    # Draw goal configuration
    if goal_config is not None:
        theta1_goal = goal_config[0]
        theta2_goal = goal_config[1]
        draw_robot_arm(ax, theta1_goal, theta2_goal, ROBOT_BASE_POSITION,
                      color='red', label='Goal Config', linestyle='-', linewidth=3, alpha=0.8)
    
    # Draw end-effector path trace
    if robot_path is not None and len(robot_path) > 0:
        ee_trace_x = []
        ee_trace_y = []
        
        for config in robot_path:
            theta1 = config[0]
            theta2 = config[1]
            base_x, base_y = ROBOT_BASE_POSITION
            ee_x = base_x + LINK1_LENGTH * np.cos(theta1) + LINK2_LENGTH * np.cos(theta1 + theta2)
            ee_y = base_y + LINK1_LENGTH * np.sin(theta1) + LINK2_LENGTH * np.sin(theta1 + theta2)
            ee_trace_x.append(ee_x)
            ee_trace_y.append(ee_y)
        
        # Draw path trace
        ax.plot(ee_trace_x, ee_trace_y, 
               color='purple', linestyle='--', linewidth=2, alpha=0.6, 
               zorder=7, label=f'Path Trace ({len(robot_path)} waypoints)')
    
    ax.set_xlabel('X (m) - Left/Right')
    ax.set_ylabel('Y (m) - Up/Down')
    ax.set_title(f'2D Working Plane (Z = {working_plane.z0:.3f}m) - Live Pose Tracking')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    plt.tight_layout()
    plt.show(block=False)
    
    # Store plot elements for updating
    pose_lines = []
    pose_points = []
    wrist_markers = []
    reach_circles = []  # Track the reachable workspace circle
    intersection_points_plot = [None]  # Track intersection points plot (use list for closure)
    
    # Map MediaPipe landmark names to indices
    landmark_name_to_idx = {
        'nose': 0, 'left_eye_inner': 1, 'left_eye': 2, 'left_eye_outer': 3,
        'right_eye_inner': 4, 'right_eye': 5, 'right_eye_outer': 6,
        'left_ear': 7, 'right_ear': 8, 'mouth_left': 9, 'mouth_right': 10,
        'left_shoulder': 11, 'right_shoulder': 12, 'left_elbow': 13, 'right_elbow': 14,
        'left_wrist': 15, 'right_wrist': 16, 'left_pinky': 17, 'right_pinky': 18,
        'left_index': 19, 'right_index': 20, 'left_thumb': 21, 'right_thumb': 22,
        'left_hip': 23, 'right_hip': 24, 'left_knee': 25, 'right_knee': 26,
        'left_ankle': 27, 'right_ankle': 28, 'left_heel': 29, 'right_heel': 30,
        'left_foot_index': 31, 'right_foot_index': 32
    }
    
    mp_pose = mp.solutions.pose
    connections = mp_pose.POSE_CONNECTIONS
    
    def update_pose(pose_data: Optional[Dict]):
        """Update the plot with new pose data."""
        # Clear previous pose elements
        for line in pose_lines:
            line.remove()
        pose_lines.clear()
        
        for point in pose_points:
            point.remove()
        pose_points.clear()
        
        for marker in wrist_markers:
            marker.remove()
        wrist_markers.clear()
        
        # Remove old reach circle
        for circle in reach_circles:
            circle.remove()
        reach_circles.clear()
        
        # Remove old intersection points
        if intersection_points_plot[0] is not None:
            intersection_points_plot[0].remove()
            intersection_points_plot[0] = None
        
        if pose_data is None:
            plt.draw()
            plt.pause(0.01)
            return
        
        landmarks_2d_plane = pose_data.get('landmarks_2d_plane', {})
        
        if len(landmarks_2d_plane) == 0:
            plt.draw()
            plt.pause(0.01)
            return
        
        # Draw skeleton connections
        for connection in connections:
            start_idx, end_idx = connection
            
            start_name = None
            end_name = None
            for name, idx in landmark_name_to_idx.items():
                if idx == start_idx:
                    start_name = name
                if idx == end_idx:
                    end_name = name
            
            if start_name and end_name and start_name in landmarks_2d_plane and end_name in landmarks_2d_plane:
                x1, y1 = landmarks_2d_plane[start_name]
                x2, y2 = landmarks_2d_plane[end_name]
                
                # Skip connections that are too long (likely projection errors)
                dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                if dist > 2.0:  # Skip connections longer than 2 meters (likely wrong)
                    continue
                
                line, = ax.plot([x1, x2], [y1, y2], 'b-', linewidth=2, alpha=0.7, zorder=5)
                pose_lines.append(line)
        
        # Draw all keypoints
        for name, (x, y) in landmarks_2d_plane.items():
            if 'wrist' in name:
                color = 'green'
                size = 150
                marker = 'o'
            elif 'elbow' in name or 'shoulder' in name:
                color = 'blue'
                size = 100
                marker = 'o'
            elif 'hip' in name or 'knee' in name or 'ankle' in name:
                color = 'purple'
                size = 80
                marker = 'o'
            else:
                color = 'gray'
                size = 50
                marker = '.'
            
            point = ax.scatter([x], [y], c=color, s=size, marker=marker, 
                             edgecolors='black', linewidths=1, zorder=10, alpha=0.8)
            pose_points.append(point)
        
        # Highlight wrists
        if 'left_wrist' in landmarks_2d_plane:
            x, y = landmarks_2d_plane['left_wrist']
            marker = ax.scatter([x], [y], c='lime', s=300, marker='*', 
                             edgecolors='black', linewidths=2, zorder=12)
            wrist_markers.append(marker)
        
        if 'right_wrist' in landmarks_2d_plane:
            x, y = landmarks_2d_plane['right_wrist']
            marker = ax.scatter([x], [y], c='lime', s=300, marker='*', 
                             edgecolors='black', linewidths=2, zorder=12)
            wrist_markers.append(marker)
        
        # Draw human reachable workspace (disk around left shoulder)
        if max_arm_reach is not None and 'left_shoulder' in landmarks_2d_plane:
            shoulder_x, shoulder_y = landmarks_2d_plane['left_shoulder']
            circle = plt.Circle((shoulder_x, shoulder_y), max_arm_reach,
                               fill=False, edgecolor='red', linewidth=2,
                               linestyle='--', alpha=0.7, zorder=3)
            ax.add_patch(circle)
            reach_circles.append(circle)
            
            # Compute and visualize intersection with robot workspace
            if robot_arm is not None:
                workspace_points = robot_arm.compute_workspace(grid_resolution=100)
                intersection_points = robot_arm.compute_intersection_with_circle(
                    workspace_points,
                    (shoulder_x, shoulder_y),
                    max_arm_reach
                )
                
                if len(intersection_points) > 0:
                    # Plot intersection points in purple with high opacity
                    intersection_points_plot[0] = ax.scatter(
                        intersection_points[:, 0], intersection_points[:, 1],
                        c='purple', s=8, alpha=0.7, zorder=4,
                        edgecolors='darkviolet', linewidths=0.5
                    )
        
        plt.draw()
        plt.pause(0.01)
    
    return fig, ax, update_pose


def animate_robot_path(ax, robot_path: List[np.ndarray], 
                      base_position: Tuple[float, float],
                      animation_speed: float = 0.05):
    """
    Animate the robot arm moving along the planned path.
    
    Args:
        ax: Matplotlib axes to draw on
        robot_path: List of configurations [θ1, θ2, 0, 0] to animate
        base_position: Robot base position (x, y)
        animation_speed: Delay between frames in seconds (default: 0.05)
    """
    if robot_path is None or len(robot_path) == 0:
        print("  No path to animate")
        return
    
    from robot.robot_config import LINK1_LENGTH, LINK2_LENGTH
    
    print(f"\n[Animation] Animating {len(robot_path)} waypoints...")
    
    # Create animated arm elements (will be updated each frame)
    animated_lines = []
    animated_joints = []
    
    # Create initial empty lines and joints
    base_x, base_y = base_position
    
    # Line for link 1 (base to elbow) - use bright blue to distinguish from static start/goal
    line1, = ax.plot([], [], color='cyan', linestyle='-', 
                    linewidth=5, alpha=1.0, zorder=12, label='Animated Arm')
    animated_lines.append(line1)
    
    # Line for link 2 (elbow to end-effector)
    line2, = ax.plot([], [], color='cyan', linestyle='-', 
                    linewidth=5, alpha=1.0, zorder=12)
    animated_lines.append(line2)
    
    # Joint markers (larger and brighter for animation)
    base_marker = ax.scatter([], [], c='cyan', s=200, marker='s', 
                            zorder=13, edgecolors='darkblue', linewidths=3, alpha=1.0)
    elbow_marker = ax.scatter([], [], c='cyan', s=150, marker='o', 
                             zorder=13, edgecolors='darkblue', linewidths=3, alpha=1.0)
    ee_marker = ax.scatter([], [], c='yellow', s=200, marker='D', 
                          zorder=13, edgecolors='darkblue', linewidths=3, alpha=1.0)
    animated_joints = [base_marker, elbow_marker, ee_marker]
    
    # Update legend to include animated arm
    ax.legend(loc='upper right')
    
    # Animate through path
    for i, config in enumerate(robot_path):
        theta1 = config[0]
        theta2 = config[1]
        
        # Compute joint positions
        elbow_x = base_x + LINK1_LENGTH * np.cos(theta1)
        elbow_y = base_y + LINK1_LENGTH * np.sin(theta1)
        
        ee_x = base_x + LINK1_LENGTH * np.cos(theta1) + LINK2_LENGTH * np.cos(theta1 + theta2)
        ee_y = base_y + LINK1_LENGTH * np.sin(theta1) + LINK2_LENGTH * np.sin(theta1 + theta2)
        
        # Update link 1 (base to elbow)
        animated_lines[0].set_data([base_x, elbow_x], [base_y, elbow_y])
        
        # Update link 2 (elbow to end-effector)
        animated_lines[1].set_data([elbow_x, ee_x], [elbow_y, ee_y])
        
        # Update joint markers
        animated_joints[0].set_offsets([[base_x, base_y]])
        animated_joints[1].set_offsets([[elbow_x, elbow_y]])
        animated_joints[2].set_offsets([[ee_x, ee_y]])
        
        # Update title to show progress
        ax.set_title(f'2D Working Plane - Animating Path ({i+1}/{len(robot_path)})', 
                    fontsize=12, fontweight='bold')
        
        # Redraw
        plt.draw()
        plt.pause(animation_speed)
    
    # Final update - keep animated arm visible
    ax.set_title(f'2D Working Plane - Animation Complete ({len(robot_path)} waypoints)', 
                fontsize=12, fontweight='bold')
    plt.draw()
    print(f"  ✓ Animation complete!")

