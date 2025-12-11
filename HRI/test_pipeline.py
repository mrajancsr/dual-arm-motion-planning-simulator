"""
End-to-End Pipeline Test

Tests the full perception → 3D environment → goal extraction pipeline.
Uses Depth Anything V2 and simple bounding box obstacles.
"""

import numpy as np
from typing import Optional, List, Dict
import matplotlib
# Try MacOSX backend first (native macOS), fall back to Agg if needed
try:
    matplotlib.use('MacOSX')
except:
    matplotlib.use('Agg')  # Non-interactive backend (saves to file)
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from perception.depth_estimator import DepthEstimator
from environment.environment_3d import Environment3D
from calibration.pixel_to_3d import pixel_to_3d, load_depth_snapshot


def test_pipeline():
    """Test the full pipeline: snapshot → environment → goal extraction."""
    print("=" * 70)
    print("End-to-End Pipeline Test (Depth Anything V2 + Bounding Boxes)")
    print("=" * 70)
    
    # Step 1: Capture depth snapshot
    print("\n[Step 1] Capturing depth snapshot...")
    estimator = DepthEstimator()
    
    # Try to use calibrated intrinsics if available
    import os
    intrinsics_file = "calibration/camera_intrinsics.json"
    if not os.path.exists(intrinsics_file):
        intrinsics_file = None
        print("  Note: Using estimated intrinsics. Run calibration for better accuracy.")
    else:
        print(f"  Using calibrated intrinsics from {intrinsics_file}")
    
    rgb_frame, depth_map, intrinsics = estimator.capture_depth_snapshot(
        camera_id=0,
        save_path="depth_snapshot",
        intrinsics_file=intrinsics_file
    )
    
    if rgb_frame is None or depth_map is None:
        print("ERROR: Failed to capture snapshot")
        return
    
    print("✓ Snapshot captured and saved")
    
    # Step 2: Build 3D environment and extract obstacle boxes
    print("\n[Step 2] Building 3D environment from snapshot...")
    env = Environment3D(
        snapshot_path="depth_snapshot",
        safety_margin=0.1
    )
    
    # Extract obstacle boxes with workspace filtering
    # Define reasonable workspace bounds (adjust based on your setup)
    workspace_bounds = {
        'x_min': -1.0, 'x_max': 1.0,   # 2m wide workspace
        'y_min': -1.0, 'y_max': 1.0,   # 2m deep workspace  
        'z_min': 0.3, 'z_max': 1.5     # Desk height to ~1.5m above
    }
    
    obstacle_boxes = env.extract_obstacle_boxes(
        threshold_percentile=50.0,  # Lower percentile = tighter threshold (closer objects only)
        grid_cell_size=30,  # Smaller cells = more boxes, better accuracy (30x30 pixels)
        min_box_size=0.02,  # Minimum 2cm box size (smaller boxes)
        workspace_bounds=workspace_bounds,
        desk_height=0.6  # Assume desk at 60cm height
    )
    
    print(f"✓ Environment built with {len(obstacle_boxes)} obstacle boxes")
    
    # Step 3: Test pixel to 3D conversion (for hand goal)
    print("\n[Step 3] Testing pixel to 3D conversion...")
    
    # Use the environment's depth map (already scaled to meters) and intrinsics
    depth_map_loaded = env.depth_map
    intrinsics_loaded = env.intrinsics
    
    # Test with center pixel (or you can specify a hand pixel here)
    h, w = depth_map_loaded.shape
    test_u = w / 2.0
    test_v = h / 2.0
    
    print(f"Testing pixel: ({test_u:.1f}, {test_v:.1f})")
    
    # Check depth value at this pixel
    u_int, v_int = int(test_u), int(test_v)
    depth_at_pixel = depth_map_loaded[v_int, u_int]
    print(f"  Depth at pixel: {depth_at_pixel:.2f}m")
    
    goal_3d = pixel_to_3d(test_u, test_v, depth_map_loaded, intrinsics_loaded)
    
    if goal_3d is None:
        print("ERROR: Failed to convert pixel to 3D")
        print(f"  Depth value {depth_at_pixel:.2f}m may be invalid (must be > 0.05m and < 20.0m)")
        # Try a different pixel (maybe find a valid one)
        valid_pixels = np.where((depth_map_loaded > 0.05) & (depth_map_loaded < 20.0))
        if len(valid_pixels[0]) > 0:
            idx = len(valid_pixels[0]) // 2  # Use middle valid pixel
            test_u = valid_pixels[1][idx]
            test_v = valid_pixels[0][idx]
            print(f"  Trying alternative pixel: ({test_u:.1f}, {test_v:.1f})")
            goal_3d = pixel_to_3d(test_u, test_v, depth_map_loaded, intrinsics_loaded)
            if goal_3d is None:
                print("ERROR: Still failed with alternative pixel")
                return
        else:
            return
    
    print(f"✓ Goal 3D point: ({goal_3d[0]:.3f}, {goal_3d[1]:.3f}, {goal_3d[2]:.3f})m")
    
    # Step 4: Check if goal is free
    print("\n[Step 4] Checking collision at goal point...")
    is_free = env.is_free(goal_3d[0], goal_3d[1], goal_3d[2], radius=0.05)
    status = "FREE" if is_free else "COLLISION"
    print(f"✓ Goal point is {status}")
    
    # Step 5: Visualize
    print("\n[Step 5] Creating 3D visualization...")
    # Get point cloud for visualization
    print("  Generating point cloud for visualization...")
    # Don't filter by workspace bounds for visualization - show everything
    point_cloud = env.get_point_cloud(
        downsample=4,  # Sample every 4nd pixel for visualization (much more points)
        workspace_bounds=None,  # Don't filter - show all points
        desk_height=None  # Don't filter by desk height
    )
    print(f"  Point cloud: {len(point_cloud)} points")
    
    # First, show just the point cloud to verify it looks correct
    print("  Visualizing point cloud only (no boxes)...")
    visualize_point_cloud_only(point_cloud, goal_3d)
    
    print("\n" + "=" * 70)
    print("Pipeline test complete!")
    print("=" * 70)


def visualize_point_cloud_only(point_cloud: np.ndarray, goal_point: Optional[np.ndarray] = None):
    """
    Visualize just the point cloud (for debugging).
    
    Args:
        point_cloud: Point cloud (N, 3) to visualize
        goal_point: Optional 3D goal point [x, y, z]
    """
    if len(point_cloud) == 0:
        print("  Warning: Point cloud is empty!")
        return
    
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot point cloud colored by depth
    scatter = ax.scatter(point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2],
                        c=point_cloud[:, 2], cmap='viridis', alpha=0.6, s=2,
                        edgecolors='none')
    
    # Add colorbar
    cbar = plt.colorbar(scatter, ax=ax, label='Depth (m)', shrink=0.6, pad=0.1)
    
    # Plot goal point if provided
    if goal_point is not None:
        ax.scatter([goal_point[0]], [goal_point[1]], [goal_point[2]],
                  c='red', s=300, marker='*', label='Goal', 
                  edgecolors='black', linewidths=2, zorder=10)
        ax.legend()
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Point Cloud from Depth Anything V2')
    
    # Set plot limits from point cloud
    min_bounds = point_cloud.min(axis=0)
    max_bounds = point_cloud.max(axis=0)
    
    if goal_point is not None:
        min_bounds = np.minimum(min_bounds, goal_point - 0.2)
        max_bounds = np.maximum(max_bounds, goal_point + 0.2)
    
    # Set equal aspect ratio
    max_range = (max_bounds - min_bounds).max() / 2.0
    mid = (min_bounds + max_bounds) / 2.0
    
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)
    
    plt.tight_layout()
    
    # Save plot to file
    output_file = "point_cloud_visualization.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"  Point cloud visualization saved to: {output_file}")
    
    # Print point cloud stats
    print(f"\n  Point cloud statistics:")
    print(f"    Points: {len(point_cloud)}")
    print(f"    X range: {min_bounds[0]:.3f} to {max_bounds[0]:.3f} m")
    print(f"    Y range: {min_bounds[1]:.3f} to {max_bounds[1]:.3f} m")
    print(f"    Z range: {min_bounds[2]:.3f} to {max_bounds[2]:.3f} m")
    
    # Try to show interactively if backend supports it
    try:
        print("  Displaying 3D plot (close window to continue)...")
        plt.show()
    except Exception as e:
        print(f"  Could not display plot interactively: {e}")
        print(f"  Plot saved to {output_file} instead")


def visualize_3d(env: Environment3D, goal_point: np.ndarray, 
                 obstacle_boxes: List[Dict],
                 point_cloud: Optional[np.ndarray] = None):
    """
    Visualize obstacle boxes, point cloud, and goal point in 3D.
    
    Args:
        env: Environment3D instance
        goal_point: 3D goal point [x, y, z]
        obstacle_boxes: List of obstacle bounding boxes
        point_cloud: Optional point cloud (N, 3) to visualize
    """
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot point cloud first (so boxes appear on top)
    if point_cloud is not None and len(point_cloud) > 0:
        # Color by Z-depth for better visualization
        scatter = ax.scatter(point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2],
                            c=point_cloud[:, 2], cmap='viridis', alpha=0.4, s=1,
                            label='Point Cloud', edgecolors='none')
        # Add colorbar
        plt.colorbar(scatter, ax=ax, label='Depth (m)', shrink=0.6, pad=0.1)
        print(f"  Visualized {len(point_cloud)} point cloud points")
    
    # Plot obstacle boxes
    if len(obstacle_boxes) > 0:
        colors = plt.cm.tab10(np.linspace(0, 1, len(obstacle_boxes)))
        
        for i, box in enumerate(obstacle_boxes):
            min_bounds = np.array(box['min'])
            max_bounds = np.array(box['max'])
            
            # Create 8 vertices of the bounding box
            x_min, y_min, z_min = min_bounds
            x_max, y_max, z_max = max_bounds
            
            vertices_box = np.array([
                [x_min, y_min, z_min], [x_max, y_min, z_min],
                [x_max, y_max, z_min], [x_min, y_max, z_min],
                [x_min, y_min, z_max], [x_max, y_min, z_max],
                [x_max, y_max, z_max], [x_min, y_max, z_max]
            ])
            
            # Define 6 faces of the box (each face is a quad with 4 vertices)
            faces_box = [
                [0, 1, 2, 3],  # bottom face (z_min)
                [4, 7, 6, 5],  # top face (z_max)
                [0, 4, 5, 1],  # front face (y_min)
                [2, 6, 7, 3],  # back face (y_max)
                [0, 3, 7, 4],  # left face (x_min)
                [1, 5, 6, 2]   # right face (x_max)
            ]
            
            # Create wireframe box
            box_mesh = Poly3DCollection(vertices_box[faces_box], 
                                       alpha=0.3, 
                                       facecolor=colors[i],
                                       edgecolor=colors[i],
                                       linewidths=2,
                                       label=f'Obstacle {i}')
            ax.add_collection3d(box_mesh)
        
        print(f"  Visualized {len(obstacle_boxes)} obstacle boxes")
    
    # Plot goal point
    ax.scatter([goal_point[0]], [goal_point[1]], [goal_point[2]],
              c='red', s=200, marker='*', label='Goal', edgecolors='black', linewidths=2)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Environment with Obstacle Boxes and Goal Point')
    ax.legend()
    
    # Set plot limits from point cloud or obstacle boxes
    if point_cloud is not None and len(point_cloud) > 0:
        # Use point cloud bounds
        min_bounds = point_cloud.min(axis=0)
        max_bounds = point_cloud.max(axis=0)
        
        # Expand to include goal point and boxes
        if len(obstacle_boxes) > 0:
            box_mins = np.array([box['min'] for box in obstacle_boxes])
            box_maxs = np.array([box['max'] for box in obstacle_boxes])
            min_bounds = np.minimum(min_bounds, box_mins.min(axis=0))
            max_bounds = np.maximum(max_bounds, box_maxs.max(axis=0))
        
        min_bounds = np.minimum(min_bounds, goal_point - 0.2)
        max_bounds = np.maximum(max_bounds, goal_point + 0.2)
    elif len(obstacle_boxes) > 0:
        # Fall back to box bounds
        all_mins = np.array([box['min'] for box in obstacle_boxes])
        all_maxs = np.array([box['max'] for box in obstacle_boxes])
        
        min_bounds = all_mins.min(axis=0)
        max_bounds = all_maxs.max(axis=0)
        
        # Expand bounds to include goal point
        min_bounds = np.minimum(min_bounds, goal_point - 0.5)
        max_bounds = np.maximum(max_bounds, goal_point + 0.5)
    else:
        # Default bounds around goal
        min_bounds = goal_point - 0.5
        max_bounds = goal_point + 0.5
    
    # Set equal aspect ratio
    max_range = (max_bounds - min_bounds).max() / 2.0
    mid = (min_bounds + max_bounds) / 2.0
    
    ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
    ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
    ax.set_zlim(mid[2] - max_range, mid[2] + max_range)
    
    plt.tight_layout()
    
    # Save plot to file
    output_file = "environment_3d_visualization.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"3D visualization saved to: {output_file}")
    
    # Try to show interactively if backend supports it
    try:
        print("Displaying 3D plot (close window to continue)...")
        plt.show()
    except Exception as e:
        print(f"Could not display plot interactively: {e}")
        print(f"Plot saved to {output_file} instead")


if __name__ == "__main__":
    test_pipeline()
