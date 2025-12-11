"""
3D Environment Module

Simple 3D environment representation using axis-aligned bounding boxes.
Extracts obstacles from depth maps using thresholding and connected components.
"""

import numpy as np
import cv2
from typing import List, Dict, Optional, Tuple
from calibration.pixel_to_3d import pixel_to_3d


class Environment3D:
    """
    Simple 3D environment from depth model.
    
    Stores obstacles as axis-aligned bounding boxes for fast collision checking.
    """
    
    def __init__(self, depth_map: Optional[np.ndarray] = None, 
                 intrinsics: Optional[dict] = None,
                 snapshot_path: Optional[str] = None,
                 safety_margin: float = 0.1):
        """
        Initialize 3D environment from depth map.
        
        Args:
            depth_map: Depth map (H, W) in meters (optional if snapshot_path provided)
            intrinsics: Camera intrinsics dict with 'fx', 'fy', 'cx', 'cy' (optional if snapshot_path provided)
            snapshot_path: Optional base path to load from (loads {snapshot_path}_depth.npy and {snapshot_path}_intrinsics.npy)
            safety_margin: Safety margin around obstacles in meters (default: 10cm)
        """
        self.safety_margin = safety_margin
        
        # Load from snapshot if provided
        if snapshot_path is not None:
            depth_path = f"{snapshot_path}_depth.npy"
            intrinsics_path = f"{snapshot_path}_intrinsics.npy"
            
            print(f"Loading environment from snapshot: {snapshot_path}")
            depth_map = np.load(depth_path)
            intrinsics = np.load(intrinsics_path, allow_pickle=True).item()
            print(f"  Loaded depth map: {depth_map.shape}")
            print(f"  Loaded intrinsics: {intrinsics}")
        
        if depth_map is None or intrinsics is None:
            raise ValueError("Must provide either (depth_map, intrinsics) or snapshot_path")
        
        # Handle negative depth values (Depth Anything V2 may output signed values)
        # Convert to absolute values first
        depth_map = np.abs(depth_map)
        
        # Load and apply depth calibration scale (convert raw units to meters)
        depth_scale = None
        try:
            import json
            import os
            calibration_file = "calibration/depth_calibration.json"
            if os.path.exists(calibration_file):
                with open(calibration_file, 'r') as f:
                    calib_data = json.load(f)
                    depth_scale = calib_data['depth_scale']
                    print(f"  Loaded depth calibration scale: {depth_scale:.6f}")
                    depth_map = depth_map * depth_scale
            else:
                print(f"  No depth calibration found - using raw depth values")
        except Exception as e:
            print(f"  Warning: Could not load depth calibration: {e}")
        
        # Now filter invalid depths (in meters after scaling)
        # Filter out very close depths (< 6 inches = 0.15m) - these are usually noise
        depth_map[depth_map < 0.15] = 0  # Filter out very close/invalid depths (< 15cm / 6 inches)
        depth_map[depth_map > 20.0] = 0  # Filter out very far depths (> 20m, likely invalid)
        
        self.depth_map = depth_map  # Now in meters (scaled)
        self.intrinsics = intrinsics
        self.obstacle_boxes = []  # Will be populated by extract_obstacle_boxes()
        
        valid_depths = depth_map[depth_map > 0]
        if len(valid_depths) > 0:
            print(f"Environment initialized:")
            print(f"  Depth map: {depth_map.shape}, range: {valid_depths.min():.2f}m - {valid_depths.max():.2f}m (in meters)")
            print(f"  Safety margin: {self.safety_margin:.3f}m")
        else:
            print(f"Environment initialized:")
            print(f"  Depth map: {depth_map.shape}, no valid depths found")
            print(f"  Safety margin: {self.safety_margin:.3f}m")
    
    def extract_obstacle_boxes(self, threshold_percentile: float = 50.0, 
                              grid_cell_size: int = 30,
                              min_box_size: float = 0.02,
                              workspace_bounds: Optional[Dict] = None,
                              desk_height: Optional[float] = None) -> List[Dict]:
        """
        Extract axis-aligned bounding boxes from depth map using grid-based approach.
        
        Divides depth map into small grid cells and creates boxes for each cell with obstacles.
        This produces many small, accurate boxes that better match the environment geometry.
        
        Args:
            threshold_percentile: Percentile of depth values to use as threshold (lower = closer objects)
            grid_cell_size: Size of grid cells in pixels (smaller = more boxes, more accurate)
            min_box_size: Minimum size of bounding box in meters (filters noise)
            workspace_bounds: Optional dict with 'x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max' to filter obstacles
            desk_height: Optional desk height in meters (objects below this are ignored)
            
        Returns:
            List of dicts, each containing:
            - 'min': [x, y, z] minimum corner of bounding box
            - 'max': [x, y, z] maximum corner of bounding box
            - 'center': [x, y, z] center of bounding box
            - 'size': [x, y, z] size of bounding box
        """
        print(f"Extracting obstacle boxes from depth map (grid-based)...")
        print(f"  Threshold percentile: {threshold_percentile}%")
        print(f"  Grid cell size: {grid_cell_size}x{grid_cell_size} pixels")
        print(f"  Min box size: {min_box_size}m")
        
        # Use depth map directly (already scaled to meters in __init__)
        depth_map_scaled = self.depth_map.copy()
        
        # Compute threshold - objects closer than this are obstacles
        # Only consider depths in reasonable range (0.15m to 10m) - filter out very close noise
        valid_depths = depth_map_scaled[(depth_map_scaled >= 0.15) & (depth_map_scaled < 10.0)]  # Reasonable range in meters
        if len(valid_depths) == 0:
            print("Warning: No valid depth values found")
            return []
        
        threshold = np.percentile(valid_depths, threshold_percentile)
        print(f"  Depth threshold: {threshold:.2f}m (objects closer are obstacles)")
        
        # Create obstacle mask
        # Minimum depth is 0.15m (6 inches) to filter out close noise
        min_depth = 0.15
        max_depth = threshold
        if workspace_bounds is not None:
            if 'z_min' in workspace_bounds and 'z_max' in workspace_bounds:
                min_depth = max(min_depth, workspace_bounds['z_min'] - 0.2)
                max_depth = min(max_depth, workspace_bounds['z_max'] + 0.2)
        
        obstacle_mask = (depth_map_scaled >= min_depth) & (depth_map_scaled < max_depth)
        
        # Apply morphological operations to clean up the mask (remove noise)
        kernel = np.ones((3, 3), np.uint8)
        obstacle_mask = cv2.morphologyEx(obstacle_mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
        obstacle_mask = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)
        
        h, w = depth_map_scaled.shape
        bounding_boxes = []
        
        # Grid-based approach: divide image into cells and create boxes for each cell
        for v_start in range(0, h, grid_cell_size):
            for u_start in range(0, w, grid_cell_size):
                v_end = min(v_start + grid_cell_size, h)
                u_end = min(u_start + grid_cell_size, w)
                
                # Check if this cell has obstacles (at least 10% obstacle pixels)
                cell_mask = obstacle_mask[v_start:v_end, u_start:u_end]
                if np.sum(cell_mask) < (grid_cell_size * grid_cell_size * 0.1):
                    continue
                
                # Get all pixels in this cell that are obstacles
                cell_pixels = np.column_stack(np.where(cell_mask))
                if len(cell_pixels) < 3:
                    continue
                
                # Convert to global pixel coordinates
                cell_pixels[:, 0] += v_start  # v coordinates
                cell_pixels[:, 1] += u_start   # u coordinates
                
                # Convert pixels to 3D points
                points_3d = []
                for v, u in cell_pixels:
                    point_3d = pixel_to_3d(u, v, depth_map_scaled, self.intrinsics)
                    if point_3d is not None:
                        # Filter by workspace bounds if provided
                        if workspace_bounds is not None:
                            x, y, z = point_3d
                            if ('x_min' in workspace_bounds and x < workspace_bounds['x_min']) or \
                               ('x_max' in workspace_bounds and x > workspace_bounds['x_max']) or \
                               ('y_min' in workspace_bounds and y < workspace_bounds['y_min']) or \
                               ('y_max' in workspace_bounds and y > workspace_bounds['y_max']) or \
                               ('z_min' in workspace_bounds and z < workspace_bounds['z_min']) or \
                               ('z_max' in workspace_bounds and z > workspace_bounds['z_max']):
                                continue
                        
                        # Filter by desk height if provided
                        if desk_height is not None and point_3d[2] < desk_height - 0.05:
                            continue
                        
                        points_3d.append(point_3d)
                
                if len(points_3d) < 3:
                    continue
                
                points_3d = np.array(points_3d)
                
                # Compute axis-aligned bounding box for this cell
                min_bounds = points_3d.min(axis=0)
                max_bounds = points_3d.max(axis=0)
                
                # Check minimum size
                size = max_bounds - min_bounds
                if np.any(size < min_box_size):
                    continue
                
                # Sanity check: box size should be reasonable (not tens of meters)
                if np.any(size > 1.0):  # More than 1m is suspicious for a grid cell
                    continue
                
                # Add safety margin
                min_bounds -= self.safety_margin
                max_bounds += self.safety_margin
                
                bounding_boxes.append({
                    'min': min_bounds.tolist(),
                    'max': max_bounds.tolist(),
                    'center': ((min_bounds + max_bounds) / 2).tolist(),
                    'size': (max_bounds - min_bounds).tolist()
                })
        
        self.obstacle_boxes = bounding_boxes
        print(f"Extracted {len(bounding_boxes)} obstacle boxes (grid-based)")
        return bounding_boxes
    
    def is_point_in_box(self, point: np.ndarray, box: Dict) -> bool:
        """
        Check if a point is inside a bounding box.
        
        Args:
            point: 3D point [x, y, z]
            box: Bounding box dict with 'min' and 'max' keys
            
        Returns:
            True if point is inside box
        """
        min_bounds = np.array(box['min'])
        max_bounds = np.array(box['max'])
        return np.all(point >= min_bounds) and np.all(point <= max_bounds)
    
    def is_free(self, x: float, y: float, z: float, 
                radius: float = 0.05) -> bool:
        """
        Check if a point (or sphere) is collision-free.
        
        Args:
            x, y, z: 3D point coordinates in camera frame
            radius: Radius of sphere to check (default: 5cm)
            
        Returns:
            True if point is free, False if collides with obstacles
        """
        if len(self.obstacle_boxes) == 0:
            return True  # No obstacles defined
        
        point = np.array([x, y, z])
        
        # Check if point (with radius) intersects any bounding box
        for box in self.obstacle_boxes:
            min_bounds = np.array(box['min'])
            max_bounds = np.array(box['max'])
            
            # Find closest point on box to our point
            closest_point = np.clip(point, min_bounds, max_bounds)
            
            # Check if point (with radius) is inside or too close to box
            dist_to_box = np.linalg.norm(point - closest_point)
            if dist_to_box <= radius:
                return False  # Collision
        
        return True  # Free
    
    def is_path_free(self, start: np.ndarray, end: np.ndarray, 
                    num_checks: int = 10, radius: float = 0.05) -> bool:
        """
        Check if a path segment is collision-free.
        
        Args:
            start: Start point [x, y, z]
            end: End point [x, y, z]
            num_checks: Number of intermediate points to check
            radius: Radius of sphere to check
            
        Returns:
            True if path is free, False if collides
        """
        # Check endpoints
        if not self.is_free(start[0], start[1], start[2], radius):
            return False
        if not self.is_free(end[0], end[1], end[2], radius):
            return False
        
        # Check intermediate points
        for i in range(1, num_checks):
            alpha = i / num_checks
            intermediate = start + alpha * (end - start)
            if not self.is_free(intermediate[0], intermediate[1], intermediate[2], radius):
                return False
        
        return True
    
    def get_obstacle_boxes(self) -> List[Dict]:
        """Get list of obstacle bounding boxes."""
        return self.obstacle_boxes
    
    def get_point_cloud(self, downsample: int = 4, 
                       workspace_bounds: Optional[Dict] = None,
                       desk_height: Optional[float] = None,
                       max_depth: float = 10.0) -> np.ndarray:
        """
        Convert depth map to point cloud for visualization.
        
        Args:
            downsample: Downsample factor (use every Nth pixel)
            workspace_bounds: Optional workspace bounds to filter points
            desk_height: Optional desk height to filter points below
            max_depth: Maximum depth in meters to include (default: 10m)
            
        Returns:
            Point cloud array (N, 3) with [x, y, z] coordinates
        """
        h, w = self.depth_map.shape
        
        # Use depth map directly (already scaled to meters in __init__)
        depth_map_scaled = self.depth_map.copy()
        
        # Create grid of pixel coordinates
        u_coords, v_coords = np.meshgrid(
            np.arange(0, w, downsample),
            np.arange(0, h, downsample)
        )
        u_coords = u_coords.flatten()
        v_coords = v_coords.flatten()
        
        # Get depth values from SCALED depth map
        depth_values = depth_map_scaled[v_coords, u_coords]
        
        # Filter valid depth values
        # Filter out very close depths (< 6 inches = 0.15m) - these are usually noise
        depth_values_abs = np.abs(depth_values)
        # Filter to reasonable range: 0.15m (6 inches) to 20m
        valid_mask = (depth_values_abs >= 0.15) & (depth_values_abs < 20.0) & np.isfinite(depth_values_abs)
        
        print(f"    Total pixels sampled: {len(u_coords)}")
        print(f"    Valid depth pixels (0.15m <= depth < 20m): {np.sum(valid_mask)}")
        if np.sum(valid_mask) == 0:
            print(f"    WARNING: No valid depth pixels found!")
            print(f"    Depth value range in sampled pixels: {depth_values_abs.min():.2f}m to {depth_values_abs.max():.2f}m")
        
        u_coords = u_coords[valid_mask]
        v_coords = v_coords[valid_mask]
        depth_values_abs = depth_values_abs[valid_mask]
        
        # Convert to 3D points
        points_3d = []
        skipped_bounds = 0
        skipped_desk = 0
        
        for u, v, z in zip(u_coords, v_coords, depth_values_abs):
            # Use the SCALED depth map for conversion
            point_3d = pixel_to_3d(u, v, depth_map_scaled, self.intrinsics, depth_value=z)
            if point_3d is not None:
                # Filter by workspace bounds if provided
                if workspace_bounds is not None:
                    x, y, z = point_3d
                    if ('x_min' in workspace_bounds and x < workspace_bounds['x_min']) or \
                       ('x_max' in workspace_bounds and x > workspace_bounds['x_max']) or \
                       ('y_min' in workspace_bounds and y < workspace_bounds['y_min']) or \
                       ('y_max' in workspace_bounds and y > workspace_bounds['y_max']) or \
                       ('z_min' in workspace_bounds and z < workspace_bounds['z_min']) or \
                       ('z_max' in workspace_bounds and z > workspace_bounds['z_max']):
                        skipped_bounds += 1
                        continue
                
                # Filter by desk height if provided
                if desk_height is not None and point_3d[2] < desk_height - 0.05:
                    skipped_desk += 1
                    continue
                
                points_3d.append(point_3d)
        
        if skipped_bounds > 0:
            print(f"    Skipped {skipped_bounds} points outside workspace bounds")
        if skipped_desk > 0:
            print(f"    Skipped {skipped_desk} points below desk height")
        
        if len(points_3d) == 0:
            return np.array([]).reshape(0, 3)
        
        return np.array(points_3d)
    
    def get_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get environment bounds from depth map.
        
        Returns:
            Tuple of (min_bounds, max_bounds) as numpy arrays
        """
        # Convert depth map to point cloud to get bounds
        h, w = self.depth_map.shape
        fx = self.intrinsics['fx']
        fy = self.intrinsics['fy']
        cx = self.intrinsics.get('cx', w / 2.0)
        cy = self.intrinsics.get('cy', h / 2.0)
        
        # Sample points at corners and center
        sample_pixels = [
            (0, 0), (w-1, 0), (w-1, h-1), (0, h-1),  # Corners
            (w//2, h//2)  # Center
        ]
        
        points_3d = []
        for u, v in sample_pixels:
            point_3d = pixel_to_3d(u, v, self.depth_map, self.intrinsics)
            if point_3d is not None:
                points_3d.append(point_3d)
        
        if len(points_3d) == 0:
            return None, None
        
        points_3d = np.array(points_3d)
        min_bounds = points_3d.min(axis=0)
        max_bounds = points_3d.max(axis=0)
        
        return min_bounds, max_bounds
