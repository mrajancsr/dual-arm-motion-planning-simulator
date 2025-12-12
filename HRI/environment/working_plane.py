"""
Working Plane Module

Defines a plane in 3D world coordinates for 2D planning.
Plane is defined in the same coordinate system as the environment.
"""

import numpy as np
from typing import Tuple, Optional


class WorkingPlane:
    """
    Defines a working plane in 3D camera coordinates.
    
    Plane equation: Z = z0 (constant depth from camera).
    This creates a vertical plane where the robot can move in X-Y.
    """
    
    def __init__(self, z0: float, epsilon: float = 0.05):
        """
        Initialize working plane.
        
        Args:
            z0: Z coordinate (depth) of the plane (plane equation: Z = z0)
            epsilon: Thickness of the plane slice in meters (default: 5cm)
        """
        self.z0 = z0
        self.epsilon = epsilon
        # Plane normal is [0, 0, 1] (pointing in +Z direction)
        self.normal = np.array([0.0, 0.0, 1.0])
        self.d = -z0  # Plane equation: 0*x + 0*y + 1*z - z0 = 0
    
    @classmethod
    def from_point_cloud(cls, point_cloud: np.ndarray, 
                        percentile: float = 50.0,
                        epsilon: float = 0.05) -> 'WorkingPlane':
        """
        Auto-detect working plane from point cloud.
        
        Uses median Z coordinate (depth) of points as the plane position.
        
        Args:
            point_cloud: Point cloud (N, 3) with [x, y, z] coordinates
            percentile: Percentile to use for Z coordinate (default: 50 = median)
            epsilon: Thickness of plane slice
            
        Returns:
            WorkingPlane instance
        """
        if len(point_cloud) == 0:
            raise ValueError("Point cloud is empty")
        
        # Get Z coordinates (depth)
        z_coords = point_cloud[:, 2]
        valid_z = z_coords[np.isfinite(z_coords) & (z_coords > 0.15) & (z_coords < 20.0)]
        
        if len(valid_z) == 0:
            raise ValueError("No valid Z coordinates in point cloud")
        
        # Use percentile (default: median)
        z0 = np.percentile(valid_z, percentile)
        
        print(f"Auto-detected working plane: Z = {z0:.3f}m (from {percentile}th percentile)")
        
        return cls(z0, epsilon)
    
    def in_plane(self, point_3d: np.ndarray) -> bool:
        """
        Check if a 3D point is within the plane slice.
        
        Args:
            point_3d: 3D point [x, y, z]
            
        Returns:
            True if |point.z - z0| < epsilon
        """
        if isinstance(point_3d, list):
            point_3d = np.array(point_3d)
        return abs(point_3d[2] - self.z0) < self.epsilon
    
    def project_to_2d(self, point_3d: np.ndarray) -> Tuple[float, float]:
        """
        Project a 3D point to 2D plane coordinates.
        
        If point is not in plane, projects it to the plane first.
        
        Args:
            point_3d: 3D point [x, y, z]
            
        Returns:
            Tuple of (y, z) - 2D coordinates in the plane
        """
        if isinstance(point_3d, list):
            point_3d = np.array(point_3d)
        
        # Project point to plane (set Z = z0)
        projected_3d = np.array([point_3d[0], point_3d[1], self.z0])
        
        # Return X and Y as 2D coordinates (left/right, up/down)
        return (float(projected_3d[0]), float(projected_3d[1]))
    
    def project_3d_to_plane(self, point_3d: np.ndarray) -> np.ndarray:
        """
        Project a 3D point onto the plane (returns 3D point on plane).
        
        Args:
            point_3d: 3D point [x, y, z]
            
        Returns:
            3D point on the plane [x, y, z0]
        """
        if isinstance(point_3d, list):
            point_3d = np.array(point_3d)
        return np.array([point_3d[0], point_3d[1], self.z0])
    
    def distance_to_plane(self, point_3d: np.ndarray) -> float:
        """
        Calculate signed distance from point to plane.
        
        Args:
            point_3d: 3D point [x, y, z]
            
        Returns:
            Signed distance (positive if point is in +Z direction from plane)
        """
        if isinstance(point_3d, list):
            point_3d = np.array(point_3d)
        return point_3d[2] - self.z0
    
    def get_bounds_2d(self, point_cloud: np.ndarray) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """
        Get 2D bounds of points in the plane.
        
        Args:
            point_cloud: Point cloud (N, 3)
            
        Returns:
            Tuple of ((x_min, x_max), (y_min, y_max))
        """
        # Filter points in plane
        in_plane_mask = np.array([self.in_plane(p) for p in point_cloud])
        plane_points = point_cloud[in_plane_mask]
        
        if len(plane_points) == 0:
            return ((0.0, 1.0), (0.0, 1.0))
        
        x_coords = plane_points[:, 0]
        y_coords = plane_points[:, 1]
        
        return ((float(x_coords.min()), float(x_coords.max())),
                (float(y_coords.min()), float(y_coords.max())))

