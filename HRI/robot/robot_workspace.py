import numpy as np
from typing import List, Tuple, Optional, Dict
import matplotlib.pyplot as plt


class Planar2LinkArm:
    """
    Simple 2-link planar robot arm model.
    
    Forward kinematics only - computes reachable workspace in 2D plane.
    """
    
    def __init__(self, 
                 base_position: Tuple[float, float] = (0.0, 0.0),
                 link1_length: float = 0.3,  # L1 in meters
                 link2_length: float = 0.25,  # L2 in meters
                 joint1_limits: Tuple[float, float] = (-np.pi, np.pi),  # θ1 range
                 joint2_limits: Tuple[float, float] = (-np.pi, np.pi)):  # θ2 range
        """
        Initialize 2-link planar arm.
        
        Args:
            base_position: (x, y) position of robot base in plane coordinates
            link1_length: Length of first link (L1) in meters
            link2_length: Length of second link (L2) in meters
            joint1_limits: (min, max) joint angles for θ1 in radians
            joint2_limits: (min, max) joint angles for θ2 in radians
        """
        self.base_x, self.base_y = base_position
        self.L1 = link1_length
        self.L2 = link2_length
        self.theta1_min, self.theta1_max = joint1_limits
        self.theta2_min, self.theta2_max = joint2_limits
    
    def forward_kinematics(self, theta1: float, theta2: float) -> Tuple[float, float]:
        """
        Compute end-effector position from joint angles.
        
        Args:
            theta1: First joint angle (radians)
            theta2: Second joint angle (radians)
            
        Returns:
            (x, y) end-effector position in plane coordinates
        """
        # Forward kinematics for 2-link arm
        x = self.base_x + self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2)
        y = self.base_y + self.L1 * np.sin(theta1) + self.L2 * np.sin(theta1 + theta2)
        
        return (x, y)
    
    def compute_workspace(self, grid_resolution: int = 100) -> np.ndarray:
        """
        Enumerate reachable workspace by grid-sampling joint space.
        
        Args:
            grid_resolution: Number of samples per joint (grid_resolution × grid_resolution total)
            
        Returns:
            Array of shape (N, 2) with reachable (x, y) points
        """
        workspace_points = []
        
        # Create grid of joint angles
        theta1_values = np.linspace(self.theta1_min, self.theta1_max, grid_resolution)
        theta2_values = np.linspace(self.theta2_min, self.theta2_max, grid_resolution)
        
        # Enumerate all combinations
        for theta1 in theta1_values:
            for theta2 in theta2_values:
                x, y = self.forward_kinematics(theta1, theta2)
                workspace_points.append([x, y])
        
        return np.array(workspace_points)
    
    def get_workspace_bounds(self, workspace_points: np.ndarray) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """
        Get bounding box of workspace.
        
        Args:
            workspace_points: Array of (x, y) points from compute_workspace()
            
        Returns:
            ((x_min, x_max), (y_min, y_max))
        """
        if len(workspace_points) == 0:
            return ((0.0, 0.0), (0.0, 0.0))
        
        x_coords = workspace_points[:, 0]
        y_coords = workspace_points[:, 1]
        
        return ((float(x_coords.min()), float(x_coords.max())),
                (float(y_coords.min()), float(y_coords.max())))
    
    def get_convex_hull(self, workspace_points: np.ndarray):
        """
        Compute convex hull of workspace points.
        
        Args:
            workspace_points: Array of (x, y) points
            
        Returns:
            Convex hull as matplotlib Path or None if insufficient points
        """
        from scipy.spatial import ConvexHull
        
        if len(workspace_points) < 3:
            return None
        
        try:
            hull = ConvexHull(workspace_points)
            return hull
        except:
            return None
    
    def compute_intersection_with_circle(self, workspace_points: np.ndarray,
                                        circle_center: Tuple[float, float],
                                        circle_radius: float) -> np.ndarray:
        """
        Find intersection of robot workspace with a circle (human reach).
        
        Args:
            workspace_points: Array of (x, y) robot workspace points
            circle_center: (x, y) center of the circle
            circle_radius: Radius of the circle
            
        Returns:
            Array of (x, y) points that are in both robot workspace and circle
        """
        if len(workspace_points) == 0:
            return np.array([])
        
        center_x, center_y = circle_center
        
        # Compute distances from each workspace point to circle center
        distances = np.sqrt((workspace_points[:, 0] - center_x)**2 + 
                           (workspace_points[:, 1] - center_y)**2)
        
        # Points inside or on the circle
        inside_mask = distances <= circle_radius
        
        return workspace_points[inside_mask]
    
    def select_handoff_target(self, intersection_points: np.ndarray,
                             obstacle_boxes_2d: List[Dict],
                             wrist_position: Optional[Tuple[float, float]] = None,
                             clearance: float = 0.05) -> Optional[Tuple[float, float]]:
        """
        Select a safe handoff target point from intersection.
        
        Args:
            intersection_points: Array of (x, y) points in intersection
            obstacle_boxes_2d: List of obstacle boxes with 'x_min', 'x_max', 'y_min', 'y_max'
            wrist_position: Optional (x, y) position of human wrist (prefer points near this)
            clearance: Minimum clearance from obstacles in meters (default: 5cm)
            
        Returns:
            (x, y) target point, or None if no valid point found
        """
        if len(intersection_points) == 0:
            return None
        
        # Filter points with clearance from obstacles
        valid_points = []
        for point in intersection_points:
            x, y = point[0], point[1]
            
            # Check clearance from all obstacles
            too_close_to_obstacle = False
            for box in obstacle_boxes_2d:
                # Check if point is within clearance distance of box
                # Point is too close if it's inside the expanded box (box + clearance)
                if (box['x_min'] - clearance <= x <= box['x_max'] + clearance and
                    box['y_min'] - clearance <= y <= box['y_max'] + clearance):
                    too_close_to_obstacle = True
                    break
            
            if not too_close_to_obstacle:
                valid_points.append((x, y))
        
        if len(valid_points) == 0:
            # No points with sufficient clearance - return closest to wrist or center
            if wrist_position is not None:
                # Find point closest to wrist
                wrist_x, wrist_y = wrist_position
                distances = np.sqrt((intersection_points[:, 0] - wrist_x)**2 + 
                                   (intersection_points[:, 1] - wrist_y)**2)
                closest_idx = np.argmin(distances)
                return tuple(intersection_points[closest_idx])
            else:
                # Return center of intersection
                return tuple(np.mean(intersection_points, axis=0))
        
        valid_points = np.array(valid_points)
        
        # Select target: prefer closest to wrist, otherwise closest to robot base
        if wrist_position is not None:
            wrist_x, wrist_y = wrist_position
            distances = np.sqrt((valid_points[:, 0] - wrist_x)**2 + 
                               (valid_points[:, 1] - wrist_y)**2)
            closest_idx = np.argmin(distances)
            return tuple(valid_points[closest_idx])
        else:
            # Prefer point closest to robot base (shorter path)
            base_x, base_y = self.base_x, self.base_y
            distances = np.sqrt((valid_points[:, 0] - base_x)**2 + 
                               (valid_points[:, 1] - base_y)**2)
            closest_idx = np.argmin(distances)
            return tuple(valid_points[closest_idx])

