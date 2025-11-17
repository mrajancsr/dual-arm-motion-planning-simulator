"""
Dual-Arm System Module

This module provides a generic dual-arm system that can hold any two robot arms
that inherit from RobotArmBase. The system manages base positions, coordinate
transformations, and inter-arm operations.
"""

from typing import Optional, Tuple, List, Dict, Union
import numpy as np
import matplotlib.pyplot as plt

try:
    from .robot_arm_base import RobotArmBase
    from .arms.two_link_arm import TwoLinkArm
except ImportError:
    from robot_arm_base import RobotArmBase
    from arms.two_link_arm import TwoLinkArm


class DualArm:
    """Generic dual-arm system that holds two robot arms."""
    
    def __init__(self, L1=1.0, L2=0.7, separation=2.0, 
                 left_arm: Optional[RobotArmBase] = None,
                 right_arm: Optional[RobotArmBase] = None,
                 obstacles: Optional[List[Dict]] = None,
                 **arm_params):
        """
        Initialize dual arm system.
        
        Args:
            L1: Link 1 length (for backward compatibility, used if arms not provided)
            L2: Link 2 length (for backward compatibility, used if arms not provided)
            separation: Distance between arm bases
            left_arm: Left arm instance (defaults to TwoLinkArm if None)
            right_arm: Right arm instance (defaults to TwoLinkArm if None)
            obstacles: Optional list of obstacle dictionaries. Each obstacle dict can be:
                - Circle: {'type': 'circle', 'center': [x, y], 'radius': r}
                - Rectangle: {'type': 'rectangle', 'center': [x, y], 'width': w, 'height': h, 'rotation': theta (rad, optional)}
                - Polygon: {'type': 'polygon', 'vertices': [[x1, y1], [x2, y2], ...]}
            **arm_params: Additional parameters for default arm creation
        """
        # Default to TwoLinkArm if no arms provided (backward compatibility)
        # Support both old-style (L1, L2 as direct params) and new-style (arm_params)
        if left_arm is None:
            L1_val = arm_params.get('L1', L1)
            L2_val = arm_params.get('L2', L2)
            self.left_arm = TwoLinkArm(L1=L1_val, L2=L2_val, name="LeftArm")
        else:
            if not isinstance(left_arm, RobotArmBase):
                raise TypeError("left_arm must inherit from RobotArmBase")
            self.left_arm = left_arm
        
        if right_arm is None:
            L1_val = arm_params.get('L1', L1)
            L2_val = arm_params.get('L2', L2)
            self.right_arm = TwoLinkArm(L1=L1_val, L2=L2_val, name="RightArm")
        else:
            if not isinstance(right_arm, RobotArmBase):
                raise TypeError("right_arm must inherit from RobotArmBase")
            self.right_arm = right_arm

        # Fixed bases, left at (-separation/2, 0) right at (separation/2, 0)
        self.left_base = np.array([-separation / 2, 0.0])
        self.right_base = np.array([separation / 2, 0.0])
        
        # Store obstacles
        self.obstacles = obstacles if obstacles is not None else []
        
        # State management for current arm configurations
        self._current_left_config = None
        self._current_right_config = None

    def compute_positions(
        self, left_angles, right_angles
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute end-effector positions for both arms.
        
        Args:
            left_angles: Joint angles for left arm (tuple or unpacked)
            right_angles: Joint angles for right arm (tuple or unpacked)
            
        Returns:
            Tuple of (left_position, right_position) in local coordinates
        """
        left_pos = self.left_arm.forward_kinematics(*left_angles)
        right_pos = self.right_arm.forward_kinematics(*right_angles)
        return left_pos, right_pos

    def compute_fk_points(self, arm: RobotArmBase, base: np.ndarray, *joint_angles):
        """
        Returns joint and end-effector positions in base frame.
        
        This method works generically for any arm type by using forward_kinematics
        and computing intermediate points. For 2-link arms, it computes the
        elbow joint position. For 6-link arms, it computes all intermediate joints.
        For other arm types, it falls back to base and end-effector only.
        
        Args:
            arm: Robot arm instance (must inherit from RobotArmBase)
            base: Base position in global frame
            *joint_angles: Joint angles for the arm
            
        Returns:
            Tuple of (base, intermediate_points..., end_effector) positions
        """
        # For 2-link arms, we can compute the elbow joint
        if hasattr(arm, 'L1') and hasattr(arm, 'L2') and len(joint_angles) == 2:
            L1, L2 = arm.L1, arm.L2
            theta1, theta2 = joint_angles[0], joint_angles[1]
            
            joint = base + np.array([L1 * np.cos(theta1), L1 * np.sin(theta1)])
            end_eff = joint + np.array(
                [L2 * np.cos(theta1 + theta2), L2 * np.sin(theta1 + theta2)]
            )
            return base, joint, end_eff
        # For 6-link arms, compute all intermediate joints
        elif hasattr(arm, 'link_lengths') and len(joint_angles) == 6:
            positions = [base.copy()]
            current_pos = base.copy()
            cumulative_angle = 0.0
            
            for length, angle in zip(arm.link_lengths, joint_angles):
                cumulative_angle += float(angle)
                current_pos = current_pos + np.array([
                    length * np.cos(cumulative_angle),
                    length * np.sin(cumulative_angle)
                ])
                positions.append(current_pos.copy())
            
            return tuple(positions)
        else:
            # Generic fallback: just return base and end-effector
            end_eff_pos = arm.forward_kinematics(*joint_angles)
            end_eff = base + end_eff_pos
            return base, end_eff
    
    def _get_all_link_segments(self, config: np.ndarray) -> List[Tuple[np.ndarray, np.ndarray]]:
        """
        Get all link segments for both arms in a configuration.
        
        Args:
            config: Full configuration array [θ1_left, θ2_left, ..., θ1_right, ...]
            
        Returns:
            List of (start_point, end_point) tuples for all link segments
        """
        left_config, right_config = self._split_configuration(config)
        
        # Get all joint positions for both arms
        left_points = self.compute_fk_points(self.left_arm, self.left_base, *left_config)
        right_points = self.compute_fk_points(self.right_arm, self.right_base, *right_config)
        
        # Convert to list of segments
        segments = []
        
        # Left arm segments
        for i in range(len(left_points) - 1):
            segments.append((left_points[i], left_points[i + 1]))
        
        # Right arm segments
        for i in range(len(right_points) - 1):
            segments.append((right_points[i], right_points[i + 1]))
        
        return segments

    def get_configuration(self) -> Tuple[Tuple, Tuple]:
        """
        Get current configuration of both arms.
        
        Note: Not needed for RRT* - state is managed externally.
        Kept for backward compatibility.
        
        Returns:
            Tuple of (left_config, right_config)
            
        Raises:
            ValueError: If configuration has not been set
        """
        if self._current_left_config is None or self._current_right_config is None:
            raise ValueError(
                "Configuration not set. Call set_configuration() first."
            )
        return self._current_left_config, self._current_right_config

    def set_configuration(self, left_config: Tuple, right_config: Tuple):
        """
        Set configuration of both arms.
        
        Note: Not needed for RRT* - state is managed externally.
        Kept for backward compatibility.
        
        Args:
            left_config: Joint angles for left arm
            right_config: Joint angles for right arm
        """
        self._current_left_config = tuple(left_config)
        self._current_right_config = tuple(right_config)

    def _split_configuration(self, config: np.ndarray) -> Tuple[Tuple, Tuple]:
        """
        Split full configuration array into left and right arm configs.
        
        Args:
            config: Full configuration array [θ1_left, θ2_left, θ1_right, θ2_right, ...]
            
        Returns:
            Tuple of (left_config, right_config) as tuples
        """
        n_left = self.left_arm.get_num_joints()
        n_right = self.right_arm.get_num_joints()
        
        left_config = tuple(config[:n_left])
        right_config = tuple(config[n_left:n_left+n_right])
        
        return left_config, right_config

    def is_valid_configuration(self, config: np.ndarray) -> bool:
        """
        Check if a dual-arm configuration is valid.
        
        Checks:
        - Individual arm validity (joint limits, workspace, self-collision)
        - Inter-arm collision detection
        - Obstacle collision detection (if obstacles are defined)
        
        Args:
            config: Full configuration array [θ1_left, θ2_left, θ1_right, θ2_right, ...]
            
        Returns:
            True if configuration is valid, False otherwise
        """
        # Split config into left and right
        left_config, right_config = self._split_configuration(config)
        
        # Check individual arm validity
        if not self.left_arm.is_valid_configuration(*left_config):
            return False
        if not self.right_arm.is_valid_configuration(*right_config):
            return False
        
        # Get all link segments for collision checking
        segments = self._get_all_link_segments(config)
        
        # Check inter-arm collision
        n_left_segments = len(self.compute_fk_points(self.left_arm, self.left_base, *left_config)) - 1
        n_right_segments = len(self.compute_fk_points(self.right_arm, self.right_base, *right_config)) - 1
        
        left_segments = segments[:n_left_segments]
        right_segments = segments[n_left_segments:]
        
        # Check if any left and right segments intersect
        for seg_left in left_segments:
            for seg_right in right_segments:
                if self._line_segments_intersect(seg_left, seg_right):
                    return False
        
        # Check obstacle collisions if obstacles are defined
        if self.obstacles:
            for segment in segments:
                if self._segment_intersects_obstacles(segment):
                    return False
        
        return True

    def _line_segments_intersect(self, seg1: Tuple[np.ndarray, np.ndarray], 
                                 seg2: Tuple[np.ndarray, np.ndarray]) -> bool:
        """
        Check if two line segments intersect.
        
        Args:
            seg1: First line segment as (start, end)
            seg2: Second line segment as (start, end)
            
        Returns:
            True if segments intersect, False otherwise
        """
        p1, p2 = seg1
        p3, p4 = seg2
        
        # Vector calculations
        d1 = p2 - p1
        d2 = p4 - p3
        
        # Cross product for intersection test
        denom = d1[0] * d2[1] - d1[1] * d2[0]
        
        if abs(denom) < 1e-10:  # Parallel lines
            return False
        
        t1 = ((p3[0] - p1[0]) * d2[1] - (p3[1] - p1[1]) * d2[0]) / denom
        t2 = ((p3[0] - p1[0]) * d1[1] - (p3[1] - p1[1]) * d1[0]) / denom
        
        return 0 <= t1 <= 1 and 0 <= t2 <= 1
    
    def _segment_intersects_obstacles(self, segment: Tuple[np.ndarray, np.ndarray]) -> bool:
        """
        Check if a line segment intersects any obstacle.
        
        Args:
            segment: Line segment as (start, end)
            
        Returns:
            True if segment intersects any obstacle, False otherwise
        """
        for obstacle in self.obstacles:
            if self._segment_intersects_obstacle(segment, obstacle):
                return True
        return False
    
    def _segment_intersects_obstacle(self, segment: Tuple[np.ndarray, np.ndarray], 
                                     obstacle: Dict) -> bool:
        """
        Check if a line segment intersects a specific obstacle.
        
        Args:
            segment: Line segment as (start, end)
            obstacle: Obstacle dictionary with 'type' key
            
        Returns:
            True if segment intersects obstacle, False otherwise
        """
        obs_type = obstacle.get('type', '').lower()
        
        if obs_type == 'circle':
            return self._segment_intersects_circle(segment, obstacle)
        elif obs_type == 'rectangle':
            return self._segment_intersects_rectangle(segment, obstacle)
        elif obs_type == 'polygon':
            return self._segment_intersects_polygon(segment, obstacle)
        else:
            # Unknown obstacle type, skip it
            return False
    
    def _segment_intersects_circle(self, segment: Tuple[np.ndarray, np.ndarray], 
                                   obstacle: Dict) -> bool:
        """
        Check if a line segment intersects a circle obstacle.
        
        Args:
            segment: Line segment as (start, end)
            obstacle: Circle obstacle dict with 'center' and 'radius'
            
        Returns:
            True if segment intersects circle, False otherwise
        """
        p1, p2 = segment
        center = np.array(obstacle['center'])
        radius = obstacle['radius']
        
        # Vector from p1 to p2
        d = p2 - p1
        # Vector from p1 to circle center
        f = p1 - center
        
        # Quadratic equation: a*t^2 + b*t + c = 0
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - radius * radius
        
        # Handle degenerate case (zero-length segment)
        if a < 1e-10:
            # Segment is a point, check if point is inside circle
            return np.linalg.norm(p1 - center) <= radius
        
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0:
            return False  # No intersection
        
        # Check if intersection point is on the segment
        sqrt_disc = np.sqrt(discriminant)
        t1 = (-b - sqrt_disc) / (2 * a)
        t2 = (-b + sqrt_disc) / (2 * a)
        
        # Check if any solution is in [0, 1]
        return (0 <= t1 <= 1) or (0 <= t2 <= 1)
    
    def _segment_intersects_rectangle(self, segment: Tuple[np.ndarray, np.ndarray], 
                                       obstacle: Dict) -> bool:
        """
        Check if a line segment intersects a rectangle obstacle.
        
        Args:
            segment: Line segment as (start, end)
            obstacle: Rectangle obstacle dict with 'center', 'width', 'height', optional 'rotation'
            
        Returns:
            True if segment intersects rectangle, False otherwise
        """
        center = np.array(obstacle['center'])
        width = obstacle['width']
        height = obstacle['height']
        rotation = obstacle.get('rotation', 0.0)
        
        # Get rectangle vertices (unrotated, centered at origin)
        half_w = width / 2
        half_h = height / 2
        vertices = np.array([
            [-half_w, -half_h],
            [half_w, -half_h],
            [half_w, half_h],
            [-half_w, half_h]
        ])
        
        # Rotate vertices
        if rotation != 0:
            cos_r = np.cos(rotation)
            sin_r = np.sin(rotation)
            rotation_matrix = np.array([[cos_r, -sin_r], [sin_r, cos_r]])
            vertices = vertices @ rotation_matrix.T
        
        # Translate to center
        vertices = vertices + center
        
        # Convert rectangle to polygon and check intersection
        polygon_obstacle = {'type': 'polygon', 'vertices': vertices.tolist()}
        return self._segment_intersects_polygon(segment, polygon_obstacle)
    
    def _segment_intersects_polygon(self, segment: Tuple[np.ndarray, np.ndarray], 
                                     obstacle: Dict) -> bool:
        """
        Check if a line segment intersects a polygon obstacle.
        
        Uses ray-casting algorithm: check if segment intersects any polygon edge.
        
        Args:
            segment: Line segment as (start, end)
            obstacle: Polygon obstacle dict with 'vertices' list
            
        Returns:
            True if segment intersects polygon, False otherwise
        """
        vertices = np.array(obstacle['vertices'])
        
        # Check if segment intersects any polygon edge
        n_vertices = len(vertices)
        for i in range(n_vertices):
            edge_start = vertices[i]
            edge_end = vertices[(i + 1) % n_vertices]
            edge_segment = (edge_start, edge_end)
            
            if self._line_segments_intersect(segment, edge_segment):
                return True
        
        # Also check if segment endpoints are inside polygon
        p1, p2 = segment
        if self._point_in_polygon(p1, vertices) or self._point_in_polygon(p2, vertices):
            return True
        
        return False
    
    def _point_in_polygon(self, point: np.ndarray, vertices: np.ndarray) -> bool:
        """
        Check if a point is inside a polygon using ray-casting algorithm.
        
        Args:
            point: Point to check [x, y]
            vertices: Array of polygon vertices
            
        Returns:
            True if point is inside polygon, False otherwise
        """
        x, y = point[0], point[1]
        n = len(vertices)
        inside = False
        
        p1x, p1y = vertices[0]
        for i in range(1, n + 1):
            p2x, p2y = vertices[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                            if p1x == p2x or x <= xinters:
                                inside = not inside
                        else:
                            # Horizontal edge - point is on edge if x is between endpoints
                            if min(p1x, p2x) <= x <= max(p1x, p2x):
                                inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside

    def get_end_effector_positions(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get end-effector positions for both arms in global frame.
        
        Note: Not needed for RRT* - use compute_positions() with explicit configs.
        Kept for backward compatibility.
        
        Returns:
            Tuple of (left_end_effector, right_end_effector) in global coordinates
            
        Raises:
            ValueError: If configuration has not been set
        """
        if self._current_left_config is None or self._current_right_config is None:
            raise ValueError(
                "Configuration not set. Call set_configuration() first."
            )
        
        left_local = self.left_arm.forward_kinematics(*self._current_left_config)
        right_local = self.right_arm.forward_kinematics(*self._current_right_config)
        
        left_global = self.left_base + left_local
        right_global = self.right_base + right_local
        
        return left_global, right_global

    def plot_arms(self, left_angles, right_angles, left_goal=None, right_goal=None):
        """
        Plot both arms with their configurations.
        
        Args:
            left_angles: Joint angles for left arm
            right_angles: Joint angles for right arm
            left_goal: Optional goal position for left arm
            right_goal: Optional goal position for right arm
        """
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.set_aspect("equal")
        ax.set_xlim(-3, 3)
        ax.set_ylim(-0.5, 3)

        # Compute FK points (returns variable number of points depending on arm type)
        left_points = self.compute_fk_points(self.left_arm, self.left_base, *left_angles)
        right_points = self.compute_fk_points(
            self.right_arm, self.right_base, *right_angles
        )

        # Extract x, y coordinates from points (handles variable number of points)
        left_x = [p[0] for p in left_points]
        left_y = [p[1] for p in left_points]
        right_x = [p[0] for p in right_points]
        right_y = [p[1] for p in right_points]

        # Plot arms
        ax.plot(
            left_x, left_y,
            "-o",
            color="tab:blue",
            label="Left Arm",
        )
        ax.plot(
            right_x, right_y,
            "-o",
            color="tab:red",
            label="Right Arm",
        )

        # Plot goals
        if left_goal is not None:
            ax.plot(
                left_goal[0],
                left_goal[1],
                "x",
                color="blue",
                markersize=10,
                label="Left Goal",
            )
        if right_goal is not None:
            ax.plot(
                right_goal[0],
                right_goal[1],
                "x",
                color="red",
                markersize=10,
                label="Right Goal",
            )

        ax.legend()
        ax.set_title("Dual-Arm Robot Visualization")
        plt.show()

