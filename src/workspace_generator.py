"""
Workspace Generator Module

This module provides functionality to generate and visualize the workspace
of 2R planar robot arms, both individually and for dual-arm systems.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List, Optional
from .two_link_arm import TwoLinkArm, DualArm


class WorkspaceGenerator:
    """Generates and visualizes workspace for 2R planar robot arms."""
    
    def __init__(self, arm: TwoLinkArm, resolution: int = 100):
        """
        Initialize workspace generator.
        
        Args:
            arm: TwoLinkArm instance
            resolution: Resolution for workspace sampling (default: 100)
        """
        self.arm = arm
        self.resolution = resolution
        
    def generate_workspace(self, theta1_range: Tuple[float, float] = (-np.pi, np.pi),
                          theta2_range: Tuple[float, float] = (-np.pi, np.pi)) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate workspace points by sampling joint angles.
        
        Args:
            theta1_range: Range for theta1 (min, max)
            theta2_range: Range for theta2 (min, max)
            
        Returns:
            Tuple of (x_points, y_points) arrays
        """
        theta1_vals = np.linspace(theta1_range[0], theta1_range[1], self.resolution)
        theta2_vals = np.linspace(theta2_range[0], theta2_range[1], self.resolution)
        
        x_points = []
        y_points = []
        
        for theta1 in theta1_vals:
            for theta2 in theta2_vals:
                pos = self.arm.forward_kinematics(theta1, theta2)
                x_points.append(pos[0])
                y_points.append(pos[1])
                
        return np.array(x_points), np.array(y_points)
    
    def generate_workspace_boundary(self, num_points: int = 360) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generate workspace boundary by sampling at joint limits.
        
        Args:
            num_points: Number of points to sample on boundary
            
        Returns:
            Tuple of (x_boundary, y_boundary) arrays
        """
        theta1_vals = np.linspace(-np.pi, np.pi, num_points)
        theta2_vals = np.linspace(-np.pi, np.pi, num_points)
        
        x_boundary = []
        y_boundary = []
        
        # Sample at theta2 = -π and θ2 = π (elbow up and down configurations)
        for theta1 in theta1_vals:
            # Elbow up configuration (theta2 = -π)
            pos_up = self.arm.forward_kinematics(theta1, -np.pi)
            x_boundary.append(pos_up[0])
            y_boundary.append(pos_up[1])
            
            # Elbow down configuration (theta2 = π)
            pos_down = self.arm.forward_kinematics(theta1, np.pi)
            x_boundary.append(pos_down[0])
            y_boundary.append(pos_down[1])
        
        return np.array(x_boundary), np.array(y_boundary)
    
    def plot_workspace(self, show_boundary: bool = True, show_points: bool = False,
                      figsize: Tuple[int, int] = (8, 8)) -> plt.Figure:
        """
        Plot the workspace of the robot arm.
        
        Args:
            show_boundary: Whether to show workspace boundary
            show_points: Whether to show sampled workspace points
            figsize: Figure size (width, height)
            
        Returns:
            matplotlib Figure object
        """
        fig, ax = plt.subplots(figsize=figsize)
        ax.set_aspect('equal')
        
        if show_points:
            x_points, y_points = self.generate_workspace()
            ax.scatter(x_points, y_points, s=1, alpha=0.3, color='lightblue', label='Workspace Points')
        
        if show_boundary:
            x_boundary, y_boundary = self.generate_workspace_boundary()
            ax.plot(x_boundary, y_boundary, 'b-', linewidth=2, label='Workspace Boundary')
        
        # Plot arm configuration at origin
        ax.plot(0, 0, 'ko', markersize=8, label='Base')
        
        # Add circles for reach limits
        inner_radius = abs(self.arm.L1 - self.arm.L2)
        outer_radius = self.arm.L1 + self.arm.L2
        
        circle_inner = plt.Circle((0, 0), inner_radius, fill=False, linestyle='--', 
                                 color='red', alpha=0.7, label=f'Inner Limit (r={inner_radius:.2f})')
        circle_outer = plt.Circle((0, 0), outer_radius, fill=False, linestyle='--', 
                                 color='red', alpha=0.7, label=f'Outer Limit (r={outer_radius:.2f})')
        
        ax.add_patch(circle_inner)
        ax.add_patch(circle_outer)
        
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_title(f'{self.arm.name} Workspace\n(L1={self.arm.L1}, L2={self.arm.L2})')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        return fig


class DualArmWorkspaceGenerator:
    """Generates and visualizes workspace for dual-arm systems."""
    
    def __init__(self, dual_arm: DualArm, resolution: int = 100):
        """
        Initialize dual-arm workspace generator.
        
        Args:
            dual_arm: DualArm instance
            resolution: Resolution for workspace sampling
        """
        self.dual_arm = dual_arm
        self.resolution = resolution
        
    def generate_dual_workspace(self, theta1_range: Tuple[float, float] = (-np.pi, np.pi),
                               theta2_range: Tuple[float, float] = (-np.pi, np.pi)) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate workspace points for both arms.
        
        Args:
            theta1_range: Range for theta1 (min, max)
            theta2_range: Range for theta2 (min, max)
            
        Returns:
            Tuple of (left_x, left_y, right_x, right_y) arrays
        """
        left_generator = WorkspaceGenerator(self.dual_arm.left_arm, self.resolution)
        right_generator = WorkspaceGenerator(self.dual_arm.right_arm, self.resolution)
        
        left_x, left_y = left_generator.generate_workspace(theta1_range, theta2_range)
        right_x, right_y = right_generator.generate_workspace(theta1_range, theta2_range)
        
        # Transform to global coordinates
        left_x_global = left_x + self.dual_arm.left_base[0]
        left_y_global = left_y + self.dual_arm.left_base[1]
        right_x_global = right_x + self.dual_arm.right_base[0]
        right_y_global = right_y + self.dual_arm.right_base[1]
        
        return left_x_global, left_y_global, right_x_global, right_y_global
    
    def plot_dual_workspace(self, show_boundary: bool = True, show_points: bool = False,
                           figsize: Tuple[int, int] = (12, 8)) -> plt.Figure:
        """
        Plot the workspace of both arms.
        
        Args:
            show_boundary: Whether to show workspace boundaries
            show_points: Whether to show sampled workspace points
            figsize: Figure size (width, height)
            
        Returns:
            matplotlib Figure object
        """
        fig, ax = plt.subplots(figsize=figsize)
        ax.set_aspect('equal')
        
        # Generate workspaces
        left_generator = WorkspaceGenerator(self.dual_arm.left_arm, self.resolution)
        right_generator = WorkspaceGenerator(self.dual_arm.right_arm, self.resolution)
        
        if show_points:
            left_x, left_y, right_x, right_y = self.generate_dual_workspace()
            ax.scatter(left_x, left_y, s=1, alpha=0.3, color='lightblue', label='Left Arm Workspace')
            ax.scatter(right_x, right_y, s=1, alpha=0.3, color='lightcoral', label='Right Arm Workspace')
        
        if show_boundary:
            # Left arm boundary
            left_x_boundary, left_y_boundary = left_generator.generate_workspace_boundary()
            left_x_global = left_x_boundary + self.dual_arm.left_base[0]
            left_y_global = left_y_boundary + self.dual_arm.left_base[1]
            ax.plot(left_x_global, left_y_global, 'b-', linewidth=2, label='Left Arm Boundary')
            
            # Right arm boundary
            right_x_boundary, right_y_boundary = right_generator.generate_workspace_boundary()
            right_x_global = right_x_boundary + self.dual_arm.right_base[0]
            right_y_global = right_y_boundary + self.dual_arm.right_base[1]
            ax.plot(right_x_global, right_y_global, 'r-', linewidth=2, label='Right Arm Boundary')
        
        # Plot arm bases
        ax.plot(self.dual_arm.left_base[0], self.dual_arm.left_base[1], 'bo', 
                markersize=10, label='Left Base')
        ax.plot(self.dual_arm.right_base[0], self.dual_arm.right_base[1], 'ro', 
                markersize=10, label='Right Base')
        
        # Plot reach limits
        separation = np.linalg.norm(self.dual_arm.right_base - self.dual_arm.left_base)
        L1, L2 = self.dual_arm.left_arm.L1, self.dual_arm.left_arm.L2
        
        # Left arm limits
        left_inner = plt.Circle(self.dual_arm.left_base, abs(L1 - L2), fill=False, 
                               linestyle='--', color='blue', alpha=0.5)
        left_outer = plt.Circle(self.dual_arm.left_base, L1 + L2, fill=False, 
                               linestyle='--', color='blue', alpha=0.5)
        
        # Right arm limits
        right_inner = plt.Circle(self.dual_arm.right_base, abs(L1 - L2), fill=False, 
                                linestyle='--', color='red', alpha=0.5)
        right_outer = plt.Circle(self.dual_arm.right_base, L1 + L2, fill=False, 
                                linestyle='--', color='red', alpha=0.5)
        
        ax.add_patch(left_inner)
        ax.add_patch(left_outer)
        ax.add_patch(right_inner)
        ax.add_patch(right_outer)
        
        # Highlight overlapping workspace
        if show_boundary:
            # Find overlapping region (simplified - just show intersection of circles)
            overlap_center = (self.dual_arm.left_base + self.dual_arm.right_base) / 2
            overlap_radius = min(L1 + L2, separation / 2)
            overlap_circle = plt.Circle(overlap_center, overlap_radius, fill=False, 
                                      linestyle=':', color='green', linewidth=3, 
                                      label='Collaborative Workspace')
            ax.add_patch(overlap_circle)
        
        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')
        ax.set_title('Dual-Arm Workspace Visualization')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Set appropriate limits
        max_reach = L1 + L2
        ax.set_xlim(-separation/2 - max_reach - 0.5, separation/2 + max_reach + 0.5)
        ax.set_ylim(-0.5, max_reach + 0.5)
        
        return fig


def demo_workspace():
    """Demonstrate workspace generation for single and dual arms."""
    # Single arm demo
    print("Generating single arm workspace...")
    arm = TwoLinkArm(L1=1.0, L2=0.7, name="DemoArm")
    ws_gen = WorkspaceGenerator(arm, resolution=50)
    fig1 = ws_gen.plot_workspace(show_boundary=True, show_points=True)
    plt.show()
    
    # Dual arm demo
    print("Generating dual arm workspace...")
    dual_arm = DualArm(L1=1.0, L2=0.7, separation=2.0)
    dual_ws_gen = DualArmWorkspaceGenerator(dual_arm, resolution=50)
    fig2 = dual_ws_gen.plot_dual_workspace(show_boundary=True, show_points=True)
    plt.show()


if __name__ == "__main__":
    demo_workspace()
