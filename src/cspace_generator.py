"""
Configuration Space (C-space) Generator Module

This module provides functionality to generate and visualize the configuration space
of 2R planar robot arms, including joint limits, collision detection, and C-space
visualization for motion planning.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List, Optional, Dict
try:
    from .two_link_arm import TwoLinkArm, DualArm
except ImportError:
    from two_link_arm import TwoLinkArm, DualArm


class CSpaceGenerator:
    """Generates and visualizes configuration space for 2R planar robot arms."""
    
    def __init__(self, arm: TwoLinkArm, resolution: int = 100):
        """
        Initialize C-space generator.
        
        Args:
            arm: TwoLinkArm instance
            resolution: Resolution for C-space sampling (default: 100)
        """
        self.arm = arm
        self.resolution = resolution
        self.joint_limits = {
            'theta1': (-np.pi, np.pi),
            'theta2': (-np.pi, np.pi)
        }
        
    def set_joint_limits(self, theta1_range: Tuple[float, float], 
                        theta2_range: Tuple[float, float]):
        """
        Set joint angle limits for the robot.
        
        Args:
            theta1_range: (min, max) for theta1
            theta2_range: (min, max) for theta2
        """
        self.joint_limits['theta1'] = theta1_range
        self.joint_limits['theta2'] = theta2_range
    
    def generate_cspace_grid(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate a grid of configuration space points.
        
        Returns:
            Tuple of (theta1_grid, theta2_grid, valid_configs) where:
            - theta1_grid: 2D array of theta1 values
            - theta2_grid: 2D array of theta2 values  
            - valid_configs: 2D boolean array indicating valid configurations
        """
        theta1_min, theta1_max = self.joint_limits['theta1']
        theta2_min, theta2_max = self.joint_limits['theta2']
        
        theta1_vals = np.linspace(theta1_min, theta1_max, self.resolution)
        theta2_vals = np.linspace(theta2_min, theta2_max, self.resolution)
        
        theta1_grid, theta2_grid = np.meshgrid(theta1_vals, theta2_vals)
        valid_configs = np.ones_like(theta1_grid, dtype=bool)
        
        return theta1_grid, theta2_grid, valid_configs
    
    def check_self_collision(self, theta1: float, theta2: float) -> bool:
        """
        Check if a configuration results in self-collision.
        For 2R arms, this is typically not an issue, but can be extended.
        
        Args:
            theta1: First joint angle
            theta2: Second joint angle
            
        Returns:
            True if collision detected, False otherwise
        """
        # For 2R planar arms, self-collision is rare
        # This is a placeholder for more complex collision detection
        return False
    
    def check_workspace_limits(self, theta1: float, theta2: float) -> bool:
        """
        Check if configuration is within workspace limits.
        
        Args:
            theta1: First joint angle
            theta2: Second joint angle
            
        Returns:
            True if within limits, False otherwise
        """
        pos = self.arm.forward_kinematics(theta1, theta2)
        r = np.sqrt(pos[0]**2 + pos[1]**2)
        
        # Check reachability
        L1, L2 = self.arm.L1, self.arm.L2
        return r <= (L1 + L2) and r >= abs(L1 - L2)
    
    def generate_valid_cspace(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate valid configuration space with collision checking.
        
        Returns:
            Tuple of (theta1_grid, theta2_grid, valid_configs)
        """
        theta1_grid, theta2_grid, valid_configs = self.generate_cspace_grid()
        
        # Check each configuration
        for i in range(self.resolution):
            for j in range(self.resolution):
                theta1 = theta1_grid[i, j]
                theta2 = theta2_grid[i, j]
                
                # Check workspace limits
                if not self.check_workspace_limits(theta1, theta2):
                    valid_configs[i, j] = False
                    continue
                
                # Check self-collision
                if self.check_self_collision(theta1, theta2):
                    valid_configs[i, j] = False
                    continue
        
        return theta1_grid, theta2_grid, valid_configs
    
    def plot_cspace(self, show_invalid: bool = True, figsize: Tuple[int, int] = (10, 8)) -> plt.Figure:
        """
        Plot the configuration space.
        
        Args:
            show_invalid: Whether to show invalid configurations
            figsize: Figure size (width, height)
            
        Returns:
            matplotlib Figure object
        """
        theta1_grid, theta2_grid, valid_configs = self.generate_valid_cspace()
        
        fig, ax = plt.subplots(figsize=figsize)
        
        # Plot valid configurations
        valid_mask = valid_configs
        ax.scatter(theta1_grid[valid_mask], theta2_grid[valid_mask], 
                  c='green', s=1, alpha=0.6, label='Valid Configurations')
        
        if show_invalid:
            # Plot invalid configurations
            invalid_mask = ~valid_configs
            ax.scatter(theta1_grid[invalid_mask], theta2_grid[invalid_mask], 
                      c='red', s=1, alpha=0.6, label='Invalid Configurations')
        
        ax.set_xlabel('Theta1 (radians)')
        ax.set_ylabel('Theta2 (radians)')
        ax.set_title(f'{self.arm.name} Configuration Space\n(L1={self.arm.L1}, L2={self.arm.L2})')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Set limits
        theta1_min, theta1_max = self.joint_limits['theta1']
        theta2_min, theta2_max = self.joint_limits['theta2']
        ax.set_xlim(theta1_min, theta1_max)
        ax.set_ylim(theta2_min, theta2_max)
        
        return fig
    
    def plot_cspace_heatmap(self, figsize: Tuple[int, int] = (10, 8)) -> plt.Figure:
        """
        Plot configuration space as a heatmap.
        
        Args:
            figsize: Figure size (width, height)
            
        Returns:
            matplotlib Figure object
        """
        theta1_grid, theta2_grid, valid_configs = self.generate_valid_cspace()
        
        fig, ax = plt.subplots(figsize=figsize)
        
        # Create heatmap data (1 for valid, 0 for invalid)
        heatmap_data = valid_configs.astype(float)
        
        im = ax.imshow(heatmap_data, extent=[
            self.joint_limits['theta1'][0], self.joint_limits['theta1'][1],
            self.joint_limits['theta2'][0], self.joint_limits['theta2'][1]
        ], origin='lower', cmap='RdYlGn', aspect='auto')
        
        ax.set_xlabel('Theta1 (radians)')
        ax.set_ylabel('Theta2 (radians)')
        ax.set_title(f'{self.arm.name} Configuration Space Heatmap\n(Green=Valid, Red=Invalid)')
        
        # Add colorbar
        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Configuration Validity')
        
        ax.grid(True, alpha=0.3)
        
        return fig


class DualArmCSpaceGenerator:
    """Generates and visualizes configuration space for dual-arm systems."""
    
    def __init__(self, dual_arm: DualArm, resolution: int = 50):
        """
        Initialize dual-arm C-space generator.
        
        Args:
            dual_arm: DualArm instance
            resolution: Resolution for C-space sampling
        """
        self.dual_arm = dual_arm
        self.resolution = resolution
        self.joint_limits = {
            'left_theta1': (-np.pi, np.pi),
            'left_theta2': (-np.pi, np.pi),
            'right_theta1': (-np.pi, np.pi),
            'right_theta2': (-np.pi, np.pi)
        }
    
    def set_joint_limits(self, left_theta1_range: Tuple[float, float],
                        left_theta2_range: Tuple[float, float],
                        right_theta1_range: Tuple[float, float],
                        right_theta2_range: Tuple[float, float]):
        """Set joint angle limits for both arms."""
        self.joint_limits['left_theta1'] = left_theta1_range
        self.joint_limits['left_theta2'] = left_theta2_range
        self.joint_limits['right_theta1'] = right_theta1_range
        self.joint_limits['right_theta2'] = right_theta2_range
    
    def check_arm_collision(self, left_angles: Tuple[float, float], 
                           right_angles: Tuple[float, float]) -> bool:
        """
        Check if the two arms collide with each other.
        
        Args:
            left_angles: (theta1, theta2) for left arm
            right_angles: (theta1, theta2) for right arm
            
        Returns:
            True if collision detected, False otherwise
        """
        # Get arm configurations
        left_base, left_joint, left_end = self.dual_arm.compute_fk_points(
            self.dual_arm.left_arm, self.dual_arm.left_base, *left_angles)
        right_base, right_joint, right_end = self.dual_arm.compute_fk_points(
            self.dual_arm.right_arm, self.dual_arm.right_base, *right_angles)
        
        # Check for collisions between arm segments
        # This is a simplified collision check - can be made more sophisticated
        segments_left = [
            (left_base, left_joint),
            (left_joint, left_end)
        ]
        segments_right = [
            (right_base, right_joint),
            (right_joint, right_end)
        ]
        
        # Check intersection between all segment pairs
        for seg_left in segments_left:
            for seg_right in segments_right:
                if self._line_segments_intersect(seg_left, seg_right):
                    return True
        
        return False
    
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
    
    def generate_dual_cspace(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Generate configuration space for dual-arm system.
        
        Returns:
            Tuple of (left_theta1, left_theta2, right_theta1, right_theta2, valid_configs)
        """
        # Generate angle grids
        left_theta1_vals = np.linspace(*self.joint_limits['left_theta1'], self.resolution)
        left_theta2_vals = np.linspace(*self.joint_limits['left_theta2'], self.resolution)
        right_theta1_vals = np.linspace(*self.joint_limits['right_theta1'], self.resolution)
        right_theta2_vals = np.linspace(*self.joint_limits['right_theta2'], self.resolution)
        
        # Create 4D grid (simplified to 2D for visualization)
        # We'll sample a subset of the full 4D space
        valid_configs = []
        left_theta1_sample = []
        left_theta2_sample = []
        right_theta1_sample = []
        right_theta2_sample = []
        
        # Sample configurations (this is computationally expensive for full 4D)
        for i in range(0, self.resolution, 5):  # Sample every 5th point
            for j in range(0, self.resolution, 5):
                for k in range(0, self.resolution, 5):
                    for l in range(0, self.resolution, 5):
                        left_angles = (left_theta1_vals[i], left_theta2_vals[j])
                        right_angles = (right_theta1_vals[k], right_theta2_vals[l])
                        
                        # Always append to all arrays
                        left_theta1_sample.append(left_theta1_vals[i])
                        left_theta2_sample.append(left_theta2_vals[j])
                        right_theta1_sample.append(right_theta1_vals[k])
                        right_theta2_sample.append(right_theta2_vals[l])

                        # Check if configuration is valid
                        is_valid = not self.check_arm_collision(left_angles, right_angles)
                        valid_configs.append(is_valid)
        
        return (np.array(left_theta1_sample), np.array(left_theta2_sample),
                np.array(right_theta1_sample), np.array(right_theta2_sample),
                np.array(valid_configs))
    
    def plot_dual_cspace_2d(self, figsize: Tuple[int, int] = (12, 8)) -> plt.Figure:
        """
        Plot 2D projection of dual-arm configuration space.
        
        Args:
            figsize: Figure size (width, height)
            
        Returns:
            matplotlib Figure object
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=figsize)
        
        # Generate sample configurations
        left_theta1, left_theta2, right_theta1, right_theta2, valid = self.generate_dual_cspace()
        
        # Convert valid to boolean array and ensure same length
        if len(valid) > 0:
            valid_bool = np.array(valid, dtype=bool)
            # Ensure arrays have same length
            min_len = min(len(left_theta1), len(left_theta2), len(right_theta1), len(right_theta2), len(valid_bool))
            left_theta1 = left_theta1[:min_len]
            left_theta2 = left_theta2[:min_len]
            right_theta1 = right_theta1[:min_len]
            right_theta2 = right_theta2[:min_len]
            valid_bool = valid_bool[:min_len]
            
            # Plot left arm C-space
            if np.any(valid_bool):
                ax1.scatter(left_theta1[valid_bool], left_theta2[valid_bool], 
                           c='blue', s=1, alpha=0.6, label='Valid Configurations')
            ax1.set_xlabel('Left Theta1 (radians)')
            ax1.set_ylabel('Left Theta2 (radians)')
            ax1.set_title('Left Arm C-space')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # Plot right arm C-space
            if np.any(valid_bool):
                ax2.scatter(right_theta1[valid_bool], right_theta2[valid_bool], 
                           c='red', s=1, alpha=0.6, label='Valid Configurations')
            ax2.set_xlabel('Right Theta1 (radians)')
            ax2.set_ylabel('Right Theta2 (radians)')
            ax2.set_title('Right Arm C-space')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
        else:
            ax1.set_xlabel('Left Theta1 (radians)')
            ax1.set_ylabel('Left Theta2 (radians)')
            ax1.set_title('Left Arm C-space (No valid configs)')
            ax2.set_xlabel('Right Theta1 (radians)')
            ax2.set_ylabel('Right Theta2 (radians)')
            ax2.set_title('Right Arm C-space (No valid configs)')
        
        plt.suptitle('Dual-Arm Configuration Space (2D Projections)')
        plt.tight_layout()
        
        return fig


def demo_cspace():
    """Demonstrate C-space generation for single and dual arms."""
    # Single arm demo
    print("Generating single arm C-space...")
    arm = TwoLinkArm(L1=1.0, L2=0.7, name="DemoArm")
    cspace_gen = CSpaceGenerator(arm, resolution=50)
    
    # Set some joint limits
    cspace_gen.set_joint_limits((-np.pi, np.pi), (-np.pi/2, np.pi/2))
    
    fig1 = cspace_gen.plot_cspace()
    plt.show()
    
    fig2 = cspace_gen.plot_cspace_heatmap()
    plt.show()
    
    # Dual arm demo
    print("Generating dual arm C-space...")
    dual_arm = DualArm(L1=1.0, L2=0.7, separation=2.0)
    dual_cspace_gen = DualArmCSpaceGenerator(dual_arm, resolution=20)
    
    fig3 = dual_cspace_gen.plot_dual_cspace_2d()
    plt.show()


if __name__ == "__main__":
    demo_cspace()
