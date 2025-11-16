"""
Object/Item Classes Module

This module provides classes for representing objects/items that need to be
moved by the dual-arm system. Supports various shapes: point, circle, rectangle, etc.
"""

from abc import ABC, abstractmethod
from typing import Tuple
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class Object(ABC):
    """Abstract base class for objects/items."""
    
    @abstractmethod
    def contains_point(self, x: float, y: float) -> bool:
        """
        Check if a point is inside the object.
        
        Args:
            x: X coordinate
            y: Y coordinate
            
        Returns:
            True if point is inside object, False otherwise
        """
        pass
    
    @abstractmethod
    def get_center(self) -> np.ndarray:
        """
        Get the center position of the object.
        
        Returns:
            Center position as numpy array [x, y]
        """
        pass
    
    @abstractmethod
    def get_bounds(self) -> Tuple[float, float, float, float]:
        """
        Get bounding box of the object.
        
        Returns:
            Tuple of (x_min, x_max, y_min, y_max)
        """
        pass
    
    @abstractmethod
    def visualize(self, ax):
        """
        Visualize the object on matplotlib axes.
        
        Args:
            ax: Matplotlib axes object
        """
        pass


class PointObject(Object):
    """Point object at a specific location."""
    
    def __init__(self, x: float, y: float):
        """
        Initialize point object.
        
        Args:
            x: X coordinate
            y: Y coordinate
        """
        self.x = x
        self.y = y
    
    def contains_point(self, x: float, y: float, tolerance: float = 1e-6) -> bool:
        """Check if point is at this location (within tolerance)."""
        return abs(x - self.x) < tolerance and abs(y - self.y) < tolerance
    
    def get_center(self) -> np.ndarray:
        """Get center position (same as point location)."""
        return np.array([self.x, self.y])
    
    def get_bounds(self) -> Tuple[float, float, float, float]:
        """Get bounding box (point has zero size)."""
        return (self.x, self.x, self.y, self.y)
    
    def visualize(self, ax, color='green', marker='o', size=50, **kwargs):
        """Visualize point object."""
        ax.scatter(self.x, self.y, c=color, marker=marker, s=size, 
                  label='Object', zorder=10, **kwargs)


class CircularRegion(Object):
    """Circular region object."""
    
    def __init__(self, center_x: float, center_y: float, radius: float):
        """
        Initialize circular region.
        
        Args:
            center_x: X coordinate of center
            center_y: Y coordinate of center
            radius: Radius of circle
        """
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius
    
    def contains_point(self, x: float, y: float) -> bool:
        """Check if point is inside circle."""
        dx = x - self.center_x
        dy = y - self.center_y
        return dx * dx + dy * dy <= self.radius * self.radius
    
    def get_center(self) -> np.ndarray:
        """Get center position."""
        return np.array([self.center_x, self.center_y])
    
    def get_bounds(self) -> Tuple[float, float, float, float]:
        """Get bounding box."""
        return (self.center_x - self.radius, self.center_x + self.radius,
                self.center_y - self.radius, self.center_y + self.radius)
    
    def visualize(self, ax, color='green', alpha=0.3, edgecolor='darkgreen'):
        """Visualize circular region."""
        circle = plt.Circle((self.center_x, self.center_y), self.radius,
                           color=color, alpha=alpha, edgecolor=edgecolor,
                           linewidth=2, label='Object', zorder=5)
        ax.add_patch(circle)
        # Also mark center
        ax.scatter(self.center_x, self.center_y, c=edgecolor, marker='x',
                  s=100, zorder=10)


class RectangularRegion(Object):
    """Rectangular region object."""
    
    def __init__(self, center_x: float, center_y: float, width: float,
                 height: float, rotation: float = 0.0):
        """
        Initialize rectangular region.
        
        Args:
            center_x: X coordinate of center
            center_y: Y coordinate of center
            width: Width of rectangle
            height: Height of rectangle
            rotation: Rotation angle in radians (default: 0)
        """
        self.center_x = center_x
        self.center_y = center_y
        self.width = width
        self.height = height
        self.rotation = rotation
    
    def contains_point(self, x: float, y: float) -> bool:
        """Check if point is inside rectangle."""
        # Translate to center
        dx = x - self.center_x
        dy = y - self.center_y
        
        # Rotate back to axis-aligned
        cos_r = np.cos(-self.rotation)
        sin_r = np.sin(-self.rotation)
        dx_rot = dx * cos_r - dy * sin_r
        dy_rot = dx * sin_r + dy * cos_r
        
        # Check if in axis-aligned rectangle
        return abs(dx_rot) <= self.width / 2 and abs(dy_rot) <= self.height / 2
    
    def get_center(self) -> np.ndarray:
        """Get center position."""
        return np.array([self.center_x, self.center_y])
    
    def get_bounds(self) -> Tuple[float, float, float, float]:
        """Get bounding box (axis-aligned)."""
        # Approximate with rotated corners
        half_w = self.width / 2
        half_h = self.height / 2
        cos_r = abs(np.cos(self.rotation))
        sin_r = abs(np.sin(self.rotation))
        
        # Maximum extent
        max_x = half_w * cos_r + half_h * sin_r
        max_y = half_w * sin_r + half_h * cos_r
        
        return (self.center_x - max_x, self.center_x + max_x,
                self.center_y - max_y, self.center_y + max_y)
    
    def visualize(self, ax, color='green', alpha=0.3, edgecolor='darkgreen'):
        """Visualize rectangular region."""
        rect = patches.Rectangle(
            (self.center_x - self.width/2, self.center_y - self.height/2),
            self.width, self.height,
            angle=np.degrees(self.rotation),
            color=color, alpha=alpha, edgecolor=edgecolor,
            linewidth=2, label='Object', zorder=5
        )
        ax.add_patch(rect)
        # Also mark center
        ax.scatter(self.center_x, self.center_y, c=edgecolor, marker='x',
                  s=100, zorder=10)

