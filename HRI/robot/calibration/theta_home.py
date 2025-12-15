"""
θ_home Helper: Loads and applies the home offset for FK/visualization.

This module provides:
- load_theta_home(): Loads θ_home from calibration file
- apply_offset(): Converts θ_sim → θ_physical (for FK/rendering)
- remove_offset(): Converts θ_physical → θ_sim (when reading robot state)

The offset is applied exactly once at the kinematics boundary.
"""

import json
import os
import numpy as np
from typing import Tuple, Optional

CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "home_pose.json")

_theta_home_cache: Optional[Tuple[float, float]] = None

def load_theta_home() -> Tuple[float, float]:
    """
    Load θ_home from calibration file.
    
    Returns:
        Tuple of (theta1_home, theta2_home) in radians.
        Defaults to (0.0, 0.0) if not found.
    """
    global _theta_home_cache
    
    if _theta_home_cache is not None:
        return _theta_home_cache
    
    if not os.path.exists(CALIBRATION_FILE):
        print(f"⚠️  {CALIBRATION_FILE} not found, using θ_home = (0, 0)")
        _theta_home_cache = (0.0, 0.0)
        return _theta_home_cache
    
    try:
        with open(CALIBRATION_FILE, "r") as f:
            data = json.load(f)
        
        if "theta_home" in data:
            theta_home = data["theta_home"]
            _theta_home_cache = (float(theta_home["theta1"]), float(theta_home["theta2"]))
            return _theta_home_cache
        else:
            print(f"⚠️  θ_home not found in {CALIBRATION_FILE}, using (0, 0)")
            _theta_home_cache = (0.0, 0.0)
            return _theta_home_cache
    except Exception as e:
        print(f"⚠️  Error loading θ_home: {e}, using (0, 0)")
        _theta_home_cache = (0.0, 0.0)
        return _theta_home_cache

def apply_offset(theta1_sim: float, theta2_sim: float) -> Tuple[float, float]:
    """
    Convert θ_sim (planner coordinates) to θ_physical (FK/rendering coordinates).
    
    This is applied at the kinematics boundary:
    - Planner works in θ_sim (zero-centered)
    - FK/visualization uses θ_physical = θ_sim + θ_home
    
    Args:
        theta1_sim: Simulator angle for joint 1 (radians)
        theta2_sim: Simulator angle for joint 2 (radians)
        
    Returns:
        Tuple of (theta1_physical, theta2_physical)
    """
    theta1_home, theta2_home = load_theta_home()
    return (theta1_sim + theta1_home, theta2_sim + theta2_home)

def remove_offset(theta1_physical: float, theta2_physical: float) -> Tuple[float, float]:
    """
    Convert θ_physical to θ_sim (inverse of apply_offset).
    
    Used when reading robot state that's already in physical coordinates.
    
    Args:
        theta1_physical: Physical angle for joint 1 (radians)
        theta2_physical: Physical angle for joint 2 (radians)
        
    Returns:
        Tuple of (theta1_sim, theta2_sim)
    """
    theta1_home, theta2_home = load_theta_home()
    return (theta1_physical - theta1_home, theta2_physical - theta2_home)

