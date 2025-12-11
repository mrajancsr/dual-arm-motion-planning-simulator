"""
Pixel to 3D Calibration Module

Converts hand pixel coordinates (u, v) + depth to 3D point (x, y, z).
Backprojects pixels into camera coordinate frame.
"""

import numpy as np
from typing import Optional, Tuple, Union


def pixel_to_3d(u: float, v: float, 
                depth_map: np.ndarray,
                intrinsics: dict,
                depth_value: Optional[float] = None) -> Optional[np.ndarray]:
    """
    Convert pixel coordinates to 3D point in camera coordinates.
    
    Args:
        u, v: Pixel coordinates (u is column, v is row, 0-based)
        depth_map: Depth map (H, W) in meters
        intrinsics: Camera intrinsics dict with 'fx', 'fy', 'cx', 'cy'
        depth_value: Optional depth value in meters (if None, looks up from depth_map)
        
    Returns:
        3D point [x, y, z] in camera coordinates, or None if invalid
    """
    # Get intrinsics
    fx = intrinsics['fx']
    fy = intrinsics['fy']
    cx = intrinsics['cx']
    cy = intrinsics['cy']
    
    # Clamp pixel coordinates to valid range
    h, w = depth_map.shape
    u = np.clip(u, 0, w - 1)
    v = np.clip(v, 0, h - 1)
    
    # Get depth value
    if depth_value is None:
        # Look up depth at pixel (use bilinear interpolation for sub-pixel accuracy)
        u_int = int(u)
        v_int = int(v)
        u_frac = u - u_int
        v_frac = v - v_int
        
        # Bilinear interpolation
        if u_int < w - 1 and v_int < h - 1:
            z00 = depth_map[v_int, u_int]
            z01 = depth_map[v_int, u_int + 1]
            z10 = depth_map[v_int + 1, u_int]
            z11 = depth_map[v_int + 1, u_int + 1]
            
            z = (z00 * (1 - u_frac) * (1 - v_frac) +
                 z01 * u_frac * (1 - v_frac) +
                 z10 * (1 - u_frac) * v_frac +
                 z11 * u_frac * v_frac)
        else:
            z = depth_map[v_int, u_int]
    else:
        z = depth_value
    
    # Check if depth is valid (now in meters after scaling in Environment3D)
    # Filter out very close depths (< 6 inches = 0.15m) - these are usually noise
    if z < 0.15 or z > 20.0 or not np.isfinite(z):  # At least 15cm (6 inches), max 20m, and finite
        return None
    
    # Backproject using pinhole camera model
    # Standard camera coordinate system:
    # - X: right (positive to the right)
    # - Y: down (positive downward in image, but we want up in 3D, so negate)
    # - Z: forward (depth into scene)
    # 
    # x = (u - cx) * z / fx
    # y = -(v - cy) * z / fy  (negate Y because image v increases downward)
    x = (u - cx) * z / fx
    y = -(v - cy) * z / fy  # Negate Y: image coordinates have Y down, camera coords have Y up
    
    return np.array([x, y, z])


def load_depth_snapshot(snapshot_path: str, apply_depth_scale: bool = True) -> Tuple[np.ndarray, dict]:
    """
    Load depth snapshot from disk.
    
    Args:
        snapshot_path: Base path (loads {snapshot_path}_depth.npy and {snapshot_path}_intrinsics.npy)
        apply_depth_scale: If True, applies depth calibration scale to convert to meters
        
    Returns:
        Tuple of (depth_map, intrinsics_dict)
        depth_map is in meters if apply_depth_scale=True, otherwise raw units
    """
    depth_path = f"{snapshot_path}_depth.npy"
    intrinsics_path = f"{snapshot_path}_intrinsics.npy"
    
    depth_map = np.load(depth_path)
    intrinsics = np.load(intrinsics_path, allow_pickle=True).item()
    
    # Apply depth calibration scale if requested
    if apply_depth_scale:
        try:
            import json
            import os
            calibration_file = "calibration/depth_calibration.json"
            if os.path.exists(calibration_file):
                with open(calibration_file, 'r') as f:
                    calib_data = json.load(f)
                    depth_scale = calib_data['depth_scale']
                    depth_map = np.abs(depth_map) * depth_scale
                    # Filter invalid depths - filter out very close (< 6 inches = 0.15m)
                    depth_map[depth_map < 0.15] = 0  # Filter out very close noise
                    depth_map[depth_map > 20.0] = 0  # Filter out very far depths
            else:
                # No calibration, just use absolute values
                depth_map = np.abs(depth_map)
        except Exception:
            # If calibration fails, just use absolute values
            depth_map = np.abs(depth_map)
    else:
        depth_map = np.abs(depth_map)
    
    return depth_map, intrinsics


if __name__ == "__main__":
    # Test pixel to 3D conversion
    print("Testing Pixel to 3D Conversion")
    print("=" * 50)
    
    # Load snapshot
    snapshot_path = "depth_snapshot"
    try:
        depth_map, intrinsics = load_depth_snapshot(snapshot_path)
        print(f"Loaded depth map: {depth_map.shape}")
        print(f"Intrinsics: {intrinsics}")
    except FileNotFoundError:
        print(f"Snapshot not found at {snapshot_path}")
        print("Run depth_estimator.py first to create snapshot")
        exit(1)
    
    # Test with center pixel
    h, w = depth_map.shape
    center_u = w / 2.0
    center_v = h / 2.0
    
    print(f"\nTesting center pixel: ({center_u:.1f}, {center_v:.1f})")
    point_3d = pixel_to_3d(center_u, center_v, depth_map, intrinsics)
    
    if point_3d is not None:
        print(f"3D point: ({point_3d[0]:.3f}, {point_3d[1]:.3f}, {point_3d[2]:.3f})m")
    else:
        print("Invalid depth at center pixel")
    
    # Test with a few more pixels
    test_pixels = [
        (w * 0.25, h * 0.25, "Top-left"),
        (w * 0.75, h * 0.25, "Top-right"),
        (w * 0.5, h * 0.75, "Bottom-center"),
    ]
    
    print("\nTesting additional pixels:")
    for u, v, desc in test_pixels:
        point_3d = pixel_to_3d(u, v, depth_map, intrinsics)
        if point_3d is not None:
            print(f"  {desc:15s} ({u:.0f}, {v:.0f}) → ({point_3d[0]:.3f}, {point_3d[1]:.3f}, {point_3d[2]:.3f})m")
        else:
            print(f"  {desc:15s} ({u:.0f}, {v:.0f}) → Invalid depth")
