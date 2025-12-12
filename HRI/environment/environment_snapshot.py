"""
Environment Snapshot Module

Reusable, saveable/loadable 3D environment representation.
Contains depth map, intrinsics, obstacle boxes, and metadata.
"""

import numpy as np
import json
import os
from typing import Optional, Dict, List
from environment.environment_3d import Environment3D


class EnvironmentSnapshot:
    """
    Reusable environment snapshot that can be saved and loaded.
    
    Contains:
    - Depth map (scaled to meters)
    - Camera intrinsics
    - Obstacle boxes (3D)
    - Metadata (calibration info, creation time, etc.)
    """
    
    def __init__(self, environment: Optional[Environment3D] = None, 
                 snapshot_path: Optional[str] = None):
        """
        Initialize snapshot from Environment3D or load from disk.
        
        Args:
            environment: Environment3D instance to snapshot (if None, loads from snapshot_path)
            snapshot_path: Path to load snapshot from (if None, uses environment)
        """
        if snapshot_path is not None:
            self.load(snapshot_path)
        elif environment is not None:
            self.depth_map = environment.depth_map.copy()
            self.intrinsics = environment.intrinsics.copy()
            self.obstacle_boxes = environment.obstacle_boxes.copy()
            self.safety_margin = environment.safety_margin
            self.metadata = {
                'created': None,  # Will be set on save
                'depth_shape': self.depth_map.shape,
                'num_obstacles': len(self.obstacle_boxes)
            }
        else:
            raise ValueError("Must provide either environment or snapshot_path")
    
    def save(self, save_path: str):
        """
        Save snapshot to disk.
        
        Args:
            save_path: Base path for saving (will create {save_path}_snapshot.npz and {save_path}_snapshot_meta.json)
        """
        import time
        
        # Update metadata
        self.metadata['created'] = time.strftime('%Y-%m-%d %H:%M:%S')
        self.metadata['depth_shape'] = self.depth_map.shape
        self.metadata['num_obstacles'] = len(self.obstacle_boxes)
        
        # Save depth map and obstacle boxes as NPZ
        npz_path = f"{save_path}_snapshot.npz"
        np.savez_compressed(
            npz_path,
            depth_map=self.depth_map,
            intrinsics=self.intrinsics,
            obstacle_boxes=self.obstacle_boxes if len(self.obstacle_boxes) > 0 else None
        )
        
        # Save metadata as JSON
        meta_path = f"{save_path}_snapshot_meta.json"
        with open(meta_path, 'w') as f:
            json.dump(self.metadata, f, indent=2)
        
        print(f"Environment snapshot saved:")
        print(f"  Data: {npz_path}")
        print(f"  Metadata: {meta_path}")
        print(f"  Obstacles: {len(self.obstacle_boxes)} boxes")
    
    def load(self, snapshot_path: str):
        """
        Load snapshot from disk.
        
        Args:
            snapshot_path: Base path to load from (loads {snapshot_path}_snapshot.npz and {snapshot_path}_snapshot_meta.json)
        """
        npz_path = f"{snapshot_path}_snapshot.npz"
        meta_path = f"{snapshot_path}_snapshot_meta.json"
        
        if not os.path.exists(npz_path):
            raise FileNotFoundError(f"Snapshot not found: {npz_path}")
        
        # Load NPZ file
        data = np.load(npz_path, allow_pickle=True)
        self.depth_map = data['depth_map']
        self.intrinsics = data['intrinsics'].item()
        
        # Load obstacle boxes (may be None if empty)
        obstacle_boxes = data.get('obstacle_boxes', None)
        if obstacle_boxes is None:
            self.obstacle_boxes = []
        else:
            # Handle numpy array with None value
            if hasattr(obstacle_boxes, 'tolist'):
                try:
                    self.obstacle_boxes = obstacle_boxes.tolist()
                except (AttributeError, TypeError):
                    self.obstacle_boxes = []
            elif isinstance(obstacle_boxes, list):
                self.obstacle_boxes = obstacle_boxes
            else:
                self.obstacle_boxes = []
        
        # Ensure it's always a list, never None
        if self.obstacle_boxes is None:
            self.obstacle_boxes = []
        
        # Load metadata
        if os.path.exists(meta_path):
            with open(meta_path, 'r') as f:
                self.metadata = json.load(f)
        else:
            self.metadata = {}
        
        # Safety margin (default if not in metadata)
        self.safety_margin = self.metadata.get('safety_margin', 0.1)
        
        print(f"Environment snapshot loaded:")
        print(f"  Depth map: {self.depth_map.shape}")
        print(f"  Obstacles: {len(self.obstacle_boxes)} boxes")
        if 'created' in self.metadata:
            print(f"  Created: {self.metadata['created']}")
    
    def to_environment_3d(self) -> Environment3D:
        """
        Convert snapshot back to Environment3D instance.
        
        Returns:
            Environment3D instance with this snapshot's data
        """
        # Depth map is already scaled, so skip calibration
        env = Environment3D(
            depth_map=self.depth_map,
            intrinsics=self.intrinsics,
            safety_margin=self.safety_margin,
            depth_already_scaled=True  # Skip depth calibration - already scaled
        )
        env.obstacle_boxes = self.obstacle_boxes.copy()
        return env
    
    def get_point_cloud(self, downsample: int = 4) -> np.ndarray:
        """
        Get point cloud from snapshot's depth map.
        
        Args:
            downsample: Downsample factor
            
        Returns:
            Point cloud array (N, 3) with [x, y, z] coordinates
        """
        env = self.to_environment_3d()
        return env.get_point_cloud(downsample=downsample)

