"""
Side Camera Stub Module

Synthetic side camera interface for testing dual-camera pipeline.
Returns synthetic (u2, v2) pixel coordinates matching real camera interface.
"""

import numpy as np
from typing import Optional, Tuple


class SideCameraStub:
    """
    Stub implementation of side camera for testing.
    
    Provides same interface as real camera but returns synthetic data.
    This allows the rest of the pipeline to be built and tested without
    a physical second camera.
    """
    
    def __init__(self, mode: str = "transform", front_camera_width: int = 640, front_camera_height: int = 480):
        """
        Initialize side camera stub.
        
        Args:
            mode: How to generate synthetic data:
                - "transform": Apply transform to front camera (u1, v1)
                - "fixed": Return fixed hardcoded values
                - "random": Return random but stable values
            front_camera_width: Width of front camera (for coordinate scaling)
            front_camera_height: Height of front camera (for coordinate scaling)
        """
        self.mode = mode
        self.front_width = front_camera_width
        self.front_height = front_camera_height
        
        # Fixed values for "fixed" mode
        self.fixed_u2 = front_camera_width * 0.6
        self.fixed_v2 = front_camera_height * 0.5
        
        # Random seed for "random" mode (makes it stable)
        self.rng = np.random.RandomState(42)
        
        # Transform parameters for "transform" mode
        # Simulates side camera viewing from ~90 degrees
        self.transform_scale = 0.8  # Side camera might have different FOV
        self.transform_offset_u = front_camera_width * 0.1
        self.transform_offset_v = front_camera_height * 0.05
    
    def get_hand_pixels(self, front_u: Optional[float] = None, front_v: Optional[float] = None) -> Tuple[float, float]:
        """
        Get synthetic hand pixel coordinates from side camera.
        
        Args:
            front_u: Front camera u coordinate (used in "transform" mode)
            front_v: Front camera v coordinate (used in "transform" mode)
            
        Returns:
            Tuple of (u2, v2) pixel coordinates for side camera
        """
        if self.mode == "fixed":
            return (self.fixed_u2, self.fixed_v2)
        
        elif self.mode == "random":
            # Return random but stable values (same seed = same values)
            u2 = self.rng.uniform(0.2 * self.front_width, 0.8 * self.front_width)
            v2 = self.rng.uniform(0.2 * self.front_height, 0.8 * self.front_height)
            return (u2, v2)
        
        elif self.mode == "transform":
            # Apply transform to front camera coordinates
            # Simulates side camera viewing from different angle
            if front_u is None or front_v is None:
                # Default to center if front camera data not provided
                front_u = self.front_width / 2
                front_v = self.front_height / 2
            
            # Simple transform: scale and offset
            # In reality, this would be a proper stereo calibration
            u2 = front_u * self.transform_scale + self.transform_offset_u
            v2 = front_v * self.transform_scale + self.transform_offset_v
            
            # Clamp to valid pixel range
            u2 = np.clip(u2, 0, self.front_width - 1)
            v2 = np.clip(v2, 0, self.front_height - 1)
            
            return (u2, v2)
        
        else:
            raise ValueError(f"Unknown mode: {self.mode}. Use 'fixed', 'random', or 'transform'")
    
    def set_mode(self, mode: str):
        """
        Change the synthetic data generation mode.
        
        Args:
            mode: 'fixed', 'random', or 'transform'
        """
        if mode not in ['fixed', 'random', 'transform']:
            raise ValueError(f"Invalid mode: {mode}")
        self.mode = mode
    
    def get_camera_info(self) -> dict:
        """
        Get camera information (for compatibility with real camera interface).
        
        Returns:
            Dictionary with camera properties
        """
        return {
            'width': self.front_width,
            'height': self.front_height,
            'mode': self.mode,
            'is_stub': True
        }


if __name__ == "__main__":
    # Test side camera stub
    stub = SideCameraStub(mode="transform")
    
    print("Testing Side Camera Stub")
    print("=" * 50)
    
    # Test with front camera coordinates
    test_cases = [
        (320, 240),  # Center
        (100, 100),  # Top-left
        (540, 380),  # Bottom-right
        (None, None)  # No front camera data
    ]
    
    for front_u, front_v in test_cases:
        u2, v2 = stub.get_hand_pixels(front_u, front_v)
        print(f"Front: ({front_u}, {front_v}) -> Side: ({u2:.1f}, {v2:.1f})")
    
    print("\nTesting different modes:")
    for mode in ['fixed', 'random', 'transform']:
        stub.set_mode(mode)
        u2, v2 = stub.get_hand_pixels(320, 240)
        print(f"{mode:10s}: ({u2:.1f}, {v2:.1f})")
