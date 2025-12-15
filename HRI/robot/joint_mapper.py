"""
Joint Mapper: Maps between simulation angles (θ1, θ2) and robot normalized positions.

Provides bidirectional mapping:
- Hardware → Sim: Read robot position, convert to θ1, θ2 (outputs θ_sim)
- Sim → Hardware: Convert θ1, θ2 to robot commands (takes θ_sim as input)

Note: The mapper works in θ_sim space (zero-centered). The θ_home offset is
applied separately at the FK/visualization boundary.
"""

import numpy as np
import json
import os
from typing import Tuple, Optional


class JointMapper:
    """
    Maps between 2-link planar arm simulation angles and SO-ARM101 normalized positions.
    
    Uses normalized space [-100, 100] (MotorNormMode.RANGE_M100_100).
    Current robot position is treated as θ=0 reference.
    """
    
    def __init__(self,
                 u0_shoulder: float,
                 u0_elbow: float,
                 k_shoulder: float = 25.0,
                 k_elbow: float = 50.0,
                 sign_shoulder: int = 1,
                 sign_elbow: int = 1,
                 min_shoulder: Optional[float] = None,
                 max_shoulder: Optional[float] = None,
                 min_elbow: Optional[float] = None,
                 max_elbow: Optional[float] = None,
                 max_delta_per_step: float = 5.0):
        """
        Initialize joint mapper.
        
        Args:
            u0_shoulder: Current normalized shoulder position (θ1=0 reference)
            u0_elbow: Current normalized elbow position (θ2=0 reference)
            k_shoulder: Normalized units per radian for shoulder (default: 25.0)
            k_elbow: Normalized units per radian for elbow (default: 50.0)
            sign_shoulder: Direction sign for shoulder (+1 or -1, default: +1)
            sign_elbow: Direction sign for elbow (+1 or -1, default: +1)
            min_shoulder: Minimum safe normalized position for shoulder (None = no limit)
            max_shoulder: Maximum safe normalized position for shoulder (None = no limit)
            min_elbow: Minimum safe normalized position for elbow (None = no limit)
            max_elbow: Maximum safe normalized position for elbow (None = no limit)
            max_delta_per_step: Maximum change per step in normalized units (rate limiting)
        """
        # Reference positions (θ=0)
        self.u0_shoulder = u0_shoulder
        self.u0_elbow = u0_elbow
        
        # Per-joint gains (normalized units per radian)
        self.k_shoulder = k_shoulder
        self.k_elbow = k_elbow
        
        # Direction signs
        self.sign_shoulder = sign_shoulder
        self.sign_elbow = sign_elbow
        
        # Limits (clamp to safe ranges)
        self.min_shoulder = min_shoulder if min_shoulder is not None else -100.0
        self.max_shoulder = max_shoulder if max_shoulder is not None else 100.0
        self.min_elbow = min_elbow if min_elbow is not None else -100.0
        self.max_elbow = max_elbow if max_elbow is not None else 100.0
        
        # Rate limiting
        self.max_delta_per_step = max_delta_per_step
        self._last_u_shoulder = u0_shoulder
        self._last_u_elbow = u0_elbow
    
    def sim_to_robot(self, theta1: float, theta2: float, 
                    rate_limit: bool = True) -> Tuple[float, float]:
        """
        Convert simulation angles to robot normalized positions.
        
        Args:
            theta1: Shoulder angle in radians
            theta2: Elbow angle in radians
            rate_limit: Whether to rate-limit the delta (default: True)
        
        Returns:
            Tuple of (u_shoulder, u_elbow) in normalized space
        """
        # Compute target positions
        u_shoulder_target = self.u0_shoulder + self.sign_shoulder * self.k_shoulder * theta1
        u_elbow_target = self.u0_elbow + self.sign_elbow * self.k_elbow * theta2
        
        # Rate limit (don't jump too far in one step)
        if rate_limit:
            delta_shoulder = u_shoulder_target - self._last_u_shoulder
            delta_elbow = u_elbow_target - self._last_u_elbow
            
            delta_shoulder = np.clip(delta_shoulder, -self.max_delta_per_step, self.max_delta_per_step)
            delta_elbow = np.clip(delta_elbow, -self.max_delta_per_step, self.max_delta_per_step)
            
            u_shoulder = self._last_u_shoulder + delta_shoulder
            u_elbow = self._last_u_elbow + delta_elbow
        else:
            u_shoulder = u_shoulder_target
            u_elbow = u_elbow_target
        
        # Clamp to limits
        u_shoulder = np.clip(u_shoulder, self.min_shoulder, self.max_shoulder)
        u_elbow = np.clip(u_elbow, self.min_elbow, self.max_elbow)
        
        # Update last positions for next rate-limited step
        self._last_u_shoulder = u_shoulder
        self._last_u_elbow = u_elbow
        
        return float(u_shoulder), float(u_elbow)
    
    def robot_to_sim(self, u_shoulder: float, u_elbow: float) -> Tuple[float, float]:
        """
        Convert robot normalized positions to simulation angles.
        
        Args:
            u_shoulder: Shoulder normalized position
            u_elbow: Elbow normalized position
        
        Returns:
            Tuple of (theta1, theta2) in radians
        """
        # Compute angles relative to reference
        theta1 = (u_shoulder - self.u0_shoulder) / (self.sign_shoulder * self.k_shoulder)
        theta2 = (u_elbow - self.u0_elbow) / (self.sign_elbow * self.k_elbow)
        
        return float(theta1), float(theta2)
    
    def update_reference(self, u_shoulder: float, u_elbow: float):
        """
        Update the reference position (θ=0 point).
        
        Useful when you want to reset the reference to current robot position.
        
        Args:
            u_shoulder: New reference shoulder position
            u_elbow: New reference elbow position
        """
        self.u0_shoulder = u_shoulder
        self.u0_elbow = u_elbow
        self._last_u_shoulder = u_shoulder
        self._last_u_elbow = u_elbow
    
    def set_gains(self, k_shoulder: Optional[float] = None, k_elbow: Optional[float] = None):
        """
        Update the per-joint gains (for calibration).
        
        Args:
            k_shoulder: New gain for shoulder (None = keep current)
            k_elbow: New gain for elbow (None = keep current)
        """
        if k_shoulder is not None:
            self.k_shoulder = k_shoulder
        if k_elbow is not None:
            self.k_elbow = k_elbow
    
    def set_signs(self, sign_shoulder: Optional[int] = None, sign_elbow: Optional[int] = None):
        """
        Update the direction signs (for calibration).
        
        Args:
            sign_shoulder: New sign for shoulder (+1 or -1, None = keep current)
            sign_elbow: New sign for elbow (+1 or -1, None = keep current)
        """
        if sign_shoulder is not None:
            self.sign_shoulder = sign_shoulder
        if sign_elbow is not None:
            self.sign_elbow = sign_elbow
    
    @classmethod
    def from_calibration_file(cls, calibration_file: Optional[str] = None) -> 'JointMapper':
        """
        Create JointMapper from calibration file.
        
        Args:
            calibration_file: Path to home_pose.json (default: robot/calibration/home_pose.json)
            
        Returns:
            JointMapper instance configured from calibration
        """
        if calibration_file is None:
            # Default path
            calib_dir = os.path.join(os.path.dirname(__file__), 'calibration')
            calibration_file = os.path.join(calib_dir, 'home_pose.json')
        
        if not os.path.exists(calibration_file):
            raise FileNotFoundError(
                f"Calibration file not found: {calibration_file}\n"
                f"Run robot/calibration/record_home_pose.py first"
            )
        
        with open(calibration_file, "r") as f:
            data = json.load(f)
        
        # Extract hardware HOME
        hardware_home = data["hardware_home"]
        u0_shoulder = hardware_home["shoulder_lift"]["normalized"]
        u0_elbow = hardware_home["elbow_flex"]["normalized"]
        
        # Extract mapping parameters
        mapping_params = data.get("mapping_parameters", {})
        k_shoulder = mapping_params.get("k_shoulder", 25.0)
        k_elbow = mapping_params.get("k_elbow", 15.0)
        sign_shoulder = mapping_params.get("sign_shoulder", 1)
        sign_elbow = mapping_params.get("sign_elbow", 1)
        
        return cls(
            u0_shoulder=u0_shoulder,
            u0_elbow=u0_elbow,
            k_shoulder=k_shoulder,
            k_elbow=k_elbow,
            sign_shoulder=sign_shoulder,
            sign_elbow=sign_elbow
        )

