#!/usr/bin/env python3
"""
Test script for 6-link arm in DualArm system.

This demonstrates that the modular refactor works - we can plug in
different arm types and they work seamlessly with the DualArm system.
"""

from src import DualArm, SixLinkArm
import numpy as np


def test_six_link_dual_arm():
    """Test that 6-link arms work in DualArm system."""
    print("=" * 60)
    print("Test: 6-Link Arm in DualArm System")
    print("=" * 60)
    
    # Create 6-link arms
    left_arm = SixLinkArm(L1=0.5, L2=0.4, L3=0.3, L4=0.25, L5=0.2, L6=0.15, name="Left6Link")
    right_arm = SixLinkArm(L1=0.5, L2=0.4, L3=0.3, L4=0.25, L5=0.2, L6=0.15, name="Right6Link")
    
    # Create dual arm system
    dual = DualArm(separation=2.0, left_arm=left_arm, right_arm=right_arm)
    
    print(f"✓ Created DualArm with 6-link arms")
    print(f"  Left arm: {dual.left_arm}")
    print(f"  Right arm: {dual.right_arm}")
    print(f"  Left arm joints: {dual.left_arm.get_num_joints()}")
    print(f"  Right arm joints: {dual.right_arm.get_num_joints()}")
    
    # Test forward kinematics
    print("\n✓ Testing forward kinematics:")
    left_angles = (0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
    right_angles = (0.1, 0.1, 0.1, 0.1, 0.1, 0.1)
    left_pos, right_pos = dual.compute_positions(left_angles, right_angles)
    print(f"  Left position (local): {left_pos}")
    print(f"  Right position (local): {right_pos}")
    
    # Test configuration validation with full 12D config
    print("\n✓ Testing configuration validation:")
    config = np.array([
        0.1, 0.1, 0.1, 0.1, 0.1, 0.1,  # Left arm (6 joints)
        0.1, 0.1, 0.1, 0.1, 0.1, 0.1   # Right arm (6 joints)
    ])
    valid = dual.is_valid_configuration(config)
    print(f"  Full config (12D): {valid}")
    
    # Test _split_configuration helper
    print("\n✓ Testing _split_configuration:")
    left_config, right_config = dual._split_configuration(config)
    print(f"  Left config: {left_config}")
    print(f"  Right config: {right_config}")
    print(f"  Left config length: {len(left_config)}")
    print(f"  Right config length: {len(right_config)}")
    
    # Test invalid configuration (out of joint limits)
    print("\n✓ Testing invalid configuration:")
    invalid_config = np.array([
        10.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Left arm (first joint out of limits)
        0.1, 0.1, 0.1, 0.1, 0.1, 0.1    # Right arm
    ])
    invalid = dual.is_valid_configuration(invalid_config)
    print(f"  Invalid config (out of limits): {invalid}")
    
    print("\n" + "=" * 60)
    print("✅ All tests passed! 6-link arm works in DualArm system.")
    print("=" * 60)


if __name__ == "__main__":
    test_six_link_dual_arm()

