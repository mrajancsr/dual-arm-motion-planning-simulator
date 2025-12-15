"""
Test script for JointMapper: Verify sim ↔ robot mapping works.

Tests small scripted motions to verify the mapping is correct.
"""

import sys
import os
import time
import json
from types import SimpleNamespace
from typing import Tuple

# Try to import lerobot modules
try:
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS
    from pathlib import Path
    LEROBOT_AVAILABLE = True
except ImportError as e:
    print(f"❌ Could not import lerobot: {e}")
    LEROBOT_AVAILABLE = False
    sys.exit(1)

from joint_mapper import JointMapper

# Configuration
PORT = "/dev/tty.usbmodem5AAF2186901"  # Update if needed

MOTORS = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
}

def load_calibration():
    """Load calibration if available."""
    try:
        calibration_dir = HF_LEROBOT_CALIBRATION / ROBOTS / "so101_follower"
        calibration_fpath = calibration_dir / "follower_arm.json"
        
        if not calibration_fpath.exists():
            return None
        
        with open(calibration_fpath, "r") as f:
            cal_raw = json.load(f)
        
        # Convert dict -> object with attributes
        cal = {}
        for name, cfg in cal_raw.items():
            if name in MOTORS:
                cal[name] = SimpleNamespace(**cfg)
        
        return cal
    except Exception as e:
        print(f"⚠️  Error loading calibration: {e}")
        return None

def setup_robot():
    """Connect to robot and enable torque."""
    print("Connecting to robot...")
    calibration = load_calibration()
    bus = FeetechMotorsBus(port=PORT, motors=MOTORS, calibration=calibration)
    bus.connect(handshake=False)
    
    print("Enabling torque...")
    bus.enable_torque(list(MOTORS.keys()))
    
    # Set speed and acceleration
    for name in MOTORS:
        bus.write("Goal_Time", name, 0)
        bus.write("Goal_Velocity", name, 200)
        bus.write("Acceleration", name, 50)
    
    print("✓ Robot connected and ready!")
    return bus

def wait_until(bus, joint_name: str, target_norm: float, 
               tol: float = 0.1, timeout: float = 3.0) -> Tuple[bool, float]:
    """
    Wait until joint reaches target position within tolerance.
    
    Args:
        bus: FeetechMotorsBus instance
        joint_name: Name of joint to wait for
        target_norm: Target normalized position
        tol: Tolerance in normalized units (default: 0.1)
        timeout: Maximum time to wait in seconds (default: 3.0)
    
    Returns:
        Tuple of (success, final_position)
    """
    t0 = time.time()
    while time.time() - t0 < timeout:
        q = bus.read("Present_Position", joint_name, normalize=True)
        if abs(q - target_norm) <= tol:
            return True, q
        time.sleep(0.05)
    return False, bus.read("Present_Position", joint_name, normalize=True)

def test_single_joint_motion(bus, mapper: JointMapper, joint_name: str, 
                             theta_delta: float, test_name: str):
    """
    Test moving a single joint by a small angle.
    
    Args:
        bus: FeetechMotorsBus instance
        mapper: JointMapper instance
        joint_name: "shoulder_lift" or "elbow_flex"
        theta_delta: Angle change in radians (e.g., 0.2)
        test_name: Description of test
    """
    print(f"\n{'='*60}")
    print(f"Test: {test_name}")
    print(f"{'='*60}")
    
    # Read current robot position
    u_current = bus.read("Present_Position", joint_name, normalize=True)
    print(f"Current {joint_name} position: {u_current:.6f} (normalized)")
    
    # Convert to sim angle
    if joint_name == "shoulder_lift":
        theta1_current, _ = mapper.robot_to_sim(u_current, mapper.u0_elbow)
        theta1_target = theta1_current + theta_delta
        print(f"Current θ1: {theta1_current:.4f} rad")
        print(f"Target θ1: {theta1_target:.4f} rad (delta: {theta_delta:.4f} rad)")
        
        # Convert to robot command
        u_target, _ = mapper.sim_to_robot(theta1_target, 0.0, rate_limit=False)
        print(f"Target {joint_name} position: {u_target:.6f} (normalized)")
        
        # Send command
        print(f"\nMoving {joint_name}...")
        bus.write("Goal_Position", joint_name, u_target, normalize=True)
        
        # Wait for movement
        success, u_final = wait_until(bus, joint_name, u_target, tol=0.5, timeout=3.0)
        
        if success:
            print(f"✓ Reached target! Final position: {u_final:.6f}")
            # Convert back to sim
            theta1_final, _ = mapper.robot_to_sim(u_final, mapper.u0_elbow)
            print(f"  Final θ1: {theta1_final:.4f} rad (target: {theta1_target:.4f} rad)")
            print(f"  Error: {abs(theta1_final - theta1_target):.4f} rad")
        else:
            print(f"⚠️  Timeout. Final position: {u_final:.6f} (target: {u_target:.6f})")
            print(f"  Error: {abs(u_final - u_target):.4f} normalized units")
        
        return success
    
    elif joint_name == "elbow_flex":
        _, theta2_current = mapper.robot_to_sim(mapper.u0_shoulder, u_current)
        theta2_target = theta2_current + theta_delta
        print(f"Current θ2: {theta2_current:.4f} rad")
        print(f"Target θ2: {theta2_target:.4f} rad (delta: {theta_delta:.4f} rad)")
        
        # Convert to robot command
        _, u_target = mapper.sim_to_robot(0.0, theta2_target, rate_limit=False)
        print(f"Target {joint_name} position: {u_target:.6f} (normalized)")
        
        # Send command
        print(f"\nMoving {joint_name}...")
        bus.write("Goal_Position", joint_name, u_target, normalize=True)
        
        # Wait for movement
        success, u_final = wait_until(bus, joint_name, u_target, tol=0.5, timeout=3.0)
        
        if success:
            print(f"✓ Reached target! Final position: {u_final:.6f}")
            # Convert back to sim
            _, theta2_final = mapper.robot_to_sim(mapper.u0_shoulder, u_final)
            print(f"  Final θ2: {theta2_final:.4f} rad (target: {theta2_target:.4f} rad)")
            print(f"  Error: {abs(theta2_final - theta2_target):.4f} rad")
        else:
            print(f"⚠️  Timeout. Final position: {u_final:.6f} (target: {u_target:.6f})")
            print(f"  Error: {abs(u_final - u_target):.4f} normalized units")
        
        return success

def main():
    """Main test function."""
    print("="*60)
    print("Joint Mapper Test")
    print("="*60)
    print("\nThis test verifies the sim ↔ robot mapping works correctly.")
    print("We'll test small scripted motions to verify direction and magnitude.\n")
    
    if not LEROBOT_AVAILABLE:
        print("❌ Lerobot not available. Exiting.")
        return
    
    try:
        # Setup
        bus = setup_robot()
        
        # Read current positions to establish reference
        print("\n" + "="*60)
        print("Establishing Reference (θ=0)")
        print("="*60)
        u0_shoulder = bus.read("Present_Position", "shoulder_lift", normalize=True)
        u0_elbow = bus.read("Present_Position", "elbow_flex", normalize=True)
        print(f"Shoulder (θ1=0): {u0_shoulder:.6f}")
        print(f"Elbow (θ2=0): {u0_elbow:.6f}")
        
        # Note: Calibration limits are in raw ticks (0-4095), but we're using normalized space [-100, 100]
        # So we use normalized limits, not tick limits
        # For RANGE_M100_100, normalized space is roughly [-100, 100]
        min_shoulder = -100.0
        max_shoulder = 100.0
        min_elbow = -100.0
        max_elbow = 100.0
        
        print(f"Using normalized limits: [-100, 100] (RANGE_M100_100 mode)")
        
        # Create mapper with initial gains (will tune)
        mapper = JointMapper(
            u0_shoulder=u0_shoulder,
            u0_elbow=u0_elbow,
            k_shoulder=25.0,  # Initial estimate (seems good from test 1)
            k_elbow=15.0,     # Reduced from 50.0 - elbow seems to have smaller range
            sign_shoulder=1,
            sign_elbow=1,
            min_shoulder=min_shoulder,
            max_shoulder=max_shoulder,
            min_elbow=min_elbow,
            max_elbow=max_elbow
        )
        print(f"\nInitial gains:")
        print(f"  k_shoulder: {mapper.k_shoulder} (normalized units per radian)")
        print(f"  k_elbow: {mapper.k_elbow} (normalized units per radian)")
        
        # Test 1: Move θ1 by +0.2 rad
        print(f"\n{'='*60}")
        print("TEST 1: Move θ1 by +0.2 rad")
        print(f"{'='*60}")
        success1 = test_single_joint_motion(
            bus, mapper, "shoulder_lift", 0.2, 
            "Shoulder forward motion (θ1 +0.2 rad)"
        )
        
        # Return to reference
        print(f"\nReturning to reference...")
        bus.write("Goal_Position", "shoulder_lift", u0_shoulder, normalize=True)
        wait_until(bus, "shoulder_lift", u0_shoulder, tol=0.5, timeout=3.0)
        time.sleep(0.5)
        
        # Test 2: Move θ2 by +0.1 rad (smaller movement to avoid limits)
        print(f"\n{'='*60}")
        print("TEST 2: Move θ2 by +0.1 rad")
        print(f"{'='*60}")
        success2 = test_single_joint_motion(
            bus, mapper, "elbow_flex", 0.1,
            "Elbow forward motion (θ2 +0.1 rad)"
        )
        
        # Return to reference
        print(f"\nReturning to reference...")
        bus.write("Goal_Position", "elbow_flex", u0_elbow, normalize=True)
        wait_until(bus, "elbow_flex", u0_elbow, tol=0.5, timeout=3.0)
        time.sleep(0.5)
        
        # Summary
        print(f"\n{'='*60}")
        print("TEST SUMMARY")
        print(f"{'='*60}")
        print(f"  Test 1 (θ1 +0.2 rad): {'✓ PASS' if success1 else '✗ FAIL'}")
        print(f"  Test 2 (θ2 +0.1 rad): {'✓ PASS' if success2 else '✗ FAIL'}")
        
        if success1 and success2:
            print(f"\n✓ Both tests passed! Mapping appears to work.")
            print(f"  You may need to tune gains (k_shoulder, k_elbow) for better accuracy.")
        else:
            print(f"\n⚠️  Some tests failed. Check:")
            print(f"  - Direction (may need to flip signs)")
            print(f"  - Gains (may need adjustment)")
            print(f"  - Robot limits or constraints")
        
        # Cleanup
        print("\nDisabling torque...")
        for name in MOTORS:
            try:
                bus.write("Torque_Enable", name, 0)
            except:
                pass
        
        time.sleep(0.5)
        bus.disconnect(disable_torque=False)
        print("✓ Disconnected")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        print("Disabling torque and disconnecting...")
        try:
            for name in MOTORS:
                bus.write("Torque_Enable", name, 0)
        except:
            pass
        try:
            bus.disconnect(disable_torque=False)
        except:
            pass
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()

