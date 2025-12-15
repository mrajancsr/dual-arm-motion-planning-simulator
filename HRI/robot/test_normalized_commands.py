"""
Test if normalized commands work with FeetechMotorsBus.

This is the gate test: if normalized works, use it. If not, fall back to ticks with safety.
"""

import sys
import os
import time
import json
from types import SimpleNamespace

# Try to import lerobot modules
try:
    from lerobot.motors.feetech import FeetechMotorsBus
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS
    from pathlib import Path
    import draccus
    LEROBOT_AVAILABLE = True
except ImportError as e:
    print(f"❌ Could not import lerobot: {e}")
    LEROBOT_AVAILABLE = False
    sys.exit(1)

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
            print(f"⚠️  No calibration file found")
            return None
        
        print(f"✓ Loading calibration from {calibration_fpath}")
        with open(calibration_fpath, "r") as f:
            cal_raw = json.load(f)
        
        # Debug: print keys of one calibration entry
        if cal_raw and "shoulder_lift" in cal_raw:
            print(f"  Calibration keys for shoulder_lift: {list(cal_raw['shoulder_lift'].keys())}")
        
        # Convert dict -> object with attributes (SimpleNamespace)
        # Keep only joints we use
        cal = {}
        for name, cfg in cal_raw.items():
            if name in MOTORS:
                cal[name] = SimpleNamespace(**cfg)
        
        print(f"  Loaded calibration for: {list(cal.keys())}")
        return cal
        
    except Exception as e:
        print(f"⚠️  Error loading calibration: {e}")
        import traceback
        traceback.print_exc()
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

def test_normalized_command(bus, joint_name, delta=0.02):
    """
    Test if normalized commands work for a single joint.
    
    Args:
        bus: FeetechMotorsBus instance
        joint_name: Name of joint to test (e.g., "shoulder_lift")
        delta: Small delta to move (in normalized units)
    
    Returns:
        True if normalized commands work, False otherwise
    """
    print(f"\n{'='*60}")
    print(f"Testing normalized commands for: {joint_name}")
    print(f"{'='*60}")
    
    try:
        # Read initial position (both normalized and raw)
        print(f"\n1. Reading initial position...")
        q0_norm = bus.read("Present_Position", joint_name, normalize=True)
        q0_raw = bus.read("Present_Position", joint_name, normalize=False)
        print(f"   Normalized: {q0_norm}")
        print(f"   Raw ticks:  {q0_raw}")
        
        # Check if normalized value is reasonable (should be float, often in [0,1] or [-1,1])
        if not isinstance(q0_norm, (int, float)):
            print(f"   ❌ Normalized value is not a number: {type(q0_norm)}")
            return False
        
        # Move a meaningful amount in normalized space
        # Note: RANGE_M100_100 means normalized space is [-100, 100], not [-1, 1]
        target_norm = q0_norm + delta
        print(f"\n2. Moving {joint_name} by {delta} in normalized space...")
        print(f"   Target normalized: {target_norm}")
        
        bus.write("Goal_Position", joint_name, target_norm, normalize=True)
        print(f"   ✓ Command sent (normalize=True)")
        
        # Verify goal register was actually written
        g_norm = bus.read("Goal_Position", joint_name, normalize=True)
        g_raw = bus.read("Goal_Position", joint_name, normalize=False)
        print(f"   Goal register: norm={g_norm:.6f}, raw={g_raw}")
        
        if abs(g_norm - target_norm) > 0.1:
            print(f"   ⚠️  Warning: Goal register doesn't match target (expected {target_norm:.6f}, got {g_norm:.6f})")
        
        # Wait for movement
        print(f"\n3. Waiting 0.5 seconds...")
        time.sleep(0.5)
        
        # Read back position
        print(f"\n4. Reading new position...")
        q1_norm = bus.read("Present_Position", joint_name, normalize=True)
        q1_raw = bus.read("Present_Position", joint_name, normalize=False)
        print(f"   Normalized: {q1_norm} (was {q0_norm}, delta: {q1_norm - q0_norm:.6f})")
        print(f"   Raw ticks:  {q1_raw} (was {q0_raw}, delta: {q1_raw - q0_raw})")
        
        # Check if it moved
        norm_delta = abs(q1_norm - q0_norm)
        raw_delta = abs(q1_raw - q0_raw)
        
        if norm_delta < 0.001:
            print(f"   ❌ Normalized value didn't change (delta: {norm_delta:.6f})")
            print(f"   ⚠️  Normalized commands may not be working")
            return False
        
        if raw_delta < 1:
            print(f"   ❌ Raw ticks didn't change (delta: {raw_delta})")
            print(f"   ⚠️  Robot may not have moved")
            return False
        
        # Check direction (should move in same direction as delta)
        if delta > 0 and q1_norm < q0_norm:
            print(f"   ⚠️  Warning: Moved in opposite direction (expected increase)")
        elif delta < 0 and q1_norm > q0_norm:
            print(f"   ⚠️  Warning: Moved in opposite direction (expected decrease)")
        
        print(f"   ✓ Robot moved! Normalized delta: {norm_delta:.6f}, Raw delta: {raw_delta}")
        
        # Return to original position
        print(f"\n5. Returning to original position...")
        bus.write("Goal_Position", joint_name, q0_norm, normalize=True)
        time.sleep(0.5)
        
        q2_norm = bus.read("Present_Position", joint_name, normalize=True)
        print(f"   Final normalized: {q2_norm} (target: {q0_norm}, error: {abs(q2_norm - q0_norm):.6f})")
        
        if abs(q2_norm - q0_norm) < 0.01:
            print(f"   ✓ Returned to original position")
        else:
            print(f"   ⚠️  Did not fully return (error: {abs(q2_norm - q0_norm):.6f})")
        
        return True
        
    except Exception as e:
        print(f"   ❌ Error during test: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main test function."""
    print("="*60)
    print("Normalized Commands Test")
    print("="*60)
    print("\nThis test checks if normalized commands work with FeetechMotorsBus.")
    print("If they work, we'll use normalized space (cleaner).")
    print("If not, we'll fall back to ticks with proper safety.\n")
    
    if not LEROBOT_AVAILABLE:
        print("❌ Lerobot not available. Exiting.")
        return
    
    try:
        # Setup
        bus = setup_robot()
        
        # Test both joints we care about
        # Note: RANGE_M100_100 means normalized space is [-100, 100]
        # So delta=5.0 is a meaningful movement (5% of range)
        joints_to_test = ["shoulder_lift", "elbow_flex"]
        results = {}
        
        for joint in joints_to_test:
            results[joint] = test_normalized_command(bus, joint, delta=5.0)
        
        # Summary
        print(f"\n{'='*60}")
        print("TEST SUMMARY")
        print(f"{'='*60}")
        
        all_passed = True
        for joint, passed in results.items():
            status = "✓ PASS" if passed else "❌ FAIL"
            print(f"  {joint}: {status}")
            if not passed:
                all_passed = False
        
        print(f"\n{'='*60}")
        if all_passed:
            print("✓ SUCCESS: Normalized commands work!")
            print("  → Use normalized space for control")
            print("  → You get calibration, limits, and clean mapping 'for free'")
        else:
            print("❌ FAILED: Normalized commands don't work")
            print("  → Fall back to ticks with proper safety:")
            print("    - Per-joint sign/scale/zero")
            print("    - Measured safe min/max (not guessed)")
            print("    - Delta rate limiting")
        print(f"{'='*60}")
        
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

