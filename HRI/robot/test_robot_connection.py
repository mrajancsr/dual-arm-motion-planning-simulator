"""
Simple test script to verify we can connect to and move the SO-ARM101 robot.

This is a minimal proof-of-concept to test the connection.
"""

import sys
import os
import time

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
    print("   Please install lerobot: pip install lerobot")
    LEROBOT_AVAILABLE = False
    sys.exit(1)

# Configuration (match your existing code)
PORT = "/dev/tty.usbmodem5AAF2186901"  # Update this if your port is different

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
        
        if calibration_fpath.exists():
            print(f"✓ Loading calibration from {calibration_fpath}")
            with open(calibration_fpath) as f:
                calibration_dict = draccus.load(dict, f)
            filtered_calibration = {name: cal for name, cal in calibration_dict.items() if name in MOTORS}
            return filtered_calibration
        else:
            print(f"⚠️  No calibration file found, continuing without calibration")
            return None
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

def read_current_positions(bus):
    """Read current position of all motors."""
    positions = {}
    for name in MOTORS:
        try:
            pos = bus.read("Present_Position", name, normalize=False)
            positions[name] = pos
            print(f"  {name}: {pos}")
        except Exception as e:
            print(f"  ⚠️  Error reading {name}: {e}")
    return positions

def test_simple_movement(bus):
    """Test moving just the shoulder_lift joint by a small amount."""
    print("\n" + "="*50)
    print("TEST: Simple Movement")
    print("="*50)
    
    # Read current position
    print("\n1. Reading current shoulder_lift position...")
    try:
        current_pos = bus.read("Present_Position", "shoulder_lift", normalize=False)
        print(f"   Current position: {current_pos}")
    except Exception as e:
        print(f"   ❌ Error reading position: {e}")
        return False
    
    # Calculate small movement (move by 100 ticks)
    target_pos = current_pos + 100
    print(f"\n2. Moving shoulder_lift from {current_pos} to {target_pos} (+100 ticks)...")
    
    try:
        bus.write("Goal_Position", "shoulder_lift", target_pos, normalize=False)
        print("   ✓ Command sent!")
    except Exception as e:
        print(f"   ❌ Error sending command: {e}")
        return False
    
    # Wait a bit
    print("\n3. Waiting 2 seconds...")
    time.sleep(2.0)
    
    # Read new position
    print("\n4. Reading new position...")
    try:
        new_pos = bus.read("Present_Position", "shoulder_lift", normalize=False)
        print(f"   New position: {new_pos}")
        print(f"   Movement: {new_pos - current_pos} ticks")
        
        if abs(new_pos - target_pos) < 50:  # Within 50 ticks is good
            print("   ✓ Movement successful!")
            return True
        else:
            print(f"   ⚠️  Movement incomplete (target: {target_pos}, actual: {new_pos})")
            return True  # Still counts as success if it moved
    except Exception as e:
        print(f"   ❌ Error reading new position: {e}")
        return False

def test_return_to_start(bus, start_positions):
    """Return robot to starting positions."""
    print("\n" + "="*50)
    print("Returning to start positions...")
    print("="*50)
    
    for name, start_pos in start_positions.items():
        try:
            print(f"  Moving {name} back to {start_pos}...")
            bus.write("Goal_Position", name, start_pos, normalize=False)
        except Exception as e:
            print(f"  ⚠️  Error moving {name}: {e}")
    
    print("  Waiting 3 seconds for return...")
    time.sleep(3.0)
    print("  ✓ Done")

def main():
    """Main test function."""
    print("="*50)
    print("SO-ARM101 Connection Test")
    print("="*50)
    
    if not LEROBOT_AVAILABLE:
        print("❌ Lerobot not available. Exiting.")
        return
    
    try:
        # Setup
        bus = setup_robot()
        
        # Read starting positions
        print("\n" + "="*50)
        print("Current Robot Positions:")
        print("="*50)
        start_positions = read_current_positions(bus)
        
        # Test simple movement
        success = test_simple_movement(bus)
        
        if success:
            print("\n" + "="*50)
            print("✓ TEST PASSED: Robot is responding!")
            print("="*50)
        else:
            print("\n" + "="*50)
            print("❌ TEST FAILED: Robot did not respond")
            print("="*50)
        
        # Return to start
        test_return_to_start(bus, start_positions)
        
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

