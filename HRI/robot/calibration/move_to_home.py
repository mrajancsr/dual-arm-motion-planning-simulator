"""
Move Robot to HOME Pose

Moves the robot to the recorded HOME pose.
Use this on program start/exit.
"""

import json
import os
import time
from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS
from types import SimpleNamespace

PORT = "/dev/tty.usbmodem5AAF2186901"
CALIBRATION_FILE = os.path.join(os.path.dirname(__file__), "home_pose.json")

MOTORS = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
}

def load_calibration():
    """Load motor calibration."""
    try:
        calibration_dir = HF_LEROBOT_CALIBRATION / ROBOTS / "so101_follower"
        calibration_fpath = calibration_dir / "follower_arm.json"
        if not calibration_fpath.exists():
            return None
        with open(calibration_fpath, "r") as f:
            cal_raw = json.load(f)
        cal = {}
        for name, cfg in cal_raw.items():
            if name in MOTORS:
                cal[name] = SimpleNamespace(**cfg)
        return cal
    except Exception as e:
        print(f"⚠️  Error loading calibration: {e}")
        return None

def load_home_pose():
    """Load HOME pose from file."""
    if not os.path.exists(CALIBRATION_FILE):
        raise FileNotFoundError(f"HOME pose not found: {CALIBRATION_FILE}\nRun record_home_pose.py first")
    
    with open(CALIBRATION_FILE, "r") as f:
        data = json.load(f)
    
    # Extract normalized positions (new format)
    if "hardware_home" in data:
        return {
            "shoulder_lift": data["hardware_home"]["shoulder_lift"]["normalized"],
            "elbow_flex": data["hardware_home"]["elbow_flex"]["normalized"]
        }
    else:
        # Legacy format (just direct keys)
        return data

def wait_until(bus, joint_name, target, tol=2.0, timeout=5.0):
    """Wait until joint reaches target. Returns (success, final_position)."""
    t0 = time.time()
    final_pos = None
    while time.time() - t0 < timeout:
        pos = bus.read("Present_Position", joint_name, normalize=True)
        final_pos = pos
        if abs(pos - target) <= tol:
            return True, pos
        time.sleep(0.05)
    # Return final position even on timeout
    if final_pos is None:
        final_pos = bus.read("Present_Position", joint_name, normalize=True)
    return False, final_pos

def move_to_home(bus, verbose=True):
    """Move robot to HOME pose."""
    home_pose = load_home_pose()
    
    if verbose:
        print(f"Moving to HOME pose...")
        print(f"  shoulder_lift: {home_pose['shoulder_lift']:.3f}")
        print(f"  elbow_flex: {home_pose['elbow_flex']:.3f}")
    
    # Read current positions
    current_sh = bus.read("Present_Position", "shoulder_lift", normalize=True)
    current_el = bus.read("Present_Position", "elbow_flex", normalize=True)
    
    if verbose:
        print(f"  Current: shoulder={current_sh:.3f}, elbow={current_el:.3f}")
        print(f"  Distance: shoulder={abs(current_sh - home_pose['shoulder_lift']):.3f}, "
              f"elbow={abs(current_el - home_pose['elbow_flex']):.3f}")
    
    # Send commands
    bus.write("Goal_Position", "shoulder_lift", home_pose["shoulder_lift"], normalize=True)
    bus.write("Goal_Position", "elbow_flex", home_pose["elbow_flex"], normalize=True)
    
    # Wait to reach (longer timeout for larger moves)
    max_distance = max(abs(current_sh - home_pose["shoulder_lift"]), 
                       abs(current_el - home_pose["elbow_flex"]))
    timeout = max(5.0, max_distance / 10.0)  # At least 5s, or more for large moves
    
    if verbose:
        print(f"  Waiting up to {timeout:.1f}s to reach HOME...")
    
    success_sh, final_sh = wait_until(bus, "shoulder_lift", home_pose["shoulder_lift"], tol=2.0, timeout=timeout)
    success_el, final_el = wait_until(bus, "elbow_flex", home_pose["elbow_flex"], tol=2.0, timeout=timeout)
    
    if verbose:
        if success_sh and success_el:
            print(f"✓ Reached HOME pose (shoulder={final_sh:.3f}, elbow={final_el:.3f})")
        else:
            print(f"⚠️  May not have fully reached HOME pose")
            print(f"   Final: shoulder={final_sh:.3f} (target: {home_pose['shoulder_lift']:.3f}, "
                  f"error: {abs(final_sh - home_pose['shoulder_lift']):.3f})")
            print(f"   Final: elbow={final_el:.3f} (target: {home_pose['elbow_flex']:.3f}, "
                  f"error: {abs(final_el - home_pose['elbow_flex']):.3f})")
    
    return success_sh and success_el

def main():
    print("=" * 60)
    print("Move to HOME Pose")
    print("=" * 60)
    
    # Connect
    print("\nConnecting to robot...")
    calibration = load_calibration()
    bus = FeetechMotorsBus(port=PORT, motors=MOTORS, calibration=calibration)
    bus.connect(handshake=False)
    bus.enable_torque(list(MOTORS.keys()))
    
    # Set speed
    for name in MOTORS:
        bus.write("Goal_Time", name, 0)
        bus.write("Goal_Velocity", name, 200)
        bus.write("Acceleration", name, 50)
    
    print("✓ Connected")
    
    # Move to home
    move_to_home(bus)
    
    # Disconnect
    print("\nDisconnecting...")
    for name in MOTORS:
        bus.write("Torque_Enable", name, 0)
    time.sleep(0.5)
    bus.disconnect(disable_torque=False)
    print("✓ Disconnected")

if __name__ == "__main__":
    main()

