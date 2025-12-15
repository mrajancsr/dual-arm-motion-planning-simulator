"""
Record HOME Pose for Robot

Physically place the robot arm in your desired "home" pose,
then run this script to record the joint positions as HOME.

This records:
- Hardware HOME: normalized positions (u_home)
- Simulator HOME: what this pose should be in simulation (theta_home_sim)
- Initial mapping parameters (signs, gains)
"""

import json
import os
from datetime import datetime
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
        import json
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

def validate_position(pos_norm, pos_raw, name):
    """Validate position readings."""
    if pos_norm is None or pos_raw is None:
        raise ValueError(f"{name}: Read None value")
    if not (-100 <= pos_norm <= 100):
        raise ValueError(f"{name}: Normalized position {pos_norm:.3f} outside [-100, 100]")
    if not (0 <= pos_raw <= 4095):
        raise ValueError(f"{name}: Raw position {pos_raw} outside [0, 4095]")
    return True

def main():
    print("=" * 60)
    print("Record HOME Pose")
    print("=" * 60)
    print("\n1. Physically place the robot arm in your desired HOME pose")
    print("   (You can place it manually with torque OFF)")
    print("\n2. What should this pose be in the simulator?")
    print("   Enter theta1 (radians) for shoulder [default: 0.0]: ", end="")
    theta1_input = input().strip()
    theta1_home = float(theta1_input) if theta1_input else 0.0
    
    print("   Enter theta2 (radians) for elbow [default: 0.0]: ", end="")
    theta2_input = input().strip()
    theta2_home = float(theta2_input) if theta2_input else 0.0
    
    print("\n3. Press Enter when the arm is in position...")
    input()
    
    # Connect to robot
    print("\nConnecting to robot...")
    calibration = load_calibration()
    bus = FeetechMotorsBus(port=PORT, motors=MOTORS, calibration=calibration)
    bus.connect(handshake=False)
    
    # Ask if torque should be enabled (for fine positioning)
    print("\nEnable torque? (y/n) [default: n]: ", end="")
    enable_torque = input().strip().lower() == 'y'
    
    if enable_torque:
        bus.enable_torque(list(MOTORS.keys()))
        # Set low speed/accel to avoid snapping
        for name in MOTORS:
            bus.write("Goal_Time", name, 0)
            bus.write("Goal_Velocity", name, 100)  # Low speed
            bus.write("Acceleration", name, 30)     # Low acceleration
        print("✓ Torque enabled (low speed)")
    else:
        print("✓ Torque OFF (reading positions only)")
    
    # Read current positions (both normalized and raw)
    print("\nReading current joint positions...")
    home_data = {
        "timestamp": datetime.now().isoformat(),
        "hardware_home": {},
        "simulator_home": {
            "theta1": theta1_home,
            "theta2": theta2_home
        },
        "mapping_parameters": {
            "sign_shoulder": 1,  # Placeholder - adjust in test_mapping.py
            "sign_elbow": 1,     # Placeholder - adjust in test_mapping.py
            "k_shoulder": 25.0,  # Placeholder - adjust in test_mapping.py
            "k_elbow": 15.0      # Placeholder - adjust in test_mapping.py
        }
    }
    
    for name in ["shoulder_lift", "elbow_flex"]:
        pos_norm = bus.read("Present_Position", name, normalize=True)
        pos_raw = bus.read("Present_Position", name, normalize=False)
        
        # Validate
        validate_position(pos_norm, pos_raw, name)
        
        home_data["hardware_home"][name] = {
            "normalized": float(pos_norm),
            "raw_ticks": int(pos_raw)
        }
        print(f"  {name}: normalized={pos_norm:.6f}, raw={pos_raw}")
    
    # Save to file
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(home_data, f, indent=2)
    
    print(f"\n✓ HOME pose saved to: {CALIBRATION_FILE}")
    print(f"  Hardware HOME: u_sh={home_data['hardware_home']['shoulder_lift']['normalized']:.3f}, "
          f"u_el={home_data['hardware_home']['elbow_flex']['normalized']:.3f}")
    print(f"  Simulator HOME: θ1={theta1_home:.4f}, θ2={theta2_home:.4f}")
    print(f"\n  Next step: Run test_mapping.py to verify/calibrate signs and gains")
    
    # Disconnect
    if enable_torque:
        for name in MOTORS:
            bus.write("Torque_Enable", name, 0)
    bus.disconnect(disable_torque=False)
    print("✓ Disconnected")

if __name__ == "__main__":
    main()

