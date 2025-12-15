"""
Test Sim ↔ Robot Mapping Around HOME

Makes small moves around HOME to verify sign and scale.
"""

import json
import os
import time
import numpy as np
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
    """Load HOME pose and calibration data."""
    if not os.path.exists(CALIBRATION_FILE):
        raise FileNotFoundError(f"HOME pose not found: {CALIBRATION_FILE}")
    with open(CALIBRATION_FILE, "r") as f:
        data = json.load(f)
    
    # Extract data (new format)
    if "hardware_home" in data:
        home_pose = {
            "shoulder_lift": data["hardware_home"]["shoulder_lift"]["normalized"],
            "elbow_flex": data["hardware_home"]["elbow_flex"]["normalized"]
        }
        sim_home = data.get("simulator_home", {"theta1": 0.0, "theta2": 0.0})
        mapping_params = data.get("mapping_parameters", {})
        return home_pose, sim_home, mapping_params
    else:
        # Legacy format
        return data, {"theta1": 0.0, "theta2": 0.0}, {}

def wait_until(bus, joint_name, target, tol=2.0, timeout=5.0):
    """Wait until joint reaches target position."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        pos = bus.read("Present_Position", joint_name, normalize=True)
        if abs(pos - target) <= tol:
            return True, pos
        time.sleep(0.05)
    final_pos = bus.read("Present_Position", joint_name, normalize=True)
    return False, final_pos

def test_mapping(bus, home_pose, theta_delta=0.05, k_shoulder=25.0, k_elbow=15.0, 
                 sign_shoulder=1, sign_elbow=1):
    """
    Test mapping by making small moves around HOME.
    
    Args:
        bus: Connected bus
        home_pose: Dict with 'shoulder_lift' and 'elbow_flex'
        theta_delta: Small angle delta in radians to test
        k_shoulder, k_elbow: Gains (normalized units per radian)
        sign_shoulder, sign_elbow: Direction signs (+1 or -1)
    """
    print("\n" + "=" * 60)
    print("Testing Mapping Around HOME")
    print("=" * 60)
    print(f"\nHOME pose: u_sh={home_pose['shoulder_lift']:.3f}, u_el={home_pose['elbow_flex']:.3f}")
    print(f"Mapping: θ_sim = sign * (u - u_home) / k")
    print(f"  Shoulder: sign={sign_shoulder}, k={k_shoulder}")
    print(f"  Elbow: sign={sign_elbow}, k={k_elbow}")
    
    # Test shoulder: +theta_delta
    print(f"\n[Test 1] Shoulder: θ_sim = +{theta_delta:.3f} rad")
    u_sh_target = home_pose['shoulder_lift'] + sign_shoulder * theta_delta * k_shoulder
    bus.write("Goal_Position", "shoulder_lift", u_sh_target, normalize=True)
    reached, u_sh_actual = wait_until(bus, "shoulder_lift", u_sh_target, tol=2.0, timeout=3.0)
    if not reached:
        print(f"  ⚠️  Did not reach target (within 2.0 units) in 3s")
    delta_u = u_sh_actual - home_pose['shoulder_lift']
    theta_actual = sign_shoulder * delta_u / k_shoulder
    print(f"  Commanded: u={u_sh_target:.3f}, Actual: u={u_sh_actual:.3f}")
    print(f"  Δu={delta_u:.3f}, θ_actual={theta_actual:.4f} rad")
    print(f"  Direction: {'✓ Correct' if np.sign(delta_u) == sign_shoulder else '✗ Wrong sign'}")
    
    # Return to home
    bus.write("Goal_Position", "shoulder_lift", home_pose['shoulder_lift'], normalize=True)
    reached_home, _ = wait_until(bus, "shoulder_lift", home_pose['shoulder_lift'], tol=2.0, timeout=3.0)
    if not reached_home:
        print(f"  ⚠️  Did not return to HOME")
    
    # Test shoulder: -theta_delta
    print(f"\n[Test 2] Shoulder: θ_sim = -{theta_delta:.3f} rad")
    u_sh_target = home_pose['shoulder_lift'] - sign_shoulder * theta_delta * k_shoulder
    bus.write("Goal_Position", "shoulder_lift", u_sh_target, normalize=True)
    reached, u_sh_actual = wait_until(bus, "shoulder_lift", u_sh_target, tol=2.0, timeout=3.0)
    if not reached:
        print(f"  ⚠️  Did not reach target (within 2.0 units) in 3s")
    delta_u = u_sh_actual - home_pose['shoulder_lift']
    theta_actual = -sign_shoulder * delta_u / k_shoulder
    print(f"  Commanded: u={u_sh_target:.3f}, Actual: u={u_sh_actual:.3f}")
    print(f"  Δu={delta_u:.3f}, θ_actual={theta_actual:.4f} rad")
    print(f"  Direction: {'✓ Correct' if np.sign(delta_u) == -sign_shoulder else '✗ Wrong sign'}")
    
    # Return to home
    bus.write("Goal_Position", "shoulder_lift", home_pose['shoulder_lift'], normalize=True)
    reached_home, _ = wait_until(bus, "shoulder_lift", home_pose['shoulder_lift'], tol=2.0, timeout=3.0)
    if not reached_home:
        print(f"  ⚠️  Did not return to HOME")
    
    # Test elbow: +theta_delta
    print(f"\n[Test 3] Elbow: θ_sim = +{theta_delta:.3f} rad")
    u_el_target = home_pose['elbow_flex'] + sign_elbow * theta_delta * k_elbow
    bus.write("Goal_Position", "elbow_flex", u_el_target, normalize=True)
    reached, u_el_actual = wait_until(bus, "elbow_flex", u_el_target, tol=2.0, timeout=3.0)
    if not reached:
        print(f"  ⚠️  Did not reach target (within 2.0 units) in 3s")
    delta_u = u_el_actual - home_pose['elbow_flex']
    theta_actual = sign_elbow * delta_u / k_elbow
    print(f"  Commanded: u={u_el_target:.3f}, Actual: u={u_el_actual:.3f}")
    print(f"  Δu={delta_u:.3f}, θ_actual={theta_actual:.4f} rad")
    print(f"  Direction: {'✓ Correct' if np.sign(delta_u) == sign_elbow else '✗ Wrong sign'}")
    
    # Return to home
    bus.write("Goal_Position", "elbow_flex", home_pose['elbow_flex'], normalize=True)
    reached_home, _ = wait_until(bus, "elbow_flex", home_pose['elbow_flex'], tol=2.0, timeout=3.0)
    if not reached_home:
        print(f"  ⚠️  Did not return to HOME")
    
    # Test elbow: -theta_delta
    print(f"\n[Test 4] Elbow: θ_sim = -{theta_delta:.3f} rad")
    u_el_target = home_pose['elbow_flex'] - sign_elbow * theta_delta * k_elbow
    bus.write("Goal_Position", "elbow_flex", u_el_target, normalize=True)
    reached, u_el_actual = wait_until(bus, "elbow_flex", u_el_target, tol=2.0, timeout=3.0)
    if not reached:
        print(f"  ⚠️  Did not reach target (within 2.0 units) in 3s")
    delta_u = u_el_actual - home_pose['elbow_flex']
    theta_actual = -sign_elbow * delta_u / k_elbow
    print(f"  Commanded: u={u_el_target:.3f}, Actual: u={u_el_actual:.3f}")
    print(f"  Δu={delta_u:.3f}, θ_actual={theta_actual:.4f} rad")
    print(f"  Direction: {'✓ Correct' if np.sign(delta_u) == -sign_elbow else '✗ Wrong sign'}")
    
    # Return to home
    bus.write("Goal_Position", "elbow_flex", home_pose['elbow_flex'], normalize=True)
    reached_home, _ = wait_until(bus, "elbow_flex", home_pose['elbow_flex'], tol=2.0, timeout=3.0)
    if not reached_home:
        print(f"  ⚠️  Did not return to HOME")
    
    print("\n" + "=" * 60)
    print("Test Complete")
    print("=" * 60)
    print("\nIf directions are wrong, flip the signs:")
    print("  sign_shoulder = -1 if Test 1/2 show wrong direction")
    print("  sign_elbow = -1 if Test 3/4 show wrong direction")
    print("\nIf scale is off, adjust k_shoulder and k_elbow")

def main():
    print("=" * 60)
    print("Test Sim ↔ Robot Mapping")
    print("=" * 60)
    
    # Load home pose and calibration data
    home_pose, sim_home, mapping_params = load_home_pose()
    
    print(f"\nSimulator HOME: θ1={sim_home.get('theta1', 0.0):.4f}, θ2={sim_home.get('theta2', 0.0):.4f}")
    
    # Connect
    print("\nConnecting to robot...")
    calibration = load_calibration()
    bus = FeetechMotorsBus(port=PORT, motors=MOTORS, calibration=calibration)
    bus.connect(handshake=False)
    bus.enable_torque(list(MOTORS.keys()))
    
    for name in MOTORS:
        bus.write("Goal_Time", name, 0)
        bus.write("Goal_Velocity", name, 200)
        bus.write("Acceleration", name, 50)
    
    print("✓ Connected")
    
    # Move to home first
    print("\nMoving to HOME pose...")
    bus.write("Goal_Position", "shoulder_lift", home_pose["shoulder_lift"], normalize=True)
    bus.write("Goal_Position", "elbow_flex", home_pose["elbow_flex"], normalize=True)
    time.sleep(2.0)
    
    # Test with parameters from file (or defaults)
    test_mapping(
        bus, home_pose,
        theta_delta=0.05,
        k_shoulder=mapping_params.get("k_shoulder", 25.0),
        k_elbow=mapping_params.get("k_elbow", 15.0),
        sign_shoulder=mapping_params.get("sign_shoulder", 1),
        sign_elbow=mapping_params.get("sign_elbow", 1)
    )
    
    # Disconnect
    print("\nDisconnecting...")
    for name in MOTORS:
        bus.write("Torque_Enable", name, 0)
    time.sleep(0.5)
    bus.disconnect(disable_torque=False)
    print("✓ Disconnected")

if __name__ == "__main__":
    main()

