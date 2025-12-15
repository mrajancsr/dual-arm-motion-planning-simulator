import time
import numpy as np
import json
from types import SimpleNamespace

from lerobot.motors.feetech import FeetechMotorsBus
from lerobot.motors import Motor, MotorNormMode
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, ROBOTS

from joint_mapper import JointMapper
from path_executor import PathExecutor

PORT = "/dev/tty.usbmodem5AAF2186901"

MOTORS = {
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
}

def load_calibration():
    """Load calibration and convert to SimpleNamespace objects."""
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

def main():
    bus = setup_robot()
    
    try:
        # Read current positions to establish reference (θ=0)
        print("\nEstablishing reference (θ=0)...")
        u0_shoulder = bus.read("Present_Position", "shoulder_lift", normalize=True)
        u0_elbow = bus.read("Present_Position", "elbow_flex", normalize=True)
        print(f"  Shoulder (θ1=0): {u0_shoulder:.6f}")
        print(f"  Elbow (θ2=0): {u0_elbow:.6f}")
        
        # Create mapper with current position as reference
        mapper = JointMapper(
            u0_shoulder=u0_shoulder,
            u0_elbow=u0_elbow,
            k_shoulder=25.0,
            k_elbow=15.0,
            sign_shoulder=1,
            sign_elbow=1
        )
        print(f"  Gains: k_shoulder={mapper.k_shoulder}, k_elbow={mapper.k_elbow}")
        
        # Create executor
        executor = PathExecutor(
            bus=bus,
            mapper=mapper,
            control_rate=30.0,
            max_delta_per_step=1.0,
            goal_tolerance=2.0,
            stall_timeout=1.0
        )
        
        # Simple test path: go out and back
        print("\n" + "="*60)
        print("Test Path: Go out and back")
        print("="*60)
        path = [
            np.array([0.00, 0.00, 0.0, 0.0]),  # Start (θ=0)
            np.array([0.15, 0.08, 0.0, 0.0]),  # Out
            np.array([0.00, 0.00, 0.0, 0.0]),  # Back
        ]
        print(f"Path: {len(path)} waypoints")
        for i, wp in enumerate(path):
            print(f"  [{i}] θ1={wp[0]:.3f}, θ2={wp[1]:.3f}")
        
        # Execute path
        print("\nExecuting path...")
        result = executor.execute_path(
            path,
            joint_names=("shoulder_lift", "elbow_flex"),
            verbose=True
        )
        
        # Summary
        print("\n" + "="*60)
        print("Execution Summary")
        print("="*60)
        print(f"  Success: {result['success']}")
        print(f"  Waypoints: {result['waypoints_executed']}/{result['total_waypoints']}")
        print(f"  Time: {result['execution_time']:.2f}s")
        print(f"  Max error (shoulder): {result['max_error_shoulder']:.3f}")
        print(f"  Max error (elbow): {result['max_error_elbow']:.3f}")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
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

if __name__ == "__main__":
    main()

