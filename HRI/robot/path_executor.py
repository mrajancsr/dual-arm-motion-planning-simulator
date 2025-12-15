"""
Path Executor: Executes planned paths on the real robot.

Converts simulation waypoints to robot commands and executes them safely
with rate limiting, error checking, and stall detection.
"""

import time
import numpy as np
from typing import List, Tuple, Optional, Dict
from joint_mapper import JointMapper


class PathExecutor:
    """
    Executes planned paths on the SO-ARM101 robot.
    
    Handles:
    - Converting sim waypoints to robot commands
    - Rate limiting (smooth motion)
    - Error checking and stall detection
    - Logging execution data
    """
    
    def __init__(self,
                 bus,
                 mapper: JointMapper,
                 control_rate: float = 30.0,
                 max_delta_per_step: float = 1.0,
                 goal_tolerance: float = 2.0,
                 stall_timeout: float = 1.0):
        """
        Initialize path executor.
        
        Args:
            bus: FeetechMotorsBus instance (connected and torque enabled)
            mapper: JointMapper instance (configured with reference and gains)
            control_rate: Control loop frequency in Hz (default: 30.0)
            max_delta_per_step: Maximum change per step in normalized units (default: 1.0)
            goal_tolerance: Goal tolerance in normalized units (default: 2.0)
            stall_timeout: Time to wait before aborting on stall in seconds (default: 1.0)
        """
        self.bus = bus
        self.mapper = mapper
        self.control_rate = control_rate
        self.dt = 1.0 / control_rate
        self.max_delta_per_step = max_delta_per_step
        self.goal_tolerance = goal_tolerance
        self.stall_timeout = stall_timeout
        
        # Execution state
        self.is_running = False
        self.execution_log = []
    
    def resample_path(self, path: List[np.ndarray], dt: Optional[float] = None) -> List[np.ndarray]:
        """
        Resample path to fixed timestep for smooth execution.
        
        Args:
            path: List of waypoints [θ1, θ2, 0, 0]
            dt: Timestep in seconds (default: self.dt)
        
        Returns:
            Resampled path with waypoints at fixed intervals
        """
        if dt is None:
            dt = self.dt
        
        if len(path) < 2:
            return path
        
        resampled = [path[0]]  # Start with first waypoint
        
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i + 1]
            
            # Extract angles
            theta1_start = start[0]
            theta2_start = start[1]
            theta1_end = end[0]
            theta2_end = end[1]
            
            # Compute distance
            delta_theta1 = theta1_end - theta1_start
            delta_theta2 = theta2_end - theta2_start
            
            # Estimate number of steps needed (based on max delta per step)
            # Convert angle delta to normalized units
            delta_u1 = abs(delta_theta1 * self.mapper.k_shoulder)
            delta_u2 = abs(delta_theta2 * self.mapper.k_elbow)
            max_delta_u = max(delta_u1, delta_u2)
            
            num_steps = max(1, int(np.ceil(max_delta_u / self.max_delta_per_step)))
            
            # Interpolate
            for j in range(1, num_steps + 1):
                alpha = j / num_steps
                theta1 = theta1_start + alpha * delta_theta1
                theta2 = theta2_start + alpha * delta_theta2
                resampled.append(np.array([theta1, theta2, 0.0, 0.0]))
        
        return resampled
    
    def execute_path(self, path: List[np.ndarray], 
                    joint_names: Tuple[str, str] = ("shoulder_lift", "elbow_flex"),
                    verbose: bool = True) -> Dict:
        """
        Execute a planned path on the robot.
        
        Args:
            path: List of waypoints [θ1, θ2, 0, 0] from planner
            joint_names: Tuple of (shoulder_joint_name, elbow_joint_name)
            verbose: Whether to print progress
        
        Returns:
            Dict with execution results:
            {
                'success': bool,
                'waypoints_executed': int,
                'total_waypoints': int,
                'execution_time': float,
                'max_error_shoulder': float,
                'max_error_elbow': float,
                'log': List of (t, theta_cmd, u_cmd, u_meas, error) tuples
            }
        """
        if self.is_running:
            raise RuntimeError("Path executor is already running")
        
        self.is_running = True
        self.execution_log = []
        
        shoulder_name, elbow_name = joint_names
        
        # Resample path for smooth execution
        if verbose:
            print(f"\n[Path Executor] Resampling path ({len(path)} waypoints)...")
        resampled_path = self.resample_path(path)
        if verbose:
            print(f"  Resampled to {len(resampled_path)} waypoints (dt={self.dt:.3f}s)")
        
        start_time = time.time()
        waypoints_executed = 0
        max_error_shoulder = 0.0
        max_error_elbow = 0.0
        
        # Track last error for stall detection
        last_error_shoulder = float('inf')
        last_error_elbow = float('inf')
        stall_start_time = None
        
        try:
            for i, waypoint in enumerate(resampled_path):
                if not self.is_running:
                    if verbose:
                        print(f"\n[Path Executor] Aborted by user")
                    break
                
                theta1 = waypoint[0]
                theta2 = waypoint[1]
                
                # Convert to robot commands (with rate limiting)
                u_shoulder_cmd, u_elbow_cmd = self.mapper.sim_to_robot(
                    theta1, theta2, rate_limit=False
                )
                
                # Send commands
                self.bus.write("Goal_Position", shoulder_name, u_shoulder_cmd, normalize=True)
                self.bus.write("Goal_Position", elbow_name, u_elbow_cmd, normalize=True)
                
                # Wait for control period
                time.sleep(self.dt)
                
                # Read back actual positions
                u_shoulder_meas = self.bus.read("Present_Position", shoulder_name, normalize=True)
                u_elbow_meas = self.bus.read("Present_Position", elbow_name, normalize=True)
                
                # Compute errors
                error_shoulder = abs(u_shoulder_meas - u_shoulder_cmd)
                error_elbow = abs(u_elbow_meas - u_elbow_cmd)
                
                max_error_shoulder = max(max_error_shoulder, error_shoulder)
                max_error_elbow = max(max_error_elbow, error_elbow)
                
                # Log
                t = time.time() - start_time
                self.execution_log.append({
                    't': t,
                    'theta_cmd': (theta1, theta2),
                    'u_cmd': (u_shoulder_cmd, u_elbow_cmd),
                    'u_meas': (u_shoulder_meas, u_elbow_meas),
                    'error': (error_shoulder, error_elbow)
                })
                
                # Check for stall using combined error with deadband
                err = max(error_shoulder, error_elbow)
                last_err = max(last_error_shoulder, last_error_elbow)
                
                # Improved if error decreased by more than deadband (0.05 normalized units)
                improved = err < last_err - 0.05
                if improved:
                    stall_start_time = None  # Error is decreasing, not stalled
                else:
                    if stall_start_time is None:
                        stall_start_time = time.time()
                    elif time.time() - stall_start_time > self.stall_timeout:
                        if verbose:
                            print(f"\n[Path Executor] ⚠️  Stall detected! Aborting...")
                            print(f"  Combined error: {err:.3f} (shoulder: {error_shoulder:.3f}, elbow: {error_elbow:.3f})")
                        break
                
                last_error_shoulder = error_shoulder
                last_error_elbow = error_elbow
                
                waypoints_executed += 1
                
                # Progress update
                if verbose and (i + 1) % 10 == 0:
                    print(f"  [{i+1}/{len(resampled_path)}] t={t:.2f}s, "
                          f"error_sh={error_shoulder:.2f}, error_el={error_elbow:.2f}")
        
        except KeyboardInterrupt:
            if verbose:
                print(f"\n[Path Executor] Interrupted by user")
            self.is_running = False
        
        except Exception as e:
            if verbose:
                print(f"\n[Path Executor] Error: {e}")
            import traceback
            traceback.print_exc()
            self.is_running = False
        
        execution_time = time.time() - start_time
        success = waypoints_executed == len(resampled_path)
        
        if verbose:
            print(f"\n[Path Executor] Execution complete:")
            print(f"  Success: {success}")
            print(f"  Waypoints: {waypoints_executed}/{len(resampled_path)}")
            print(f"  Time: {execution_time:.2f}s")
            print(f"  Max error (shoulder): {max_error_shoulder:.3f} normalized units")
            print(f"  Max error (elbow): {max_error_elbow:.3f} normalized units")
        
        self.is_running = False
        
        return {
            'success': success,
            'waypoints_executed': waypoints_executed,
            'total_waypoints': len(resampled_path),
            'execution_time': execution_time,
            'max_error_shoulder': max_error_shoulder,
            'max_error_elbow': max_error_elbow,
            'log': self.execution_log
        }
    
    def stop(self):
        """Stop execution gracefully."""
        self.is_running = False

