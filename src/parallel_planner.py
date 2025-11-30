"""
Parallel RRT* Planner with Mac Compatibility

This module provides parallel planning by running multiple RRT* instances
simultaneously and returning the best result. Designed to work on macOS
with proper multiprocessing configuration.
"""

from multiprocessing import Pool, set_start_method
import numpy as np
from typing import List, Optional, Tuple
import platform

try:
    from .rrt_star import RRTStar
    from .dual_arm_system import DualArm
except ImportError:
    from rrt_star import RRTStar
    from dual_arm_system import DualArm


def _plan_worker(args: Tuple) -> Optional[Tuple[int, List[np.ndarray]]]:
    """
    Worker function for parallel planning (must be top-level for pickling).
    
    Args:
        args: Tuple of (dual_arm, start_config, goal_config, planner_params, seed)
        
    Returns:
        Tuple of (path_length, path) if successful, None otherwise
    """
    dual_arm, start_config, goal_config, planner_params, seed = args
    
    # Set random seed for reproducibility
    np.random.seed(seed)
    
    try:
        # Create planner instance
        planner = RRTStar(dual_arm, **planner_params)
        
        # Plan path
        path = planner.plan(start_config, goal_config)
        
        if path is None:
            return None
        
        # Return path length as cost metric
        return (len(path), path)
    
    except Exception as e:
        # Handle any errors gracefully
        print(f"Worker {seed} encountered error: {e}")
        return None


class ParallelRRTStar:
    """
    Run multiple RRT* planners in parallel and return best result.
    
    Mac-compatible implementation using 'spawn' method for multiprocessing.
    Safe for use on macOS 10.13+ and other Unix systems.
    """
    
    def __init__(self, dual_arm: DualArm, num_workers: int = 4, **planner_params):
        """
        Initialize parallel RRT* planner.
        
        Args:
            dual_arm: DualArm system instance
            num_workers: Number of parallel workers (default: 4)
            **planner_params: Parameters to pass to each RRTStar instance
                (max_iterations, step_size, goal_threshold, etc.)
        """
        self.dual_arm = dual_arm
        self.num_workers = num_workers
        self.planner_params = planner_params
        
        # Mac compatibility: use 'spawn' on macOS
        if platform.system() == 'Darwin':
            try:
                set_start_method('spawn', force=True)
            except RuntimeError:
                # Already set, which is fine
                pass
    
    def plan(self, start_config: np.ndarray, goal_config: np.ndarray,
             verbose: bool = False) -> Optional[List[np.ndarray]]:
        """
        Plan using parallel workers and return best path.
        
        Args:
            start_config: Start configuration as numpy array
            goal_config: Goal configuration as numpy array
            verbose: Whether to print progress information
            
        Returns:
            Best path found by any worker, or None if all workers fail
        """
        if verbose:
            print(f"\n🚀 Parallel Planning with {self.num_workers} workers...")
        
        # Create work items with different random seeds
        work_items = [
            (self.dual_arm, start_config, goal_config, self.planner_params, seed)
            for seed in range(self.num_workers)
        ]
        
        # Run in parallel
        with Pool(self.num_workers) as pool:
            results = pool.map(_plan_worker, work_items)
        
        # Filter out None results
        valid_results = [r for r in results if r is not None]
        
        if not valid_results:
            if verbose:
                print("❌ All workers failed to find a path")
            return None
        
        if verbose:
            print(f"✓ {len(valid_results)}/{self.num_workers} workers found paths")
            for i, (cost, _) in enumerate(valid_results):
                print(f"  Worker {i}: path length = {cost}")
        
        # Return path with shortest length
        best_cost, best_path = min(valid_results, key=lambda x: x[0])
        
        if verbose:
            print(f"✓ Best path has {best_cost} waypoints")
        
        return best_path


# Example usage
if __name__ == "__main__":
    from dual_arm_system import DualArm
    from arms.two_link_arm import TwoLinkArm
    
    # Create dual-arm system
    left_arm = TwoLinkArm(L1=1.0, L2=0.7, name="LeftArm")
    right_arm = TwoLinkArm(L1=1.0, L2=0.7, name="RightArm")
    dual_arm = DualArm(left_arm=left_arm, right_arm=right_arm, separation=2.0)
    
    # Define start and goal configurations
    start_config = np.array([0.5, 0.3, -0.2, 0.4])
    goal_config = np.array([1.0, -0.5, 0.8, -0.3])
    
    # Create parallel planner
    planner = ParallelRRTStar(
        dual_arm,
        num_workers=4,
        max_iterations=3000,
        step_size=0.15,
        verbose=True
    )
    
    # Plan path
    path = planner.plan(start_config, goal_config, verbose=True)
    
    if path:
        print(f"\n✓ Path found with {len(path)} configurations")
    else:
        print("\n❌ No path found")

