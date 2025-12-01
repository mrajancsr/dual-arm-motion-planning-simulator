"""
Handoff Planner Module

Implements intelligent handoff planning that determines which arm should grab/deliver
based on workspace reachability and calculates optimal handoff points.
"""

import numpy as np
from typing import Optional, Dict, List, Tuple
from .dual_arm_system import DualArm
from .rrt_star import RRTStar
from .workspace_generator import WorkspaceGenerator
from .robot_arm_base import RobotArmBase


class HandoffPlanner:
    """
    Planner that intelligently determines grab/delivery arms and executes
    phase-based RRT* planning with optional handoff.
    """
    
    def __init__(self, dual_arm: DualArm):
        """
        Initialize handoff planner.
        
        Args:
            dual_arm: DualArm system instance
        """
        self.dual_arm = dual_arm
        self.left_arm = dual_arm.left_arm
        self.right_arm = dual_arm.right_arm
        
    def check_reachability(self, point: np.ndarray, verbose: bool = False) -> Dict:
        """
        Check if a point is reachable by each arm using IK verification.
        
        Args:
            point: [x, y] position in workspace
            
        Returns:
            Dictionary with reachability info:
            {
                'left_reachable': bool,
                'right_reachable': bool,
                'point': np.ndarray
            }
        """
        # Calculate distances from bases for quick filtering
        dist_left = np.linalg.norm(point - self.dual_arm.left_base)
        dist_right = np.linalg.norm(point - self.dual_arm.right_base)
        
        # Get arm link lengths for distance check (works for N-link arms)
        left_max_reach = 1.7  # Default 2-link
        left_min_reach = 0.3
        right_max_reach = 1.7
        right_min_reach = 0.3
        
        # Try to get actual link lengths from arms
        if hasattr(self.left_arm, 'link_lengths'):
            try:
                ll = self.left_arm.link_lengths
                if isinstance(ll, (list, tuple, np.ndarray)) and len(ll) > 0:
                    # Sum all link lengths for max reach (works for 2-link, 3-link, 6-link, etc.)
                    left_max_reach = float(np.sum(ll))
                    # Min reach: sum - 2*max_link (approximate)
                    left_min_reach = max(0.1, float(left_max_reach - 2 * np.max(ll)))
            except:
                pass
        
        if hasattr(self.right_arm, 'link_lengths'):
            try:
                ll = self.right_arm.link_lengths
                if isinstance(ll, (list, tuple, np.ndarray)) and len(ll) > 0:
                    right_max_reach = float(np.sum(ll))
                    right_min_reach = max(0.1, float(right_max_reach - 2 * np.max(ll)))
            except:
                pass
        
        # First check distance bounds (fast pre-filter)
        # But don't skip IK if distance check fails - IK is the authoritative check
        left_distance_ok = left_min_reach <= dist_left <= left_max_reach
        right_distance_ok = right_min_reach <= dist_right <= right_max_reach
        
        # Now verify with actual IK (checks if point is actually reachable, not behind base, etc.)
        # IK is the authoritative check - distance check is just a fast pre-filter
        # We still try IK even if distance check fails (distance might be approximate)
        left_reachable = False
        right_reachable = False
        
        # Always try IK for left arm (distance check is just a hint)
        local_left = point - self.dual_arm.left_base
        try:
            ik_result = self.left_arm.ik_iterative(local_left[0], local_left[1], max_iters=200, tol=1e-3)
            
            # If failed, try with different initial guesses (up to 5 attempts)
            if ik_result is None:
                for attempt in range(5):
                    init_guess = tuple(np.random.uniform(-np.pi, np.pi, self.left_arm.get_num_joints()))
                    ik_result = self.left_arm.ik_iterative(
                        local_left[0], local_left[1],
                        theta_init=init_guess,
                        max_iters=200,
                        tol=1e-3
                    )
                    if ik_result is not None:
                        break
            
            if ik_result is not None:
                left_reachable = True
                if verbose:
                    print(f"[check_reachability] Left IK SUCCESS for global {point}, local {local_left}, dist={dist_left:.3f}", flush=True)
            else:
                if verbose:
                    if not left_distance_ok:
                        print(f"[check_reachability] Left distance check FAILED (dist={dist_left:.3f}, range=[{left_min_reach:.3f}, {left_max_reach:.3f}]) AND IK FAILED after 6 attempts", flush=True)
                    else:
                        print(f"[check_reachability] Left IK FAILED after 6 attempts for global {point}, local {local_left}, dist={dist_left:.3f}", flush=True)
        except Exception as e:
            if verbose:
                print(f"[check_reachability] Left IK EXCEPTION for global {point}, local {local_left}: {e}", flush=True)
        
        # Always try IK for right arm (distance check is just a hint)
        local_right = point - self.dual_arm.right_base
        try:
            ik_result = self.right_arm.ik_iterative(local_right[0], local_right[1], max_iters=200, tol=1e-3)
            
            # If failed, try with different initial guesses (up to 5 attempts)
            if ik_result is None:
                for attempt in range(5):
                    init_guess = tuple(np.random.uniform(-np.pi, np.pi, self.right_arm.get_num_joints()))
                    ik_result = self.right_arm.ik_iterative(
                        local_right[0], local_right[1],
                        theta_init=init_guess,
                        max_iters=200,
                        tol=1e-3
                    )
                    if ik_result is not None:
                        break
            
            if ik_result is not None:
                right_reachable = True
                if verbose:
                    print(f"[check_reachability] Right IK SUCCESS for global {point}, local {local_right}, dist={dist_right:.3f}", flush=True)
            else:
                if verbose:
                    if not right_distance_ok:
                        print(f"[check_reachability] Right distance check FAILED (dist={dist_right:.3f}, range=[{right_min_reach:.3f}, {right_max_reach:.3f}]) AND IK FAILED after 6 attempts", flush=True)
                    else:
                        print(f"[check_reachability] Right IK FAILED after 6 attempts for global {point}, local {local_right}, dist={dist_right:.3f}", flush=True)
        except Exception as e:
            if verbose:
                print(f"[check_reachability] Right IK EXCEPTION for global {point}, local {local_right}: {e}", flush=True)
        
        return {
            'left_reachable': left_reachable,
            'right_reachable': right_reachable,
            'point': point,
            'left_distance': dist_left,
            'right_distance': dist_right
        }
    
    def determine_arms(self, start_pos: np.ndarray, goal_pos: np.ndarray) -> Dict:
        """
        Determine which arms should be used for grabbing and delivery.
        
        Args:
            start_pos: Item start position [x, y]
            goal_pos: Item goal position [x, y]
            
        Returns:
            Dictionary with arm assignment:
            {
                'grab_arm': 'left' | 'right' | 'either',
                'delivery_arm': 'left' | 'right' | 'either',
                'needs_handoff': bool,
                'single_arm_possible': bool
            }
        """
        print(f"[determine_arms] Checking start pos {start_pos}...", flush=True)
        start_reach = self.check_reachability(start_pos, verbose=True)
        print(f"[determine_arms] Start reach: left={start_reach['left_reachable']}, right={start_reach['right_reachable']}", flush=True)
        
        print(f"[determine_arms] Checking goal pos {goal_pos}...", flush=True)
        goal_reach = self.check_reachability(goal_pos, verbose=True)
        print(f"[determine_arms] Goal reach: left={goal_reach['left_reachable']}, right={goal_reach['right_reachable']}", flush=True)
        
        # Determine which arms can reach start and goal
        left_reaches_start = start_reach['left_reachable']
        right_reaches_start = start_reach['right_reachable']
        left_reaches_goal = goal_reach['left_reachable']
        right_reaches_goal = goal_reach['right_reachable']
        
        # Check if single arm can do both
        left_can_do_both = left_reaches_start and left_reaches_goal
        right_can_do_both = right_reaches_start and right_reaches_goal
        
        # Determine strategy
        if left_can_do_both and not right_can_do_both:
            # Only left arm can do everything
            return {
                'grab_arm': 'left',
                'delivery_arm': 'left',
                'needs_handoff': False,
                'single_arm_possible': True,
                'chosen_arm': 'left'
            }
        
        elif right_can_do_both and not left_can_do_both:
            # Only right arm can do everything
            return {
                'grab_arm': 'right',
                'delivery_arm': 'right',
                'needs_handoff': False,
                'single_arm_possible': True,
                'chosen_arm': 'right'
            }
        
        elif left_can_do_both and right_can_do_both:
            # Both arms can do everything - choose based on proximity
            left_total_dist = start_reach['left_distance'] + goal_reach['left_distance']
            right_total_dist = start_reach['right_distance'] + goal_reach['right_distance']
            
            chosen = 'left' if left_total_dist <= right_total_dist else 'right'
            return {
                'grab_arm': chosen,
                'delivery_arm': chosen,
                'needs_handoff': False,
                'single_arm_possible': True,
                'chosen_arm': chosen
            }
        
        else:
            # Need handoff
            # Determine grab arm (which arm reaches start)
            if left_reaches_start and not right_reaches_start:
                grab_arm = 'left'
            elif right_reaches_start and not left_reaches_start:
                grab_arm = 'right'
            elif left_reaches_start and right_reaches_start:
                # Both can grab - choose closer one
                grab_arm = 'left' if start_reach['left_distance'] <= start_reach['right_distance'] else 'right'
            else:
                # Neither can reach start - problem is unsolvable
                grab_arm = None
            
            # Determine delivery arm (which arm reaches goal)
            if left_reaches_goal and not right_reaches_goal:
                delivery_arm = 'left'
            elif right_reaches_goal and not left_reaches_goal:
                delivery_arm = 'right'
            elif left_reaches_goal and right_reaches_goal:
                # Both can deliver - choose closer one
                delivery_arm = 'left' if goal_reach['left_distance'] <= goal_reach['right_distance'] else 'right'
            else:
                # Neither can reach goal - problem is unsolvable
                delivery_arm = None
            
            if grab_arm is None:
                raise ValueError(f"Item start position {start_pos} is unreachable by any arm")
            if delivery_arm is None:
                raise ValueError(f"Item goal position {goal_pos} is unreachable by any arm")
            
            return {
                'grab_arm': grab_arm,
                'delivery_arm': delivery_arm,
                'needs_handoff': grab_arm != delivery_arm,
                'single_arm_possible': False
            }
    
    def find_handoff_point(self, start_pos: np.ndarray, goal_pos: np.ndarray,
                           obstacles: List[dict], resolution: int = 30) -> np.ndarray:
        """
        Find optimal handoff point in workspace intersection.
        
        Args:
            start_pos: Item start position
            goal_pos: Item goal position
            obstacles: List of obstacle dictionaries
            resolution: Resolution for workspace sampling
            
        Returns:
            [x, y] handoff position
        """
        # Generate workspace points for both arms
        left_ws_gen = WorkspaceGenerator(self.left_arm, resolution=resolution)
        right_ws_gen = WorkspaceGenerator(self.right_arm, resolution=resolution)
        
        left_x, left_y = left_ws_gen.generate_workspace()
        right_x, right_y = right_ws_gen.generate_workspace()
        
        # Convert to global coordinates (workspace returns LOCAL coords)
        left_points = np.column_stack([
            left_x + self.dual_arm.left_base[0], 
            left_y + self.dual_arm.left_base[1]
        ])
        right_points = np.column_stack([
            right_x + self.dual_arm.right_base[0], 
            right_y + self.dual_arm.right_base[1]
        ])
        
        print(f"[find_handoff_point] Left workspace: {len(left_points)} points")
        print(f"[find_handoff_point] Right workspace: {len(right_points)} points")
        print(f"[find_handoff_point] Left base: {self.dual_arm.left_base}")
        print(f"[find_handoff_point] Right base: {self.dual_arm.right_base}")
        
        # Find intersection using spatial sets (with rounding for discrete comparison)
        # Use coarser rounding to find more intersections
        left_set = set((round(p[0], 1), round(p[1], 1)) for p in left_points)
        right_set = set((round(p[0], 1), round(p[1], 1)) for p in right_points)
        intersection = left_set & right_set
        
        print(f"[find_handoff_point] Left set size: {len(left_set)}")
        print(f"[find_handoff_point] Right set size: {len(right_set)}")
        print(f"[find_handoff_point] Intersection size: {len(intersection)}")
        
        if not intersection:
            # Try even coarser resolution
            left_set_coarse = set((round(p[0], 0), round(p[1], 0)) for p in left_points)
            right_set_coarse = set((round(p[0], 0), round(p[1], 0)) for p in right_points)
            intersection = left_set_coarse & right_set_coarse
            
            print(f"[find_handoff_point] Retrying with coarser resolution: {len(intersection)} points")
            
            if not intersection:
                raise ValueError("No workspace intersection found between arms")
        
        # Convert back to array
        intersection_points = np.array(list(intersection))
        
        # Filter points: check obstacles, height, AND IK reachability
        clearance = 0.3
        min_height = 0.3  # Minimum y-coordinate for handoff (lowered from 0.5)
        safe_points = []
        
        print(f"[find_handoff_point] Filtering {len(intersection_points)} intersection points...")
        for i, point in enumerate(intersection_points):
            # Skip points too low
            if point[1] < min_height:
                continue
            
            # Check obstacle clearance
            is_safe = True
            for obstacle in obstacles:
                if obstacle['type'] == 'circle':
                    dist = np.linalg.norm(point - np.array(obstacle['center']))
                    if dist < obstacle['radius'] + clearance:
                        is_safe = False
                        break
            
            if is_safe:
                # Verify IK can actually reach this point (CRITICAL CHECK!)
                reach_check = self.check_reachability(point)
                if reach_check['left_reachable'] and reach_check['right_reachable']:
                    safe_points.append(point)
                elif (i % 10 == 0):  # Log every 10th rejected point to reduce spam
                    print(f"[find_handoff_point] Point {point} rejected: "
                          f"left={reach_check['left_reachable']}, right={reach_check['right_reachable']}", 
                          flush=True)
        
        print(f"[find_handoff_point] Safe IK-reachable points (y >= {min_height}): {len(safe_points)}", flush=True)
        
        if not safe_points:
            # This is a real failure - no points are reachable by both arms via IK
            raise ValueError(
                f"No IK-reachable handoff points found! "
                f"Intersection had {len(intersection_points)} points, but none passed IK check. "
                f"Left base: {self.dual_arm.left_base}, Right base: {self.dual_arm.right_base}"
            )
        
        safe_points = np.array(safe_points)
        
        # Find point that minimizes sum of distances from both bases
        # Also bias toward points on the line from start to goal
        distances = []
        for p in safe_points:
            # Distance metric
            base_dist = (np.linalg.norm(p - self.dual_arm.left_base) + 
                        np.linalg.norm(p - self.dual_arm.right_base))
            
            # Bias toward line from start to goal
            line_vec = goal_pos - start_pos
            line_length = np.linalg.norm(line_vec)
            if line_length > 0:
                line_dir = line_vec / line_length
                start_to_p = p - start_pos
                projection = np.dot(start_to_p, line_dir)
                # Penalize points far from the line
                dist_from_line = np.linalg.norm(start_to_p - projection * line_dir)
                total_dist = base_dist + dist_from_line * 2.0  # Weight line distance more
            else:
                total_dist = base_dist
            
            distances.append(total_dist)
        
        distances = np.array(distances)
        best_idx = np.argmin(distances)
        handoff_point = safe_points[best_idx]
        
        print(f"[find_handoff_point] Selected handoff point: {handoff_point}")
        return handoff_point
    
    def plan_with_handoff(self, item_start: np.ndarray, item_goal: np.ndarray,
                         rrt_params: dict, progress_callback=None) -> Optional[Dict]:
        """
        Main planning method that determines strategy and executes planning.
        
        Args:
            item_start: Item start position [x, y]
            item_goal: Item goal position [x, y]
            rrt_params: RRT* parameters (max_iterations, step_size, etc.)
            
        Returns:
            Dictionary with results:
            {
                'strategy': dict with arm assignments,
                'paths': list of path segments,
                'handoff_point': np.ndarray or None,
                'full_path': combined path array,
                'phases': list of phase descriptions
            }
        """
        # Step 1: Determine which arms to use
        strategy = self.determine_arms(item_start, item_goal)
        print(f"[HandoffPlanner] Strategy: {strategy}", flush=True)
        
        result = {
            'strategy': strategy,
            'handoff_point': None,
            'phases': [],
            'paths': [],
            'full_path': None
        }
        
        # Step 2: Execute planning based on strategy
        if not strategy['needs_handoff']:
            # Single arm planning
            chosen_arm = strategy['chosen_arm']
            print(f"[HandoffPlanner] Single arm planning with {chosen_arm} arm", flush=True)
            
            # Plan from start to goal with single arm
            path = self._plan_single_arm_task(
                arm=chosen_arm,
                start_pos=item_start,
                goal_pos=item_goal,
                rrt_params=rrt_params,
                progress_callback=progress_callback
            )
            
            if path:
                result['paths'] = [path]
                result['full_path'] = path
                result['phases'] = [{
                    'arm': chosen_arm,
                    'from': 'start',
                    'to': 'goal',
                    'path_length': len(path)
                }]
            
            return result
        
        else:
            # Handoff required
            print(f"[HandoffPlanner] Handoff required: {strategy['grab_arm']} -> {strategy['delivery_arm']}", flush=True)
            
            # Step 3: Find handoff point
            print(f"[HandoffPlanner] Finding handoff point...", flush=True)
            handoff_point = self.find_handoff_point(
                item_start, 
                item_goal, 
                self.dual_arm.obstacles
            )
            result['handoff_point'] = handoff_point
            print(f"[HandoffPlanner] Handoff point: {handoff_point}", flush=True)
            
            # Step 4: Plan grab phase
            print(f"[HandoffPlanner] Planning grab phase ({strategy['grab_arm']} arm)...", flush=True)
            grab_path = self._plan_single_arm_task(
                arm=strategy['grab_arm'],
                start_pos=item_start,
                goal_pos=handoff_point,
                rrt_params=rrt_params,
                progress_callback=progress_callback
            )
            
            if not grab_path:
                print("[HandoffPlanner] Grab phase planning failed", flush=True)
                return None
            
            print(f"[HandoffPlanner] Grab phase complete: {len(grab_path)} waypoints", flush=True)
            
            # Step 5: Plan delivery phase
            print(f"[HandoffPlanner] Planning delivery phase ({strategy['delivery_arm']} arm)...", flush=True)
            delivery_path = self._plan_single_arm_task(
                arm=strategy['delivery_arm'],
                start_pos=handoff_point,
                goal_pos=item_goal,
                rrt_params=rrt_params,
                progress_callback=progress_callback
            )
            
            if not delivery_path:
                print("[HandoffPlanner] Delivery phase planning failed", flush=True)
                return None
            
            print(f"[HandoffPlanner] Delivery phase complete: {len(delivery_path)} waypoints", flush=True)
            
            # Step 6: Stitch paths together
            full_path = self._stitch_paths(grab_path, delivery_path, strategy)
            
            result['paths'] = [grab_path, delivery_path]
            result['full_path'] = full_path
            result['phases'] = [
                {
                    'arm': strategy['grab_arm'],
                    'from': 'start',
                    'to': 'handoff',
                    'path_length': len(grab_path)
                },
                {
                    'arm': strategy['delivery_arm'],
                    'from': 'handoff',
                    'to': 'goal',
                    'path_length': len(delivery_path)
                }
            ]
            
            return result
    
    def _plan_single_arm_task(self, arm: str, start_pos: np.ndarray, 
                              goal_pos: np.ndarray, rrt_params: dict,
                              progress_callback=None) -> Optional[List[np.ndarray]]:
        """
        Plan for a single arm to move item from start_pos to goal_pos.
        
        Args:
            arm: 'left' or 'right'
            start_pos: Starting position for item
            goal_pos: Goal position for item
            rrt_params: RRT* parameters
            
        Returns:
            List of configurations or None
        """
        print(f"[_plan_single_arm_task] Getting start config for {arm} arm at {start_pos}", flush=True)
        # Get start and goal configurations
        start_config = self._get_config_for_position(arm, start_pos)
        
        if start_config is None:
            print(f"[_plan_single_arm_task] Failed to get start config", flush=True)
            return None
        
        print(f"[_plan_single_arm_task] Getting goal config for {arm} arm at {goal_pos}", flush=True)
        goal_config = self._get_config_for_position(arm, goal_pos)
        
        if goal_config is None:
            print(f"[_plan_single_arm_task] Failed to get goal config", flush=True)
            return None
        
        # Create RRT* planner
        print(f"[_plan_single_arm_task] Creating RRTStar planner...", flush=True)
        planner = RRTStar(
            self.dual_arm,
            max_iterations=rrt_params.get('max_iterations', 5000),
            step_size=rrt_params.get('step_size', 0.15),
            goal_threshold=rrt_params.get('goal_threshold', 0.2),
            use_kdtree=rrt_params.get('use_kdtree', True),
            workspace_weight=rrt_params.get('workspace_weight', 0.3),
            use_adaptive_step=rrt_params.get('use_adaptive_step', True),
            verbose=True
        )
        
        # Plan with single arm active
        print(f"[_plan_single_arm_task] Starting RRT* planning for {arm} arm...", flush=True)
        path = planner.plan_single_arm(arm, start_config, goal_config, progress_callback=progress_callback)
        
        if path:
            print(f"[_plan_single_arm_task] RRT* found path with {len(path)} waypoints", flush=True)
        else:
            print(f"[_plan_single_arm_task] RRT* failed to find path", flush=True)
        
        return path
    
    def _get_config_for_position(self, arm: str, position: np.ndarray) -> Optional[np.ndarray]:
        """
        Get full dual-arm configuration for a position that the specified arm should reach.
        Other arm stays in idle position (pointing up).
        
        Args:
            arm: 'left' or 'right'
            position: [x, y] position to reach (GLOBAL coordinates)
            
        Returns:
            Full configuration array or None
        """
        arm_obj = self.left_arm if arm == 'left' else self.right_arm
        base = self.dual_arm.left_base if arm == 'left' else self.dual_arm.right_base
        
        # Convert to local coordinates
        local_pos = position - base
        
        print(f"[_get_config_for_position] {arm} arm: global={position}, local={local_pos}, base={base}")
        
        # Use arm's IK solver with multiple attempts
        arm_config = None
        try:
            print(f"[_get_config_for_position] Calling IK for {arm} arm...", flush=True)
            # Try IK with default initial guess (note: use max_iters and tol, not max_iterations and tolerance)
            arm_config = arm_obj.ik_iterative(local_pos[0], local_pos[1], max_iters=200, tol=1e-3)
            print(f"[_get_config_for_position] IK returned: {arm_config}", flush=True)
            
            # If failed, try with different initial guesses
            if arm_config is None:
                for _ in range(5):
                    init_guess = tuple(np.random.uniform(-np.pi, np.pi, arm_obj.get_num_joints()))
                    arm_config = arm_obj.ik_iterative(
                        local_pos[0], local_pos[1],
                        theta_init=init_guess,
                        max_iters=200,
                        tol=1e-3
                    )
                    if arm_config is not None:
                        break
            
            if arm_config is None:
                print(f"[_get_config_for_position] IK failed for {arm} arm to reach {local_pos}")
                return None
                
        except Exception as e:
            print(f"[_get_config_for_position] IK exception for {arm}: {e}")
            return None
        
        # Get config for other arm
        # For handoff scenarios, the other arm should ALSO be at the handoff point (both arms meet there)
        # For single-arm scenarios, the other arm can be idle
        num_joints = arm_obj.get_num_joints()
        
        # Try to have the other arm also reach the same position (for handoff)
        other_arm = self.right_arm if arm == 'left' else self.left_arm
        other_base = self.dual_arm.right_base if arm == 'left' else self.dual_arm.left_base
        other_local_pos = position - other_base
        
        try:
            other_arm_config = other_arm.ik_iterative(other_local_pos[0], other_local_pos[1], max_iters=200, tol=1e-3)
            if other_arm_config is None:
                # If other arm can't reach the position, use idle config
                print(f"[_get_config_for_position] Other arm can't reach position, using idle", flush=True)
                other_arm_config = np.zeros(num_joints)
            else:
                print(f"[_get_config_for_position] Other arm can also reach position", flush=True)
        except:
            # Fall back to idle if IK fails
            other_arm_config = np.zeros(num_joints)
        
        # Construct full dual-arm config
        if arm == 'left':
            full_config = np.concatenate([np.array(arm_config), np.array(other_arm_config)])
        else:
            full_config = np.concatenate([np.array(other_arm_config), np.array(arm_config)])
        
        # Validate the configuration
        if not self.dual_arm.is_valid_configuration(full_config):
            print(f"[_get_config_for_position] Configuration with other arm at position is invalid, trying with other arm idle", flush=True)
            # Try with other arm idle as fallback
            idle_config = np.zeros(num_joints)
            if arm == 'left':
                full_config = np.concatenate([np.array(arm_config), idle_config])
            else:
                full_config = np.concatenate([idle_config, np.array(arm_config)])
            
            if not self.dual_arm.is_valid_configuration(full_config):
                print(f"[_get_config_for_position] Configuration is invalid even with other arm idle (collision or limits)")
                return None
        
        print(f"[_get_config_for_position] ✓ Valid config found: {full_config}")
        return full_config
    
    def _stitch_paths(self, grab_path: List[np.ndarray], delivery_path: List[np.ndarray],
                     strategy: Dict) -> List[np.ndarray]:
        """
        Stitch grab and delivery paths together at handoff point.
        
        For sequential handoff:
        - grab_path ends with grab arm at handoff
        - delivery_path starts with delivery arm at handoff
        - Combined: grab_path + delivery_path (handoff config shared)
        
        Args:
            grab_path: Path from start to handoff
            delivery_path: Path from handoff to goal
            strategy: Strategy dictionary
            
        Returns:
            Combined path
        """
        # The handoff configuration is the last config of grab_path
        # and first config of delivery_path
        # For sequential handoff, we just concatenate
        
        # Remove duplicate handoff config
        full_path = grab_path + delivery_path[1:]
        
        return full_path

