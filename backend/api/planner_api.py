"""
Flask API for Dual-Arm Motion Planning

Provides REST endpoints for:
- Starting planning jobs
- Getting planning status and progress
- Retrieving RRT* tree data
- Retrieving solution paths
"""

import sys
import os
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from flask import Flask, request, jsonify
from flask_cors import CORS
import numpy as np
import threading
import uuid
import time
from typing import Dict, List, Optional

from src import (
    DualArm, TwoLinkArm, ThreeLinkArm, SixLinkArm,
    RRTStar, HandoffPlanner
)

# Initialize Flask app
app = Flask(__name__)
CORS(app)  # Enable CORS for local development

# Job storage
jobs: Dict[str, Dict] = {}
job_lock = threading.Lock()


# Helper functions
def create_arm(arm_type: str, params: Dict):
    """Create an arm instance based on type."""
    if arm_type == '2-link':
        return TwoLinkArm(
            L1=params.get('L1', 1.0),
            L2=params.get('L2', 0.7),
            name=params.get('name', 'Arm')
        )
    elif arm_type == '3-link':
        return ThreeLinkArm(
            L1=params.get('L1', 1.0),
            L2=params.get('L2', 0.8),
            L3=params.get('L3', 0.6),
            name=params.get('name', 'Arm')
        )
    elif arm_type == '6-link':
        return SixLinkArm(
            L1=params.get('L1', 0.5),
            L2=params.get('L2', 0.4),
            L3=params.get('L3', 0.3),
            L4=params.get('L4', 0.25),
            L5=params.get('L5', 0.2),
            L6=params.get('L6', 0.15),
            name=params.get('name', 'Arm')
        )
    else:
        raise ValueError(f"Unknown arm type: {arm_type}")


def handoff_planning_worker(job_id: str, config: Dict):
    """Background worker for handoff-based planning."""
    try:
        # Update job status
        start_time = time.time()
        with job_lock:
            jobs[job_id]['status'] = 'running'
            jobs[job_id]['start_time'] = start_time
            jobs[job_id]['progress']['start_time'] = start_time
        
        # Extract configuration
        arm_type = config['arm_type']
        item_start = np.array(config['item_start'])
        item_goal = np.array(config['item_goal'])
        left_base_x = config['left_base_x']
        right_base_x = config['right_base_x']
        obstacles = config.get('obstacles', [])
        rrt_params = config.get('rrt_params', {})
        
        print(f"[Handoff Planning] Starting for job {job_id}", flush=True)
        print(f"  Item start: {item_start}", flush=True)
        print(f"  Item goal: {item_goal}", flush=True)
        print(f"  Left base: [{left_base_x}, 0]", flush=True)
        print(f"  Right base: [{right_base_x}, 0]", flush=True)
        
        # Update progress to show we're starting
        with job_lock:
            jobs[job_id]['progress']['current_phase'] = 'Analyzing workspace...'
        
        # Create arms
        arm_params = config.get('arm_params', {})
        left_arm = create_arm(arm_type, {'name': 'LeftArm', **arm_params})
        right_arm = create_arm(arm_type, {'name': 'RightArm', **arm_params})
        
        # Create dual-arm system with specific base positions
        separation = abs(right_base_x - left_base_x)
        dual_arm = DualArm(
            left_arm=left_arm,
            right_arm=right_arm,
            separation=separation,
            obstacles=obstacles
        )
        
        # Override base positions to match user's dragged positions
        dual_arm.left_base = np.array([left_base_x, 0.0])
        dual_arm.right_base = np.array([right_base_x, 0.0])
        
        # Create handoff planner
        print(f"[Handoff Planning] Creating HandoffPlanner...", flush=True)
        handoff_planner = HandoffPlanner(dual_arm)
        
        # Determine strategy immediately and update job
        strategy = handoff_planner.determine_arms(item_start, item_goal)
        print(f"[Handoff Planning] Strategy: {strategy}", flush=True)
        
        with job_lock:
            if job_id in jobs:
                jobs[job_id]['strategy'] = strategy
                jobs[job_id]['progress']['current_phase'] = 'Strategy determined'
        
        # Find handoff point if needed
        if strategy['needs_handoff']:
            handoff_point = handoff_planner.find_handoff_point(item_start, item_goal, obstacles)
            print(f"[Handoff Planning] Handoff point: {handoff_point}", flush=True)
            
            with job_lock:
                if job_id in jobs:
                    jobs[job_id]['handoff_point'] = handoff_point.tolist()
                    jobs[job_id]['progress']['current_phase'] = f'Planning {strategy["grab_arm"]} arm (grab phase)'
        else:
            with job_lock:
                if job_id in jobs:
                    jobs[job_id]['progress']['current_phase'] = f'Planning {strategy["chosen_arm"]} arm (single arm solution)'
        
        # Create progress callback
        def progress_callback(tree_data, iteration):
            with job_lock:
                if job_id in jobs:
                    jobs[job_id]['progress']['iterations'] = iteration
                    jobs[job_id]['progress']['tree_size'] = len(tree_data)
                    # Store tree snapshot EVERY iteration for real-time visualization
                    jobs[job_id]['tree'] = {
                        'nodes': [
                            {
                                'id': i,
                                'config': node['config'].tolist(),
                                'parent': node['parent'],
                                'cost': float(node['cost'])
                            }
                            for i, node in enumerate(tree_data)
                        ]
                    }
                    # Only print every 50 iterations to reduce log spam
                    if iteration % 50 == 0:
                        print(f"[Progress] Iteration {iteration}, tree size {len(tree_data)}", flush=True)
        
        # Execute planning (skip strategy/handoff determination since we already did it)
        print(f"[Handoff Planning] Starting RRT* planning phases...", flush=True)
        
        # Call the internal planning logic
        if not strategy['needs_handoff']:
            # Single arm planning
            chosen_arm = strategy['chosen_arm']
            path = handoff_planner._plan_single_arm_task(
                arm=chosen_arm,
                start_pos=item_start,
                goal_pos=item_goal,
                rrt_params=rrt_params,
                progress_callback=progress_callback
            )
            
            result = {
                'strategy': strategy,
                'handoff_point': None,
                'phases': [{
                    'arm': chosen_arm,
                    'from': 'start',
                    'to': 'goal',
                    'path_length': len(path) if path else 0
                }],
                'paths': [path] if path else [],
                'full_path': path
            }
        else:
            # Handoff planning - plan each phase
            # Phase 1: Grab
            with job_lock:
                if job_id in jobs:
                    jobs[job_id]['progress']['current_phase'] = f'Planning {strategy["grab_arm"]} arm (grab)'
            
            grab_path = handoff_planner._plan_single_arm_task(
                arm=strategy['grab_arm'],
                start_pos=item_start,
                goal_pos=handoff_point,
                rrt_params=rrt_params,
                progress_callback=progress_callback
            )
            
            if not grab_path:
                print("[Handoff Planning] Grab phase failed", flush=True)
                result = None
            else:
                # Phase 2: Delivery
                with job_lock:
                    if job_id in jobs:
                        jobs[job_id]['progress']['current_phase'] = f'Planning {strategy["delivery_arm"]} arm (delivery)'
                        jobs[job_id]['progress']['iterations'] = 0  # Reset for phase 2
                
                delivery_path = handoff_planner._plan_single_arm_task(
                    arm=strategy['delivery_arm'],
                    start_pos=handoff_point,
                    goal_pos=item_goal,
                    rrt_params=rrt_params,
                    progress_callback=progress_callback
                )
                
                if not delivery_path:
                    print("[Handoff Planning] Delivery phase failed", flush=True)
                    result = None
                else:
                    # Stitch paths
                    full_path = handoff_planner._stitch_paths(grab_path, delivery_path, strategy)
                    
                    result = {
                        'strategy': strategy,
                        'handoff_point': handoff_point,
                        'phases': [
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
                        ],
                        'paths': [grab_path, delivery_path],
                        'full_path': full_path
                    }
        
        
        end_time = time.time()
        
        # Update job with results
        with job_lock:
            if job_id in jobs:
                if result and result.get('full_path'):
                    jobs[job_id]['status'] = 'completed'
                    jobs[job_id]['end_time'] = end_time
                    jobs[job_id]['planning_time'] = end_time - jobs[job_id]['start_time']
                    jobs[job_id]['strategy'] = result['strategy']
                    jobs[job_id]['handoff_point'] = result['handoff_point'].tolist() if result['handoff_point'] is not None else None
                    jobs[job_id]['phases'] = result['phases']
                    
                    # Store path
                    path = result['full_path']
                    jobs[job_id]['path'] = [config.tolist() for config in path]
                    jobs[job_id]['path_length'] = len(path)
                    
                    # Compute end-effector positions for path
                    path_with_ee = []
                    for config_item in path:
                        left_config, right_config = dual_arm._split_configuration(config_item)
                        left_ee = dual_arm.left_arm.forward_kinematics(*left_config)
                        right_ee = dual_arm.right_arm.forward_kinematics(*right_config)
                        
                        path_with_ee.append({
                            'config': config_item.tolist(),
                            'left_ee': (dual_arm.left_base + left_ee).tolist(),
                            'right_ee': (dual_arm.right_base + right_ee).tolist()
                        })
                    
                    jobs[job_id]['path_with_ee'] = path_with_ee
                    jobs[job_id]['progress']['goal_found'] = True
                    
                    print(f"[Handoff Planning] SUCCESS - Path found with {len(path)} waypoints", flush=True)
                    print(f"  Strategy: {result['strategy']}", flush=True)
                    print(f"  Handoff point: {result['handoff_point']}", flush=True)
                    print(f"  Phases: {result['phases']}", flush=True)
                else:
                    jobs[job_id]['status'] = 'failed'
                    jobs[job_id]['error'] = 'No path found'
                    print(f"[Handoff Planning] FAILED - No path found", flush=True)
                    
    except Exception as e:
        import traceback
        error_trace = traceback.format_exc()
        print(f"[ERROR] Handoff planning exception:", flush=True)
        print(error_trace, flush=True)
        
        with job_lock:
            if job_id in jobs:
                jobs[job_id]['status'] = 'failed'
                jobs[job_id]['error'] = str(e)
                jobs[job_id]['error_trace'] = error_trace


def planning_worker(job_id: str, config: Dict):
    """Background worker for planning."""
    try:
        # Update job status
        with job_lock:
            jobs[job_id]['status'] = 'running'
            jobs[job_id]['start_time'] = time.time()
        
        # Extract configuration
        arm_type = config['arm_type']
        start_config = np.array(config['start'])
        goal_config = np.array(config['goal'])
        obstacles = config.get('obstacles', [])
        separation = config.get('separation', 2.0)
        
        print(f"[Planning] Arm type: {arm_type}")
        print(f"[Planning] Start config: {start_config}")
        print(f"[Planning] Goal config: {goal_config}")
        print(f"[Planning] Separation: {separation}")
        print(f"[Planning] Obstacles: {len(obstacles)}")
        
        # Planning parameters
        max_iterations = config.get('max_iterations', 5000)
        step_size = config.get('step_size', 0.15)
        goal_threshold = config.get('goal_threshold', 0.2)
        
        # Optimization parameters
        use_kdtree = config.get('use_kdtree', True)
        workspace_weight = config.get('workspace_weight', 0.3)
        use_adaptive_step = config.get('use_adaptive_step', True)
        
        # Create arms
        arm_params = config.get('arm_params', {})
        left_arm = create_arm(arm_type, {'name': 'LeftArm', **arm_params})
        right_arm = create_arm(arm_type, {'name': 'RightArm', **arm_params})
        
        # Create dual-arm system
        dual_arm = DualArm(
            left_arm=left_arm,
            right_arm=right_arm,
            separation=separation,
            obstacles=obstacles
        )
        
        # Validate start and goal configurations
        if not dual_arm.is_valid_configuration(start_config):
            error_msg = "Start configuration is invalid!"
            print(f"[ERROR] {error_msg}")
            print(f"[ERROR] Start config: {start_config}")
            with job_lock:
                jobs[job_id]['status'] = 'failed'
                jobs[job_id]['error'] = error_msg
            return
        
        if not dual_arm.is_valid_configuration(goal_config):
            error_msg = "Goal configuration is invalid!"
            print(f"[ERROR] {error_msg}")
            print(f"[ERROR] Goal config: {goal_config}")
            
            # Detailed debugging
            num_joints = left_arm.get_num_joints()
            left_config = goal_config[:num_joints]
            right_config = goal_config[num_joints:]
            
            print(f"[DEBUG] Left goal config: {left_config}")
            print(f"[DEBUG] Left valid: {left_arm.is_valid_configuration(*left_config)}")
            print(f"[DEBUG] Right goal config: {right_config}")
            print(f"[DEBUG] Right valid: {right_arm.is_valid_configuration(*right_config)}")
            
            # Check FK to see where arms end up
            try:
                left_ee = left_arm.forward_kinematics(*left_config)
                right_ee = right_arm.forward_kinematics(*right_config)
                print(f"[DEBUG] Left EE (from base): {left_ee}")
                print(f"[DEBUG] Right EE (from base): {right_ee}")
                print(f"[DEBUG] Left base: {dual_arm.left_base}")
                print(f"[DEBUG] Right base: {dual_arm.right_base}")
            except Exception as fk_error:
                print(f"[DEBUG] FK error: {fk_error}")
            
            with job_lock:
                jobs[job_id]['status'] = 'failed'
                jobs[job_id]['error'] = f"{error_msg} Details: Left valid={left_arm.is_valid_configuration(*left_config)}, Right valid={right_arm.is_valid_configuration(*right_config)}"
            return
        
        print(f"[SUCCESS] Both start and goal configurations are valid")
        
        # Create planner
        planner = RRTStar(
            dual_arm,
            max_iterations=max_iterations,
            step_size=step_size,
            goal_threshold=goal_threshold,
            use_kdtree=use_kdtree,
            workspace_weight=workspace_weight,
            use_adaptive_step=use_adaptive_step,
            verbose=False
        )
        
        # Progress callback - update every 10 iterations
        def progress_callback(tree, iteration):
            if iteration % 10 == 0:  # Update every 10 iterations
                with job_lock:
                    if job_id in jobs:
                        jobs[job_id]['progress']['iterations'] = iteration
                        jobs[job_id]['progress']['tree_size'] = len(tree)
                        # Store tree snapshot every 100 iterations for visualization
                        if iteration % 100 == 0:
                            jobs[job_id]['current_tree'] = planner._serialize_tree(tree)
        
        # Tree callback for visualization
        def tree_callback(tree_data):
            with job_lock:
                if job_id in jobs:
                    jobs[job_id]['tree_snapshots'].append(tree_data)
        
        # Plan
        path = planner.plan(
            start_config,
            goal_config,
            progress_callback=progress_callback
        )
        
        end_time = time.time()
        
        # Update job with results
        with job_lock:
            if job_id in jobs:
                jobs[job_id]['status'] = 'completed' if path else 'failed'
                jobs[job_id]['end_time'] = end_time
                jobs[job_id]['planning_time'] = end_time - jobs[job_id]['start_time']
                
                if path:
                    # Store path
                    jobs[job_id]['path'] = [config.tolist() for config in path]
                    jobs[job_id]['path_length'] = len(path)
                    
                    # Compute end-effector positions for path
                    path_with_ee = []
                    for config in path:
                        left_config, right_config = dual_arm._split_configuration(config)
                        left_ee = dual_arm.left_arm.forward_kinematics(*left_config)
                        right_ee = dual_arm.right_arm.forward_kinematics(*right_config)
                        
                        path_with_ee.append({
                            'config': config.tolist(),
                            'left_ee': (dual_arm.left_base + left_ee).tolist(),
                            'right_ee': (dual_arm.right_base + right_ee).tolist()
                        })
                    
                    jobs[job_id]['path_with_ee'] = path_with_ee
                    
                    # Get final tree state
                    jobs[job_id]['final_tree'] = planner._serialize_tree(planner._last_tree) if hasattr(planner, '_last_tree') else None
                
                jobs[job_id]['progress']['goal_found'] = path is not None
                
    except Exception as e:
        with job_lock:
            if job_id in jobs:
                jobs[job_id]['status'] = 'failed'
                jobs[job_id]['error'] = str(e)


# API Endpoints

@app.route('/api/plan', methods=['POST'])
def start_planning():
    """Start a new planning job."""
    try:
        config = request.json
        
        # Validate required fields
        required = ['arm_type', 'start', 'goal']
        for field in required:
            if field not in config:
                return jsonify({'error': f'Missing required field: {field}'}), 400
        
        # Generate job ID
        job_id = str(uuid.uuid4())
        
        # Initialize job
        with job_lock:
            jobs[job_id] = {
                'job_id': job_id,
                'status': 'queued',
                'config': config,
                'progress': {
                    'iterations': 0,
                    'max_iterations': config.get('max_iterations', 5000),
                    'tree_size': 0,
                    'goal_found': False
                },
                'tree_snapshots': [],
                'current_tree': None,  # Current tree for visualization
                'path': None,
                'path_with_ee': None,
                'final_tree': None,
                'error': None
            }
        
        # Start planning in background thread
        thread = threading.Thread(target=planning_worker, args=(job_id, config))
        thread.daemon = True
        thread.start()
        
        return jsonify({
            'job_id': job_id,
            'status': 'queued'
        })
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/status/<job_id>', methods=['GET'])
def get_status(job_id):
    """Get status of a planning job."""
    with job_lock:
        if job_id not in jobs:
            return jsonify({'error': 'Job not found'}), 404
        
        job = jobs[job_id]
        response = {
            'job_id': job_id,
            'status': job['status'],
            'progress': job['progress'],
            'config': job['config'],
            'planning_time': job.get('planning_time'),
            'error': job.get('error')
        }
        
        # Add handoff-specific data if available
        if job.get('planning_mode') == 'handoff':
            response['strategy'] = job.get('strategy')
            response['handoff_point'] = job.get('handoff_point')
            response['phases'] = job.get('phases')
        
        return jsonify(response)


@app.route('/api/tree/<job_id>', methods=['GET'])
def get_tree(job_id):
    """Get RRT* tree data for visualization."""
    with job_lock:
        if job_id not in jobs:
            return jsonify({'error': 'Job not found'}), 404
        
        job = jobs[job_id]
        
        # Return tree data - check multiple possible keys for compatibility
        # 'tree' is used by handoff planner, 'current_tree' by regular planner
        tree_data = job.get('tree') or job.get('current_tree') or job.get('final_tree')
        if not tree_data and job.get('tree_snapshots'):
            tree_data = job['tree_snapshots'][-1]
        
        if not tree_data:
            return jsonify({'nodes': [], 'path_nodes': []})
        
        return jsonify(tree_data)


@app.route('/api/path/<job_id>', methods=['GET'])
def get_path(job_id):
    """Get solution path."""
    with job_lock:
        if job_id not in jobs:
            return jsonify({'error': 'Job not found'}), 404
        
        job = jobs[job_id]
        
        if job['status'] != 'completed' or not job.get('path_with_ee'):
            return jsonify({'path': [], 'cost': 0, 'length': 0})
        
        return jsonify({
            'path': job['path_with_ee'],
            'cost': 0,  # Could calculate from tree if needed
            'length': job['path_length']
        })


@app.route('/api/validate-config', methods=['POST'])
def validate_config():
    """Validate an arm configuration with detailed error messages."""
    try:
        data = request.json
        arm_type = data.get('arm_type')
        config = np.array(data.get('config', []))
        obstacles = data.get('obstacles', [])
        separation = data.get('separation', 2.0)
        
        # Create arms
        arm_params = data.get('arm_params', {})
        left_arm = create_arm(arm_type, {'name': 'LeftArm', **arm_params})
        right_arm = create_arm(arm_type, {'name': 'RightArm', **arm_params})
        
        # Create dual-arm system
        dual_arm = DualArm(
            left_arm=left_arm,
            right_arm=right_arm,
            separation=separation,
            obstacles=obstacles
        )
        
        # Detailed validation
        reasons = []
        
        # Split configuration
        num_joints = left_arm.get_num_joints()
        if len(config) != num_joints * 2:
            return jsonify({
                'valid': False,
                'reasons': [f'Invalid config length: expected {num_joints*2}, got {len(config)}']
            })
        
        left_config = config[:num_joints]
        right_config = config[num_joints:]
        
        # Check individual arms first
        left_valid = left_arm.is_valid_configuration(*left_config)
        right_valid = right_arm.is_valid_configuration(*right_config)
        
        if not left_valid:
            # Try to get FK to see where it's trying to reach
            try:
                left_ee = left_arm.forward_kinematics(*left_config)
                left_global = dual_arm.left_base + left_ee
                reasons.append(f'Left arm invalid: trying to reach {left_global} (may be out of reach)')
            except:
                reasons.append('Left arm configuration invalid (joint limits or unreachable)')
        
        if not right_valid:
            try:
                right_ee = right_arm.forward_kinematics(*right_config)
                right_global = dual_arm.right_base + right_ee
                reasons.append(f'Right arm invalid: trying to reach {right_global} (may be out of reach)')
            except:
                reasons.append('Right arm configuration invalid (joint limits or unreachable)')
        
        # Use dual_arm's full validation if individual arms are OK
        if not reasons:
            if not dual_arm.is_valid_configuration(config):
                reasons.append('Configuration has collisions (arms or obstacles)')
        
        is_valid = len(reasons) == 0
        
        return jsonify({
            'valid': is_valid,
            'reasons': reasons if not is_valid else ['Configuration is valid'],
            'config': config.tolist()
        })
        
    except Exception as e:
        import traceback
        error_trace = traceback.format_exc()
        print(f"[ERROR] Validation exception: {error_trace}")
        return jsonify({
            'valid': False,
            'error': str(e),
            'traceback': error_trace
        }), 500


@app.route('/api/arm-types', methods=['GET'])
def get_arm_types():
    """Get available arm types and their parameters."""
    return jsonify({
        'arm_types': [
            {
                'id': '2-link',
                'name': '2-Link Arm',
                'num_joints': 2,
                'default_params': {'L1': 1.0, 'L2': 0.7},
                'config_dim': 4
            },
            {
                'id': '3-link',
                'name': '3-Link Arm',
                'num_joints': 3,
                'default_params': {'L1': 1.0, 'L2': 0.8, 'L3': 0.6},
                'config_dim': 6
            },
            {
                'id': '6-link',
                'name': '6-Link Arm',
                'num_joints': 6,
                'default_params': {
                    'L1': 0.5, 'L2': 0.4, 'L3': 0.3,
                    'L4': 0.25, 'L5': 0.2, 'L6': 0.15
                },
                'config_dim': 12
            }
        ]
    })


@app.route('/api/plan-handoff', methods=['POST'])
def plan_with_handoff():
    """
    Start a planning job using intelligent handoff planning.
    
    Request body:
    {
        'arm_type': '2-link',
        'item_start': [x, y],
        'item_goal': [x, y],
        'left_base_x': -1.0,
        'right_base_x': 1.0,
        'obstacles': [...],
        'rrt_params': {
            'max_iterations': 5000,
            'step_size': 0.15,
            ...
        }
    }
    """
    try:
        config = request.json
        
        # Validate required fields
        required = ['arm_type', 'item_start', 'item_goal', 'left_base_x', 'right_base_x']
        for field in required:
            if field not in config:
                return jsonify({'error': f'Missing required field: {field}'}), 400
        
        # Generate job ID
        job_id = str(uuid.uuid4())
        
        # Initialize job with handoff-specific data
        with job_lock:
            jobs[job_id] = {
                'job_id': job_id,
                'status': 'queued',
                'config': config,
                'planning_mode': 'handoff',
                'progress': {
                    'iterations': 0,
                    'max_iterations': config.get('rrt_params', {}).get('max_iterations', 5000),
                    'tree_size': 0,
                    'goal_found': False,
                    'current_phase': None
                },
                'strategy': None,
                'handoff_point': None,
                'tree_snapshots': [],
                'current_tree': None,
                'path': None,
                'path_with_ee': None,
                'final_tree': None,
                'phases': [],
                'error': None
            }
        
        # Start planning in background thread
        thread = threading.Thread(target=handoff_planning_worker, args=(job_id, config))
        thread.daemon = True
        thread.start()
        
        return jsonify({
            'job_id': job_id,
            'status': 'queued',
            'planning_mode': 'handoff'
        })
        
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/health', methods=['GET'])
def health_check():
    """Health check endpoint."""
    return jsonify({'status': 'ok', 'jobs_count': len(jobs)})


if __name__ == '__main__':
    print("Starting Flask API server...")
    print("API will be available at http://localhost:5001")
    app.run(debug=True, host='0.0.0.0', port=5001)

