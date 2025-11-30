# RRT* Optimization Guide

This guide explains the optimizations implemented in the RRT* planner and how to use them.

## Overview of Optimizations

All optimizations have been implemented in [`src/rrt_star.py`](src/rrt_star.py) and [`src/dual_arm_system.py`](src/dual_arm_system.py).

### Phase 1: KD-Tree for Nearest Neighbor Search ✓

**What it does**: Replaces O(n) linear search with O(log n) KD-tree search for finding nearest nodes in the tree.

**Expected speedup**: 10-100x for large trees (>1000 nodes)

**How to use**:
```python
from src import DualArm, TwoLinkArm, RRTStar

dual_arm = DualArm(...)
planner = RRTStar(
    dual_arm,
    use_kdtree=True,  # Enable KD-tree (default: True)
    ...
)
```

To disable (for comparison):
```python
planner = RRTStar(dual_arm, use_kdtree=False, ...)
```

---

### Phase 2: Improved Distance Metrics ✓

**What it does**: Combines three improvements:
1. **Angular wrapping**: Handles ±π discontinuity (no more "going the long way")
2. **Joint weighting**: Base joints weighted higher than end-effector joints
3. **Workspace distance**: Considers end-effector position in workspace

**Expected improvement**: Significantly shorter paths, fewer rotational inefficiencies

**How to use**:
```python
planner = RRTStar(
    dual_arm,
    workspace_weight=0.3,  # 30% workspace, 70% joint-space (default: 0.3)
    ...
)
```

To adjust weighting:
- `workspace_weight=0.0`: Pure joint-space distance (baseline)
- `workspace_weight=0.3`: Balanced (recommended, default)
- `workspace_weight=0.5`: More emphasis on workspace distance
- `workspace_weight=1.0`: Pure workspace distance

---

### Phase 3: Adaptive Step Size ✓

**What it does**: Uses smaller steps near obstacles, larger steps in open space.

**Expected improvement**: Better navigation around obstacles, fewer collision check failures

**How to use**:
```python
planner = RRTStar(
    dual_arm,
    use_adaptive_step=True,  # Enable adaptive stepping (default: True)
    step_size=0.15,  # Base step size
    ...
)
```

Adaptive behavior:
- Clearance < 0.2m: step_size × 0.5 (small steps near obstacles)
- Clearance > 0.5m: step_size × 1.5 (large steps in open space)
- Otherwise: step_size (normal steps)

---

### Phase 4: Bounding Box Collision Pre-Check ✓

**What it does**: Adds fast AABB (axis-aligned bounding box) check before expensive segment intersection tests.

**Expected speedup**: 20-40% in collision checking

**How to use**: Always enabled automatically in `DualArm.is_valid_configuration()`. No configuration needed.

---

### Phase 5: Parallel RRT* Planning ✓

**What it does**: Runs multiple RRT* instances in parallel and returns the best path.

**Expected speedup**: 3-4x with 4 cores (but individual paths may vary)

**How to use**:
```python
from src import ParallelRRTStar

parallel_planner = ParallelRRTStar(
    dual_arm,
    num_workers=4,  # Number of parallel workers
    max_iterations=3000,
    step_size=0.15,
    verbose=True  # Print which worker found best path
)

path = parallel_planner.plan(start_config, goal_config, verbose=True)
```

**Mac Compatibility**: Uses 'spawn' method for multiprocessing, safe on all macOS versions.

---

## Complete Example

```python
import numpy as np
from src import DualArm, TwoLinkArm, RRTStar, ParallelRRTStar

# Create dual-arm system
left_arm = TwoLinkArm(L1=1.0, L2=0.7, name="LeftArm")
right_arm = TwoLinkArm(L1=1.0, L2=0.7, name="RightArm")

# Optional: add obstacles
obstacles = [{'type': 'circle', 'center': [-0.5, 0.8], 'radius': 0.3}]
dual_arm = DualArm(
    left_arm=left_arm, 
    right_arm=right_arm, 
    separation=2.0,
    obstacles=obstacles
)

# Define start and goal
start_config = np.array([0.5, 0.3, -0.2, 0.4])
goal_config = np.array([1.0, -0.5, 0.8, -0.3])

# Option 1: Single-threaded with all optimizations
planner = RRTStar(
    dual_arm,
    max_iterations=5000,
    step_size=0.15,
    goal_threshold=0.2,
    use_kdtree=True,           # KD-tree search
    workspace_weight=0.3,      # Hybrid distance metric
    use_adaptive_step=True,    # Adaptive stepping
    verbose=True
)
path = planner.plan(start_config, goal_config)

# Option 2: Parallel planning
parallel_planner = ParallelRRTStar(
    dual_arm,
    num_workers=4,
    max_iterations=3000,
    step_size=0.15,
    verbose=True
)
path = parallel_planner.plan(start_config, goal_config, verbose=True)
```

---

## Testing the Optimizations

Run the test suite:
```bash
./venv/bin/python test_optimizations.py
```

Or test individual files:
```bash
./venv/bin/python test_rrt_star.py
./venv/bin/python test_rrt_star_obstacles.py
```

---

## Performance Tuning

### For 2-Link Arms (faster planning)
```python
planner = RRTStar(
    dual_arm,
    max_iterations=3000,      # Fewer iterations needed
    step_size=0.2,            # Larger steps
    goal_threshold=0.2,       # Looser threshold
    workspace_weight=0.3,
    use_adaptive_step=True
)
```

### For 6-Link Arms (more careful planning)
```python
planner = RRTStar(
    dual_arm,
    max_iterations=8000,      # More iterations
    step_size=0.08,           # Smaller steps
    goal_threshold=0.3,       # Looser threshold (harder to reach exact config)
    workspace_weight=0.4,     # More workspace emphasis
    use_adaptive_step=True
)
```

### With Dense Obstacles
```python
planner = RRTStar(
    dual_arm,
    max_iterations=10000,     # More iterations to find path
    step_size=0.1,            # Smaller base step
    use_adaptive_step=True,   # Critical for obstacle navigation
    workspace_weight=0.2      # More joint-space focus
)
```

---

## Fallback to Baseline

If optimizations cause issues, you can disable them individually:

```python
# Baseline configuration (no optimizations)
planner = RRTStar(
    dual_arm,
    use_kdtree=False,          # Linear search
    workspace_weight=0.0,      # Pure Euclidean distance
    use_adaptive_step=False,   # Fixed step size
    ...
)
```

---

## Implementation Details

### Files Modified
- **[src/rrt_star.py](src/rrt_star.py)**: All RRT* optimizations (Phases 1-3)
- **[src/dual_arm_system.py](src/dual_arm_system.py)**: Bounding box pre-check (Phase 4)
- **[src/parallel_planner.py](src/parallel_planner.py)**: Parallel planning (Phase 5) - NEW FILE
- **[src/__init__.py](src/__init__.py)**: Export ParallelRRTStar

### Dependencies
- `scipy.spatial.KDTree`: Used for KD-tree nearest neighbor search (already in requirements.txt)

---

## Troubleshooting

### "ValueError: numpy.dtype size changed"
This indicates a numpy/scipy version mismatch. Use the project's virtual environment:
```bash
./venv/bin/python test_optimizations.py
```

### Planning is slower than expected
- Check if `use_kdtree=True` (should be default)
- Try increasing `step_size` for faster (but less optimal) planning
- Use `ParallelRRTStar` for faster results

### Paths are still inefficient
- Increase `workspace_weight` (try 0.4 or 0.5)
- Angular wrapping is always enabled - if paths still go the long way, it might be a local minimum issue
- Try running multiple times or use `ParallelRRTStar`

### Parallel planning doesn't work on Mac
- Should work automatically with 'spawn' method
- If issues persist, check Python version (3.7+ recommended)
- Reduce `num_workers` to 2 if system has limited cores

---

## Next Steps

Consider implementing:
1. **Path smoothing**: Post-process paths to remove unnecessary waypoints
2. **Informed RRT***: Use heuristics to guide sampling
3. **Bidirectional RRT***: Grow trees from both start and goal
4. **Dynamic obstacles**: Handle moving obstacles during planning

These are outlined in the original analysis document.

