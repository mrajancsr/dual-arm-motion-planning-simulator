# Testing & Optimization Guide

Complete guide for testing the dual-arm motion planning simulator and using RRT* optimizations.

---

## 🚀 Quick Start

```bash
# Quick test (verify setup)
./run_tests.sh quick

# Full test suite
./run_tests.sh all

# Interactive GUI
./run_tests.sh gui

# See all options
./run_tests.sh help
```

**Always use the virtual environment:** `./venv/bin/python` or `./run_tests.sh`

---

## 📋 Table of Contents

1. [Test Files Overview](#test-files-overview)
2. [Running Tests](#running-tests)
3. [RRT* Optimizations](#rrt-optimizations)
4. [Performance Tuning](#performance-tuning)
5. [Troubleshooting](#troubleshooting)

---

## Test Files Overview

### Primary Test Suite

#### **`test_comprehensive.py`** ⭐
**Consolidated test for all arm types and optimizations.**

```bash
./venv/bin/python test_comprehensive.py --quick
./venv/bin/python test_comprehensive.py --test optimization
```

**Features:**
- Tests 2-link, 3-link, and 6-link arms
- Compares all optimization configurations
- Tests obstacle avoidance
- Tests parallel planning
- Provides performance benchmarks

### Test Wrapper Script

#### **`run_tests.sh`**
**Convenient command-line interface for all tests.**

```bash
./run_tests.sh quick          # Quick test
./run_tests.sh optimization   # Compare optimizations
./run_tests.sh benchmark      # Benchmark all arms
./run_tests.sh obstacles      # Test with obstacles
./run_tests.sh parallel       # Test parallel planning
./run_tests.sh gui           # Interactive obstacle GUI
./run_tests.sh help          # Show all commands
```

### Individual Test Files

| File | Purpose | Command |
|------|---------|---------|
| `test_rrt_star_obstacles.py` | Interactive GUI | `./run_tests.sh gui` |
| `test_rrt_star_animation.py` | Animated visualization | `./venv/bin/python test_rrt_star_animation.py` |
| `test_rrt_star.py` | Basic RRT* test | `./venv/bin/python test_rrt_star.py` |
| `test_end_to_end.py` | Complete workflow | `./venv/bin/python test_end_to_end.py` |

---

## Running Tests

### Quick Verification

**Start here to verify your setup:**
```bash
./run_tests.sh quick
```

Expected output:
```
🚀 Quick Test Mode

1. Baseline:
✓ Path found: 23 waypoints in 1.567s

2. All Optimizations:
✓ Path found: 15 waypoints in 0.234s

📊 Results:
   Speedup: 6.70x
   Path improvement: 34.8%
```

### Optimization Comparison

**Compare all optimization configurations:**
```bash
./run_tests.sh optimization

# Or specific arm types
./venv/bin/python test_comprehensive.py --test optimization --arm-types 2-link 3-link
```

Compares:
- `baseline`: No optimizations
- `kdtree-only`: Only KD-tree enabled
- `distance-only`: Only improved distance metric
- `adaptive-only`: Only adaptive step size
- `all-optimizations`: All optimizations enabled

### Benchmark All Arms

**Test all arm types with best settings:**
```bash
./run_tests.sh benchmark
```

Expected results:
```
✓ 2-link     ( 4D):  0.234s,  15 waypoints
✓ 3-link     ( 6D):  0.567s,  23 waypoints
✓ 6-link     (12D):  2.345s,  45 waypoints
```

### Obstacle Avoidance

**Test planning with obstacles:**
```bash
./run_tests.sh obstacles
```

Uses all optimizations (especially adaptive stepping for better obstacle navigation).

### Parallel Planning

**Test multi-core planning:**
```bash
./run_tests.sh parallel
```

Runs 4 parallel planners and returns best result.

### Interactive GUI

**Visual testing and debugging:**
```bash
./run_tests.sh gui
```

Features:
- Drag start/goal positions
- Drag obstacles around
- Click "Run RRT*" to plan
- See planning progress in real-time

### Command-Line Options

```bash
# Test specific arm types
./venv/bin/python test_comprehensive.py --arm-types 2-link 3-link

# Run specific test
./venv/bin/python test_comprehensive.py --test optimization

# Include parallel tests
./venv/bin/python test_comprehensive.py --parallel

# Quick mode
./venv/bin/python test_comprehensive.py --quick
```

Available options:
- `--arm-types`: Choose from `2-link`, `3-link`, `6-link`
- `--test`: Choose from `optimization`, `benchmark`, `obstacles`, `parallel`, `all`
- `--quick`: Run quick comparison (2-link only)
- `--parallel`: Include parallel planning tests

---

## RRT* Optimizations

All optimizations are implemented and enabled by default in [`src/rrt_star.py`](src/rrt_star.py).

### 1. KD-Tree Nearest Neighbor Search

**What it does:** Replaces O(n) linear search with O(log n) KD-tree search.

**Expected speedup:** 10-100x for large trees (>1000 nodes)

**Usage:**
```python
from src import DualArm, RRTStar

planner = RRTStar(
    dual_arm,
    use_kdtree=True,  # Default: True
    ...
)
```

### 2. Improved Distance Metrics

**What it does:** Combines three improvements:
- **Angular wrapping**: Handles ±π discontinuity (no more "going the long way")
- **Joint weighting**: Base joints weighted higher than end-effector joints
- **Workspace distance**: Considers end-effector position in task space

**Expected improvement:** Significantly shorter paths, fewer rotational inefficiencies

**Usage:**
```python
planner = RRTStar(
    dual_arm,
    workspace_weight=0.3,  # 0.0-1.0, default: 0.3
    ...
)
```

Weighting guide:
- `0.0`: Pure joint-space distance (baseline)
- `0.3`: Balanced (recommended, default)
- `0.5`: More workspace emphasis
- `1.0`: Pure workspace distance

### 3. Adaptive Step Size

**What it does:** Adjusts step size based on obstacle proximity.

**Expected improvement:** Better obstacle navigation, fewer collision failures

**Usage:**
```python
planner = RRTStar(
    dual_arm,
    use_adaptive_step=True,  # Default: True
    step_size=0.15,          # Base step size
    ...
)
```

Behavior:
- Near obstacles (<0.2m): 50% step size
- Far from obstacles (>0.5m): 150% step size
- Medium distance: 100% step size

### 4. Bounding Box Collision Pre-Check

**What it does:** Fast AABB check before expensive segment intersection tests.

**Expected speedup:** 20-40% in collision checking

**Usage:** Always enabled automatically in `DualArm.is_valid_configuration()`. No configuration needed.

### 5. Parallel RRT* Planning

**What it does:** Runs multiple planners in parallel, returns best path.

**Expected speedup:** 3-4x with 4 cores

**Usage:**
```python
from src import ParallelRRTStar

parallel_planner = ParallelRRTStar(
    dual_arm,
    num_workers=4,        # Number of parallel workers
    max_iterations=3000,
    step_size=0.15,
    verbose=True
)

path = parallel_planner.plan(start_config, goal_config, verbose=True)
```

**Mac Compatibility:** Uses 'spawn' method, safe on all macOS versions.

### Complete Example

```python
import numpy as np
from src import DualArm, TwoLinkArm, RRTStar, ParallelRRTStar

# Create dual-arm system
left_arm = TwoLinkArm(L1=1.0, L2=0.7, name="LeftArm")
right_arm = TwoLinkArm(L1=1.0, L2=0.7, name="RightArm")

# Add obstacles (optional)
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
    use_kdtree=True,          # KD-tree search
    workspace_weight=0.3,     # Hybrid distance
    use_adaptive_step=True,   # Adaptive stepping
    verbose=True
)
path = planner.plan(start_config, goal_config)

# Option 2: Parallel planning (faster)
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

## Performance Tuning

### For 2-Link Arms
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

### For 3-Link Arms
```python
planner = RRTStar(
    dual_arm,
    max_iterations=4000,
    step_size=0.12,
    goal_threshold=0.25,
    workspace_weight=0.3,
    use_adaptive_step=True
)
```

### For 6-Link Arms
```python
planner = RRTStar(
    dual_arm,
    max_iterations=8000,      # More iterations
    step_size=0.08,           # Smaller steps
    goal_threshold=0.3,       # Looser threshold
    workspace_weight=0.4,     # More workspace emphasis
    use_adaptive_step=True
)
```

### With Dense Obstacles
```python
planner = RRTStar(
    dual_arm,
    max_iterations=10000,     # More iterations
    step_size=0.1,            # Smaller base step
    use_adaptive_step=True,   # Critical!
    workspace_weight=0.2      # More joint-space focus
)
```

### Disable Optimizations (Baseline)
```python
planner = RRTStar(
    dual_arm,
    use_kdtree=False,          # Linear search
    workspace_weight=0.0,      # Pure Euclidean distance
    use_adaptive_step=False,   # Fixed step size
    ...
)
```

---

## Performance Expectations

Typical results with all optimizations:

| Arm Type | Config Dim | Planning Time | Path Length |
|----------|-----------|---------------|-------------|
| 2-link   | 4D        | 0.2-0.5s      | 10-20       |
| 3-link   | 6D        | 0.5-1.0s      | 15-30       |
| 6-link   | 12D       | 2.0-5.0s      | 30-60       |

Times vary based on:
- Start/goal configuration difficulty
- Presence of obstacles
- Random sampling variations

---

## Troubleshooting

### "ValueError: numpy.dtype size changed"

**Problem:** Using system Python with incompatible numpy/scipy versions.

**Solution:** Use virtual environment Python:
```bash
./venv/bin/python test_comprehensive.py  # ✓ Correct
python test_comprehensive.py            # ✗ Wrong
```

Or use wrapper:
```bash
./run_tests.sh quick  # Always uses venv
```

### "No module named 'src'"

**Problem:** Not in project root directory.

**Solution:**
```bash
cd /Users/kayacelebi/Projects/Columbia-git/dual-arm-motion-planning-simulator
./run_tests.sh quick
```

### Planning is slower than expected

**Possible causes:**
- KD-tree disabled: Check `use_kdtree=True`
- Too many iterations: Try reducing `max_iterations`
- Small step size: Increase `step_size`

**Solutions:**
```python
# Faster (less optimal)
planner = RRTStar(dual_arm, step_size=0.2, max_iterations=2000, ...)

# Or use parallel planning
parallel_planner = ParallelRRTStar(dual_arm, num_workers=4, ...)
```

### Paths are still inefficient

**Solutions:**
- Increase `workspace_weight` (try 0.4 or 0.5)
- Run multiple times or use `ParallelRRTStar`
- Adjust `goal_threshold` (looser = easier to reach)

### Planning fails consistently

**Solutions:**
```python
# Adjust parameters
planner = RRTStar(
    dual_arm,
    max_iterations=10000,     # More attempts
    goal_threshold=0.3,       # Looser goal
    step_size=0.08,           # Finer resolution
    ...
)
```

### Parallel planning doesn't work on Mac

**Should work automatically** with 'spawn' method.

**If issues persist:**
- Check Python version (3.7+ recommended)
- Reduce `num_workers` to 2
- Run sequential planner instead

---

## Best Practices

1. **Always use virtual environment:**
   ```bash
   ./venv/bin/python script.py
   # Or
   ./run_tests.sh command
   ```

2. **Start with quick test:**
   ```bash
   ./run_tests.sh quick
   ```

3. **Test incrementally:**
   - Test one arm type before all
   - Test one optimization at a time when debugging

4. **Save results for analysis:**
   ```bash
   ./run_tests.sh all > results.txt
   ```

5. **Run multiple trials:**
   RRT* is randomized, run 3-5 times for statistical significance

6. **Use appropriate parameters:**
   - Higher dimensions → more iterations, smaller steps
   - Obstacles → enable adaptive stepping
   - Speed priority → parallel planning

---

## Additional Resources

### Implementation Details

**Files modified:**
- [`src/rrt_star.py`](src/rrt_star.py) - RRT* optimizations (Phases 1-3)
- [`src/dual_arm_system.py`](src/dual_arm_system.py) - Bounding box (Phase 4)
- [`src/parallel_planner.py`](src/parallel_planner.py) - Parallel planning (Phase 5)

**Dependencies:**
- `scipy.spatial.KDTree` - Already in requirements.txt

### Future Enhancements

Consider implementing:
1. **Path smoothing** - Remove unnecessary waypoints
2. **Informed RRT*** - Use heuristics to guide sampling
3. **Bidirectional RRT*** - Grow trees from both ends
4. **Dynamic obstacles** - Handle moving obstacles

---

## Quick Reference

### Common Commands

```bash
# Quick test
./run_tests.sh quick

# Full test suite
./run_tests.sh all

# Compare optimizations
./run_tests.sh optimization

# Test with obstacles
./run_tests.sh obstacles

# Parallel planning
./run_tests.sh parallel

# Interactive GUI
./run_tests.sh gui

# Help
./run_tests.sh help
```

### Test Parameters

```python
# RRT* Parameters
max_iterations=5000       # Number of planning iterations
step_size=0.15           # Step size in radians
goal_threshold=0.2       # Distance to goal for success

# Optimization Parameters
use_kdtree=True          # Enable KD-tree (default: True)
workspace_weight=0.3     # Workspace vs joint-space (0-1)
use_adaptive_step=True   # Adaptive stepping (default: True)

# Parallel Parameters
num_workers=4            # Number of parallel workers
```

---

## Summary

**Quick Start:**
```bash
./run_tests.sh quick
```

**Full Testing:**
```bash
./run_tests.sh all
```

**Best Performance:**
```python
planner = RRTStar(
    dual_arm,
    use_kdtree=True,
    workspace_weight=0.3,
    use_adaptive_step=True,
    verbose=True
)
```

For detailed documentation, see:
- [README.md](README.md) - Project overview
- Source code comments in `src/rrt_star.py`

