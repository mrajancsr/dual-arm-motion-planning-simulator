# 🦾 Dual-Arm Motion Planning Simulator

A Python-based simulator for dual-arm robot motion planning using RRT* in configuration space. Supports 2-link, 3-link, and 6-link planar robot arms with collision detection and path planning.

---

## 🧠 Project Overview

This simulator is developed for the **Computational Robotics** course (Fall 2025). It demonstrates fundamental robotics principles including forward/inverse kinematics, configuration space visualization, and sampling-based motion planning (RRT*).

### ✨ Key Features
- Forward and Inverse Kinematics for 2-link, 3-link, and 6-link planar arms  
- Dual-arm configuration and independent motion control  
- Workspace and C-space (configuration space) visualization  
- RRT* (Rapidly-exploring Random Tree Star) motion planning with optimizations:
  - KD-tree nearest neighbor search (10-100x speedup)
  - Improved distance metrics (angular wrapping, joint weighting, workspace distance)
  - Adaptive step sizing for obstacle navigation
  - Parallel planning with multi-core support
- Collision detection and path validation (with bounding box optimization)
- Joint limit enforcement in IK solver

---

## 👥 Team Members
| Name | Role |
|------|------|
| **Rajan Subramanian** | Kinematics, C-space mapping, report writing |
| **Kaya Celebi** | Motion planning, visualization, presentation prep |
| **Nico Bykhovsky** | Motion Planning, visualization, presentation | 

---

## 📁 Module Structure

```
src/
├── __init__.py                 # Package initialization
├── arms/
│   ├── two_link_arm.py        # 2-link arm implementation
│   └── six_link_arm.py        # 6-link arm implementation
├── dual_arm_system.py         # Dual-arm system coordination & collision checking
├── robot_arm_base.py          # Base class for robot arms
├── rrt_star.py                # RRT* motion planning algorithm (C-space planning)
├── motion_planner.py          # High-level planner (workspace → C-space → RRT*)
├── simple_problem.py          # Simple problem generator
├── workspace_generator.py     # Workspace generation and visualization
├── cspace_generator.py        # Configuration space generation and visualization
├── problem_generator.py       # Legacy problem generator (object manipulation)
└── objects.py                 # Object definitions
```

### Key Files
- **`rrt_star.py`**: Core RRT* implementation - plans in C-space using `DualArm.is_valid_configuration()`
- **`motion_planner.py`**: High-level interface - converts workspace goals to C-space and plans
- **`dual_arm_system.py`**: Manages two arms and provides collision checking

---

## 🏗️ Architecture and End-to-End Process

### Data Flow

```
Workspace Goals (x₁, y₁), (x₂, y₂)
    ↓ [IK Conversion - MotionPlanner]
C-space Configs [θ₁, θ₂, θ₃, θ₄] → [θ₅, θ₆, θ₇, θ₈]
    ↓ [RRT* Planning]
Path: List of configs [[θ₁, θ₂, θ₃, θ₄], ..., [θ₅, θ₆, θ₇, θ₈]]
    ↓ [Execution]
Visualization in Workspace
```

### Component Responsibilities

**`DualArm` System**
- Manages two robot arms
- Provides `is_valid_configuration(config)` for collision checking
- Checks individual arm validity (joint limits, workspace) and inter-arm collisions
- Handles forward kinematics (C-space → workspace)

**`RRTStar` Planner**
- Plans paths in C-space
- Takes start/goal configs as numpy arrays
- Format: `[θ1_left, θ2_left, θ1_right, θ2_right]` for 2-link arms
- Format: `[θ1_left...θ6_left, θ1_right...θ6_right]` for 6-link arms
- Returns list of configs forming path
- Uses `DualArm.is_valid_configuration()` for validation

**`MotionPlanner`**
- High-level interface that bridges workspace goals with RRT*
- Converts workspace positions to C-space using IK
- Validates IK solutions before planning
- Handles multiple IK attempts with different initial guesses

### Key Design Decisions

1. **C-space Planning**: RRT* works in configuration space (joint angles), not workspace
2. **Separation of Concerns**: 
   - IK handles workspace ↔ C-space conversion
   - RRT* handles path planning in C-space
   - DualArm handles collision checking
3. **Generic Design**: Works with any arm type (2-link, 6-link, etc.)
4. **Joint Limit Enforcement**: IK solver clips joint angles to limits during iteration

---

## ⚙️ Setup Instructions

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```

### 2. Run Comprehensive Test Suite (Recommended) ⭐
```bash
# Quick test (2-link, baseline vs optimized)
./run_tests.sh quick

# Full test suite (all arm types, all optimizations)
./run_tests.sh all

# Or use Python directly
./venv/bin/python test_comprehensive.py --quick
```

The comprehensive test suite tests:
- 2-link, 3-link, and 6-link arms
- All optimization configurations
- Obstacle avoidance
- Parallel planning
- Performance benchmarks

See **[TESTING.md](TESTING.md)** for detailed testing guide and optimization usage.

### 3. Run Specific Tests
```bash
# Interactive obstacle planning GUI
./run_tests.sh gui

# Test 3-link arm with visualization
./run_tests.sh 3link

# Compare optimizations
./run_tests.sh optimization

# Benchmark all arm types
./run_tests.sh benchmark
```

### 4. Legacy Tests
```bash
# Original optimization tests
./venv/bin/python test_optimizations.py

# End-to-end workflow
./venv/bin/python test_end_to_end.py
```

---

## 🌐 Web Application

An interactive web-based visualization is now available! This provides a modern UI for planning, visualizing RRT* tree growth, and analyzing results in real-time.

### Quick Start

```bash
# Install web dependencies (one-time)
./venv/bin/pip install -r backend/requirements-web.txt
cd frontend && npm install && cd ..

# Start web app (backend + frontend)
./run_webapp.sh
```

Access the app at **http://localhost:5173**

### Features

- 🎯 **Interactive Planning**: Real-time RRT* visualization with handoff planning
- 🦾 **Multiple Arm Types**: Switch between 2-link, 3-link, and 6-link arms
- 🎨 **Visual Feedback**: See tree growth, solution paths, and handoff points
- ⚙️ **Live Configuration**: Adjust parameters on-the-fly
- 📊 **Performance Metrics**: View planning statistics with ETA
- 🤝 **Handoff Planning**: Intelligent arm assignment and handoff point calculation
- 🎯 **Item-Based Planning**: Drag item start/goal positions (not end-effectors)

### Handoff Planning

The web app supports intelligent handoff planning:
- Automatically determines which arm should grab and deliver items
- Calculates optimal handoff points in workspace intersection
- Executes phase-based RRT* planning (grab phase → handoff → delivery phase)
- Visualizes handoff strategy and point on canvas

### API Endpoints

- `POST /api/plan-handoff` - Start handoff-based planning
- `POST /api/plan` - Start traditional dual-arm planning
- `GET /api/status/:job_id` - Get planning status and progress
- `GET /api/tree/:job_id` - Get RRT* tree data for visualization
- `GET /api/path/:job_id` - Get solution path
- `POST /api/validate-config` - Validate arm configuration
- `GET /api/arm-types` - Get available arm types

See **[webapp/README.md](webapp/README.md)** for detailed API documentation and troubleshooting.

---

## 💻 Usage Examples

### High-Level Interface (Recommended)

```python
from src import DualArm, MotionPlanner, SimpleProblemGenerator
import numpy as np

# Create dual-arm system
dual_arm = DualArm(L1=1.0, L2=0.7, separation=2.0)

# Option 1: Generate random problem
prob_gen = SimpleProblemGenerator(dual_arm)
problem = prob_gen.generate_random_problem()

# Option 2: Create custom problem
problem = prob_gen.create_problem(
    left_start=np.array([-0.8, 0.5]),
    left_goal=np.array([-0.3, 1.2]),
    right_start=np.array([1.2, 0.5]),
    right_goal=np.array([1.7, 1.2])
)

# Create planner and plan
planner = MotionPlanner(dual_arm, max_iterations=3000, step_size=0.15)
path = planner.plan_from_workspace_goals(
    problem.left_start_pos,
    problem.left_goal_pos,
    problem.right_start_pos,
    problem.right_goal_pos
)

if path:
    print(f"Path found! Length: {len(path)} configurations")
else:
    print("Planning failed")
```

### Low-Level: Direct C-Space Planning

```python
from src import DualArm, RRTStar
import numpy as np

dual_arm = DualArm(L1=1.0, L2=0.7, separation=2.0)

# Define configs directly: [θ1_left, θ2_left, θ1_right, θ2_right]
start_config = np.array([0.5, 0.3, -0.2, 0.4])
goal_config = np.array([1.0, -0.5, 0.8, -0.3])

# Plan directly in C-space
planner = RRTStar(dual_arm, max_iterations=3000, step_size=0.15)
path = planner.plan(start_config, goal_config)
```

### With 6-Link Arms

```python
from src import DualArm, SixLinkArm, MotionPlanner
import numpy as np

# Create 6-link arms
left_arm = SixLinkArm(L1=0.5, L2=0.4, L3=0.3, L4=0.25, L5=0.2, L6=0.15)
right_arm = SixLinkArm(L1=0.5, L2=0.4, L3=0.3, L4=0.25, L5=0.2, L6=0.15)
dual_arm = DualArm(left_arm=left_arm, right_arm=right_arm, separation=2.0)

# Use high-level interface (handles IK conversion)
planner = MotionPlanner(dual_arm, max_iterations=8000, step_size=0.08, goal_threshold=0.2)
prob_gen = SimpleProblemGenerator(dual_arm)
problem = prob_gen.generate_random_problem()

path = planner.plan_from_workspace_goals(
    problem.left_start_pos, problem.left_goal_pos,
    problem.right_start_pos, problem.right_goal_pos
)
```

### Workspace Visualization

```python
from src import DualArm, WorkspaceGenerator, DualArmWorkspaceGenerator

# Single arm workspace
arm = TwoLinkArm(L1=1.0, L2=0.7)
ws_gen = WorkspaceGenerator(arm, resolution=100)
fig = ws_gen.plot_workspace(show_boundary=True, show_points=True)

# Dual arm workspace
dual_arm = DualArm(L1=1.0, L2=0.7, separation=2.0)
dual_ws_gen = DualArmWorkspaceGenerator(dual_arm, resolution=100)
fig = dual_ws_gen.plot_dual_workspace(show_boundary=True, show_points=True)
```

---

## 🔧 Implementation Details

### RRT* Algorithm

The RRT* implementation:
- Samples random configurations in C-space
- Builds a tree from start to goal
- Uses `DualArm.is_valid_configuration()` to check:
  - Joint limits (individual arms)
  - Workspace reachability
  - Inter-arm collisions
- Optimizes path via rewiring
- Returns collision-free path as list of configurations

### IK Solver Improvements

The IK solver (`ik_iterative`) now:
- **Enforces joint limits** during iteration by clipping angles to `[-π, π]`
- Tries multiple initial guesses for better convergence
- Validates solutions before returning them
- Works with both 2-link and 6-link arms

### Configuration Validation

`DualArm.is_valid_configuration(config)` checks:
1. **Joint limits**: Each joint angle within `[-π, π]`
2. **Workspace reachability**: End-effector within reachable region
3. **Inter-arm collisions**: Arm segments don't intersect

---

## 🐛 Troubleshooting

### Planning Fails
- **Increase `max_iterations`**: Try 5000-10000 for 2-link, 8000-15000 for 6-link
- **Increase `goal_threshold`**: Try 0.2-0.3 (especially for 6-link arms)
- **Check IK solutions**: Ensure workspace goals are reachable
- **Verify configurations**: Use `dual_arm.is_valid_configuration()` before planning

### IK Conversion Fails
- **Workspace position unreachable**: Check if position is within arm's workspace
- **Multiple attempts**: `MotionPlanner` tries 20 different initial guesses automatically
- **6-link arms**: More challenging - may need more attempts or better initial guesses

### Path Has Collisions
- **Decrease `step_size`**: For finer path checking (try 0.05-0.1)
- **Increase validation checks**: More intermediate points in path validation
- **Verify collision detection**: Test `is_valid_configuration()` directly

### Slow Planning
- **Reduce `max_iterations`**: Faster but less optimal planning
- **Increase `step_size`**: Explore faster (but may miss narrow passages)
- **Reduce `rewire_radius`**: Check fewer neighbors during optimization

---

## 📦 Dependencies

- `numpy`: Numerical computations
- `matplotlib`: Visualization
- `typing`: Type hints

---

## 🧪 Test Files

- **`test_end_to_end.py`**: Complete workflow test (workspace → IK → RRT* → visualization)
- **`test_rrt_star.py`**: RRT* component tests
- **`test_six_link_arm.py`**: 6-link arm integration tests

---

## 📝 Recent Changes

### Handoff Planning Architecture
- **Added**: Intelligent handoff planner that determines grab/delivery arms
- **Added**: Workspace intersection analysis for handoff point calculation
- **Added**: Phase-based RRT* planning (grab → handoff → delivery)
- **Files**: `src/handoff_planner.py`, `backend/api/planner_api.py`

### Web Application Enhancements
- **Added**: Real-time RRT* tree visualization
- **Added**: Item-based planning (drag item positions, not end-effectors)
- **Added**: Draggable arm bases
- **Added**: Handoff strategy visualization
- **Fixed**: Early termination when goal is found (10x speedup)
- **Fixed**: IK reachability verification for handoff points

### RRT* Optimizations
- **Added**: Early termination when goal is reached
- **Improved**: Progress callbacks every 10 iterations (was 1000)
- **Fixed**: Tree visualization data synchronization

### Multi-Link Arm Support
- **Added**: Full 3-link arm support in workspace generation
- **Fixed**: Generic workspace generator for N-link arms

---

## 🚀 Future Enhancements

- Trajectory smoothing/post-processing
- Visualization of RRT* planning tree
- Support for static obstacles in workspace
- Better workspace reachability analysis for 6-link arms
- Performance optimizations (KD-tree for nearest neighbor search)

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
