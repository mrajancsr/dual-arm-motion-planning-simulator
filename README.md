# 🦾 Interactive Dual-Arm Motion Planning Simulator for Planar Robot Arms

A Python-based simulator that models and visualizes two planar 2R robot arms performing motion planning in obstacle-rich environments.  
This project demonstrates fundamental robotics principles including forward/inverse kinematics, configuration space visualization, and sampling-based motion planning (RRT).

---

## 🧠 Project Overview

This simulator is developed for the **Computational Robotics** course (Fall 2025).  
It provides an interactive environment for understanding how dual-arm manipulators compute and execute collision-free trajectories in a shared workspace.

### ✨ Key Features
- Forward and Inverse Kinematics for 2-link planar arms  
- Dual-arm configuration and independent motion control  
- Workspace and C-space (configuration space) visualization  
- Sampling-based motion planning (RRT)  
- Optional trajectory smoothing and animation  

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
├── two_link_arm.py            # Core robot arm classes
├── workspace_generator.py     # Workspace generation and visualization
└── cspace_generator.py        # Configuration space generation and visualization
```

---

## 🔧 Module Documentation

### Workspace Generator Module (`workspace_generator.py`)

#### Purpose
Generates and visualizes the workspace of 2R planar robot arms, both individually and for dual-arm systems.

#### Classes

**`WorkspaceGenerator`**
- **Purpose**: Generates workspace for a single 2R arm
- **Key Methods**:
  - `generate_workspace()`: Samples joint angles to generate workspace points
  - `generate_workspace_boundary()`: Computes workspace boundary
  - `plot_workspace()`: Visualizes workspace with boundary and reach limits

**`DualArmWorkspaceGenerator`**
- **Purpose**: Generates workspace for dual-arm systems
- **Key Methods**:
  - `generate_dual_workspace()`: Generates workspace for both arms
  - `plot_dual_workspace()`: Visualizes both arm workspaces and collaborative region

#### Features
- Workspace boundary computation using joint limits
- Reachability analysis with inner/outer radius limits
- Collaborative workspace identification
- High-quality matplotlib visualizations
- Configurable resolution for sampling

### Configuration Space Generator Module (`cspace_generator.py`)

#### Purpose
Generates and visualizes the configuration space (C-space) of 2R planar robot arms, including collision detection and validity checking.

#### Classes

**`CSpaceGenerator`**
- **Purpose**: Generates C-space for a single 2R arm
- **Key Methods**:
  - `generate_cspace_grid()`: Creates configuration space grid
  - `check_workspace_limits()`: Validates reachability
  - `check_self_collision()`: Checks for self-collisions
  - `plot_cspace()`: Visualizes valid/invalid configurations
  - `plot_cspace_heatmap()`: Shows C-space as heatmap

**`DualArmCSpaceGenerator`**
- **Purpose**: Generates C-space for dual-arm systems
- **Key Methods**:
  - `check_arm_collision()`: Detects collisions between arms
  - `generate_dual_cspace()`: Generates 4D C-space (simplified)
  - `plot_dual_cspace_2d()`: Shows 2D projections of dual C-space

#### Features
- Joint limit enforcement
- Workspace reachability checking
- Inter-arm collision detection
- 2D and heatmap visualizations
- Configurable joint angle ranges

---

## 💻 Usage Examples

### Basic Workspace Generation
```python
from src.workspace_generator import WorkspaceGenerator
from src.two_link_arm import TwoLinkArm

# Create arm and workspace generator
arm = TwoLinkArm(L1=1.0, L2=0.7)
ws_gen = WorkspaceGenerator(arm, resolution=100)

# Generate and plot workspace
fig = ws_gen.plot_workspace(show_boundary=True, show_points=True)
```

### Dual-Arm Workspace
```python
from src.workspace_generator import DualArmWorkspaceGenerator
from src.two_link_arm import DualArm

# Create dual arm system
dual_arm = DualArm(L1=1.0, L2=0.7, separation=2.0)
dual_ws_gen = DualArmWorkspaceGenerator(dual_arm, resolution=100)

# Plot dual workspace
fig = dual_ws_gen.plot_dual_workspace(show_boundary=True, show_points=True)
```

### Configuration Space Analysis
```python
from src.cspace_generator import CSpaceGenerator

# Create C-space generator
cspace_gen = CSpaceGenerator(arm, resolution=100)
cspace_gen.set_joint_limits((-np.pi, np.pi), (-np.pi/2, np.pi/2))

# Plot C-space
fig = cspace_gen.plot_cspace(show_invalid=True)
```

---

## 🔗 Integration with Motion Planning

These modules provide the foundation for motion planning algorithms:

1. **Workspace Analysis**: Identifies reachable regions for path planning
2. **C-space Visualization**: Shows valid configurations for sampling-based planners
3. **Collision Detection**: Enables obstacle avoidance in motion planning
4. **Dual-Arm Coordination**: Supports collaborative manipulation planning

---

## ⚙️ Setup Instructions

### 1. Clone the repository
```bash
git clone https://github.com/<your-username>/dual-arm-motion-planning-simulator.git
cd dual-arm-motion-planning-simulator
```

### 2. Install dependencies
```bash
pip install -r requirements.txt
```

### 3. Run the demo
```bash
python demo.py
```

---

## 📦 Dependencies

- `numpy`: Numerical computations
- `matplotlib`: Visualization
- `typing`: Type hints

---

## 🚀 Future Enhancements

- 3D workspace visualization
- Advanced collision detection algorithms
- Real-time C-space updates
- Integration with RRT/PRM planners
- Trajectory optimization visualization

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
