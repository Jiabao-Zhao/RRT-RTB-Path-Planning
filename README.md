# RRT-Connect Path Planning for Robot Arm Using Roboticstoolbox-Python

This project plans a collision-free joint-space path for a 6-DOF robot arm (modeled as **xArm6**) from a start end-effector pose to a goal end-effector pose using **RRT-Connect**, with collision checking against a **ground plane** and optional **spherical obstacles**. The result can be visualized (joint trajectories + 3D end-effector path) and optionally executed on a real xArm.

---

## What’s included

- **Robot model + IK/FK wrapper** (DH model via `roboticstoolbox-python`)
- **World model** with a ground plane and spherical obstacles
- **RRT-Connect planner** operating in joint space (radians internally)
- **Collision checking + path smoothing + plotting utilities** 
- **Main entry point** that wires everything together and (optionally) moves the physical robot 

---

## File structure

- `Main.py`  
  Runs the whole pipeline: define start/goal poses → build world → run RRT-Connect → smooth path → plot results → (optional) command xArm.

- `RoboticsToolbox.py`  
  Creates a DH-based `xArm6` model and exposes:
  - `forward_kinematics(q_deg)` → end-effector pose
  - `inverse_kinematics(x,y,z, rx,ry,rz)` → joint angles (deg)

- `Obstacles_Creator.py`  
  Defines:
  - `WorldModel(ground_z)`  
  - `add_sphere(center, radius)` for spherical obstacles in **mm** 

- `RRT.py`  
  Implements RRT-Connect:
  - Converts start/goal Cartesian pose → IK → radians
  - Grows two trees and tries to connect them
  - Uses collision checking for each added segment 

- `Utility.py`  
  Implements:
  - Link sampling for collision (`sample_link_points`)
  - Ground + obstacle collision checks (`check_state_validity`)
  - Segment collision checking (`segment_collision_free`)
  - Shortcut smoothing (`smooth_path_shortcut`)
  - Plotting (`plot_rrt_results`)  

---

## Dependencies

Core:
- `numpy` 
- `matplotlib` 
- `roboticstoolbox-python`
- `spatialmath-python`

Optional (only if commanding a real xArm):
- `xarmlib` (for `XArmAPI`) 

Example install (adjust as needed for your environment):
```bash
pip install numpy matplotlib roboticstoolbox-python spatialmath-python
# Optional for real robot:
pip install xarmlib or deploy the xarmlib locally
