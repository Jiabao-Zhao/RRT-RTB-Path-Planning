# RRT-Connect Path Planning for xArm6 (Roboticstoolbox-Python)

This project plans a collision-free joint-space path for a 6-DOF robot arm (modeled as **xArm6**) from a start end-effector pose to a goal end-effector pose using **RRT-Connect**, with collision checking against a **ground plane** and optional **spherical obstacles**. The result can be visualized (joint trajectories + 3D end-effector path) and optionally executed on a real xArm.

---

## What’s included

- **Robot model + IK/FK wrapper** (DH model via `roboticstoolbox-python`) :contentReference[oaicite:0]{index=0}  
- **World model** with a ground plane and spherical obstacles :contentReference[oaicite:1]{index=1}  
- **RRT-Connect planner** operating in joint space (radians internally) :contentReference[oaicite:2]{index=2}  
- **Collision checking + path smoothing + plotting utilities** :contentReference[oaicite:3]{index=3}  
- **Main entry point** that wires everything together and (optionally) moves the physical robot :contentReference[oaicite:4]{index=4}  

---

## File structure

- `Main.py`  
  Runs the whole pipeline: define start/goal poses → build world → run RRT-Connect → smooth path → plot results → (optional) command xArm. :contentReference[oaicite:5]{index=5}  

- `RoboticsToolbox.py`  
  Creates a DH-based `xArm6` model and exposes:
  - `forward_kinematics(q_deg)` → end-effector pose
  - `inverse_kinematics(x,y,z, rx,ry,rz)` → joint angles (deg) :contentReference[oaicite:6]{index=6}  

- `Obstacles_Creator.py`  
  Defines:
  - `WorldModel(ground_z)`  
  - `add_sphere(center, radius)` for spherical obstacles in **mm** :contentReference[oaicite:7]{index=7}  

- `RRT.py`  
  Implements RRT-Connect:
  - Converts start/goal Cartesian pose → IK → radians
  - Grows two trees and tries to connect them
  - Uses collision checking for each added segment :contentReference[oaicite:8]{index=8}  

- `Utility.py`  
  Implements:
  - Link sampling for collision (`sample_link_points`)
  - Ground + obstacle collision checks (`check_state_validity`)
  - Segment collision checking (`segment_collision_free`)
  - Shortcut smoothing (`smooth_path_shortcut`)
  - Plotting (`plot_rrt_results`) :contentReference[oaicite:9]{index=9}  

---

## Dependencies

Core:
- `numpy` :contentReference[oaicite:10]{index=10}
- `matplotlib` :contentReference[oaicite:11]{index=11}
- `roboticstoolbox-python` :contentReference[oaicite:12]{index=12}
- `spatialmath-python` :contentReference[oaicite:13]{index=13}

Optional (only if commanding a real xArm):
- `xarmlib` (for `XArmAPI`) :contentReference[oaicite:14]{index=14}  

Example install (adjust as needed for your environment):
```bash
pip install numpy matplotlib roboticstoolbox-python spatialmath-python
# Optional for real robot:
pip install xarmlib
