# xArm 6 Path Planner - Improved Version

A robust, production-ready path planning controller for the xArm 6 robotic arm with collision avoidance, trajectory optimization, and comprehensive safety checks.

## Features

### Core Capabilities
- ✅ **Proper xArm 6 Model Integration** - Supports URDF loading with fallback options
- ✅ **Full 6-DOF Pose Planning** - Complete position and orientation specification
- ✅ **Enhanced Collision Checking** - Multi-point sampling along links for accurate collision detection
- ✅ **Dual Planning Strategy** - Fast straight-line planner with RRT-Connect fallback
- ✅ **Path Smoothing** - Shortcut smoothing to reduce waypoints and improve trajectory quality
- ✅ **Trajectory Generation** - Time-optimal trajectories with velocity/acceleration limits
- ✅ **Comprehensive Safety Checks** - Pre-execution verification and user confirmation
- ✅ **Detailed Logging** - Real-time progress tracking and debugging information

### Safety Features
- Joint limit verification
- Collision checking with configurable clearance
- Current position validation before execution
- Emergency stop support (Ctrl+C)
- User confirmation before hardware execution
- Graceful error handling and cleanup

## Installation

### Required Dependencies

```bash
# Core dependencies
pip install numpy scipy

# Robotics libraries
pip install roboticstoolbox-python
pip install spatialmath-python

# xArm SDK
pip install xarm-python-sdk
```

### Robot Model Setup

For best results, obtain the xArm 6 URDF file:

1. **Download from UFACTORY:**
   - Visit [xArm-Python-SDK GitHub](https://github.com/xArm-Developer/xArm-Python-SDK)
   - Or download from [UFACTORY Support](https://www.ufactory.cc/support/)

2. **Place URDF in your project directory:**
   ```
   your_project/
   ├── xarm6_path_planner_improved.py
   ├── xarm6.urdf  <-- Place here
   └── xarm6_description/  <-- Or here
       └── urdf/
           └── xarm6.urdf
   ```

3. **Update the code if using custom path:**
   ```python
   # In make_xarm6_robot() function
   urdf_paths = [
       'xarm6.urdf',
       'xarm6_description/urdf/xarm6.urdf',
       '/your/custom/path/to/xarm6.urdf'  # Add your path here
   ]
   ```

## Configuration

### Network Settings

Update the xArm IP address in `PlannerConfig`:

```python
class PlannerConfig:
    XARM_IP = '192.168.1.244'  # <-- UPDATE THIS
```

### Planning Parameters

Adjust these parameters based on your needs:

```python
class PlannerConfig:
    # Execution speed
    DT = 0.02  # 50 Hz control rate
    
    # Joint limits (rad/s and rad/s²)
    MAX_JOINT_VELOCITY = 1.0
    MAX_JOINT_ACCELERATION = 2.0
    
    # Planning quality vs. speed tradeoff
    RRT_MAX_ITERS = 5000  # Increase for better success rate
    RRT_STEP_SIZE = 0.15  # Smaller = smoother but slower
    
    # Collision safety
    CLEARANCE = 0.05  # 5cm minimum clearance (meters)
    POINTS_PER_LINK = 5  # More points = better collision detection
    
    # Safety
    MAX_START_DEVIATION = 0.2  # ~11 degrees
    REQUIRE_USER_CONFIRMATION = True
```

## Usage

### Basic Example

```python
from xarm6_path_planner_improved import *

# 1. Create robot model
robot = make_xarm6_robot()

# 2. Define obstacles (spherical)
world = WorldModel()
world.add_sphere(
    center=[0.3, 0.2, 0.3],  # x, y, z in meters
    radius=0.10,              # 10cm radius
    name="table_obstacle"
)

# 3. Set start configuration (current joint angles)
q_start = np.zeros(6)  # Home position

# 4. Define goal pose (full 6-DOF)
T_goal = (SE3.Tx(0.4) *       # 40cm forward
          SE3.Ty(0.1) *       # 10cm right
          SE3.Tz(0.3) *       # 30cm up
          SE3.Rx(180, 'deg')) # Gripper down

# 5. Plan path
path = plan_path_to_pose(robot, world, q_start, T_goal)

if path is not None:
    # 6. Generate trajectory
    trajectory, timestamps = generate_time_optimal_trajectory(path)
    
    # 7. Execute (uncomment when ready)
    # execute_trajectory_on_xarm(trajectory, robot)
```

### Running the Example

```bash
# Run with default example
python xarm6_path_planner_improved.py

# The script will:
# 1. Load robot model
# 2. Create example obstacles
# 3. Plan path to example goal
# 4. Generate trajectory
# 5. Show execution-ready message (hardware execution commented out)
```

### Defining Goal Poses

The xArm 6 has 6 DOF, so you must specify full pose (position + orientation):

```python
from spatialmath import SE3

# Example 1: Pick pose (gripper pointing down)
T_pick = (SE3.Tx(0.4) * SE3.Ty(0.0) * SE3.Tz(0.2) * 
          SE3.Rx(180, 'deg'))

# Example 2: Place pose (gripper pointing forward)
T_place = (SE3.Tx(0.3) * SE3.Ty(0.2) * SE3.Tz(0.35) * 
           SE3.Ry(90, 'deg'))

# Example 3: Custom orientation with RPY angles
T_custom = (SE3.Tx(0.35) * SE3.Ty(-0.1) * SE3.Tz(0.4) * 
            SE3.RPY([45, 0, 90], unit='deg'))

# Example 4: From transformation matrix
T_matrix = SE3([
    [0, 0, 1, 0.4],
    [0, 1, 0, 0.0],
    [-1, 0, 0, 0.3],
    [0, 0, 0, 1]
])
```

### Adding Obstacles

```python
world = WorldModel()

# Sphere 1: Table
world.add_sphere(
    center=[0.4, 0.0, 0.1],
    radius=0.15,
    name="table"
)

# Sphere 2: Tool holder
world.add_sphere(
    center=[0.3, -0.2, 0.25],
    radius=0.08,
    name="tool_holder"
)

# Sphere 3: Camera mount
world.add_sphere(
    center=[0.2, 0.3, 0.4],
    radius=0.05,
    name="camera"
)

# Clear all obstacles
world.clear()
```

## Hardware Execution

### Safety Checklist

Before executing on real hardware:

1. ✅ Verify workspace is clear of people and objects
2. ✅ Check emergency stop button is accessible
3. ✅ Verify xArm IP address is correct
4. ✅ Test with slow speeds first
5. ✅ Keep hand near emergency stop
6. ✅ Verify obstacle models match real workspace

### Execution Steps

1. **Uncomment execution line:**
   ```python
   # In main() function, uncomment:
   success = execute_trajectory_on_xarm(trajectory, robot)
   ```

2. **Run the script:**
   ```bash
   python xarm6_path_planner_improved.py
   ```

3. **The system will:**
   - Connect to xArm
   - Verify arm state
   - Check current position vs. trajectory start
   - Validate all waypoints
   - **Ask for confirmation** (type 'EXECUTE' to proceed)
   - Move to start position
   - Execute trajectory in servo mode
   - Show progress updates

4. **Emergency stop:**
   - Press `Ctrl+C` at any time
   - Or use physical emergency stop button

### Execution Output

```
============================================================
HARDWARE SAFETY CHECKS
============================================================
Check 1: Arm state = 0 (READY)
Check 2: Position deviation from trajectory start
  Current:    [0.00 0.00 0.00 0.00 0.00 0.00] deg
  Trajectory: [0.00 0.00 0.00 0.00 0.00 0.00] deg
  Deviation:  0.0000 rad (0.00 deg)
Check 3: Verifying 150 waypoints...
✓ All safety checks passed
============================================================

!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
READY TO EXECUTE TRAJECTORY ON HARDWARE
Total waypoints: 150
Estimated time: 3.0 seconds
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

Type 'EXECUTE' to continue or anything else to abort: EXECUTE

Moving to start position...
✓ At start position

Switching to servo mode...

Executing trajectory (150 waypoints at 50 Hz)...
Press Ctrl+C to emergency stop

Progress:  33.3% (50/150) - Elapsed: 1.0s
Progress:  66.7% (100/150) - Elapsed: 2.0s
Progress: 100.0% (150/150) - Elapsed: 3.0s

============================================================
✓ EXECUTION COMPLETE
  Time: 3.02s
  Errors: 0
============================================================
```

## Troubleshooting

### IK Fails

**Problem:** `IK failed - no solution found`

**Solutions:**
1. Check if goal pose is reachable (within workspace)
2. Verify goal pose doesn't have singular configuration
3. Try different initial guess: `plan_path_to_pose(..., ik_initial_guess=q_guess)`
4. Adjust goal position/orientation slightly

### RRT Fails

**Problem:** `RRT-Connect failed after N iterations`

**Solutions:**
1. Increase iterations: `PlannerConfig.RRT_MAX_ITERS = 10000`
2. Reduce step size: `PlannerConfig.RRT_STEP_SIZE = 0.1`
3. Check if obstacles block the path entirely
4. Increase clearance tolerance: `PlannerConfig.CLEARANCE = 0.03`

### Collision Detection Too Sensitive

**Problem:** Path fails even though obstacles seem far away

**Solutions:**
1. Reduce clearance: `PlannerConfig.CLEARANCE = 0.02`
2. Check obstacle positions are correct (use meters, not cm)
3. Reduce points per link: `PlannerConfig.POINTS_PER_LINK = 3`

### Robot Model Issues

**Problem:** Joint configurations have wrong dimensions

**Solutions:**
1. Verify URDF file is for xArm 6 (not xArm 7)
2. Check robot model loads correctly: `print(robot.n)` should be 6
3. Manually verify DH parameters if using fallback model

### Hardware Connection Issues

**Problem:** Cannot connect to xArm

**Solutions:**
1. Verify IP address: `ping 192.168.1.244`
2. Check network connection
3. Verify xArm is powered on
4. Check firewall settings

### Large Start Deviation

**Problem:** `Deviation exceeds limit`

**Solutions:**
1. Move robot to trajectory start position first
2. Or plan from current position: Use `arm.get_servo_angle()` as `q_start`
3. Increase tolerance: `PlannerConfig.MAX_START_DEVIATION = 0.3`

## Advanced Usage

### Custom Collision Geometry

Extend the collision checking for more complex shapes:

```python
class CapsuleObstacle:
    """Capsule (cylinder with hemispherical caps)"""
    def __init__(self, start, end, radius):
        self.start = np.array(start)
        self.end = np.array(end)
        self.radius = radius
    
    def distance_to_point(self, point):
        # Project point onto line segment
        v = self.end - self.start
        w = point - self.start
        
        c1 = np.dot(w, v)
        if c1 <= 0:
            return np.linalg.norm(point - self.start) - self.radius
        
        c2 = np.dot(v, v)
        if c1 >= c2:
            return np.linalg.norm(point - self.end) - self.radius
        
        b = c1 / c2
        pb = self.start + b * v
        return np.linalg.norm(point - pb) - self.radius
```

### Real-time Replanning

For dynamic obstacles:

```python
def execute_with_replanning(robot, world, T_goal, replan_interval=1.0):
    """Execute trajectory with periodic replanning"""
    arm = XArmAPI(PlannerConfig.XARM_IP)
    arm.connect()
    
    try:
        while True:
            # Get current position
            _, current_angles = arm.get_servo_angle(is_radian=True)
            q_current = np.array(current_angles[:6])
            
            # Check if goal reached
            T_current = robot.fkine(q_current)
            if np.linalg.norm(T_current.t - T_goal.t) < 0.01:
                break
            
            # Plan from current to goal
            path = plan_path_to_pose(robot, world, q_current, T_goal)
            if path is None:
                logger.error("Replanning failed!")
                break
            
            # Execute partial trajectory
            traj, times = generate_time_optimal_trajectory(path)
            execute_time = min(replan_interval, times[-1])
            
            for i, q in enumerate(traj):
                if times[i] > execute_time:
                    break
                arm.set_servo_angle_j(angles=list(q), is_radian=True)
                time.sleep(PlannerConfig.DT)
    
    finally:
        arm.disconnect()
```

### Visualization

Add visualization for debugging:

```python
def visualize_trajectory(robot, trajectory, world):
    """Visualize trajectory using matplotlib"""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot end-effector path
    ee_points = []
    for q in trajectory:
        T = robot.fkine(q)
        ee_points.append(T.t)
    ee_points = np.array(ee_points)
    
    ax.plot(ee_points[:, 0], ee_points[:, 1], ee_points[:, 2], 
            'b-', linewidth=2, label='EE path')
    ax.scatter(ee_points[0, 0], ee_points[0, 1], ee_points[0, 2],
              c='g', s=100, marker='o', label='Start')
    ax.scatter(ee_points[-1, 0], ee_points[-1, 1], ee_points[-1, 2],
              c='r', s=100, marker='*', label='Goal')
    
    # Plot obstacles
    for obs in world.obstacles:
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        x = obs.center[0] + obs.radius * np.outer(np.cos(u), np.sin(v))
        y = obs.center[1] + obs.radius * np.outer(np.sin(u), np.sin(v))
        z = obs.center[2] + obs.radius * np.outer(np.ones(np.size(u)), np.cos(v))
        ax.plot_surface(x, y, z, alpha=0.3, color='red')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    ax.set_title('Planned Trajectory')
    plt.show()
```

## Performance Tips

1. **Faster Planning:**
   - Use straight-line when possible (no obstacles)
   - Reduce `RRT_MAX_ITERS` for time-critical applications
   - Increase `RRT_STEP_SIZE` for faster but less optimal paths

2. **Better Paths:**
   - Increase `SMOOTHING_ITERATIONS`
   - Use smaller `RRT_STEP_SIZE`
   - Add more `POINTS_PER_LINK` for accuracy

3. **Safer Execution:**
   - Reduce `MAX_JOINT_VELOCITY` and `MAX_JOINT_ACCELERATION`
   - Increase `CLEARANCE`
   - Add more safety checks

## API Reference

### Main Functions

#### `plan_path_to_pose(robot, world, q_start, T_goal, ik_initial_guess=None)`
Plan collision-free path from start to goal pose.

**Returns:** `List[np.ndarray]` or `None`

#### `execute_trajectory_on_xarm(q_trajectory, robot, dt, ip)`
Execute trajectory on hardware.

**Returns:** `bool` (success status)

#### `generate_time_optimal_trajectory(path, max_velocity, max_acceleration, dt)`
Generate time-parameterized trajectory.

**Returns:** `(trajectory, timestamps)`

### Classes

#### `WorldModel`
- `add_sphere(center, radius, name)` - Add spherical obstacle
- `clear()` - Remove all obstacles

#### `PlannerConfig`
Configuration parameters (modify as needed)

## Contributing

Improvements welcome! Areas for enhancement:
- Support for cylindrical/box obstacles
- Real-time obstacle detection integration
- Path optimization algorithms
- Better visualization tools
- Integration with motion planning libraries

## License

MIT License - Feel free to use and modify for your projects.

## Support

For issues:
1. Check troubleshooting section
2. Verify all dependencies installed correctly
3. Test with simple examples first
4. Check xArm SDK documentation: https://github.com/xArm-Developer/xArm-Python-SDK

## Credits

Improved version based on original planning code with enhancements for production use with xArm 6.
