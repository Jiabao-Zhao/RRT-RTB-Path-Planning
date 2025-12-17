import numpy as np

def sample_link_points(robot, q_rad):
    """
    Sample 5 points along each link for collision detection.
    """
    q_rad = np.asarray(q_rad, dtype=float)
    points = []

    # Sample points along each link
    for i in range(1, robot.n):
        link_start = robot.fkine(q_rad, end=i)
        link_end = robot.fkine(q_rad, end=i + 1)

        for alpha in np.linspace(0, 1, 5):
            p_start = np.array(link_start.t)
            p_end = np.array(link_end.t)
            p = (1 - alpha) * p_start + alpha * p_end
            points.append(p)
    try:
        ee = robot.fkine(q_rad)
        points.append(np.array(ee.t))
    except:
        pass

    return points

def check_state_validity(robot, q_rad, world):
    """Check if robot arm pose is collision-free"""
    points = sample_link_points(robot, q_rad)

    skip_count = 5 # skip the first link since it does not move

    # collision check with ground
    for i, point in enumerate(points):
        if i < skip_count:
            continue  # ignore base + first link for ground
        if point[2] < world.ground_z + 10:
            return False, f"Ground collision: z={point[2]:.1f}mm < {world.ground_z + 10:.1f}mm"

    # collision check with an obstacle
    for obs in world.obstacles:
        for point in points:
            dist = obs.distance_to_point(point)
            if dist < 30:
                return False, f"Collision with obstacle: clearance={dist:.1f}mm"

    return True, None


def segment_collision_free(robot, q_from, q_to, world):
    """check if the path of the robot is collision-free"""
    diff = q_to - q_from
    diff = np.arctan2(np.sin(diff), np.cos(diff))

    dist = np.linalg.norm(diff)
    if dist == 0:
        is_valid, _ = check_state_validity(robot, q_from, world)
        return is_valid

    n_steps = max(2, int(np.ceil(dist / 0.05)))

    for i in range(n_steps + 1):
        alpha = i / n_steps
        q = q_from + alpha * diff
        is_valid, reason = check_state_validity(robot, q, world)
        if not is_valid:
            return False

    return True

def smooth_path_shortcut(robot, world, rrt_path_rad):
    """Smooth path using shortcut method (works in RADIANS)"""
    if len(rrt_path_rad) <= 2:
        return rrt_path_rad

    smoothed = [np.array(q) for q in rrt_path_rad]

    for _ in range(100):
        if len(smoothed) <= 2:
            break

        i = np.random.randint(0, len(smoothed) - 2)
        j = np.random.randint(i + 2, len(smoothed))

        if segment_collision_free(robot, smoothed[i], smoothed[j], world):
            smoothed = smoothed[:i + 1] + smoothed[j:]

    return smoothed



def set_axes_equal(ax):
    """Make 3D axes have equal scale so the path isn't visually distorted."""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])

    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range])
    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

# ---- After you compute smoothed_path ----
# smoothed_path = smooth_path_shortcut(toolbox.robot, world, path_planning)

robot = toolbox.robot

# FK for each configuration -> end-effector positions
ee_xyz = []
for q in smoothed_path:  # q in radians
    T = robot.fkine(q)   # SE3
    ee_xyz.append(np.array(T.t).reshape(3))
ee_xyz = np.vstack(ee_xyz)

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Plot path + start/goal
ax.plot(ee_xyz[:, 0], ee_xyz[:, 1], ee_xyz[:, 2], linewidth=2)
ax.scatter(ee_xyz[0, 0], ee_xyz[0, 1], ee_xyz[0, 2], s=60)
ax.scatter(ee_xyz[-1, 0], ee_xyz[-1, 1], ee_xyz[-1, 2], s=60)

# Plot obstacle spheres
for obs in world.obstacles:
    c = np.asarray(obs.center, dtype=float)
    r = float(obs.radius)
    u = np.linspace(0, 2*np.pi, 30)
    v = np.linspace(0, np.pi, 15)
    xs = c[0] + r * np.outer(np.cos(u), np.sin(v))
    ys = c[1] + r * np.outer(np.sin(u), np.sin(v))
    zs = c[2] + r * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_wireframe(xs, ys, zs, linewidth=0.5)

# Plot ground plane (z = ground_z)
gz = float(world.ground_z)
pad = 100.0
xmin, xmax = ee_xyz[:, 0].min() - pad, ee_xyz[:, 0].max() + pad
ymin, ymax = ee_xyz[:, 1].min() - pad, ee_xyz[:, 1].max() + pad
X, Y = np.meshgrid(np.linspace(xmin, xmax, 2), np.linspace(ymin, ymax, 2))
Z = np.full_like(X, gz)
ax.plot_surface(X, Y, Z, alpha=0.2)

ax.set_xlabel("x (mm)")
ax.set_ylabel("y (mm)")
ax.set_zlabel("z (mm)")
ax.set_title("RRT Path (End-Effector)")
set_axes_equal(ax)

plt.show()