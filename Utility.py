import numpy as np
import matplotlib.pyplot as plt

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

import numpy as np
import matplotlib.pyplot as plt

def plot_rrt_results(robot, smoothed_path, world=None, x_goal=None, y_goal=None, z_goal=None):
    """
    1) Joint angles vs step index
    2) End-effector x-y-z trajectory in 3D (+ spheres + ground plane)
    """

    q = np.asarray(smoothed_path, dtype=float)
    q_step, n = q.shape
    steps = np.arange(q_step)

    # ---- end-effector positions (N x 3) ----
    ee_xyz = np.zeros((q_step, 3), dtype=float)
    for i, k in enumerate(q):
        t = robot.fkine(k)
        ee_xyz[i, :] = np.asarray(t.t).reshape(3)

    plt.figure()
    for j in range(n):
        plt.plot(steps, q[:, j], label=f"Joint {j+1}")
    plt.xlabel("Step index")
    plt.ylabel("Joint angle (rad)")
    plt.title("Joint Angles Along Planned Path")
    plt.grid(True, alpha=0.3)
    plt.legend(ncol=2, fontsize=9)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(ee_xyz[:, 0], ee_xyz[:, 1], ee_xyz[:, 2], linewidth=2)

    # start/end markers
    ax.scatter(ee_xyz[0, 0], ee_xyz[0, 1], ee_xyz[0, 2], s=60)
    ax.scatter(ee_xyz[-1, 0], ee_xyz[-1, 1], ee_xyz[-1, 2], s=60)

    # optional goal marker
    if x_goal is not None and y_goal is not None and z_goal is not None:
        ax.scatter(float(x_goal), float(y_goal), float(z_goal), s=80, marker="x")

    xmin, xmax = np.min(ee_xyz[:, 0]), np.max(ee_xyz[:, 0])
    ymin, ymax = np.min(ee_xyz[:, 1]), np.max(ee_xyz[:, 1])
    zmin, zmax = np.min(ee_xyz[:, 2]), np.max(ee_xyz[:, 2])

    if world is not None:
        # include ground in z-limits
        zmin = min(zmin, float(world.ground_z))

        # include spheres in x/y/z limits
        for obs in world.obstacles:
            c = obs.center.astype(float)
            r = float(obs.radius)
            xmin = min(xmin, c[0] - r); xmax = max(xmax, c[0] + r)
            ymin = min(ymin, c[1] - r); ymax = max(ymax, c[1] + r)
            zmin = min(zmin, c[2] - r); zmax = max(zmax, c[2] + r)

    pad = 0.05 * max(xmax - xmin, ymax - ymin, zmax - zmin, 1.0)
    xmin -= pad; xmax += pad
    ymin -= pad; ymax += pad
    zmin -= pad; zmax += pad

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_zlim(zmin, zmax)

    if world is not None:
        # Ground plane z = ground_z over the current x/y bounds
        gx = np.linspace(xmin, xmax, 2)
        gy = np.linspace(ymin, ymax, 2)
        GX, GY = np.meshgrid(gx, gy)
        GZ = np.full_like(GX, float(world.ground_z))
        ax.plot_surface(GX, GY, GZ, alpha=0.15, linewidth=0)

        # Spheres
        u = np.linspace(0, 2*np.pi, 32)
        v = np.linspace(0, np.pi, 32)
        for obs in world.obstacles:
            cx, cy, cz = obs.center.astype(float)
            r = float(obs.radius)
            x = cx + r * np.outer(np.cos(u), np.sin(v))
            y = cy + r * np.outer(np.sin(u), np.sin(v))
            z = cz + r * np.outer(np.ones_like(u), np.cos(v))
            ax.plot_surface(x, y, z, alpha=0.25, linewidth=0)

    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_zlabel("z (mm)")
    ax.set_title("End-Effector 3D Trajectory")

    ax.set_box_aspect((xmax - xmin, ymax - ymin, zmax - zmin))

    plt.show()


