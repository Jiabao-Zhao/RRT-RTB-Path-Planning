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
                return False, f"Collision with {obs.name}: clearance={dist:.1f}mm"

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