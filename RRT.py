import numpy as np
from Utility import check_state_validity, segment_collision_free

class RRT:
    def __init__(self, toolbox, world, Tree, x_start, y_start, z_start, roll_start, pitch_start, yaw_start,
                                     x_goal, y_goal, z_goal, roll_goal, pitch_goal, yaw_goal):
        q_start = toolbox.inverse_kinematics(x_start, y_start, z_start, roll_start, pitch_start, yaw_start)
        q_goal = toolbox.inverse_kinematics(x_goal, y_goal, z_goal, roll_goal, pitch_goal, yaw_goal)

        self.q_start_rad = np.deg2rad(q_start)
        self.q_goal_rad = np.deg2rad(q_goal)
        self.toolbox = toolbox
        self.world = world
        self.max_iterations = 10000
        self.bias = 0.1
        self.Tree = Tree
        self.step_size = 0.15

    def rrt_connect(self):

        # checking if the setting goal is achievable before RRT
        is_valid_goal, reason_goal = check_state_validity(self.toolbox.robot, self.q_goal_rad, self.world)
        if not is_valid_goal:
            print(f"Goal invalid (even with no obstacles): {reason_goal}")
            return None

        q_start_rad = np.asarray(self.q_start_rad, dtype=float)
        q_goal_rad = np.asarray(self.q_goal_rad, dtype=float)

        tree_start = self.Tree(q_start_rad)
        tree_goal = self.Tree(q_goal_rad)

        for iteration in range(self.max_iterations):
            if iteration % 2 == 0:
                tree_a, tree_b = tree_start, tree_goal
            else:
                tree_a, tree_b = tree_goal, tree_start

            q_rand = self.random_sample(q_goal_rad)
            idx_new, reached = self.extend_tree(q_rand)

            if idx_new is None:
                continue

            q_new = tree_a.nodes[idx_new]
            idx_b_current = None

            for _ in range(10):
                idx_b_new, reached_b = extend_tree(robot, world, tree_b, q_new)
                if idx_b_new is None:
                    break

                idx_b_current = idx_b_new

                if reached_b:
                    if tree_a is tree_start:
                        path = extract_path_from_trees(tree_start, idx_new, tree_goal, idx_b_current)
                    else:
                        path = extract_path_from_trees(tree_goal, idx_new, tree_start, idx_b_current)

                    return path

        return None

    def random_sample(self, q_goal_rad):
        """Sample random configuration in radians"""
        if np.random.random() < self.bias:
            return q_goal_rad

        robot = self.toolbox.robot

        q_random = []
        for i in range(robot.n):
            qmin = robot.qlim[0, i]
            qmax = robot.qlim[1, i]
            q_random.append(np.random.uniform(qmin, qmax))
        return np.array(q_random)

    def steer(self, q_from, q_to):
        """Steer with angle wrapping to constrain the case which the random sampling is far away from the nearest tree node"""
        q_from = np.asarray(q_from, dtype=float)
        q_to = np.asarray(q_to, dtype=float)

        diff = q_to - q_from
        diff = np.arctan2(np.sin(diff), np.cos(diff))  # Wrap to [-pi, pi]

        dist = np.linalg.norm(diff)
        if dist <= self.step_size:
            return q_from + diff

        return q_from + (diff / dist) * self.step_size

    def extend_tree(self, q_rand):
        """Extend the tree towards target"""
        robot =  self.toolbox.robot
        idx_near = self.Tree.nearest(q_rand)
        q_near = self.Tree.nodes[idx_near]
        q_new = self.steer(q_near, q_rand)

        if not segment_collision_free(robot, q_near, q_new, self.world):
            return None, False

        new_idx = self.Tree.add_node(q_new, idx_near)

        # Check if reached target
        diff = q_rand - q_new
        diff = np.arctan2(np.sin(diff), np.cos(diff))
        dist_to_target = np.linalg.norm(diff)

        if dist_to_target < self.step_size:
            if segment_collision_free(robot, q_new, q_rand, self.world):
                return new_idx, True

        return new_idx, False


class Tree:
    """Rapidly-exploring Random Tree"""
    def __init__(self, root):
        self.nodes = root
        self.parents = [-1]

    def add_node(self, q, parent_idx):
        self.nodes.append(np.asarray(q, dtype=float))
        self.parents.append(parent_idx)
        return len(self.nodes) - 1

    def nearest(self, q):
        """Find the nearest node with angle wrapping"""
        q = np.asarray(q, dtype=float)
        dists = []
        for node in self.nodes:
            diff = q - node
            diff = np.arctan2(np.sin(diff), np.cos(diff))
            dists.append(np.linalg.norm(diff))
        return int(np.argmin(dists))

    def __len__(self):
        return len(self.nodes)