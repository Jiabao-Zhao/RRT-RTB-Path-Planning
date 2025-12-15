import numpy as np

class RRT:
    def __init__(self,toolbox, world, x_start, y_start, z_start, roll_start, pitch_start, yaw_start,
                                     x_goal, y_goal, z_goal, roll_goal, pitch_goal, yaw_goal):
        q_start = toolbox.inverse_kinematics(x_start, y_start, z_start, roll_start, pitch_start, yaw_start)
        q_goal = toolbox.inverse_kinematics(x_goal, y_goal, z_goal, roll_goal, pitch_goal, yaw_goal)

        q_start_rad = np.deg2rad(q_start)
        q_goal_rad = np.deg2rad(q_goal)
