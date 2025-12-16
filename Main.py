from RoboticsToolbox import RoboticsToolBox
from xarmlib.wrapper import XArmAPI
from Obstacles_Creator import WorldModel
from RRT import RRT, Tree
from Utility import smooth_path_shortcut
import numpy as np

def move_arm(arm, path_deg):
    for q in path_deg[1:]:
        code = arm.set_servo_angle(
            angle=q.tolist(),
            is_radian=False,
            speed=20,
            mvacc=2000,
            wait=True
        )
        if code != 0:
            print("Error moving to waypoint:", code)
            break

def main():
    """
    High-level controller for path planning using RRT and RTB
    """

    # load the robot model with roboticstoolbox
    toolbox = RoboticsToolBox()

    x_start = 100; y_start = -250; z_start = 200; roll_start = 180; pitch_start = 0; yaw_start = 0
    x_goal = 614.1; y_goal = -114.6; z_goal = 184.6; roll_goal = 180; pitch_goal = 0; yaw_goal = 0      # goal coordinate in 3D (predefined)

    # load physical xArm
    arm = XArmAPI("192.168.1.240")
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    arm.set_position(x_start, y_start, z_start, roll_start, pitch_start, yaw_start) # home position (predefined)

    # initial world-class with ground plane and obstacle
    world = WorldModel(ground_z = 0.0)
    world.add_sphere(center=[385, -193.6, 200], radius=110.0)

    # initialize RRT Tree
    rrt_tree = RRT(toolbox, world, Tree, x_start, y_start, z_start, roll_start, pitch_start, yaw_start,
                        x_goal, y_goal, z_goal, roll_goal, pitch_goal, yaw_goal)

    path_planning = rrt_tree.rrt_connect()
    smoothed_path = smooth_path_shortcut(toolbox.robot,world, path_planning)
    path_deg = [np.rad2deg(q) for q in smoothed_path]

    # Verify final pose via FK
    q_final_deg = path_deg[-1]
    x_final, y_final, z_final, rx_final, ry_final, rz_final = toolbox.forward_kinematics(q_final_deg)
    print(f"\nFinal position: x={x_final:.1f}mm, y={y_final:.1f}mm, z={z_final:.1f}mm")

    # Check if we reached the goal
    distance_error = np.sqrt((x_final - x_goal) ** 2 + (y_final - y_goal) ** 2 + (z_final - z_goal) ** 2)
    print(f"Distance to goal: {distance_error:.2f}mm")

    print(path_deg)

    # move arm
    #move_arm(arm, path_deg)

if __name__ == "__main__":
    main()