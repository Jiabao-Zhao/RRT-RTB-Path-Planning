from RoboticsToolbox import RoboticsToolBox
from xarmlib.wrapper import XArmAPI
from Obstacles_Creator import WorldModel
from RRT import RRT







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

