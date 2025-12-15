from RoboticsToolbox import RoboticsToolBox
from xarmlib.wrapper import XArmAPI







def main():
    """
    High-level controller for path planning using RRT and RTB
    """

    # load the robot model with roboticstoolbox
    load_toolbox = RoboticsToolBox()
    robot = load_toolbox.robot

    # load physical xArm
    arm = XArmAPI("192.168.1.240")
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    arm.set_position(100, -250, 200, 180, 0, 0) # home position (predefined)

    # get the home joint positions
    code, start_angles = arm.get_servo_angle(is_radian=False)
    start_angles = start_angles[0:6]

    # goal coordinate in 3D (predefined)
    x_goal = 614.1; y_goal = -114.6; z_goal = 184.6