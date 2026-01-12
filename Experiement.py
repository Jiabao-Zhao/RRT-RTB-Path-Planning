from xarmlib.wrapper import XArmAPI

path_deg = [
    [-68.1985905, -20.1120182, -10.4416846, 0.0, 30.5537028, -68.1985905],
    [-29.08676044, 24.13448893, -76.10968549, 3.68530581, 42.36497815, -23.34110244],
    [-10.5706380, 43.2852224, -94.1929494, 0.0, 50.9077265, -10.5706304],
]

x_start, y_start, z_start = 100, -250, 200
roll_start, pitch_start, yaw_start = 180, 0, 0

arm = XArmAPI("192.168.1.240")
arm.motion_enable(True)
arm.set_mode(0)
arm.set_state(0)

code = arm.set_position(x_start, y_start, z_start, roll_start, pitch_start, yaw_start, wait=True)
if code != 0:
    print("Error moving to start pose:", code)
"""
for q in path_deg[1:]:  # skips the first waypoint on purpose
    code = arm.set_servo_angle(
        angle=q,
        is_radian=False,
        speed=20,
        mvacc=2000,
        wait=True
    )
    if code != 0:
        print("Error moving to waypoint:", code)
        break
code = arm.set_gripper_enable(True)
arm.set_gripper_position(100,)
arm.set_position(385, -193.6, 300, 180, 0, 0, wait=True)
arm.set_gripper_position(500)
"""