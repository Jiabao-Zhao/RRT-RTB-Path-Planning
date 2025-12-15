from roboticstoolbox import DHRobot, RevoluteDH
import numpy as np
import math
from spatialmath import SE3
from spatialmath.base import tr2angvec, angvec2r

class RoboticsToolBox:
    """
    This class uses roboticstoolbox-python to create a robot model.
    """
    def __init__(self):
        self.link = [ # Customized robot arm using Denavit-Hartenberg
            RevoluteDH(d=267, a=0.0, alpha=-np.pi / 2, offset=0.0, qlim=[-2 * np.pi, 2 * np.pi]),
            RevoluteDH(d=0.0, a=math.sqrt(284.5 ** 2 + 53.5 ** 2), alpha=0.0,offset=-math.atan(284.5 / 53.5), qlim=[-2.059, 2.0944]),
            RevoluteDH(d=0.0, a=77.5, alpha=-np.pi / 2, offset=math.atan(284.5 / 53.5), qlim=[-2 * np.pi, 2 * np.pi]),
            RevoluteDH(d=342.5, a=0.0, alpha=np.pi / 2, offset=0.0, qlim=[-2 * np.pi, 2 * np.pi]),
            RevoluteDH(d=0.0, a=76, alpha=-np.pi / 2, offset=0.0, qlim=[-2 * np.pi, 2 * np.pi]),
            RevoluteDH(d=97, a=0.0, alpha=0.0, offset=0.0, qlim=[-2 * np.pi, 2 * np.pi]),
        ]
        self.robot = DHRobot(self.link, name="xArm6")

    def forward_kinematics(self, q_deg):
        """
        Return the end effector pose
        """
        q = np.deg2rad(q_deg)

        # translational
        T = self.robot.fkine(q)
        x, y, z = T.t

        # rotational
        R = T.R
        theta, u = tr2angvec(R)
        rx, ry, rz = u * theta
        rx_deg, ry_deg, rz_deg = np.rad2deg([rx, ry, rz])

        return x, y, z, rx_deg, ry_deg, rz_deg

    def inverse_kinematics(self, x, y, z, rx_deg, ry_deg, rz_deg, q0_deg=None):
        """
        get the joint positions of robot arm
        """
        rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
        theta = np.linalg.norm([rx, ry, rz])

        if theta < 1e-6:
            R = np.eye(3)
        else:
            u = np.array([rx, ry, rz]) / theta
            R = angvec2r(theta, u)

        T_target = SE3.Rt(R, [x, y, z])

        if q0_deg is None:
            q0 = np.zeros(self.robot.n)
        else:
            q0 = np.deg2rad(q0_deg)

        sol = self.robot.ikine_LM(T_target, q0=q0)

        if sol.success:
            q_deg = np.rad2deg(sol.q)
            return q_deg
        else:
            return None