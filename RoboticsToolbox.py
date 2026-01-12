from roboticstoolbox import DHRobot, RevoluteDH
import numpy as np
import math
from spatialmath import SE3
from spatialmath.base import tr2angvec, angvec2r
from scipy.spatial.transform import Rotation as R
from scipy.io import savemat

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

        return x, y, z, rx_deg, ry_deg, rz_deg, T

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
if __name__ == "__main__":
    theta = np.array([
        [-20.830053, -16.359549, -48.270205, 0.4924, 65.285504, -20.875889],
        [-17.400499, -19.280832, -65.154583, -15.091995, 80.485673, -15.222515],
        [-18.840743, -21.497835, -52.606177, -12.685343, 64.131968, -39.162983],
        [-29.8024, -55.92578, -9.978518, 4.485916, 47.090599, -54.307232],
        [-36.037384, 2.283466, -81.582887, 5.661682, 96.705765, -49.870762],
        [-34.023781, 5.713879, -85.332839, 5.373485, 93.962844, -53.554995],
        [-42.394121, 22.995145, -93.185512, 7.063481, 95.385555, -62.330417],
        [-37.927858, 18.525444, -86.95345, 4.567104, 82.476415, -123.404567],
        [-39.95355, 13.381257, -77.115249, 22.920489, 65.882182, -140.457968],
        [3.46502, -5.674746, -50.09244, -38.821901, 61.978449, -67.865075],
        [8.870361, -5.48567, -50.670898, -24.582296, 59.310013, 117.072695],
        [-12.830645, -7.694823, -63.405572, -5.975663, 68.034155, 165.091372],
        [-12.639392, -3.244087, -59.194504, -6.449843, 59.441965, 166.330507],
        [-9.732891, 26.186692, -85.67564, -8.587606, 70.975032, 168.954539],
        [-13.946366, -14.59249, -25.209284, -2.695766, 17.13797, 165.387304],
        [-104.288403, -47.497571, -30.503356, 27.541508, 96.646807, -101.492942],
        [-139.729223, -64.606091, -45.338781, 30.48485, 135.831792, -104.715715],
        [-154.807123, -70.781315, -41.285276, 41.299142, 132.668721, -119.65261],
        [-108.855736, -20.062174, -40.729163, 31.109832, 88.369515, -133.602299],
        [-89.588598, -5.446021, -54.908895, 31.109546, 88.002994, -133.839675]
    ], dtype=float)

    robotics_toolbox = RoboticsToolBox()
    N = theta.shape[0]
    T_all = np.zeros((N, 4, 4), dtype=float)
    for i in range(N):
        q = theta[i]
        x,y,z,rx,ry,rz, T = robotics_toolbox.forward_kinematics(q)
        T_all[i] = T.A
    T_4x4xN = np.transpose(T_all, (1, 2, 0))

    savemat("endEffectorToBaseTform.mat", {"T_4x4xN": T_4x4xN})