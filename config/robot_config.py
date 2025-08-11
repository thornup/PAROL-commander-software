# This file acts as configuration file for robot you are using
# It works in conjustion with configuration file from robotics toolbox

from swift import Swift
import spatialmath.base.symbolic as sym
from roboticstoolbox import ETS as ET
from roboticstoolbox import *
import roboticstoolbox as rtb
from spatialmath import *
from spatialgeometry import *
from math import pi
import numpy as np
import time
import random

joint_num = 6  # Number of joints
microstep = 32
steps_per_revolution = 200


# robot length values (metres)
a1 = 110.50 / 1000
a2 = 23.42 / 1000
a3 = 180 / 1000
a4 = 43.5 / 1000
a5 = 176.35 / 1000
a6 = 62.8 / 1000
a7 = 45.25 / 1000

alpha_DH = [-pi / 2, pi, pi / 2, -pi / 2, pi / 2, pi]

robot = DHRobot(
    [
        RevoluteDH(d=a1, a=a2, alpha=alpha_DH[0]),
        RevoluteDH(a=a3, d=0, alpha=alpha_DH[1]),
        RevoluteDH(alpha=alpha_DH[2], a=-a4),
        RevoluteDH(d=-a5, a=0, alpha=alpha_DH[3]),
        RevoluteDH(a=0, d=0, alpha=alpha_DH[4]),
        RevoluteDH(alpha=alpha_DH[5], a=-a7, d=-a6),
    ],
    name="PAROL6",
)
# print(robot.isspherical())
# pyplot = rtb.backends.PyPlot()

# in degrees
joints_standby_position_degree = np.array([0, -90, 180, 0, 0, 180])
# in radians
joints_standby_position_radian = [np.deg2rad(angle) for angle in joints_standby_position_degree]

# values you get after homing robot and moving it to its most left and right sides
# In degrees
joint_limits_degree = [[-123.046875, 123.046875], [-145.0088, -3.375], [107.866, 287.8675], [-105.46975, 105.46975],
                       [-90, 90], [0, 360]]

# in radians
joint_limits_radian = []
for limits in joint_limits_degree:
    radian_limits = [np.deg2rad(angle) for angle in limits]
    joint_limits_radian.append(radian_limits)

# Reduction ratio we have on our joints
joint_reduction_ratio = [6.4, 20, 20 * (38 / 42), 4, 4, 10]

# min and max jog speeds. Usually slower from real maximal speeds
joint_max_jog_speed = [1500, 3000, 3600, 7000, 7000, 18000]
joint_min_jog_speed = [100, 100, 100, 100, 100, 100]

# LINEAR CARTESIAN JOG MAX MIN SPEED IN METERS PER SECOND
cartesian_linear_velocity_min_jog = 0.002
cartesian_linear_velocity_max_jog = 0.06

# LINEAR CARTESIAN MAX MIN SPEED IN METERS PER SECOND
cartesian_linear_velocity_min = 0.002
cartesian_linear_velocity_max = 0.06

# LINEAR CARTESIAN MAX MIN ACC IN METERS PER SECOND²
cartesian_linear_acc_min = 0.002
cartesian_linear_acc_max = 0.06

# ANGULAR CARTESIAN JOG MAX MIN SPEED IN DEGREES PER SECOND
cartesian_angular_velocity_min = 0.7
cartesian_angular_velocity_max = 25

joint_max_speed = [6500, 18000, 20000, 22000, 22000, 22000]  # max speed in STEP/S used
joint_min_speed = [100, 100, 100, 100, 100, 100]  # min speed in STEP/S used

joint_max_acc = 32000  # max acceleration in RAD/S²
joint_min_acc = 100  # min acceleration in RAD/S²

cart_lin_velocity_limits = [[-100, 100], [-100, 100], [-100, 100]]
cart_ang_velocity_limits = [[-100, 100], [-100, 100], [-100, 100]]

commands_list = ["Input", "Output", "Dummy", "Begin", "Home", "Delay", "End", "Loop", "MoveJoint", "MovePose",
                 "SpeedJoint", "MoveCart",
                 "MoveCart", "MoveCartRelTRF", "Gripper", "Gripper_cal"]

commands_list_true = [item + "()" for item in commands_list]


if __name__ == "__main__":
    """
    print(DEG2STEPS(180,2))
    print(STEPS2DEG(57905,2))
    print(RAD2STEPS(pi,5))
    print(STEPS2RADS(32000,5))
    print(SPEED_STEPS2DEG(1000,5))
    print(SPEED_STEP2RAD(1000,5))
    print(Joint_limits_radian)
    print(Joints_standby_position_radian)
    print(Joint_limits_steps)
    print(Joint_limits_radian)
    print(DEG2STEPS(-62.5,1))
    """
    #
    # J0_var = STEPS2RADS(1, 0)
    # J1_var = STEPS2RADS(1, 1)
    # J2_var = STEPS2RADS(1, 2)
    # J3_var = STEPS2RADS(1, 3)
    # J4_var = STEPS2RADS(1, 4)
    # J5_var = STEPS2RADS(1, 5)
    #
    # print("Joint 1 smallest step:", RAD2DEG(J0_var))
    # print("Joint 2 smallest step:", RAD2DEG(J1_var))
    # print("Joint 3 smallest step:", RAD2DEG(J2_var))
    # print("Joint 4 smallest step:", RAD2DEG(J3_var))
    # print("Joint 5 smallest step:", RAD2DEG(J4_var))
    # print("Joint 6 smallest step:", RAD2DEG(J5_var))
    # print("rad 2 step:", SPEED_RAD2STEP(-2.948504399390715 / 2, 5))
    # print("standby radian is", Joints_standby_position_radian)
    #
    # test = RAD2STEPS(0.0001, 5)
    # print(test)

