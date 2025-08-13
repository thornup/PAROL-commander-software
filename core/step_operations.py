from math import pi
import numpy as np
from config.robot_config import joint_limits_degree, joint_reduction_ratio
degree_per_step_constant = 360 / (32 * 200)
radian_per_step_constant = (2 * pi) / (32 * 200)
radian_per_sec_2_deg_per_sec_const = 360 / (2 * pi)
deg_per_sec_2_radian_per_sec_const = (2 * pi) / 360


# 360 / (200 * 32) = 0.05625
def DEG2STEPS(Degrees, index):
    Steps = Degrees / degree_per_step_constant * joint_reduction_ratio[index]
    return Steps


joint_limits_steps = [[DEG2STEPS(joint_limits_degree[0][0], 0), DEG2STEPS(joint_limits_degree[0][1], 0)],
                      [DEG2STEPS(joint_limits_degree[1][0], 1), DEG2STEPS(joint_limits_degree[1][1], 1)],
                      [DEG2STEPS(joint_limits_degree[2][0], 2), DEG2STEPS(joint_limits_degree[2][1], 2)],
                      [DEG2STEPS(joint_limits_degree[3][0], 3), DEG2STEPS(joint_limits_degree[3][1], 3)],
                      [DEG2STEPS(joint_limits_degree[4][0], 4), DEG2STEPS(joint_limits_degree[4][1], 4)],
                      [DEG2STEPS(joint_limits_degree[5][0], 5), DEG2STEPS(joint_limits_degree[5][1], 5)]]
joint_limits_steps = [[int(i[0]), int(i[1])] for i in joint_limits_steps]


def STEPS2DEG(Steps, index):
    Degrees = Steps * degree_per_step_constant / joint_reduction_ratio[index]
    return Degrees


def RAD2STEPS(Rads, index):
    deg = np.rad2deg(Rads)
    steps = DEG2STEPS(deg, index)
    return steps


def STEPS2RADS(Steps, index):
    deg = STEPS2DEG(Steps, index)
    rads = np.deg2rad(deg)
    return rads


def RAD2DEG(radian):
    return np.rad2deg(radian)


def DEG2RAD(degree):
    return np.deg2rad(degree)


def SPEED_STEPS2DEG(Steps_per_second, index):
    '''     Transform true RADS/S to true RPM.
    Both these values are true values at witch MOTORS SPIN  '''

    degrees_per_step = degree_per_step_constant / joint_reduction_ratio[index]
    degrees_per_second = Steps_per_second * degrees_per_step
    return degrees_per_second


def SPEED_DEG2STEPS(Deg_per_second, index):
    steps_per_second = Deg_per_second / degree_per_step_constant * joint_reduction_ratio[index]
    return steps_per_second


def SPEED_STEP2RAD(Steps_per_second, index):
    degrees_per_step = radian_per_step_constant / joint_reduction_ratio[index]
    rad_per_second = Steps_per_second * degrees_per_step
    return rad_per_second


def SPEED_RAD2STEP(Rad_per_second, index):
    steps_per_second = Rad_per_second / radian_per_step_constant * joint_reduction_ratio[index]
    return steps_per_second


def RAD_SEC_2_DEG_SEC(rad_per_sec):
    return rad_per_sec * radian_per_sec_2_deg_per_sec_const


def DEG_SEC_2_RAD_SEC(deg_per_sec):
    return deg_per_sec * deg_per_sec_2_radian_per_sec_const
