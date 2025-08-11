import threading
import multiprocessing
import serial
import time
from spatialmath import *
import platform
import os
import re
from oclock import Timer, loop, interactiveloop
import numpy as np
import math, random
import roboticstoolbox as rp
from roboticstoolbox import trapezoidal
from roboticstoolbox import quintic
from spatialmath.base.argcheck import (
    isvector,
    getvector,
    # assertmatrix,
    getvector,
    isscalar,
)
import struct
import logging

from data_sender import send_data
from data_receiver import receive_data
from data_display import display_data

def main(shared_string, position_out, speed_out, command_out, affected_joint_out, in_out_out, timeout_out
         , gripper_data_out,
         position_in, speed_in, homed_in, in_out_in, Temperature_error_in, Position_error_in, Timeout_error
         , Timing_data_in,
         XTR_data, Gripper_data_in,
         Joint_jog_buttons, Cart_jog_buttons, Jog_control, General_data, Buttons, ):

    t1 = threading.Thread(target = send_data, args = (shared_string , position_out , speed_out , command_out , affected_joint_out , in_out_out
                                , timeout_out , gripper_data_out,
                                                      position_in , speed_in , homed_in , in_out_in , Temperature_error_in , Position_error_in
                                , Timeout_error , Timing_data_in,
                                                      XTR_data , Gripper_data_in,
                                                      Joint_jog_buttons , Cart_jog_buttons , Jog_control , General_data , Buttons))

    t2 = threading.Thread(target = receive_data, args = (shared_string, position_in , speed_in , homed_in , in_out_in , Temperature_error_in
                                , Position_error_in , Timeout_error , Timing_data_in,
                                                         XTR_data , Gripper_data_in , General_data,))

    t3 = threading.Thread(target = display_data
                          ,args = (shared_string , position_out , speed_out , command_out , affected_joint_out , in_out_out
                                , timeout_out , gripper_data_out,
                                   position_in , speed_in , homed_in , in_out_in , Temperature_error_in , Position_error_in
                                , Timeout_error , Timing_data_in,
                                   XTR_data , Gripper_data_in,
                                   Joint_jog_buttons , Cart_jog_buttons , Jog_control , General_data , Buttons))

    t1.start()
    t2.start()
    t3.start()
