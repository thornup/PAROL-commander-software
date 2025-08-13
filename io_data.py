import threading
import multiprocessing

# Hardcoded
STARTING_PORT = 3

# Positional stuff
# Data sent by the PC to the robot
position_out = multiprocessing.Array("i", [1, 11, 111, 1111, 11111, 10], lock=False)
speed_out = multiprocessing.Array("i", [2, 21, 22, 23, 24, 25], lock=True)

command_out = multiprocessing.Value('i', 0)
affected_joint_out = multiprocessing.Array("i", [1, 1, 1, 1, 1, 1, 1, 1], lock=False)
in_out_out = multiprocessing.Array("i", [0, 0, 0, 0, 0, 0, 0, 0], lock=False)  # IN1,IN2,OUT1,OUT2,ESTOP
timeout_out = multiprocessing.Value('i', 0)
# Positon,speed,current,command,mode,ID
gripper_data_out = multiprocessing.Array("i", [1, 1, 1, 1, 0, 0], lock=False)

# Data sent from robot to PC
Position_in = multiprocessing.Array("i", [31, 32, 33, 34, 35, 36], lock=False)
speed_in = multiprocessing.Array("i", [41, 42, 43, 44, 45, 46], lock=False)
homed_in = multiprocessing.Array("i", [1, 1, 1, 1, 1, 1, 1, 1], lock=False)
in_out_in = multiprocessing.Array("i", [1, 1, 1, 1, 1, 1, 1, 1], lock=False)  # IN1,IN2,OUT1,OUT2,ESTOP
temperature_error_in = multiprocessing.Array("i", [1, 1, 1, 1, 1, 1, 1, 1], lock=False)
position_error_in = multiprocessing.Array("i", [1, 1, 1, 1, 1, 1, 1, 1], lock=False)
timeout_error = multiprocessing.Value('i', 0)
# how much time passed between 2 sent commands (2byte value, last 2 digits are decimal so max value is 655.35ms?)
timing_data_in = multiprocessing.Value('i', 0)
xtr_data = multiprocessing.Value('i', 0)

# ID,Position,speed,current,status,obj_detection
gripper_data_in = multiprocessing.Array("i", [1, 1, 1, 1, 1, 1], lock=False)

# gui core data
homed_out = multiprocessing.Array("i", [1, 1, 1, 1, 1, 1], lock=False)

# General robot vars
robot_gui_mode = multiprocessing.Value('i', 0)

# Robot jogging vars
joint_jog_buttons = multiprocessing.Array("i", [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], lock=False)
cart_jog_buttons = multiprocessing.Array("i", [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], lock=False)

# Speed slider, acc slider, WRF/TRF
jog_control = multiprocessing.Array("i", [0, 0, 0, 0], lock=False)

# COM PORT, BAUD RATE,
general_data = multiprocessing.Array("i", [STARTING_PORT, 3000000, 0], lock=False)
# Home,Enable,Disable,Clear error,Real_robot,Sim_robot, demo_app, program execution,
buttons = multiprocessing.Array("i", [0, 0, 0, 0, 1, 1, 0, 0, 0], lock=False)

# Positions for robot simulator
position_sim = multiprocessing.Array("i", [0, 0, 0, 0, 0, 0], lock=False)

shared_string = multiprocessing.Array('c', b' ' * 100)  # Create a character array of size 100
