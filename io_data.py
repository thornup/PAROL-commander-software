import threading
import multiprocessing

# Hardcoded
STARTING_PORT = 3

# Positional stuff
# Data sent by the PC to the robot
Position_out = multiprocessing.Array("i", [1, 11, 111, 1111, 11111, 10], lock=False)
Speed_out = multiprocessing.Array("i", [2, 21, 22, 23, 24, 25], lock=True)

Command_out = multiprocessing.Value('i', 0)
Affected_joint_out = multiprocessing.Array("i", [1, 1, 1, 1, 1, 1, 1, 1], lock=False)
InOut_out = multiprocessing.Array("i", [0, 0, 0, 0, 0, 0, 0, 0], lock=False)  # IN1,IN2,OUT1,OUT2,ESTOP
Timeout_out = multiprocessing.Value('i', 0)
# Positon,speed,current,command,mode,ID
Gripper_data_out = multiprocessing.Array("i", [1, 1, 1, 1, 0, 0], lock=False)

# Data sent from robot to PC
Position_in = multiprocessing.Array("i", [31, 32, 33, 34, 35, 36], lock=False)
Speed_in = multiprocessing.Array("i", [41, 42, 43, 44, 45, 46], lock=False)
Homed_in = multiprocessing.Array("i", [1, 1, 1, 1, 1, 1, 1, 1], lock=False)
InOut_in = multiprocessing.Array("i", [1, 1, 1, 1, 1, 1, 1, 1], lock=False)  # IN1,IN2,OUT1,OUT2,ESTOP
Temperature_error_in = multiprocessing.Array("i", [1, 1, 1, 1, 1, 1, 1, 1], lock=False)
Position_error_in = multiprocessing.Array("i", [1, 1, 1, 1, 1, 1, 1, 1], lock=False)
Timeout_error = multiprocessing.Value('i', 0)
# how much time passed between 2 sent commands (2byte value, last 2 digits are decimal so max value is 655.35ms?)
Timing_data_in = multiprocessing.Value('i', 0)
XTR_data = multiprocessing.Value('i', 0)

# ID,Position,speed,current,status,obj_detection
Gripper_data_in = multiprocessing.Array("i", [1, 1, 1, 1, 1, 1], lock=False)

# gui core data
Homed_out = multiprocessing.Array("i", [1, 1, 1, 1, 1, 1], lock=False)

# General robot vars
Robot_GUI_mode = multiprocessing.Value('i', 0)

# Robot jogging vars
Joint_jog_buttons = multiprocessing.Array("i", [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], lock=False)
Cart_jog_buttons = multiprocessing.Array("i", [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], lock=False)

# Speed slider, acc slider, WRF/TRF
Jog_control = multiprocessing.Array("i", [0, 0, 0, 0], lock=False)

# COM PORT, BAUD RATE,
General_data = multiprocessing.Array("i", [STARTING_PORT, 3000000], lock=False)

# Home,Enable,Disable,Clear error,Real_robot,Sim_robot, demo_app, program execution,
Buttons = multiprocessing.Array("i", [0, 0, 0, 0, 1, 1, 0, 0, 0], lock=False)

# Positions for robot simulator
Position_Sim = multiprocessing.Array("i", [0, 0, 0, 0, 0, 0], lock=False)

shared_string = multiprocessing.Array('c', b' ' * 100)  # Create a character array of size 100
