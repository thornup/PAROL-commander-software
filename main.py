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


from gui import gui
from gui import simulator




my_os = platform.system()
if my_os == "Windows":
    Image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)))
    logging.debug("Os is Windows")
else: 
    Image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)))
    logging.debug("Os is Linux")

print("run this")
logging.basicConfig(level = logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)

if my_os == "Windows": 
    STARTING_PORT = 58 # COM3
else:   
    STARTING_PORT = 0
# if using linux this will add /dev/ttyACM + 0 ---> /dev/ttyACM0
# if using windows this will add COM + 3 ---> COM3 
str_port = ''
logging.disable(logging.DEBUG)
if my_os == "Windows":
    try:
        str_port = 'COM' + str(STARTING_PORT)
        ser = serial.Serial(port=str_port, baudrate=3000000, timeout=0)
    except:
        ser = serial.Serial()
elif my_os == "Linux":
    try:
        str_port = '/dev/ttyACM' + str(STARTING_PORT)
        ser = serial.Serial(port=str_port, baudrate=3000000, timeout=0)
    except:
        ser = serial.Serial()
#ser.open()

# in big endian machines, first byte of binary representation of the multibyte data-type is stored first. 
int_to_3_bytes = struct.Struct('>I').pack # BIG endian order

# data for output string (data that is being sent to the robot)
#######################################################################################
#######################################################################################
start_bytes =  [0xff,0xff,0xff] 
start_bytes = bytes(start_bytes)

end_bytes =  [0x01,0x02] 
end_bytes = bytes(end_bytes)

# Data for testing
#######################################################################################

data_len_output = [0x05]
data_len_output = bytes(data_len_output)

test_data =  [0x9,0x4,0x5] 
test_data = bytes(test_data)
#######################################################################################


# data for input string (Data that is being sent by the robot)
#######################################################################################
#######################################################################################
input_byte = 0 # Here save incoming bytes from serial

start_cond1_byte = bytes([0xff])
start_cond2_byte = bytes([0xff])
start_cond3_byte = bytes([0xff])

end_cond1_byte = bytes([0x01])
end_cond2_byte = bytes([0x02])

start_cond1 = 0 #Flag if start_cond1_byte is received
start_cond2 = 0 #Flag if start_cond2_byte is received
start_cond3 = 0 #Flag if start_cond3_byte is received

good_start = 0 #Flag if we got all 3 start condition bytes
data_len = 0 #Length of the data after -3 start condition bytes and length byte, so -4 bytes

data_buffer = [None]*255 #Here save all data after data length byte
data_counter = 0 #Data counter for incoming bytes; compared to data length to see if we have correct length
#######################################################################################
#######################################################################################

prev_positions = [0,0,0,0,0,0]

# Set interval
INTERVAL_S = 0.01

robot_pose = [0,0,0,0,0,0] #np.array([0,0,0,0,0,0])
prev_speed = [0,0,0,0,0,0]
Tt = SE3
interval_test = 0

# Program execution variables
Program_length = 0
Program_step = 0

Robot_mode = "Dummy"







# Data that we receive from the robot
#Input is data buffer list
#Output is saved to multiproc arrays and variables
## joints(3byte)x6,speed(3byte)x6,homed(byte),I/O(byte),temp_error(byte),position_error(byte),timing_data(2byte),Timeout_error(byte),xtr2(byte)
# Gripper data == Position(2byte),speed(2byte),current(2byte),status(byte),obj_detection(byte),ID(byte)
## CRC(byte),end1(byte),end2(byte)
# Last 2 bytes are end bytes but we dont unpack then since we chech their validity elsewhere
def Unpack_data(data_buffer_list, Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in):

    Joints = []
    Speed = []

    for i in range(0,18, 3):
        variable = data_buffer_list[i:i+3] 
        Joints.append(variable)

    for i in range(18,36, 3):
        variable = data_buffer_list[i:i+3]
        Speed.append(variable)


    for i in range(6):
        var =  b'\x00' + b''.join(Joints[i]) 
        Position_in[i] = Fuse_3_bytes(var)
        var =  b'\x00' + b''.join(Speed[i]) 
        Speed_in[i] = Fuse_3_bytes(var)

    Homed = data_buffer_list[36]
    IO_var = data_buffer_list[37]
    temp_error = data_buffer_list[38]
    position_error = data_buffer_list[39]
    timing_data = data_buffer_list[40:42]
    Timeout_error_var = data_buffer_list[42]
    xtr2 = data_buffer_list[43]
    device_ID = data_buffer_list[44]
    Gripper_position = data_buffer_list[45:47]
    Gripper_speed = data_buffer_list[47:49]
    Gripper_current = data_buffer_list[49:51]
    Status = data_buffer_list[51]
    object_detection = data_buffer_list[52]
    CRC_byte = data_buffer_list[53]
    endy_byte1 = data_buffer_list[54]
    endy_byte2 = data_buffer_list[55]

    logging.debug("Robot position")
    logging.debug(Joints)
    logging.debug("Robot speed")
    logging.debug(Speed)
    logging.debug("Robot homed")
    logging.debug(Homed)

    temp = Split_2_bitfield(int.from_bytes(Homed,"big"))
    for i in range(8):
        Homed_in[i] = temp[i]

    logging.debug("Robot I/O data")
    logging.debug(IO_var)

    temp = Split_2_bitfield(int.from_bytes(IO_var,"big"))
    for i in range(8):
        InOut_in[i] = temp[i]

    logging.debug("Robot temp error data")
    logging.debug(temp_error)

    temp = Split_2_bitfield(int.from_bytes(temp_error,"big"))
    for i in range(8):
        Temperature_error_in[i] = temp[i]

    logging.debug("Robot position error data")
    logging.debug(position_error)

    temp = Split_2_bitfield(int.from_bytes(position_error,"big"))
    for i in range(8):
        Position_error_in[i] = temp[i]

    logging.debug("Robot timig data")
    logging.debug(timing_data)
    logging.debug("Robot timig data fused")
    var = b'\x00' + b'\x00' + b''.join(timing_data)
    logging.debug(var)
    logging.debug("Robot timig data fused 2")
    var2 = Fuse_3_bytes(var)
    Timing_data_in.value = var2
    logging.debug(var2)
    logging.debug("Timing in ms")
    logging.debug(var2 * 1.4222222e-6)
    logging.debug(var2)
    logging.debug("Robot timig error data")
    logging.debug(Timeout_error_var)

    Timeout_error.value = int.from_bytes(Timeout_error_var,"big")

    logging.debug("Robot additional byte 2")
    logging.debug(xtr2)

    XTR_data.value = int.from_bytes(xtr2,"big")

    logging.debug("Gripper device ID")
    logging.debug(device_ID)

    Gripper_data_in[0] = int.from_bytes(device_ID,"big") 

    logging.debug("Gripper position")
    logging.debug(Gripper_position)

    var =  b'\x00'+ b'\x00' + b''.join(Gripper_position) 
    Gripper_data_in[1] = Fuse_2_bytes(var)

    logging.debug("Gripper speed")
    logging.debug(Gripper_speed)


    var =  b'\x00'+ b'\x00' + b''.join(Gripper_speed) 
    Gripper_data_in[2] = Fuse_2_bytes(var)

    logging.debug("Gripper current")
    logging.debug(Gripper_current)


    var =  b'\x00'+ b'\x00' + b''.join(Gripper_current) 
    Gripper_data_in[3] = Fuse_2_bytes(var)

    logging.debug("Gripper status")
    logging.debug(Status)

    Gripper_data_in[4] = int.from_bytes(Status,"big")

    logging.debug("Gripper object detection")
    logging.debug(object_detection)

    Gripper_data_in[5] = int.from_bytes(object_detection,"big")

    logging.debug("CRC byte")
    logging.debug(CRC_byte)
    logging.debug("End byte 1")
    logging.debug(endy_byte1)
    logging.debug("End byte 2")
    logging.debug(endy_byte2)

# Data we send to the robot for testing
def Pack_data_test():

    # Len is defined by all bytes EXCEPT start bytes and len
    # Start bytes = 3
    len = 52 #1
    Position = [255,255,255,255,255,255]  #18
    Speed = [255,255,255,255,255,255]  #18
    Command = 123 #1 
    Affected_joint = [1,1,1,1,1,1,1,1] #1
    InOut = [0,0,0,0,0,0,0,0] #1
    Timeout = 247 #1
    Gripper_data = [-222,-223,-224,225,226,123]  #9
    CRC_byte = 228 #1
    # End bytes = 2


    test_list = []
    #print(test_list)

    #x = bytes(start_bytes)
    test_list.append((start_bytes))
    
    test_list.append(bytes([len]))

    # Position data
    for i in range(6):
        position_split = Split_2_3_bytes(Position[i])
        test_list.append(position_split[1:4])

    # Speed data
    for i in range(6):
        speed_split = Split_2_3_bytes(Speed[i])
        test_list.append(speed_split[1:4])

    # Command data
    test_list.append(bytes([Command]))

    # Affected joint data
    Affected_list = Fuse_bitfield_2_bytearray(Affected_joint)
    test_list.append(Affected_list)

    # Inputs outputs data
    InOut_list = Fuse_bitfield_2_bytearray(InOut)
    test_list.append(InOut_list)

    # Timeout data
    test_list.append(bytes([Timeout]))

    # Gripper position
    Gripper_position = Split_2_3_bytes(Gripper_data[0])
    test_list.append(Gripper_position[2:4])

    # Gripper speed
    Gripper_speed = Split_2_3_bytes(Gripper_data[1])
    test_list.append(Gripper_speed[2:4])

    # Gripper current
    Gripper_current = Split_2_3_bytes(Gripper_data[2])
    test_list.append(Gripper_current[2:4])  

    # Gripper command
    test_list.append(bytes([Gripper_data[3]]))
    # Gripper mode
    test_list.append(bytes([Gripper_data[4]]))
    # Gripper ID
    test_list.append(bytes([Gripper_data[5]]))
 
    # CRC byte
    test_list.append(bytes([CRC_byte]))

    # END bytes
    test_list.append((end_bytes))
    
    #print(test_list)
    return test_list





def Main(shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons,): 

    t1 = threading.Thread(target = Task1, args = (shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons))
    
    t2 = threading.Thread(target = Task2, args = (shared_string, Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,General_data,))
    
    t3 = threading.Thread(target = Task3,args = ( shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons))

    t1.start()
    t2.start()
    t3.start()



def GUI_process(shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons):

        gui.GUI(shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons)


def SIMULATOR_process(Position_out,Position_in,Position_Sim,Buttons):
    simulator.run(Position_out,Position_in,Position_Sim,Buttons)

# u PROCES kao argumenti idu multi proc arrays tu dolje u initi
# Gore u thredovima i funkcijama to nazovem kako oćem i pozivam stvari iz toga i tjt
if __name__ == '__main__':

    print("running")
    time.sleep(0.01) 

    try:
        ser.close()
    except:
        None

    # Data sent by the PC to the robot
    Position_out = multiprocessing.Array("i",[1,11,111,1111,11111,10], lock=False) 

    Speed_out = multiprocessing.Array("i",[2,21,22,23,24,25], lock=True)


    Command_out = multiprocessing.Value('i',0) 
    Affected_joint_out = multiprocessing.Array("i",[1,1,1,1,1,1,1,1], lock=False) 
    InOut_out = multiprocessing.Array("i",[0,0,0,0,0,0,0,0], lock=False) #IN1,IN2,OUT1,OUT2,ESTOP
    Timeout_out = multiprocessing.Value('i',0) 
    #Positon,speed,current,command,mode,ID
    Gripper_data_out = multiprocessing.Array("i",[1,1,1,1,0,0], lock=False)

    # Data sent from robot to PC
    Position_in = multiprocessing.Array("i",[31,32,33,34,35,36], lock=False) 
    Speed_in = multiprocessing.Array("i",[41,42,43,44,45,46], lock=False) 
    Homed_in = multiprocessing.Array("i",[1,1,1,1,1,1,1,1], lock=False) 
    InOut_in = multiprocessing.Array("i",[1,1,1,1,1,1,1,1], lock=False) #IN1,IN2,OUT1,OUT2,ESTOP
    Temperature_error_in = multiprocessing.Array("i",[1,1,1,1,1,1,1,1], lock=False) 
    Position_error_in = multiprocessing.Array("i",[1,1,1,1,1,1,1,1], lock=False) 
    Timeout_error = multiprocessing.Value('i',0) 
    # how much time passed between 2 sent commands (2byte value, last 2 digits are decimal so max value is 655.35ms?)
    Timing_data_in = multiprocessing.Value('i',0) 
    XTR_data =   multiprocessing.Value('i',0)

    #ID,Position,speed,current,status,obj_detection
    Gripper_data_in = multiprocessing.Array("i",[1,1,1,1,1,1], lock=False)  

    # gui control data
    Homed_out = multiprocessing.Array("i",[1,1,1,1,1,1], lock=False) 

    #General robot vars
    Robot_GUI_mode =   multiprocessing.Value('i',0)

    # Robot jogging vars
    Joint_jog_buttons = multiprocessing.Array("i",[0,0,0,0,0,0,0,0,0,0,0,0], lock=False) 
    Cart_jog_buttons = multiprocessing.Array("i",[0,0,0,0,0,0,0,0,0,0,0,0], lock=False)

    # Speed slider, acc slider, WRF/TRF 
    Jog_control = multiprocessing.Array("i",[0,0,0,0], lock=False) 

    # COM PORT, BAUD RATE, 
    General_data =  multiprocessing.Array("i",[STARTING_PORT,3000000], lock=False) 

    # Home,Enable,Disable,Clear error,Real_robot,Sim_robot, demo_app, program execution,
    Buttons =  multiprocessing.Array("i",[0,0,0,0,1,1,0,0,0], lock=False) 

    # Positions for robot simulator
    Position_Sim =  multiprocessing.Array("i",[0,0,0,0,0,0], lock=False) 

    shared_string = multiprocessing.Array('c', b' ' * 100)  # Create a character array of size 100

    # Process
    process1 = multiprocessing.Process(target=Main,args=[shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons,])
    
    process2 = multiprocessing.Process(target=GUI_process,args=[shared_string,Position_out,Speed_out,Command_out,Affected_joint_out,InOut_out,Timeout_out,Gripper_data_out,
         Position_in,Speed_in,Homed_in,InOut_in,Temperature_error_in,Position_error_in,Timeout_error,Timing_data_in,
         XTR_data,Gripper_data_in,
        Joint_jog_buttons,Cart_jog_buttons,Jog_control,General_data,Buttons,])
    

    process3 = multiprocessing.Process(target=SIMULATOR_process,args =[Position_out,Position_in,Position_Sim,Buttons])


    process1.start()
    time.sleep(1)
    process2.start()
    time.sleep(1)
    process3.start()
    process1.join()
    process2.join()
    process3.join()

    process1.terminate()
    process2.terminate()
    process3.terminate()





