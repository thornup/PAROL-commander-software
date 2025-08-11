from core.byte_operations import *
from core.unpack import unpack_data

# Data we send to the robot
# Inputs are multiproc arrays and variables
# Outputs is list of bytes objects? that need to be send by the serial
# Position is needed robot position - it is list of 6 joint position elements and packs to 3 bytes each
# Speed is needed robot position - it is list of 6 joint speed elements and packs to 3 bytes each
# Commad is single byte
# Affected joint is single byte
# InOut is byte where
# Timeout is byte
# Gripper data is list of gripper elements like speed, positon, ID...
# Positon packs to 2 bytes, speed to 2 bytes, current to 2 bytes, command to 1 byte, mode 1 byte, ID to 1  byte

# First 3 bytes are start condition
# After that is data len
# After that is data buffer
# After that is CRC
# After that is 2 end bytes
# Whole string is packed in one list of data


my_os = platform.system()
if my_os == "Windows":
    image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)))
    logging.debug("Os is Windows")
else:
    image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)))
    logging.debug("Os is Linux")


logging.basicConfig(level = logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)

if my_os == "Windows":
    STARTING_PORT = 3 # COM3
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



# Just read data and print it
def Get_data_old():
    while (ser.inWaiting() > 0):
        data_str = ser.read(ser.inWaiting()) #.decode('utf-8')
        print(data_str)
        print("\\+\\")
        time.sleep(0.01)



# Data we send to the robot
# Inputs are multiproc arrays and variables
# Outputs is list of bytes objects? that need to be send by the serial
# Position is needed robot position - it is list of 6 joint position elements and packs to 3 bytes each
# Speed is needed robot position - it is list of 6 joint speed elements and packs to 3 bytes each
# Commad is single byte
# Affected joint is single byte
# InOut is byte where
# Timeout is byte
# Gripper data is list of gripper elements like speed, positon, ID...
# Positon packs to 2 bytes, speed to 2 bytes, current to 2 bytes, command to 1 byte, mode 1 byte, ID to 1  byte

# First 3 bytes are start condition
# After that is data len
# After that is data buffer
# After that is CRC
# After that is 2 end bytes
# Whole string is packed in one list of data


def pack_data(position_out, speed_out, command_out, affected_joint_out, in_out_out, timeout_out, gripper_data_out):
    # Len is defined by all bytes EXCEPT start bytes and len
    # Start bytes = 3
    len = 52  # 1
    position = [position_out[0], position_out[1], position_out[2], position_out[3], position_out[4],
                position_out[5]]  # 18
    speed = [speed_out[0], speed_out[1], speed_out[2], speed_out[3], speed_out[4], speed_out[5], ]  # 18
    command = command_out.value  # 1
    affected_joint = affected_joint_out
    in_out = in_out_out  # 1
    timeout = timeout_out.value  # 1
    gripper_data = gripper_data_out  # 9
    crc_byte = 228  # 1
    # End bytes = 2

    test_list = []
    # print(test_list)

    # x = bytes(start_bytes)
    test_list.append((start_bytes))

    test_list.append(bytes([len]))

    # position data
    for i in range(6):
        position_split = split_2_3_bytes(position[i])
        test_list.append(position_split[1:4])

    # speed data
    for i in range(6):
        speed_split = split_2_3_bytes(speed[i])
        test_list.append(speed_split[1:4])

    # command data
    test_list.append(bytes([command]))

    # Affected joint data
    affected_list = fuse_bitfield_2_bytearray(affected_joint[:])
    test_list.append(affected_list)

    # Inputs outputs data
    in_out_list = fuse_bitfield_2_bytearray(in_out[:])
    test_list.append(in_out_list)

    # timeout data
    test_list.append(bytes([timeout]))

    # Gripper position
    gripper_position = split_2_3_bytes(gripper_data[0])
    test_list.append(gripper_position[2:4])

    # Gripper speed
    gripper_speed = split_2_3_bytes(gripper_data[1])
    test_list.append(gripper_speed[2:4])

    # Gripper current
    gripper_current = split_2_3_bytes(gripper_data[2])
    test_list.append(gripper_current[2:4])

    # Gripper command
    test_list.append(bytes([gripper_data[3]]))
    # Gripper mode
    test_list.append(bytes([gripper_data[4]]))
    # Gripper ID
    test_list.append(bytes([gripper_data[5]]))

    # CRC byte
    test_list.append(bytes([crc_byte]))

    # END bytes
    test_list.append((end_bytes))

    # print(test_list)
    return test_list


# Data we send to the robot for testing
def pack_data_test():
    # Len is defined by all bytes EXCEPT start bytes and len
    # Start bytes = 3
    len = 52  # 1
    Position = [255, 255, 255, 255, 255, 255]  # 18
    Speed = [255, 255, 255, 255, 255, 255]  # 18
    Command = 123  # 1
    Affected_joint = [1, 1, 1, 1, 1, 1, 1, 1]  # 1
    InOut = [0, 0, 0, 0, 0, 0, 0, 0]  # 1
    Timeout = 247  # 1
    Gripper_data = [-222, -223, -224, 225, 226, 123]  # 9
    CRC_byte = 228  # 1
    # End bytes = 2

    test_list = []
    # print(test_list)

    # x = bytes(start_bytes)
    test_list.append((start_bytes))

    test_list.append(bytes([len]))

    # Position data
    for i in range(6):
        position_split = split_2_3_bytes(Position[i])
        test_list.append(position_split[1:4])

    # Speed data
    for i in range(6):
        speed_split = split_2_3_bytes(Speed[i])
        test_list.append(speed_split[1:4])

    # Command data
    test_list.append(bytes([Command]))

    # Affected joint data
    Affected_list = fuse_bitfield_2_bytearray(Affected_joint)
    test_list.append(Affected_list)

    # Inputs outputs data
    InOut_list = fuse_bitfield_2_bytearray(InOut)
    test_list.append(InOut_list)

    # Timeout data
    test_list.append(bytes([Timeout]))

    # Gripper position
    Gripper_position = split_2_3_bytes(Gripper_data[0])
    test_list.append(Gripper_position[2:4])

    # Gripper speed
    Gripper_speed = split_2_3_bytes(Gripper_data[1])
    test_list.append(Gripper_speed[2:4])

    # Gripper current
    Gripper_current = split_2_3_bytes(Gripper_data[2])
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

    # print(test_list)
    return test_list

# Data that we receive from the robot
# Input is data buffer list
# Output is saved to multiproc arrays and variables
## joints(3byte)x6,speed(3byte)x6,homed(byte),I/O(byte),temp_error(byte),position_error(byte),timing_data(2byte),Timeout_error(byte),xtr2(byte)
# Gripper data == Position(2byte),speed(2byte),current(2byte),status(byte),obj_detection(byte),ID(byte)
## CRC(byte),end1(byte),end2(byte)
# Last 2 bytes are end bytes but we dont unpack then since we chech their validity elsewhere
def unpack_data_test(data_buffer_list):
    joints = []
    speed = []

    for i in range(0, 18, 3):
        variable = data_buffer_list[i:i + 3]
        joints.append(variable)

    for i in range(18, 36, 3):
        variable = data_buffer_list[i:i + 3]
        speed.append(variable)

    homed = data_buffer_list[36]
    io_var = data_buffer_list[37]
    temp_error = data_buffer_list[38]
    position_error = data_buffer_list[39]
    timing_data = data_buffer_list[40:42]
    timeout_error = data_buffer_list[42]
    xtr2 = data_buffer_list[43]
    device_id = data_buffer_list[44]
    gripper_position = data_buffer_list[45:47]
    gripper_speed = data_buffer_list[47:49]
    gripper_current = data_buffer_list[49:51]
    status = data_buffer_list[51]
    object_detection = data_buffer_list[52]
    crc_byte = data_buffer_list[53]
    endy_byte1 = data_buffer_list[54]
    endy_byte2 = data_buffer_list[55]

    logging.debug("Robot position")
    logging.debug(joints)
    logging.debug("Robot speed")
    logging.debug(speed)
    logging.debug("Robot homed")
    logging.debug(homed)
    logging.debug("Robot I/O data")
    logging.debug(io_var)
    logging.debug("Robot temp error data")
    logging.debug(temp_error)
    logging.debug("Robot position error data")
    logging.debug(position_error)
    logging.debug("Robot timig data")
    logging.debug(timing_data)
    logging.debug("Robot timig error data")
    logging.debug(timeout_error)
    logging.debug("Robot additional byte 2")
    logging.debug(xtr2)
    logging.debug("Gripper device ID")
    logging.debug(device_id)
    logging.debug("Gripper position")
    logging.debug(gripper_position)
    logging.debug("Gripper speed")
    logging.debug(gripper_speed)
    logging.debug("Gripper current")
    logging.debug(gripper_current)
    logging.debug("Gripper status")
    logging.debug(status)
    logging.debug("Gripper object detection")
    logging.debug(object_detection)
    logging.debug("CRC byte")
    logging.debug(crc_byte)
    logging.debug("End byte 1")
    logging.debug(endy_byte1)
    logging.debug("End byte 2")
    logging.debug(endy_byte2)

def get_data(position_in, speed_in, homed_in, in_out_in, temperature_error_in, position_error_in, timeout_error,
             timing_data_in,
             xtr_data, gripper_data_in):
    global input_byte

    global start_cond1_byte
    global start_cond2_byte
    global start_cond3_byte

    global end_cond1_byte
    global end_cond2_byte

    global start_cond1
    global start_cond2
    global start_cond3

    global good_start
    global data_len

    global data_buffer
    global data_counter

    while (ser.inWaiting() > 0):
        input_byte = ser.read()

        # UNCOMMENT THIS TO GET ALL DATA FROM THE ROBOT PRINTED
        # print(input_byte)

        # When data len is received start is good and after that put all data in receive buffer
        # Data len is ALL data after it; that includes input buffer, end bytes and CRC
        if good_start != 1:
            # All start bytes are good and next byte is data len
            if start_cond1 == 1 and start_cond2 == 1 and start_cond3 == 1:
                good_start = 1
                data_len = input_byte
                data_len = struct.unpack('B', data_len)[0]
                logging.debug("data len we got from robot packet= ")
                logging.debug(input_byte)
                logging.debug("good start for DATA that we received at PC")
            # Third start byte is good
            if input_byte == start_cond3_byte and start_cond2 == 1 and start_cond1 == 1:
                start_cond3 = 1
                # print("good cond 3 PC")
            # Third start byte is bad, reset all flags
            elif start_cond2 == 1 and start_cond1 == 1:
                start_cond1 = 0
                start_cond2 = 0
            # Second start byte is good
            if input_byte == start_cond2_byte and start_cond1 == 1:
                start_cond2 = 1
            # Second start byte is bad, reset all flags
            elif start_cond1 == 1:
                start_cond1 = 0
            # First start byte is good
            if input_byte == start_cond1_byte:
                start_cond1 = 1
        else:
            # Here data goes after good  start
            data_buffer[data_counter] = input_byte
            if data_counter == data_len - 1:

                logging.debug("Data len PC")
                logging.debug(data_len)
                logging.debug("End bytes are:")
                logging.debug(data_buffer[data_len - 1])
                logging.debug(data_buffer[data_len - 2])

                # Here if last 2 bytes are end condition bytes we process the data
                if data_buffer[data_len - 1] == end_cond2_byte and data_buffer[data_len - 2] == end_cond1_byte:
                    logging.debug("GOOD END CONDITION PC")
                    logging.debug("I UNPACKED RAW DATA RECEIVED FROM THE ROBOT")
                    unpack_data(data_buffer, position_in, speed_in, homed_in, in_out_in, temperature_error_in,
                                position_error_in, timeout_error, timing_data_in,
                                xtr_data, gripper_data_in)
                    logging.debug("DATA UNPACK FINISHED")
                    # ako su dobri izračunaj crc
                    # if crc dobar raspakiraj podatke
                    # ako je dobar paket je dobar i spremi ga u nove variable!

                # Print every byte
                # print("podaci u data bufferu su:")
                # for i in range(data_len):
                # print(data_buffer[i])

                good_start = 0
                start_cond1 = 0
                start_cond3 = 0
                start_cond2 = 0
                data_len = 0
                data_counter = 0
            else:
                data_counter = data_counter + 1
