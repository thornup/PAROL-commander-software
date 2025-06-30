# Dummy test task
# Best used to show data that we get from the robot and data we get from gui
def Task3(shared_string, Position_out, Speed_out, Command_out, Affected_joint_out, InOut_out, Timeout_out,
          Gripper_data_out,
          Position_in, Speed_in, Homed_in, InOut_in, Temperature_error_in, Position_error_in, Timeout_error,
          Timing_data_in,
          XTR_data, Gripper_data_in,
          Joint_jog_buttons, Cart_jog_buttons, Jog_control, General_data, Buttons):
    while (1):
        start_bytes = [0xff, 0xff, 0xff]
        print(start_bytes)
        start_bytes = bytes(start_bytes)
        print(start_bytes)
        a = Split_2_bitfield(123)
        print(a)
        b = Split_2_3_bytes(-235005)
        print(b)
        print(hex(b[0]))
        print(hex(b[1]))
        print(hex(b[2]))
        print(bytes([b[0]]))
        print(bytes([b[1]]))
        print(bytes([b[2]]))
        print("fuesd")
        c = Fuse_3_bytes(b)
        print(c)
        d = Fuse_bitfield_2_bytearray(a)
        print(d)
        test_list = [10] * 50
        elements = list(range(60))
        Unpack_data_test(elements)
        print("$$$$$$$$$$$$$$$$")
        Pack_data_test()

        print("")

        print("ROBOT DATA: ")
        print("Position from robot is: ", end="")
        print(Position_in[:])
        print("Speed from robot is: ", end="")
        print(Speed_in[:])
        print("Robot homed status is: ", end="")
        print(Homed_in[:])
        print("Robot Input/Output status is: ", end="")
        print(InOut_in[:])
        print("Robot temperature error status is: ", end="")
        print(Temperature_error_in[:])
        print("Robot temperature error status is: ", end="")
        print(Position_error_in[:])
        print("Timeout_error is: ", end="")
        print(Timeout_error.value)
        print("Time between 2 commands raw is: ", end="")
        print(Timing_data_in.value)
        print("Time between 2 commands in ms is: ", end="")
        print(Timing_data_in.value * 1.42222222e-6)
        print("XTR_DATA byte is: ", end="")
        print(XTR_data.value)
        print("Gripper ID is: ", end="")
        print(Gripper_data_in[0])
        print("Gripper position is: ", end="")
        print(Gripper_data_in[1])
        print("Gripper speed is: ", end="")
        print(Gripper_data_in[2])
        print("Gripper current is: ", end="")
        print(Gripper_data_in[3])
        print("Gripper status is: ", end="")
        print(Gripper_data_in[4])
        print("Gripper object detection is: ", end="")
        print(Gripper_data_in[5])

        print("")

        print("COMMANDED DATA: ")
        print("Robot Input/Output status is (OUT): ", end="")
        print(InOut_out[:])
        print("Robot Commands is:  ", end="")
        print(Command_out.value)
        print("Commanded robot speeds are: ", end="")
        print(Speed_out[:])

        print("")

        print("gui DATA: ")
        print("Joint jog buttons: ", end="")
        print(list(Joint_jog_buttons))
        print("Cart jog buttons: ", end="")
        print(list(Cart_jog_buttons))
        print("Home button state:", end="")
        print(Buttons[0])
        print("Enable button state:", end="")
        print(Buttons[1])
        print("Disable button state:", end="")
        print(Buttons[2])
        print("Clear error button state:", end="")
        print(Buttons[3])
        print("Real robot state: ", end="")
        print(Buttons[4])
        print("Simulator robot state: ", end="")
        print(Buttons[5])
        print("Speed slider is: ", end="")
        print(Jog_control[0])
        print("WRF/TRF is: ", end="")
        print(Jog_control[2])
        print("Demo app button state: ", end="")
        print(Buttons[6])
        print("Shared string is: ", end="")
        print(shared_string.value)
        print("Program execution variable: ", end="")
        print(Buttons[7])
        print("Park button state: ", end="")
        print(Buttons[8])

        time.sleep(3)


# Data that we receive from the robot
# Input is data buffer list
# Output is saved to multiproc arrays and variables
## joints(3byte)x6,speed(3byte)x6,homed(byte),I/O(byte),temp_error(byte),position_error(byte),timing_data(2byte),Timeout_error(byte),xtr2(byte)
# Gripper data == Position(2byte),speed(2byte),current(2byte),status(byte),obj_detection(byte),ID(byte)
## CRC(byte),end1(byte),end2(byte)
# Last 2 bytes are end bytes but we dont unpack then since we chech their validity elsewhere
def Unpack_data_test(data_buffer_list):
    Joints = []
    Speed = []

    for i in range(0, 18, 3):
        variable = data_buffer_list[i:i + 3]
        Joints.append(variable)

    for i in range(18, 36, 3):
        variable = data_buffer_list[i:i + 3]
        Speed.append(variable)

    Homed = data_buffer_list[36]
    IO_var = data_buffer_list[37]
    temp_error = data_buffer_list[38]
    position_error = data_buffer_list[39]
    timing_data = data_buffer_list[40:42]
    Timeout_error = data_buffer_list[42]
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
    logging.debug("Robot I/O data")
    logging.debug(IO_var)
    logging.debug("Robot temp error data")
    logging.debug(temp_error)
    logging.debug("Robot position error data")
    logging.debug(position_error)
    logging.debug("Robot timig data")
    logging.debug(timing_data)
    logging.debug("Robot timig error data")
    logging.debug(Timeout_error)
    logging.debug("Robot additional byte 2")
    logging.debug(xtr2)
    logging.debug("Gripper device ID")
    logging.debug(device_ID)
    logging.debug("Gripper position")
    logging.debug(Gripper_position)
    logging.debug("Gripper speed")
    logging.debug(Gripper_speed)
    logging.debug("Gripper current")
    logging.debug(Gripper_current)
    logging.debug("Gripper status")
    logging.debug(Status)
    logging.debug("Gripper object detection")
    logging.debug(object_detection)
    logging.debug("CRC byte")
    logging.debug(CRC_byte)
    logging.debug("End byte 1")
    logging.debug(endy_byte1)
    logging.debug("End byte 2")
    logging.debug(endy_byte2)
