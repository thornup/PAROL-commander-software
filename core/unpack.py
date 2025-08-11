from core.byte_operations import *

# Data that we receive from the robot
#Input is data buffer list
#Output is saved to multiproc arrays and variables
## joints(3byte)x6,speed(3byte)x6,homed(byte),I/O(byte),temp_error(byte),position_error(byte),timing_data(2byte),Timeout_error(byte),xtr2(byte)
# Gripper data == Position(2byte),speed(2byte),current(2byte),status(byte),obj_detection(byte),ID(byte)
## CRC(byte),end1(byte),end2(byte)
# Last 2 bytes are end bytes but we dont unpack then since we check their validity elsewhere
def unpack_data(data_buffer_list, position_in, speed_in, homed_in, in_out_in, temperature_error_in, position_error_in, timeout_error, timing_data_in,
                xtr_data, gripper_data_in):

    joints = []
    speed = []

    for i in range(0,18, 3):
        variable = data_buffer_list[i:i+3]
        joints.append(variable)

    for i in range(18,36, 3):
        variable = data_buffer_list[i:i+3]
        speed.append(variable)


    for i in range(6):
        var =  b'\x00' + b''.join(joints[i])
        position_in[i] = fuse_3_bytes(var)
        var =  b'\x00' + b''.join(speed[i])
        speed_in[i] = fuse_3_bytes(var)

    homed = data_buffer_list[36]
    io_var = data_buffer_list[37]
    temp_error = data_buffer_list[38]
    position_error = data_buffer_list[39]
    timing_data = data_buffer_list[40:42]
    timeout_error_var = data_buffer_list[42]
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

    temp = split_2_bitfield(int.from_bytes(homed,"big"))
    for i in range(8):
        homed_in[i] = temp[i]

    logging.debug("Robot I/O data")
    logging.debug(io_var)

    temp = split_2_bitfield(int.from_bytes(io_var,"big"))
    for i in range(8):
        in_out_in[i] = temp[i]

    logging.debug("Robot temp error data")
    logging.debug(temp_error)

    temp = split_2_bitfield(int.from_bytes(temp_error,"big"))
    for i in range(8):
        temperature_error_in[i] = temp[i]

    logging.debug("Robot position error data")
    logging.debug(position_error)

    temp = split_2_bitfield(int.from_bytes(position_error,"big"))
    for i in range(8):
        position_error_in[i] = temp[i]

    logging.debug("Robot timig data")
    logging.debug(timing_data)
    logging.debug("Robot timig data fused")
    var = b'\x00' + b'\x00' + b''.join(timing_data)
    logging.debug(var)
    logging.debug("Robot timig data fused 2")
    var2 = fuse_3_bytes(var)
    timing_data_in.value = var2
    logging.debug(var2)
    logging.debug("Timing in ms")
    logging.debug(var2 * 1.4222222e-6)
    logging.debug(var2)
    logging.debug("Robot timig error data")
    logging.debug(timeout_error_var)

    timeout_error.value = int.from_bytes(timeout_error_var, "big")

    logging.debug("Robot additional byte 2")
    logging.debug(xtr2)

    xtr_data.value = int.from_bytes(xtr2, "big")

    logging.debug("Gripper device ID")
    logging.debug(device_id)

    gripper_data_in[0] = int.from_bytes(device_id, "big")

    logging.debug("Gripper position")
    logging.debug(gripper_position)

    var =  b'\x00'+ b'\x00' + b''.join(gripper_position)
    gripper_data_in[1] = fuse_2_bytes(var)

    logging.debug("Gripper speed")
    logging.debug(gripper_speed)


    var =  b'\x00'+ b'\x00' + b''.join(gripper_speed)
    gripper_data_in[2] = fuse_2_bytes(var)

    logging.debug("Gripper current")
    logging.debug(gripper_current)


    var =  b'\x00'+ b'\x00' + b''.join(gripper_current)
    gripper_data_in[3] = fuse_2_bytes(var)

    logging.debug("Gripper status")
    logging.debug(status)

    gripper_data_in[4] = int.from_bytes(status, "big")

    logging.debug("Gripper object detection")
    logging.debug(object_detection)

    gripper_data_in[5] = int.from_bytes(object_detection, "big")

    logging.debug("CRC byte")
    logging.debug(crc_byte)
    logging.debug("End byte 1")
    logging.debug(endy_byte1)
    logging.debug("End byte 2")
    logging.debug(endy_byte2)
