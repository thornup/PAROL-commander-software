from core.byte_operations import *
from core.common import pack_data_test, unpack_data_test


# Dummy test task
# Best used to show data that we get from the robot and data we get from gui
def display_data(shared_string, position_out, speed_out, command_out, affected_joint_out, in_out_out, timeout_out,
                 gripper_data_out,
                 position_in, speed_in, homed_in, in_out_in, temperature_error_in, position_error_in, timeout_error,
                 timing_data_in,
                 xtr_data, gripper_data_in,
                 joint_jog_buttons, cart_jog_buttons, jog_control, general_data, buttons):
    while (1):
        elements = list(range(60))
        unpack_data_test(elements)
        pack_data_test()

        print("")

        print("ROBOT DATA: ")
        print("Position from robot is: ", end="")
        print(position_in[:])
        print("Speed from robot is: ", end="")
        print(speed_in[:])
        print("Robot homed status is: ", end="")
        print(homed_in[:])
        print("Robot Input/Output status is: ", end="")
        print(in_out_in[:])
        print("Robot temperature error status is: ", end="")
        print(temperature_error_in[:])
        print("Robot temperature error status is: ", end="")
        print(position_error_in[:])
        print("Timeout_error is: ", end="")
        print(timeout_error.value)
        print("Time between 2 commands raw is: ", end="")
        print(timing_data_in.value)
        print("Time between 2 commands in ms is: ", end="")
        print(timing_data_in.value * 1.42222222e-6)
        print("XTR_DATA byte is: ", end="")
        print(xtr_data.value)
        print("Gripper ID is: ", end="")
        print(gripper_data_in[0])
        print("Gripper position is: ", end="")
        print(gripper_data_in[1])
        print("Gripper speed is: ", end="")
        print(gripper_data_in[2])
        print("Gripper current is: ", end="")
        print(gripper_data_in[3])
        print("Gripper status is: ", end="")
        print(gripper_data_in[4])
        print("Gripper object detection is: ", end="")
        print(gripper_data_in[5])

        print("")

        print("COMMANDED DATA: ")
        print("Robot Input/Output status is (OUT): ", end="")
        print(in_out_out[:])
        print("Robot Commands is:  ", end="")
        print(command_out.value)
        print("Commanded robot speeds are: ", end="")
        print(speed_out[:])

        print("")

        print("gui DATA: ")
        print("Joint jog buttons: ", end="")
        print(list(joint_jog_buttons))
        print("Cart jog buttons: ", end="")
        print(list(cart_jog_buttons))
        print("Home button state:", end="")
        print(buttons[0])
        print("Enable button state:", end="")
        print(buttons[1])
        print("Disable button state:", end="")
        print(buttons[2])
        print("Clear error button state:", end="")
        print(buttons[3])
        print("Real robot state: ", end="")
        print(buttons[4])
        print("Simulator robot state: ", end="")
        print(buttons[5])
        print("Speed slider is: ", end="")
        print(jog_control[0])
        print("WRF/TRF is: ", end="")
        print(jog_control[2])
        print("Demo app button state: ", end="")
        print(buttons[6])
        print("Shared string is: ", end="")
        print(shared_string.value)
        print("Program execution variable: ", end="")
        print(buttons[7])
        print("Park button state: ", end="")
        print(buttons[8])
        time.sleep(3)
