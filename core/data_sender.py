from oclock import Timer
import time
import roboticstoolbox as rp
import logging
import numpy as np
from spatialmath import *
import re
import serial as ser
from roboticstoolbox import trapezoidal
from roboticstoolbox import quintic

from config import robot_config
from core.common import my_os, prev_speed, pack_data, image_path, INTERVAL_S

# Check if there is element 1 in the list.
# If yes return its index, if no element is 1 return -1
def check_elements(lst):
    for i, element in enumerate(lst):
        if element == 1:
            return i
    return -1  # Return -1 if no element is 1

def extract_content_from_command(command):
    '''
    Treba mi bytes format za slanje, nije baš user readable je pretvori iz hex u ascii
    ako trebam gledati vrijednosti koristi hex() funkciju
    '''
    match = re.search(r'\((.*?)\)', command)
    if match:
        return match.group(1)
    else:
        return None

# Task for sending data every x ms and performing all calculations, kinematics gui core logic...
def send_data(shared_string, position_out, speed_out, command_out, affected_joint_out, in_out_out, timeout_out,
              gripper_data_out,
              position_in, speed_in, homed_in, in_out_in, temperature_error_in, position_error_in, timeout_error,
              timing_data_in,
              xtr_data, gripper_data_in,
              joint_jog_buttons, cart_jog_buttons, jog_control, general_data, buttons):
    timer = Timer(INTERVAL_S, warnings=False, precise=True)
    cnt = 0

    while timer.elapsed_time < 110000:

        if ser.is_open == True:
            logging.debug("Task 1 alive")
            logging.debug("Data that PC will send to the robot is: ")
            # This function packs data that we will send to the robot
            s = pack_data(position_out, speed_out, command_out, affected_joint_out, in_out_out, timeout_out,
                          gripper_data_out)

            # Make sure if sending calib to gripper to send it only once
            if (gripper_data_out[4] == 1 or gripper_data_out[4] == 2):
                gripper_data_out[4] = 0

            logging.debug(s)
            logging.debug("END of data sent to the ROBOT")
            len_ = len(s)
            try:
                for i in range(len_):
                    ser.write(s[i])
            except:
                logging.debug("NO SERIAL TASK1")
                # This function packs data that we will send to the robot

            # Check if any of jog buttons is pressed
            result_joint_jog = check_elements(list(joint_jog_buttons))
            result_cart_jog = check_elements(list(cart_jog_buttons))

            ######################################################
            ######################################################

            # JOINT JOG (regular speed core) 0x123 # -1 is value if nothing is pressed
            if result_joint_jog != -1 and buttons[2] == 0 and in_out_in[4] == 1:
                robot_mode = "Joint jog"
                command_out.value = 123
                # Set speed for all other joints to 0
                for i in range(6):
                    speed_out[i] = 0
                    # ako je position in veći ili jednak nekom od limita disable tu stranu tipki
                # Set speed for the clicked joint
                if result_joint_jog in [0, 1, 2, 3, 4, 5]:
                    if position_in[result_joint_jog] >= robot_config.Joint_limits_steps[result_joint_jog][1]:
                        shared_string.value = b'Error: Robot jog -> Position out of range'
                    else:
                        speed_out[result_joint_jog] = int(np.interp(jog_control[0], [0, 100],
                                                                    [robot_config.joint_min_jog_speed[result_joint_jog],
                                                                     robot_config.joint_max_jog_speed[
                                                                         result_joint_jog]]))
                        arr = bytes(str(result_joint_jog + 1), 'utf-8')
                        shared_string.value = b'Log: Joint  ' + arr + b'  jog  '
                else:
                    if position_in[result_joint_jog - 6] <= robot_config.Joint_limits_steps[result_joint_jog - 6][0]:
                        shared_string.value = b'Error: Robot jog -> Position out of range'
                    else:
                        speed_out[result_joint_jog - 6] = int(-1 * np.interp(jog_control[0], [0, 100], [
                            robot_config.joint_min_jog_speed[result_joint_jog - 6],
                            robot_config.joint_max_jog_speed[result_joint_jog - 6]]))
                        arr = bytes(str(result_joint_jog - 6 + 1), 'utf-8')
                        shared_string.value = b'Log: Joint  ' + arr + b'  jog  '

                        ######################################################
            ######################################################
            # CART JOG (regular speed core but for multiple joints) 0x123 # -1 is value if nothing is pressed
            elif result_cart_jog != -1 and buttons[2] == 0 and in_out_in[4] == 1:  #

                command_out.value = 123
                # Set speed for all other joints to 0
                for i in range(6):
                    speed_out[i] = 0
                # if moving in positive direction
                q1 = np.array([robot_config.STEPS2RADS(position_in[0], 0),
                               robot_config.STEPS2RADS(position_in[1], 1),
                               robot_config.STEPS2RADS(position_in[2], 2),
                               robot_config.STEPS2RADS(position_in[3], 3),
                               robot_config.STEPS2RADS(position_in[4], 4),
                               robot_config.STEPS2RADS(position_in[5], 5), ])
                T = robot_config.robot.fkine(q1)

                temp_var = float(np.interp(jog_control[0], [0, 100], [robot_config.cartesian_linear_velocity_min_jog,
                                                                      robot_config.cartesian_linear_velocity_max_jog]))
                temp_var_angular = float(np.interp(jog_control[0], [0, 100],
                                                   [robot_config.cartesian_angular_velocity_min,
                                                    robot_config.cartesian_angular_velocity_max]))

                speed_temp = temp_var  # Speed is 20mm/s = 0.02m/s
                speed_temp_angular = temp_var_angular  # Speed is DEG/s

                delta_s = speed_temp * INTERVAL_S  # displacement in meters
                delta_s_angular = speed_temp_angular * INTERVAL_S  # displacement in degrees

                # WRF jogging
                if jog_control[2] == 1:  # WRF jog
                    if result_cart_jog in [1, 3, 4, 10, 7, 8]:  # For positive directions 1,3,4.
                        if result_cart_jog == 4:  # Z+ direction
                            T.t[2] = T.t[2] + delta_s  # Add to the Z+ direction in WRF
                            shared_string.value = b'Log: Cartesian WRF Z+ move'
                        elif result_cart_jog == 3:  # Y+ direction
                            T.t[1] = T.t[1] + delta_s  # Add to the Y+ direction in WRF
                            shared_string.value = b'Log: Cartesian WRF Y+ move'
                        elif result_cart_jog == 1:  # X+ direction
                            T.t[0] = T.t[0] + delta_s  # Add to the X+ direction in WRF
                            shared_string.value = b'Log: Cartesian WRF X+ move'

                        elif result_cart_jog == 10:  # Rotation in Z+ direction
                            None
                        elif result_cart_jog == 8:  # Rotation in Y+ direction
                            None
                        elif result_cart_jog == 7:  # Rotation in X+ direction
                            None

                    # if moving in negative direction
                    else:
                        if result_cart_jog == 5:  # Z- direction
                            T.t[2] = T.t[2] - delta_s  # Add to the Z- direction in WRF
                            shared_string.value = b'Log: Cartesian WRF Z- move'
                        elif result_cart_jog == 2:  # Y- direction
                            T.t[1] = T.t[1] - delta_s  # Add to the Y- direction in WRF
                            shared_string.value = b'Log: Cartesian WRF Y- move'
                        elif result_cart_jog == 0:  # X- direction
                            T.t[0] = T.t[0] - delta_s  # Add to the X- direction in WRF
                            shared_string.value = b'Log: Cartesian WRF X- move'

                        elif result_cart_jog == 11:  # Rotation in Z- direction
                            None
                        elif result_cart_jog == 9:  # Rotation in Y- direction
                            None
                        elif result_cart_jog == 6:  # Rotation in X- direction
                            None

                # TRF jogging
                else:  # TRF jog
                    if result_cart_jog in [1, 3, 4, 10, 7, 8]:  # For positive directions 1,3,4.
                        if result_cart_jog == 4:  # Z+ direction
                            x1 = [0, 0, delta_s]
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            shared_string.value = b'Log: Cartesian TRF Z+ move'
                        elif result_cart_jog == 3:  # Y+ direction
                            x1 = [0, delta_s, 0]
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            shared_string.value = b'Log: Cartesian TRF Y+ move'
                        elif result_cart_jog == 1:  # X+ direction
                            x1 = [delta_s, 0, 0]
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            shared_string.value = b'Log: Cartesian TRF X+ move'

                        elif result_cart_jog == 10:  # Rotation in Z+ direction
                            T2 = T * T.Rz(delta_s_angular, 'deg')
                            T = T2
                            shared_string.value = b'Log: TRF Z+ Rotation '
                        elif result_cart_jog == 8:  # Rotation in Y+ direction
                            T2 = T * T.Ry(delta_s_angular, 'deg')
                            T = T2
                            shared_string.value = b'Log: TRF Y+ Rotation '
                        elif result_cart_jog == 7:  # Rotation in x+ direction
                            T2 = T * T.Rx(delta_s_angular, 'deg')
                            T = T2
                            shared_string.value = b'Log: TRF X+ Rotation '


                    # if moving in negative direction
                    else:
                        if result_cart_jog == 5:  # Z- direction
                            x1 = [0, 0, -delta_s]
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            shared_string.value = b'Log: Cartesian TRF Z- move'
                        elif result_cart_jog == 2:  # Y- direction
                            x1 = [0, -delta_s, 0]
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            shared_string.value = b'Log: Cartesian TRF Y- move'
                        elif result_cart_jog == 0:  # X- direction
                            x1 = [-delta_s, 0, 0]
                            x2 = T * x1
                            Tt = T
                            Tt.t[0] = x2[0]
                            Tt.t[1] = x2[1]
                            Tt.t[2] = x2[2]
                            T = Tt
                            shared_string.value = b'Log: Cartesian TRF X- move'

                        elif result_cart_jog == 11:  # Rotation in Z- direction
                            T2 = T * T.Rz(-delta_s_angular, 'deg')
                            T = T2
                            shared_string.value = b'Log: TRF Z- Rotation '
                        elif result_cart_jog == 9:  # Rotation in Y- direction
                            T2 = T * T.Ry(-delta_s_angular, 'deg')
                            T = T2
                            shared_string.value = b'Log: TRF Y- Rotation '
                        elif result_cart_jog == 6:  # Rotation in X- direction
                            T2 = T * T.Rx(-delta_s_angular, 'deg')
                            T = T2
                            shared_string.value = b'Log: TRF X- Rotation '

                var = robot_config.robot.ikine_LMS(T, q0=q1, ilimit=6)  # Get joint angles

                temp_var = [0, 0, 0, 0, 0, 0]
                for i in range(6):

                    temp_var[i] = ((var[0][i] - q1[i]) / INTERVAL_S)
                    # print(temp_var)

                    # If solver gives error DISABLE ROBOT
                    if var.success:
                        speed_out[i] = int(robot_config.SPEED_RAD2STEP(temp_var[i], i))
                        prev_speed[i] = speed_out[i]
                    else:
                        shared_string.value = b'Error: Inverse kinematics error '
                        # Command_out.value = 102
                        # Speed_out[i] = 0
                    # If any joint passed its position limit, disable robot

                if robot_mode != "Cartesian jog":
                    for i in range(6):
                        if abs(speed_out[i]) >= 300000:
                            speed_out[i] = int(speed_out[i] / 10000)
                            arr = bytes(str(speed_out[i]), 'utf-8')
                            arr2 = bytes(str(i + 1), 'utf-8')
                            shared_string.value = b'Error: Joint  ' + arr2 + b'  speed error in cart mode  ' + arr


                else:
                    # If any joint starts moving faster than allowed DISABLE ROBOT
                    for i in range(6):
                        if abs(speed_out[i]) >= 300000:
                            command_out.value = 102
                            arr = bytes(str(speed_out[i]), 'utf-8')
                            arr2 = bytes(str(i + 1), 'utf-8')
                            shared_string.value = b'Error: Joint  ' + arr2 + b'  speed error in cart mode  ' + arr
                robot_mode = "Cartesian jog"
                # Calculate every joint speed using var and q1

                # commanded position = robot position
                # if real send to real
                # if sim send to sim
                # if both send to both
                # print(result_joint_jog)



            elif buttons[0] == 1:  # HOME COMMAND 0x100
                command_out.value = 100
                buttons[0] = 0
                shared_string.value = b'Log: Robot homing'


            elif buttons[1] == 1:  # ENABLE COMMAND 0x101
                command_out.value = 101
                buttons[1] = 0
                shared_string.value = b'Log: Robot enable'

            elif buttons[2] == 1 or in_out_in[4] == 0:  # DISABLE COMMAND 0x102
                robot_mode = "STOP"
                buttons[7] = 0  # program execution button
                command_out.value = 102
                buttons[2] = 0
                shared_string.value = b'Log: Robot disable; button or estop'

            elif buttons[3] == 1:  # CLEAR ERROR COMMAND 0x103
                command_out.value = 103
                buttons[3] = 0
                shared_string.value = b'Log: Error clear'

            elif buttons[6] == 1:  # For testing accel motions?
                command_out.value = 69


            # Program execution
            ######################################################
            ######################################################
            elif buttons[7] == 1:  # For program execution

                if robot_mode != "Program":
                    ik_error = 0  # If there is error in ik calculations
                    error_state = 0  # If 1 it is error state
                    program_len = 0  # Length of the program
                    Program_step = 1  # Start from 1 because begin is always index 0 and it does nothing
                    command_step = 0  # counter when stepping thru the command
                    # command_len = variable time / INTERVAL_S
                    command_len = 0  # Length of the command; usually in the commands it is in seconds but here it is in ticks of INTERVAL_S --> command_len = variable time / INTERVAL_S
                    VALID_COMMANDS = robot_config.commands_list_true

                    # Open execute_script.txt
                    text_file = open(image_path + "/Programs/execute_script.txt", 'r')
                    code_string = text_file.readlines()
                    text_file.close()

                    clean_string_commands = []  # Program list without \n anywhere, contains data between ()
                    clean_string = []  # like clean string but without data between ()

                    for i in range(0, len(code_string)):
                        # If there is any \n alone in the document remove them
                        if code_string[i] == '\n':
                            continue
                        else:
                            clean_string.append(code_string[i])

                    print(clean_string)
                    clean_string = [item.rstrip("\n") for item in clean_string]  # Remove \n from all elements
                    clean_string_commands = clean_string
                    print(clean_string)

                    clean_string = [re.sub(r'\(.*?\)', '()', command) for command in
                                    clean_string]  # remove data between ()
                    print(clean_string)
                    print(clean_string_commands)
                    # Now with everything clean check for all errors and see if the code is valid.
                    # Check if all commands exist
                    for command in clean_string:
                        if command not in VALID_COMMANDS:
                            print(f"Error: '{command}' is not a valid command")
                            err = bytes(str(command), 'utf-8')
                            # shared_string.value = b'Error: Command  ' + err +   b'  is not valid  '
                            shared_string.value = b'Error: has invalid commands!'
                            error_state = 1
                            # Set flag, exit program mode
                    program_len = len(clean_string)
                    if clean_string[0] != 'Begin()':
                        None
                        shared_string.value = b'Error: program needs to start with Begin()'
                        error_state = 1
                        # Set error flag, exit program mode
                    if clean_string[program_len - 1] == 'End()' or clean_string[program_len - 1] == 'Loop()':
                        None
                        # All good
                    else:
                        shared_string.value = b'Error: program needs to end with End() or Loop()'
                        error_state = 1
                        # Set error flag, exit program
                    if error_state == 0:
                        shared_string.value = b'Log: program will try to run'

                    # Check if first and last commands are valid
                robot_mode = "Program"

                if error_state == 0:
                    if Program_step < program_len:

                        # Delay command
                        if clean_string[Program_step] == 'Delay()':
                            if command_step == 0:
                                time1 = time.perf_counter()
                                shared_string.value = b'Log: Delay() command'
                                # Extract time from delay
                                Time = extract_content_from_command(clean_string_commands[Program_step])
                                # print(Time)
                                try:
                                    number_int = float(Time)
                                    # Check if the converted integer is greater than our interval
                                    if number_int > INTERVAL_S:
                                        command_len = int(number_int / (INTERVAL_S))
                                        command_out.value = 255  # Set dummy data
                                        command_step = command_step + 1
                                        # print(command_len)
                                    else:
                                        error_state = 1
                                        buttons[7] = 0
                                        shared_string.value = b'Error: Invalid Delay() command'
                                except ValueError:
                                    error_state = 1
                                    buttons[7] = 0
                                    shared_string.value = b'Error: Invalid Delay() command'
                            elif command_step != command_len:  #
                                command_step = command_step + 1
                                command_out.value = 255  # Set dummy data
                            else:
                                time2 = time.perf_counter()
                                print(command_step)
                                print(time2 - time1)
                                command_step = 0
                                command_len = 0
                                Program_step = Program_step + 1
                                command_out.value = 255  # Set dummy data

                        # Output command
                        elif clean_string[Program_step] == 'Output()':
                            # Extract data between ()
                            command_value = extract_content_from_command(clean_string_commands[Program_step])
                            cond1 = 0
                            cond2 = 0
                            if command_value.count(',') == 1:
                                x = command_value.split(',')
                                try:
                                    if int(x[0]) == 1 or int(x[0] == 2):
                                        cond1 = 1
                                    else:
                                        shared_string.value = b'Error: Invalid Output() command'
                                        error_state = 1
                                        buttons[7] = 0
                                        # Give error
                                except ValueError:
                                    shared_string.value = b'Error: Invalid Output() command'
                                    error_state = 1
                                    buttons[7] = 0
                                # Give error

                                if x[1] == 'HIGH' or x[1] == 'LOW':
                                    cond2 = 1
                                else:
                                    shared_string.value = b'Error: Invalid Output() command'
                                    error_state = 1
                                    buttons[7] = 0
                                    # Give error
                                if cond1 == 1 and cond2 == 1:
                                    command_out.value = 255  # Set dummy data
                                    if int(x[0]) == 1:
                                        if x[1] == 'HIGH':
                                            in_out_out[2] = 1
                                            in_out_in[2] = 1
                                            logging.debug('Log: Output 1 HIGH')
                                        elif x[1] == 'LOW':
                                            in_out_out[2] = 0
                                            in_out_in[2] = 0
                                            logging.debug('Log: Output 1 LOW')
                                    if int(x[0]) == 2:
                                        if x[1] == 'HIGH':
                                            in_out_out[3] = 1
                                            in_out_in[3] = 1
                                            logging.debug('Log: Output 2 HIGH')
                                        elif x[1] == 'LOW':
                                            in_out_out[3] = 0
                                            in_out_in[3] = 0
                                            logging.debug('Log: Output 2 LOW')

                                    cond1 = 0
                                    cond2 = 0
                                    Program_step = Program_step + 1
                            else:
                                error_state = 1
                                buttons[7] = 0
                                shared_string.value = b'Error: Invalid Output() command'


                        # Loop command
                        elif clean_string[Program_step] == 'Loop()':
                            logging.debug('Log: Loop() command')
                            Program_step = 1

                        # MoveJoint command
                        elif clean_string[Program_step] == 'MoveJoint()':
                            # This code will execute once per command call
                            if command_step == 0:
                                time1 = time.perf_counter()
                                # data_packet = "test(1,2,3,4,5,6,a=1,v=2,t=0,func,speed)"

                                # Define the pattern using regular expression
                                # pattern = r'MoveJoint\(\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*(?:,\s*v\s*=\s*(\d+))?(?:,\s*a\s*=\s*(\d+))?(?:,\s*t\s*=\s*(\d+))?(?:,\s*(trap|poly))?\s*\)'
                                pattern = r'MoveJoint\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*(?:,\s*v\s*=\s*(-?\d+(?:\.\d+)?))?(?:,\s*a\s*=\s*(-?\d+(?:\.\d+)?))?(?:,\s*t\s*=\s*(-?\d+(?:\.\d+)?))?(?:,\s*(trap|poly))?(?:,\s*(speed))?\s*\)?'

                                # Use re.match to find the pattern in the data packet
                                match = re.match(pattern, clean_string_commands[Program_step])

                                if match:
                                    shared_string.value = b'Log: MoveJoint() command'
                                    groups = match.groups()
                                    numbers = []
                                    for num_str in groups[:6]:
                                        try:
                                            num = float(num_str)
                                            numbers.append(num)
                                        except ValueError:
                                            print(f"Invalid number: {num_str}")
                                            error_state = 1
                                            buttons[7] = 0
                                            break

                                    v_value = float(groups[6]) if groups[6] is not None else None
                                    a_value = float(groups[7]) if groups[7] is not None else None
                                    t_value = float(groups[8]) if groups[8] is not None else None
                                    additional_element = groups[9] if groups[9] is not None else None
                                    tracking = groups[10] if groups[10] is not None else None

                                    # initial pos and needed pos
                                    initial_pos = np.array([robot_config.STEPS2RADS(position_in[0], 0),
                                                            robot_config.STEPS2RADS(position_in[1], 1),
                                                            robot_config.STEPS2RADS(position_in[2], 2),
                                                            robot_config.STEPS2RADS(position_in[3], 3),
                                                            robot_config.STEPS2RADS(position_in[4], 4),
                                                            robot_config.STEPS2RADS(position_in[5], 5), ])

                                    needed_pos = np.array([robot_config.DEG2RAD(numbers[0] + 0.0000001),
                                                           robot_config.DEG2RAD(numbers[1] + 0.0000001),
                                                           robot_config.DEG2RAD(numbers[2] + 0.0000001),
                                                           robot_config.DEG2RAD(numbers[3] + 0.0000001),
                                                           robot_config.DEG2RAD(numbers[4] + 0.0000001),
                                                           robot_config.DEG2RAD(numbers[5] + 0.0000001), ])

                                    needed_pos_steps = np.array([int(robot_config.DEG2STEPS(numbers[0], 0)),
                                                                 int(robot_config.DEG2STEPS(numbers[1], 1)),
                                                                 int(robot_config.DEG2STEPS(numbers[2], 2)),
                                                                 int(robot_config.DEG2STEPS(numbers[3], 3)),
                                                                 int(robot_config.DEG2STEPS(numbers[4], 4)),
                                                                 int(robot_config.DEG2STEPS(numbers[5], 5)), ])

                                    # Check if needed positions are in range
                                    for i in range(6):
                                        if needed_pos[i] >= robot_config.joint_limits_radian[i][1] or needed_pos[i] <= \
                                                robot_config.joint_limits_radian[i][0]:
                                            shared_string.value = b'Error: MoveJoint needed position out of range'
                                            # print(f"Joint is out of range: {i + 1}")
                                            error_state = 1
                                            buttons[7] = 0
                                            break

                                    # print(f"initial pos is : {initial_pos}")
                                    # print(f"needed pos is : {needed_pos}")

                                    if additional_element is not None and additional_element not in ("trap", "poly"):
                                        # print("Invalid additional element:", additional_element)
                                        error_state = 1
                                        buttons[7] = 0
                                    else:
                                        # If t is defined *ignore all other params, if func is defined use that func, else use poly
                                        if t_value != None and t_value > 0 and t_value != 0:
                                            command_len = int(t_value / INTERVAL_S)
                                            command_step = command_step + 1
                                            t2 = (np.arange(0, t_value, INTERVAL_S))
                                            timebase_defined = "t"
                                            # print(" t is not none")

                                            if additional_element == "poly" or additional_element == None:
                                                qx2 = rp.tools.trajectory.jtraj(initial_pos, needed_pos, command_len)

                                            elif additional_element == "trap":

                                                qx2 = rp.tools.trajectory.mtraj(trapezoidal, initial_pos, needed_pos,
                                                                                command_len)


                                        # if t is not defined; use v and a values, trapezoidal is used always
                                        elif (t_value == 0 or t_value == None) and v_value != None and a_value != None:

                                            if a_value > 100 or a_value < 0:
                                                # print("error a_value too small")
                                                error_state = 1
                                                buttons[7] = 0
                                                shared_string.value = b'Error: MoveJoint() command acceleration setpoint out of range'
                                            if v_value > 100 or v_value < 0:
                                                # print("error v_value too small")
                                                error_state = 1
                                                buttons[7] = 0
                                                shared_string.value = b'Error: MoveJoint() command velocity setpoint out of range'

                                            path_differences = np.abs(needed_pos - initial_pos)
                                            # print(path_differences)

                                            # Find the index with the maximum difference
                                            max_path_index = np.argmax(path_differences)
                                            # print("index with max path is:",max_path_index)

                                            # find if any joint angles are the same. ignore those in calculations
                                            # print("needed positons in steps:",needed_pos_steps)
                                            # print("current positons in steps:")
                                            for i in range(6):
                                                print(" positon is", position_in[i])
                                            matching_indexes = np.where(needed_pos_steps == position_in)[0]
                                            # print("matching indexes are:",matching_indexes)

                                            # init arrays
                                            v_value_array = np.array([0, 0, 0, 0, 0, 0])
                                            trap_calc = np.array([None, None, None, None, None, None])

                                            # calculate speed and acc for leading joint using set %
                                            v_value_array[max_path_index] = 2000
                                            a_value_real = 1000

                                            v_value_array[max_path_index] = (np.interp(v_value, [0, 100], [
                                                robot_config.joint_min_speed[max_path_index],
                                                robot_config.joint_max_speed[max_path_index]]))
                                            a_value_real = (np.interp(a_value, [0, 100], [robot_config.joint_min_acc,
                                                                                          robot_config.joint_max_acc]))
                                            # print("a value is:", a_value_real)
                                            # print("v value is:", v_value_array[max_path_index])
                                            # from leading profile calculate acceleration time and total duration of the move
                                            tacc = v_value_array[max_path_index] / a_value_real
                                            # print("tacc is:",tacc)
                                            total_t = abs(
                                                needed_pos_steps[max_path_index] - position_in[max_path_index]) / \
                                                      v_value_array[max_path_index] + tacc
                                            # print("total_t is:")
                                            # print(total_t)
                                            execution_time = (np.arange(0, total_t, INTERVAL_S))

                                            # calculate all speeds and position profiles, ignore ones where needed and initial positon are the same
                                            for i in range(6):
                                                # if needed and initial are the same dont calculate anything
                                                if i in matching_indexes:
                                                    # print("positions are the same. joint at index: " ,i)
                                                    continue
                                                v_value_array[i] = abs(needed_pos_steps[i] - position_in[i]) / (
                                                            total_t - tacc)
                                                try:
                                                    trap_calc[i] = trapezoidal(position_in[i], needed_pos_steps[i],
                                                                               execution_time, v_value_array[i])
                                                    # print("good trap profile was made for index",i)
                                                    # trap_calc[i].plot()
                                                except:
                                                    # if needed and initial are not the same but path is really small use 1/3 acc, 1/3 cruise and 1/3 deac
                                                    trap_calc[i] = trapezoidal(position_in[i], needed_pos_steps[i],
                                                                               execution_time)
                                                    # print("ERROR acc is too small or v is to big index is:",i)

                                            for i in range(6):
                                                if i in matching_indexes:
                                                    continue
                                                if np.any(abs(trap_calc[i].qd) > robot_config.joint_max_speed[i]):
                                                    shared_string.value = b'Error: MoveJoint() speed or acceleration too big'
                                                    # print("error in joint:", i)
                                                    error_state = 1
                                                    buttons[7] = 0

                                            # TODO make sure accel and speed values are in the range
                                            # First find what joint has largerst path to travel
                                            # Calculate the absolute difference between elements of a and b

                                            command_len = int(total_t / INTERVAL_S)
                                            command_step = command_step + 1
                                            timebase_defined = "v and a"
                                            # print("t is none, a and v are defined")
                                            # use calculations


                                        # is none are defined or just one of speed and acc. Use conservative values, trapezoidal is used always
                                        elif (t_value == 0 or t_value == None) or v_value == None or a_value == None:

                                            a_value = 27
                                            v_value = 45

                                            path_differences = np.abs(needed_pos - initial_pos)
                                            print(path_differences)

                                            # Find the index with the maximum difference
                                            max_path_index = np.argmax(path_differences)
                                            # print("index with max path is:",max_path_index)

                                            # find if any joint angles are the same. ignore those in calculations
                                            matching_indexes = np.where(needed_pos_steps == position_in)[0]
                                            # print("matching indexes are:",matching_indexes)

                                            # init arrays
                                            v_value_array = np.array([0, 0, 0, 0, 0, 0])
                                            trap_calc = np.array([None, None, None, None, None, None])

                                            # calculate speed and acc for leading joint using set %
                                            v_value_array[max_path_index] = 2000
                                            a_value_real = 1000

                                            v_value_array[max_path_index] = (np.interp(v_value, [0, 100], [
                                                robot_config.joint_min_speed[max_path_index],
                                                robot_config.joint_max_speed[max_path_index]]))
                                            a_value_real = (np.interp(a_value, [0, 100], [robot_config.joint_min_acc,
                                                                                          robot_config.joint_max_acc]))
                                            # print("a value is:", a_value_real)
                                            # print("v value is:", v_value_array[max_path_index])
                                            # from leading profile calculate acceleration time and total duration of the move
                                            tacc = v_value_array[max_path_index] / a_value_real
                                            # print("tacc is:",tacc)
                                            total_t = abs(
                                                needed_pos_steps[max_path_index] - position_in[max_path_index]) / \
                                                      v_value_array[max_path_index] + tacc
                                            # print("total_t is:")
                                            # print(total_t)
                                            execution_time = (np.arange(0, total_t, INTERVAL_S))

                                            # calculate all speeds and position profiles, ignore ones where needed and initial positon are the same
                                            for i in range(6):
                                                # if needed and initial are the same dont calculate anything
                                                if i in matching_indexes:
                                                    # print("positions are the same. joint at index: " ,i)
                                                    continue
                                                v_value_array[i] = abs(needed_pos_steps[i] - position_in[i]) / (
                                                            total_t - tacc)
                                                try:
                                                    trap_calc[i] = trapezoidal(position_in[i], needed_pos_steps[i],
                                                                               execution_time, v_value_array[i])
                                                    # print("good trap profile was made for index",i)
                                                    # trap_calc[i].plot()
                                                except:
                                                    # if needed and initial are not the same but path is really small use 1/3 acc, 1/3 cruise and 1/3 deac
                                                    trap_calc[i] = trapezoidal(position_in[i], needed_pos_steps[i],
                                                                               execution_time)
                                                    # print("ERROR acc is too small or v is to big index is:",i)

                                            for i in range(6):
                                                if i in matching_indexes:
                                                    continue
                                                if np.any(abs(trap_calc[i].qd) > robot_config.joint_max_speed[i]):
                                                    shared_string.value = b'Error: MoveJoint() speed or acceleration too big'
                                                    # print("error in joint:", i)
                                                    error_state = 1
                                                    buttons[7] = 0

                                            # TODO make sure accel and speed values are in the range
                                            # First find what joint has largerst path to travel
                                            # Calculate the absolute difference between elements of a and b

                                            command_len = int(total_t / INTERVAL_S)
                                            command_step = command_step + 1
                                            timebase_defined = "None"
                                            print("Using conservative values")
                                            # use calculations




                                        # Error state?
                                        else:
                                            command_len = 1000
                                            command_step = command_step + 1
                                            # flag error unknown state
                                            print("unknown state?")

                                            # print(qx2)
                                        print(needed_pos)

                                        print("Numbers:", numbers)
                                        print("Value of 'v':", v_value)
                                        print("Value of 'a':", a_value)
                                        print("Value of 't':", t_value)
                                        print("Function element:", additional_element)
                                        print("Tracking (speed):", tracking)
                                else:
                                    shared_string.value = b'Error: Invalid MoveJoint() command'
                                    error_state = 1
                                    buttons[7] = 0

                            elif command_step != command_len:  #

                                if timebase_defined == "t":
                                    for i in range(6):

                                        if additional_element == "trap":
                                            speed_out[i] = int(robot_config.SPEED_RAD2STEP(
                                                qx2.qd[command_step][i] / (t_value - INTERVAL_S), i)) * (
                                                                       command_len - 1)
                                            position_out[i] = int(robot_config.RAD2STEPS(qx2.q[command_step][i], i))
                                        elif additional_element == "poly" or additional_element == None:
                                            speed_out[i] = int(robot_config.SPEED_RAD2STEP(
                                                qx2.qd[command_step][i] / (t_value - INTERVAL_S), i))  # * 199
                                            position_out[i] = int(robot_config.RAD2STEPS(qx2.q[command_step][i], i))

                                elif timebase_defined == "v and a" or timebase_defined == "None":

                                    for i in range(6):
                                        if i in matching_indexes:
                                            speed_out[i] = 0
                                            position_out[i] = position_in[i]
                                            continue
                                        try:
                                            temp_var_traj = trap_calc[i]
                                            speed_out[i] = int(temp_var_traj.qd[command_step])
                                            position_out[i] = int(temp_var_traj.q[command_step])
                                        except:
                                            speed_out[i] = 0
                                            position_out[i] = position_in[i]
                                            # print("ERROR acc is too small or v is to big index is:",i)

                                # print(Speed_out[5])
                                # print(Speed_out[0])
                                command_step = command_step + 1

                                if tracking == None:
                                    command_out.value = 156
                                elif tracking == "speed":
                                    command_out.value = 123
                                else:
                                    command_out.value = 255

                            else:
                                time2 = time.perf_counter()
                                command_out.value = 255  # Send command from last index
                                print(command_step)
                                print(time2 - time1)
                                print("MoveJoint done")
                                command_step = 0
                                command_len = 0
                                Program_step = Program_step + 1

                        # Joint space move but with pose
                        elif clean_string[Program_step] == 'MovePose()':
                            # This code will execute once per command call
                            if command_step == 0:
                                time1 = time.perf_counter()
                                # data_packet = "test(1,2,3,4,5,6,a=1,v=2,t=0,func,speed)"

                                # Define the pattern using regular expression
                                pattern = r'MovePose\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*(?:,\s*v\s*=\s*(-?\d+(?:\.\d+)?))?(?:,\s*a\s*=\s*(-?\d+(?:\.\d+)?))?(?:,\s*t\s*=\s*(-?\d+(?:\.\d+)?))?(?:,\s*(trap|poly))?(?:,\s*(speed))?\s*\)?'

                                # Use re.match to find the pattern in the data packet
                                match = re.match(pattern, clean_string_commands[Program_step])

                                if match:
                                    shared_string.value = b'Log: MovePose() command'
                                    groups = match.groups()
                                    numbers = []
                                    for num_str in groups[:6]:
                                        try:
                                            num = float(num_str)
                                            numbers.append(num)
                                        except ValueError:
                                            print(f"Invalid number: {num_str}")
                                            error_state = 1
                                            buttons[7] = 0
                                            break

                                    v_value = float(groups[6]) if groups[6] is not None else None
                                    a_value = float(groups[7]) if groups[7] is not None else None
                                    t_value = float(groups[8]) if groups[8] is not None else None
                                    additional_element = groups[9] if groups[9] is not None else None
                                    tracking = groups[10] if groups[10] is not None else None

                                    # initial pos and needed pos
                                    initial_pos = np.array([robot_config.STEPS2RADS(position_in[0], 0),
                                                            robot_config.STEPS2RADS(position_in[1], 1),
                                                            robot_config.STEPS2RADS(position_in[2], 2),
                                                            robot_config.STEPS2RADS(position_in[3], 3),
                                                            robot_config.STEPS2RADS(position_in[4], 4),
                                                            robot_config.STEPS2RADS(position_in[5], 5), ])

                                    R3 = SE3.RPY([numbers[3], numbers[4], numbers[5]], unit='deg', order='xyz')
                                    R3.t[0] = numbers[0] / 1000
                                    R3.t[1] = numbers[1] / 1000
                                    R3.t[2] = numbers[2] / 1000

                                    q_pose_move = robot_config.robot.ikine_LMS(R3,
                                                                               q0=robot_config.joints_standby_position_radian,
                                                                               ilimit=60)
                                    joint_angle_pose = np.array(
                                        [q_pose_move.q[0], q_pose_move.q[1], q_pose_move.q[2], q_pose_move.q[3],
                                         q_pose_move.q[4], q_pose_move.q[5]])

                                    needed_pos = np.array([joint_angle_pose[0] + 0.0000001,
                                                           joint_angle_pose[1] + 0.0000001,
                                                           joint_angle_pose[2] + 0.0000001,
                                                           joint_angle_pose[3] + 0.0000001,
                                                           joint_angle_pose[4] + 0.0000001,
                                                           joint_angle_pose[5] + 0.0000001, ])

                                    needed_pos_steps = np.array([int(robot_config.RAD2STEPS(joint_angle_pose[0], 0)),
                                                                 int(robot_config.RAD2STEPS(joint_angle_pose[1], 1)),
                                                                 int(robot_config.RAD2STEPS(joint_angle_pose[2], 2)),
                                                                 int(robot_config.RAD2STEPS(joint_angle_pose[3], 3)),
                                                                 int(robot_config.RAD2STEPS(joint_angle_pose[4], 4)),
                                                                 int(robot_config.RAD2STEPS(joint_angle_pose[5], 5)), ])

                                    # Check if needed positions are in range
                                    for i in range(6):
                                        if needed_pos[i] >= robot_config.joint_limits_radian[i][1] or needed_pos[i] <= \
                                                robot_config.joint_limits_radian[i][0]:
                                            shared_string.value = b'Error: MovePose needed joint position out of range'
                                            # print(f"Joint is out of range: {i + 1}")
                                            error_state = 1
                                            buttons[7] = 0
                                            break

                                    # print(f"initial pos is : {initial_pos}")
                                    # print(f"needed pos is : {needed_pos}")

                                    if additional_element is not None and additional_element not in ("trap", "poly"):
                                        # print("Invalid additional element:", additional_element)
                                        error_state = 1
                                        buttons[7] = 0
                                    else:
                                        # If t is defined *ignore all other params, if func is defined use that func, else use poly
                                        if t_value != None and t_value > 0 and t_value != 0:
                                            command_len = int(t_value / INTERVAL_S)
                                            command_step = command_step + 1
                                            t2 = (np.arange(0, t_value, INTERVAL_S))
                                            timebase_defined = "t"
                                            # print(" t is not none")

                                            if additional_element == "poly" or additional_element == None:
                                                qx2 = rp.tools.trajectory.jtraj(initial_pos, needed_pos, command_len)

                                            elif additional_element == "trap":

                                                qx2 = rp.tools.trajectory.mtraj(trapezoidal, initial_pos, needed_pos,
                                                                                command_len)


                                        # if t is not defined; use v and a values, trapezoidal is used always
                                        elif (t_value == 0 or t_value == None) and v_value != None and a_value != None:

                                            if a_value > 100 or a_value < 0:
                                                # print("error a_value too small")
                                                error_state = 1
                                                buttons[7] = 0
                                                shared_string.value = b'Error: MovePose() command acceleration setpoint out of range'
                                            if v_value > 100 or v_value < 0:
                                                # print("error v_value too small")
                                                error_state = 1
                                                buttons[7] = 0
                                                shared_string.value = b'Error: MovePose() command velocity setpoint out of range'

                                            path_differences = np.abs(needed_pos - initial_pos)
                                            # print(path_differences)

                                            # Find the index with the maximum difference
                                            max_path_index = np.argmax(path_differences)
                                            # print("index with max path is:",max_path_index)

                                            # find if any joint angles are the same. ignore those in calculations
                                            # print("needed positons in steps:",needed_pos_steps)
                                            # print("current positons in steps:")
                                            for i in range(6):
                                                print(" positon is", position_in[i])
                                            matching_indexes = np.where(needed_pos_steps == position_in)[0]
                                            # print("matching indexes are:",matching_indexes)

                                            # init arrays
                                            v_value_array = np.array([0, 0, 0, 0, 0, 0])
                                            trap_calc = np.array([None, None, None, None, None, None])

                                            # calculate speed and acc for leading joint using set %
                                            v_value_array[max_path_index] = 2000
                                            a_value_real = 1000

                                            v_value_array[max_path_index] = (np.interp(v_value, [0, 100], [
                                                robot_config.joint_min_speed[max_path_index],
                                                robot_config.joint_max_speed[max_path_index]]))
                                            a_value_real = (np.interp(a_value, [0, 100], [robot_config.joint_min_acc,
                                                                                          robot_config.joint_max_acc]))
                                            # print("a value is:", a_value_real)
                                            # print("v value is:", v_value_array[max_path_index])
                                            # from leading profile calculate acceleration time and total duration of the move
                                            tacc = v_value_array[max_path_index] / a_value_real
                                            # print("tacc is:",tacc)
                                            total_t = abs(
                                                needed_pos_steps[max_path_index] - position_in[max_path_index]) / \
                                                      v_value_array[max_path_index] + tacc
                                            # print("total_t is:")
                                            # print(total_t)
                                            execution_time = (np.arange(0, total_t, INTERVAL_S))

                                            # calculate all speeds and position profiles, ignore ones where needed and initial positon are the same
                                            for i in range(6):
                                                # if needed and initial are the same dont calculate anything
                                                if i in matching_indexes:
                                                    # print("positions are the same. joint at index: " ,i)
                                                    continue
                                                v_value_array[i] = abs(needed_pos_steps[i] - position_in[i]) / (
                                                            total_t - tacc)
                                                try:
                                                    trap_calc[i] = trapezoidal(position_in[i], needed_pos_steps[i],
                                                                               execution_time, v_value_array[i])
                                                    # print("good trap profile was made for index",i)
                                                    # trap_calc[i].plot()
                                                except:
                                                    # if needed and initial are not the same but path is really small use 1/3 acc, 1/3 cruise and 1/3 deac
                                                    trap_calc[i] = trapezoidal(position_in[i], needed_pos_steps[i],
                                                                               execution_time)
                                                    # print("ERROR acc is too small or v is to big index is:",i)

                                            for i in range(6):
                                                if i in matching_indexes:
                                                    continue
                                                if np.any(abs(trap_calc[i].qd) > robot_config.joint_max_speed[i]):
                                                    shared_string.value = b'Error: MovePose() speed or acceleration too big'
                                                    # print("error in joint:", i)
                                                    error_state = 1
                                                    buttons[7] = 0

                                            # TODO make sure accel and speed values are in the range
                                            # First find what joint has largerst path to travel
                                            # Calculate the absolute difference between elements of a and b

                                            command_len = int(total_t / INTERVAL_S)
                                            command_step = command_step + 1
                                            timebase_defined = "v and a"
                                            # print("t is none, a and v are defined")
                                            # use calculations


                                        # is none are defined or just one of speed and acc. Use conservative values, trapezoidal is used always
                                        elif (t_value == 0 or t_value == None) or v_value == None or a_value == None:
                                            a_value = 30
                                            v_value = 50

                                            path_differences = np.abs(needed_pos - initial_pos)
                                            print(path_differences)

                                            # Find the index with the maximum difference
                                            max_path_index = np.argmax(path_differences)
                                            # print("index with max path is:",max_path_index)

                                            # find if any joint angles are the same. ignore those in calculations
                                            matching_indexes = np.where(needed_pos_steps == position_in)[0]
                                            # print("matching indexes are:",matching_indexes)

                                            # init arrays
                                            v_value_array = np.array([0, 0, 0, 0, 0, 0])
                                            trap_calc = np.array([None, None, None, None, None, None])

                                            # calculate speed and acc for leading joint using set %
                                            v_value_array[max_path_index] = 2000
                                            a_value_real = 1000

                                            v_value_array[max_path_index] = (np.interp(v_value, [0, 100], [
                                                robot_config.joint_min_speed[max_path_index],
                                                robot_config.joint_max_speed[max_path_index]]))
                                            a_value_real = (np.interp(a_value, [0, 100], [robot_config.joint_min_acc,
                                                                                          robot_config.joint_max_acc]))
                                            # print("a value is:", a_value_real)
                                            # print("v value is:", v_value_array[max_path_index])
                                            # from leading profile calculate acceleration time and total duration of the move
                                            tacc = v_value_array[max_path_index] / a_value_real
                                            # print("tacc is:",tacc)
                                            total_t = abs(
                                                needed_pos_steps[max_path_index] - position_in[max_path_index]) / \
                                                      v_value_array[max_path_index] + tacc
                                            # print("total_t is:")
                                            # print(total_t)
                                            execution_time = (np.arange(0, total_t, INTERVAL_S))

                                            # calculate all speeds and position profiles, ignore ones where needed and initial positon are the same
                                            for i in range(6):
                                                # if needed and initial are the same dont calculate anything
                                                if i in matching_indexes:
                                                    # print("positions are the same. joint at index: " ,i)
                                                    continue
                                                v_value_array[i] = abs(needed_pos_steps[i] - position_in[i]) / (
                                                            total_t - tacc)
                                                try:
                                                    trap_calc[i] = trapezoidal(position_in[i], needed_pos_steps[i],
                                                                               execution_time, v_value_array[i])
                                                    # print("good trap profile was made for index",i)
                                                    # trap_calc[i].plot()
                                                except:
                                                    # if needed and initial are not the same but path is really small use 1/3 acc, 1/3 cruise and 1/3 deac
                                                    trap_calc[i] = trapezoidal(position_in[i], needed_pos_steps[i],
                                                                               execution_time)
                                                    # print("ERROR acc is too small or v is to big index is:",i)

                                            for i in range(6):
                                                if i in matching_indexes:
                                                    continue
                                                if np.any(abs(trap_calc[i].qd) > robot_config.joint_max_speed[i]):
                                                    shared_string.value = b'Error: MovePose() speed or acceleration too big'
                                                    # print("error in joint:", i)
                                                    error_state = 1
                                                    buttons[7] = 0

                                            # TODO make sure accel and speed values are in the range
                                            # First find what joint has largerst path to travel
                                            # Calculate the absolute difference between elements of a and b

                                            command_len = int(total_t / INTERVAL_S)
                                            command_step = command_step + 1
                                            timebase_defined = "None"
                                            print("Using conservative values")
                                            # use calculations




                                        # Error state?
                                        else:
                                            command_len = 1000
                                            command_step = command_step + 1
                                            # flag error unknown state
                                            print("unknown state?")

                                            # print(qx2)
                                        print(needed_pos)

                                        print("Numbers:", numbers)
                                        print("Value of 'v':", v_value)
                                        print("Value of 'a':", a_value)
                                        print("Value of 't':", t_value)
                                        print("Function element:", additional_element)
                                        print("Tracking (speed):", tracking)
                                else:
                                    shared_string.value = b'Error: Invalid MovePose() command'
                                    error_state = 1
                                    buttons[7] = 0

                            elif command_step != command_len:  #

                                if timebase_defined == "t":
                                    for i in range(6):

                                        if additional_element == "trap":
                                            speed_out[i] = int(robot_config.SPEED_RAD2STEP(
                                                qx2.qd[command_step][i] / (t_value - INTERVAL_S), i)) * (
                                                                       command_len - 1)
                                            position_out[i] = int(robot_config.RAD2STEPS(qx2.q[command_step][i], i))
                                        elif additional_element == "poly" or additional_element == None:
                                            speed_out[i] = int(robot_config.SPEED_RAD2STEP(
                                                qx2.qd[command_step][i] / (t_value - INTERVAL_S), i))  # * 199
                                            position_out[i] = int(robot_config.RAD2STEPS(qx2.q[command_step][i], i))

                                elif timebase_defined == "v and a" or timebase_defined == "None":

                                    for i in range(6):
                                        if i in matching_indexes:
                                            speed_out[i] = 0
                                            position_out[i] = position_in[i]
                                            continue
                                        try:
                                            temp_var_traj = trap_calc[i]
                                            speed_out[i] = int(temp_var_traj.qd[command_step])
                                            position_out[i] = int(temp_var_traj.q[command_step])
                                        except:
                                            speed_out[i] = 0
                                            position_out[i] = position_in[i]
                                            # print("ERROR acc is too small or v is to big index is:",i)

                                # print(Speed_out[5])
                                # print(Speed_out[0])
                                command_step = command_step + 1

                                if tracking == None:
                                    command_out.value = 156
                                elif tracking == "speed":
                                    command_out.value = 123
                                else:
                                    command_out.value = 255

                            else:
                                time2 = time.perf_counter()
                                command_out.value = 255  # Send command from last index
                                print(command_step)
                                print(time2 - time1)
                                print("MovePose done")
                                command_step = 0
                                command_len = 0
                                Program_step = Program_step + 1


                        # Move in cartesian space command
                        elif clean_string[Program_step] == 'MoveCart()':
                            # This code will execute once per command call
                            if command_step == 0:
                                time1 = time.perf_counter()
                                # data_packet = "test(1,2,3,4,5,6,a=1,v=2,t=0,func,speed)"

                                # Define the pattern using regular expression
                                pattern = r'MoveCart\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*(?:,\s*v\s*=\s*(-?\d+(?:\.\d+)?))?(?:,\s*a\s*=\s*(-?\d+(?:\.\d+)?))?(?:,\s*t\s*=\s*(-?\d+(?:\.\d+)?))?(?:,\s*(trap|poly))?(?:,\s*(speed))?\s*\)?'

                                # Use re.match to find the pattern in the data packet
                                match = re.match(pattern, clean_string_commands[Program_step])

                                if match:
                                    shared_string.value = b'Log: MoveCart() command'
                                    groups = match.groups()
                                    numbers = []
                                    for num_str in groups[:6]:
                                        try:
                                            num = float(num_str)
                                            numbers.append(num)
                                        except ValueError:
                                            print(f"Invalid number: {num_str}")
                                            error_state = 1
                                            buttons[7] = 0
                                            break

                                    v_value = float(groups[6]) if groups[6] is not None else None
                                    a_value = float(groups[7]) if groups[7] is not None else None
                                    t_value = float(groups[8]) if groups[8] is not None else None
                                    additional_element = groups[9] if groups[9] is not None else None
                                    tracking = groups[10] if groups[10] is not None else None

                                    # joint positons we start from
                                    initial_joint_position = np.array(
                                        [robot_config.STEPS2RADS(position_in[0], 0) - 0.0001,
                                         robot_config.STEPS2RADS(position_in[1], 1) - 0.00015,
                                         robot_config.STEPS2RADS(position_in[2], 2) - 0.00015,
                                         robot_config.STEPS2RADS(position_in[3], 3) - 0.00015,
                                         robot_config.STEPS2RADS(position_in[4], 4) - 0.00015,
                                         robot_config.STEPS2RADS(position_in[5], 5) - 0.001, ])
                                    # current pose can be contructed from fkine of current joint positions

                                    Initial_pose = robot_config.robot.fkine(initial_joint_position)

                                    # Construct a matrix from given arguments, this will be needed pose
                                    Needed_pose = SE3.RPY([numbers[3], numbers[4], numbers[5]], unit='deg', order='xyz')
                                    Needed_pose.t[0] = numbers[0] / 1000
                                    Needed_pose.t[1] = numbers[1] / 1000
                                    Needed_pose.t[2] = numbers[2] / 1000

                                    print("needed pose is", Needed_pose)

                                    # TODO Check if needed pose joint angles are in range

                                    if additional_element is not None and additional_element not in ("trap", "poly"):
                                        # print("Invalid additional element:", additional_element)
                                        error_state = 1
                                        buttons[7] = 0
                                    else:

                                        # NOTE done!
                                        # If t is defined *ignore all other params, if func is defined use that func, else use poly
                                        if t_value != None and t_value > 0 and t_value != 0:

                                            command_len = int(t_value / INTERVAL_S)
                                            # t_t = np.arange(0,t_value,INTERVAL_S)
                                            timebase_defined = "t"

                                            if additional_element == "poly":
                                                t_tt = np.arange(0, t_value, INTERVAL_S)
                                                t_t_ = quintic(0, 1, t_tt)
                                                t_t = t_t_.q


                                            elif additional_element == "trap" or additional_element == None:
                                                t_tt = np.arange(0, t_value, INTERVAL_S)
                                                t_t_ = trapezoidal(0, 1, t_tt)
                                                t_t = t_t_.q

                                            # Calculated_distance = math.sqrt((Needed_pose.t[0] - Initial_pose.t[0])**2 + (Needed_pose.t[1] - Initial_pose.t[1])**2 + (Needed_pose.t[2] - Initial_pose.t[2])**2)
                                            # print("calculated distance is:", Calculated_distance)



                                        # if t is not defined; use v and a values, trapezoidal is used always
                                        elif (t_value == 0 or t_value == None) and v_value != None and a_value != None:

                                            if a_value > 100 or a_value < 0:
                                                # print("error a_value too small")
                                                error_state = 1
                                                buttons[7] = 0
                                                shared_string.value = b'Error: MovePose() command acceleration setpoint out of range'
                                            if v_value > 100 or v_value < 0:
                                                # print("error v_value too small")
                                                error_state = 1
                                                buttons[7] = 0
                                                shared_string.value = b'Error: MovePose() command velocity setpoint out of range'

                                            # v_value_cart_real = (np.interp(v_value,[0,100],[robot_config.Cartesian_linear_velocity_min,robot_config.Cartesian_linear_velocity_max]))
                                            # a_value_cart_real =  (np.interp(a_value,[0,100],[robot_config.Cartesian_linear_acc_min,robot_config.Cartesian_linear_acc_max]))
                                            #
                                            t_value_s = 3.6
                                            t_tt = np.arange(0, t_value_s, INTERVAL_S)
                                            t_t_ = trapezoidal(0, 1, t_tt)
                                            t_t = t_t_.q

                                            command_len = int(t_value_s / INTERVAL_S)
                                            timebase_defined = "None"
                                            print("Using conservative values")
                                            # use calculations


                                        # is none are defined or just one of speed and acc. Use conservative values, trapezoidal is used always
                                        elif (t_value == 0 or t_value == None) or v_value == None or a_value == None:
                                            t_value_s = 3.6
                                            t_tt = np.arange(0, t_value_s, INTERVAL_S)
                                            t_t_ = trapezoidal(0, 1, t_tt)
                                            t_t = t_t_.q

                                            command_len = int(t_value_s / INTERVAL_S)
                                            timebase_defined = "None"
                                            print("Using conservative values")
                                            # use calculations

                                    # perform ctraj
                                    Ctraj_traj = rp.tools.trajectory.ctraj(Initial_pose, Needed_pose, t_t)
                                    # print(Ctraj_traj)

                                    temp = [0] * len(t_t)
                                    joint_positions = [0] * len(t_t)
                                    velocity_array = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

                                    # Joint positons of the first matrix
                                    joint_positions[0] = initial_joint_position

                                    command_step = command_step + 1

                                    print("Numbers:", numbers)
                                    print("Value of 'v':", v_value)
                                    print("Value of 'a':", a_value)
                                    print("Value of 't':", t_value)
                                    print("Function element:", additional_element)
                                    print("Tracking (speed):", tracking)

                                else:
                                    shared_string.value = b'Error: Invalid MoveCart() command'
                                    error_state = 1
                                    buttons[7] = 0

                            elif command_step != command_len:  #

                                if ik_error == 0:
                                    # Calculate joint positons from matrix crated by ctraj
                                    temp[command_step] = robot_config.robot.ikine_LMS(Ctraj_traj[command_step],
                                                                                      q0=joint_positions[
                                                                                          command_step - 1], ilimit=60)
                                    joint_positions[command_step] = temp[command_step][0]
                                    # print("results")
                                    # print(temp[command_step])
                                    # print(temp[command_step].success)
                                    if str(temp[command_step].success) == 'False':
                                        print("i am false")
                                        shared_string.value = b'Error: MoveCart() IK error'
                                        error_state = 1
                                        buttons[7] = 0
                                        ik_error = 1

                                    # Check if positons are in valid range

                                    # Calculate needed speed
                                    for i in range(6):
                                        velocity_array[i] = (joint_positions[command_step][i] -
                                                             joint_positions[command_step - 1][i]) / INTERVAL_S

                                        # Set speeds and positions
                                    for i in range(6):
                                        position_out[i] = (
                                            int(robot_config.RAD2STEPS(joint_positions[command_step][i], i)))
                                        speed_out[i] = int(robot_config.SPEED_RAD2STEP(velocity_array[i], i))

                                        # print("joint positons are:", joint_positions[command_step])
                                    if tracking == None:
                                        command_out.value = 156
                                    elif tracking == "speed":
                                        command_out.value = 123
                                    else:
                                        command_out.value = 255

                                    # Check if positons are in valid range

                                    threshold_value_flip = 0.7
                                    zero_threshold = 0.0001
                                    # check if robot switches configurations by going from positive to negative angle!

                                    for i in range(6):
                                        # if values are close to zero they flactuate a lot
                                        if abs(joint_positions[command_step][i]) <= zero_threshold and abs(
                                                joint_positions[command_step - 1][i]) <= zero_threshold:
                                            None
                                        # Check if the absolute difference between the absolute values of 'a' and 'b' is less than or equal to the threshold
                                        else:
                                            if abs(abs(joint_positions[command_step][i]) - abs(
                                                    joint_positions[command_step - 1][i])) <= threshold_value_flip:
                                                # Check if 'a' and 'b' have opposite signs
                                                if (joint_positions[command_step][i] > 0 and
                                                    joint_positions[command_step - 1][i] < 0) or (
                                                        joint_positions[command_step][i] < 0 and
                                                        joint_positions[command_step - 1][i] > 0):
                                                    shared_string.value = b'Error: MoveCart() sign of position flipped'
                                                    print("ik flip error in joint:", i)
                                                    error_state = 1
                                                    buttons[7] = 0
                                                    speed_out[i] = 0
                                                    command_out.value = 255

                                    command_step = command_step + 1

                                    # check the speeds

                                    for i in range(6):
                                        if abs(speed_out[i] > robot_config.joint_max_speed[i]):
                                            shared_string.value = b'Error: MoveCart() speed is too big'
                                            print("error in joint:", i)
                                            error_state = 1
                                            buttons[7] = 0
                                            speed_out[i] = 0
                                            command_out.value = 255


                                else:
                                    ik_error = 0
                                    command_step = 0
                                    command_len = 0
                                    error_state = 1
                                    buttons[7] = 0

                            else:
                                time2 = time.perf_counter()
                                command_out.value = 255  # Send command from last index
                                print(command_step)
                                print(time2 - time1)
                                print("MoveCart done")
                                command_step = 0
                                command_len = 0
                                Program_step = Program_step + 1


                        # Move in cartesian space command
                        elif clean_string[Program_step] == 'MoveCartRelTRF()':
                            # This code will execute once per command call
                            if command_step == 0:
                                time1 = time.perf_counter()
                                # data_packet = "test(1,2,3,4,5,6,a=1,v=2,t=0,func,speed)"

                                # Define the pattern using regular expression
                                pattern = r'MoveCartRelTRF\(\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*(?:,\s*v\s*=\s*(-?\d+(?:\.\d+)?))?(?:,\s*a\s*=\s*(-?\d+(?:\.\d+)?))?(?:,\s*t\s*=\s*(-?\d+(?:\.\d+)?))?(?:,\s*(trap|poly))?(?:,\s*(speed))?\s*\)?'

                                # Use re.match to find the pattern in the data packet
                                match = re.match(pattern, clean_string_commands[Program_step])

                                if match:
                                    shared_string.value = b'Log: MoveCartRelTRF() command'
                                    groups = match.groups()
                                    numbers = []
                                    for num_str in groups[:6]:
                                        try:
                                            num = float(num_str)
                                            numbers.append(num)
                                        except ValueError:
                                            print(f"Invalid number: {num_str}")
                                            error_state = 1
                                            buttons[7] = 0
                                            break

                                    v_value = float(groups[6]) if groups[6] is not None else None
                                    a_value = float(groups[7]) if groups[7] is not None else None
                                    t_value = float(groups[8]) if groups[8] is not None else None
                                    additional_element = groups[9] if groups[9] is not None else None
                                    tracking = groups[10] if groups[10] is not None else None

                                    # joint positons we start from
                                    initial_joint_position = np.array(
                                        [robot_config.STEPS2RADS(position_in[0], 0) - 0.0001,
                                         robot_config.STEPS2RADS(position_in[1], 1) - 0.00015,
                                         robot_config.STEPS2RADS(position_in[2], 2) - 0.00015,
                                         robot_config.STEPS2RADS(position_in[3], 3) - 0.00015,
                                         robot_config.STEPS2RADS(position_in[4], 4) - 0.00015,
                                         robot_config.STEPS2RADS(position_in[5], 5) - 0.001, ])
                                    # current pose can be contructed from fkine of current joint positions

                                    Initial_pose = robot_config.robot.fkine(initial_joint_position)
                                    # print("current pose TRF first is ",Initial_pose)

                                    Ttt = Initial_pose
                                    x1_ = [numbers[0] / 1000, numbers[1] / 1000, numbers[2] / 1000]
                                    x2_ = Ttt * x1_
                                    Needed_pose = Ttt
                                    Needed_pose.t[0] = x2_[0]
                                    Needed_pose.t[1] = x2_[1]
                                    Needed_pose.t[2] = x2_[2]
                                    Needed_pose = Needed_pose * Needed_pose.Rx(numbers[3], 'deg') * Needed_pose.Ry(
                                        numbers[4], 'deg') * Needed_pose.Rz(numbers[5], 'deg')
                                    Initial_pose = robot_config.robot.fkine(initial_joint_position)

                                    # print("x1 is",x1_)
                                    # print("x2 is ",x2_)
                                    # print("current pose TRF second is ",Initial_pose)
                                    # print("needed pose  TRF is",Needed_pose)

                                    # TODO Check if needed pose joint angles are in range

                                    if additional_element is not None and additional_element not in ("trap", "poly"):
                                        # print("Invalid additional element:", additional_element)
                                        error_state = 1
                                        buttons[7] = 0
                                    else:

                                        # NOTE done!
                                        # If t is defined *ignore all other params, if func is defined use that func, else use poly
                                        if t_value != None and t_value > 0 and t_value != 0:

                                            command_len = int(t_value / INTERVAL_S)
                                            # t_t = np.arange(0,t_value,INTERVAL_S)
                                            timebase_defined = "t"

                                            if additional_element == "poly":
                                                t_tt = np.arange(0, t_value, INTERVAL_S)
                                                t_t_ = quintic(0, 1, t_tt)
                                                t_t = t_t_.q


                                            elif additional_element == "trap" or additional_element == None:
                                                t_tt = np.arange(0, t_value, INTERVAL_S)
                                                t_t_ = trapezoidal(0, 1, t_tt)
                                                t_t = t_t_.q

                                            # Calculated_distance = math.sqrt((Needed_pose.t[0] - Initial_pose.t[0])**2 + (Needed_pose.t[1] - Initial_pose.t[1])**2 + (Needed_pose.t[2] - Initial_pose.t[2])**2)
                                            # print("calculated distance is:", Calculated_distance)



                                        # if t is not defined; use v and a values, trapezoidal is used always
                                        elif (t_value == 0 or t_value == None) and v_value != None and a_value != None:

                                            if a_value > 100 or a_value < 0:
                                                # print("error a_value too small")
                                                error_state = 1
                                                buttons[7] = 0
                                                shared_string.value = b'Error: MoveCartRelTRF() command acceleration setpoint out of range'
                                            if v_value > 100 or v_value < 0:
                                                # print("error v_value too small")
                                                error_state = 1
                                                buttons[7] = 0
                                                shared_string.value = b'Error: MoveCartRelTRF() command velocity setpoint out of range'

                                            # v_value_cart_real = (np.interp(v_value,[0,100],[robot_config.Cartesian_linear_velocity_min,robot_config.Cartesian_linear_velocity_max]))
                                            # a_value_cart_real =  (np.interp(a_value,[0,100],[robot_config.Cartesian_linear_acc_min,robot_config.Cartesian_linear_acc_max]))
                                            #
                                            t_value_s = 3.6
                                            t_tt = np.arange(0, t_value_s, INTERVAL_S)
                                            t_t_ = trapezoidal(0, 1, t_tt)
                                            t_t = t_t_.q

                                            command_len = int(t_value_s / INTERVAL_S)
                                            timebase_defined = "None"
                                            print("Using conservative values")
                                            # use calculations


                                        # is none are defined or just one of speed and acc. Use conservative values, trapezoidal is used always
                                        elif (t_value == 0 or t_value == None) or v_value == None or a_value == None:
                                            t_value_s = 3.6
                                            t_tt = np.arange(0, t_value_s, INTERVAL_S)
                                            t_t_ = trapezoidal(0, 1, t_tt)
                                            t_t = t_t_.q

                                            command_len = int(t_value_s / INTERVAL_S)
                                            timebase_defined = "None"
                                            print("Using conservative values")
                                            # use calculations

                                    # perform ctraj
                                    Ctraj_traj = rp.tools.trajectory.ctraj(Initial_pose, Needed_pose, t_t)
                                    # print(Ctraj_traj)

                                    temp = [0] * len(t_t)
                                    joint_positions = [0] * len(t_t)
                                    velocity_array = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

                                    # Joint positons of the first matrix
                                    joint_positions[0] = initial_joint_position

                                    command_step = command_step + 1

                                    print("Numbers:", numbers)
                                    print("Value of 'v':", v_value)
                                    print("Value of 'a':", a_value)
                                    print("Value of 't':", t_value)
                                    print("Function element:", additional_element)
                                    print("Tracking (speed):", tracking)

                                else:
                                    shared_string.value = b'Error: Invalid MoveCartRelTRF() command'
                                    error_state = 1
                                    buttons[7] = 0

                            elif command_step != command_len:  #

                                if ik_error == 0:
                                    # Calculate joint positons from matrix crated by ctraj
                                    temp[command_step] = robot_config.robot.ikine_LMS(Ctraj_traj[command_step],
                                                                                      q0=joint_positions[
                                                                                          command_step - 1], ilimit=60)
                                    joint_positions[command_step] = temp[command_step][0]
                                    # print("results")
                                    # print(temp[command_step])
                                    # print(temp[command_step].success)
                                    if str(temp[command_step].success) == 'False':
                                        print("i am false")
                                        shared_string.value = b'Error: MoveCartRelTRF() IK error'
                                        error_state = 1
                                        buttons[7] = 0
                                        ik_error = 1

                                    # Check if positons are in valid range

                                    # Calculate needed speed
                                    for i in range(6):
                                        velocity_array[i] = (joint_positions[command_step][i] -
                                                             joint_positions[command_step - 1][i]) / INTERVAL_S

                                        # Set speeds and positions
                                    for i in range(6):
                                        position_out[i] = (
                                            int(robot_config.RAD2STEPS(joint_positions[command_step][i], i)))
                                        speed_out[i] = int(robot_config.SPEED_RAD2STEP(velocity_array[i], i))

                                        # print("joint positons are:", joint_positions[command_step])
                                    if tracking == None:
                                        command_out.value = 156
                                    elif tracking == "speed":
                                        command_out.value = 123
                                    else:
                                        command_out.value = 255

                                    # Check if positons are in valid range

                                    threshold_value_flip = 0.7
                                    zero_threshold = 0.0001
                                    # check if robot switches configurations by going from positive to negative angle!

                                    for i in range(6):
                                        # if values are close to zero they flactuate a lot
                                        if abs(joint_positions[command_step][i]) <= zero_threshold and abs(
                                                joint_positions[command_step - 1][i]) <= zero_threshold:
                                            None
                                        # Check if the absolute difference between the absolute values of 'a' and 'b' is less than or equal to the threshold
                                        else:
                                            if abs(abs(joint_positions[command_step][i]) - abs(
                                                    joint_positions[command_step - 1][i])) <= threshold_value_flip:
                                                # Check if 'a' and 'b' have opposite signs
                                                if (joint_positions[command_step][i] > 0 and
                                                    joint_positions[command_step - 1][i] < 0) or (
                                                        joint_positions[command_step][i] < 0 and
                                                        joint_positions[command_step - 1][i] > 0):
                                                    shared_string.value = b'Error: MoveCartRelTRF() sign of position flipped'
                                                    print("ik flip error in joint:", i)
                                                    error_state = 1
                                                    buttons[7] = 0
                                                    speed_out[i] = 0
                                                    command_out.value = 255

                                    command_step = command_step + 1

                                    # check the speeds

                                    for i in range(6):
                                        if abs(speed_out[i] > robot_config.joint_max_speed[i]):
                                            shared_string.value = b'Error: MoveCartRelTRF() speed is too big'
                                            print("error in joint:", i)
                                            print("command step is", command_step)
                                            error_state = 1
                                            buttons[7] = 0
                                            speed_out[i] = 0
                                            command_out.value = 255


                                else:
                                    ik_error = 0
                                    command_step = 0
                                    command_len = 0
                                    error_state = 1
                                    buttons[7] = 0

                            else:
                                time2 = time.perf_counter()
                                command_out.value = 255  # Send command from last index
                                print(command_step)
                                print(time2 - time1)
                                print("MoveCart done")
                                command_step = 0
                                command_len = 0
                                Program_step = Program_step + 1





                        # Dummy command (used for testing)
                        elif clean_string[Program_step] == 'Dummy()':
                            logging.debug('Log: Dummy() command')
                            command_out.value = 255  # Set dummy data
                            Program_step = Program_step + 1

                        # End command
                        elif clean_string[Program_step] == 'End()':
                            logging.debug('Log: End() command')
                            Program_step = 1
                            robot_mode = "Dummy"
                            buttons[7] = 0


                        # Gripper command
                        elif clean_string[Program_step] == 'Gripper()':

                            pattern = r'Gripper\(\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*\)'
                            match = re.match(pattern, clean_string_commands[Program_step])
                            if match:

                                match_1 = int(match.group(1))
                                match_2 = int(match.group(2))
                                match_3 = int(match.group(3))
                                if (
                                        match_1 >= 0 and match_1 <= 255 and match_2 >= 0 and match_2 <= 255 and match_3 >= 100 and match_3 <= 1000):
                                    shared_string.value = b'Log: Gripper() command'
                                    gripper_data_out[0] = match_1
                                    gripper_data_out[1] = match_2
                                    gripper_data_out[2] = match_3
                                else:
                                    shared_string.value = b'Log: Error: Gripper() invalid input value'
                            else:
                                shared_string.value = b'Log: Error: Gripper() command'

                            logging.debug('Log: Gripper() command')
                            Program_step = Program_step + 1
                            # Robot_mode = "Dummy"
                            # Buttons[7] = 0

                        # Gripper_cal command
                        elif clean_string[Program_step] == 'Gripper_cal()':
                            logging.debug('Log: Gripper_cal() command')
                            shared_string.value = b'Log: Gripper calibration command'
                            gripper_data_out[4] = 1
                            Program_step = Program_step + 1
                            # Robot_mode = "Dummy"
                            # Buttons[7] = 0






            ######################################################
            ######################################################
            else:  # If nothing else is done send dummy data 0x255
                robot_mode = "Dummy"
                dummy_data(position_out, speed_out, command_out, position_in)

            # Provjere
            # Svaka move funkciaj će imati svoje provjere!
            # Tu će samo biti zadnja provjera koja gleda brzine i ako su pre velike stavlja ih na nula!
            time2 = time.perf_counter()



        else:
            try:

                if my_os == 'Linux':
                    com_port = '/dev/ttyACM' + str(general_data[0])
                elif my_os == 'Windows':
                    com_port = 'COM' + str(general_data[0])

                print(com_port)
                ser.port = com_port
                ser.baudrate = 3000000
                ser.close()
                time.sleep(0.5)
                ser.open()
                time.sleep(0.5)
            except:
                time.sleep(0.5)
                logging.debug("no serial available, reconnecting!")

        timer.checkpt()

