
import serial
import time
import platform
import os
import logging


from gui.gui import run_gui
from gui.simulator import run_sim
from core.main import main
from io_data import *
from core.common import get_platform_serial, platform
logging.basicConfig(level = logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)

logging.disable(logging.DEBUG)




# u PROCES kao argumenti idu multi proc arrays tu dolje u initi
# Gore u thredovima i funkcijama to nazovem kako oćem i pozivam stvari iz toga i tjt
if __name__ == '__main__':

    print("Running")
    time.sleep(0.01)

    try:
        platform_serial = get_platform_serial()
        platform_serial.close()
    except:
        None

    # Process
    core_process = multiprocessing.Process(target=main, args=[shared_string, position_out, speed_out, command_out, affected_joint_out, in_out_out, timeout_out, gripper_data_out,
                                                              Position_in, speed_in, homed_in, in_out_in, temperature_error_in, position_error_in, timeout_error, timing_data_in,
                                                              xtr_data, gripper_data_in,
                                                              joint_jog_buttons, cart_jog_buttons, jog_control, general_data, buttons, ])
    
    gui_process = multiprocessing.Process(target=run_gui, args=[shared_string, position_out, speed_out, command_out, affected_joint_out, in_out_out, timeout_out, gripper_data_out,
                                                                Position_in, speed_in, homed_in, in_out_in, temperature_error_in, position_error_in, timeout_error, timing_data_in,
                                                                xtr_data, gripper_data_in,
                                                                joint_jog_buttons, cart_jog_buttons, jog_control, general_data, buttons, ])

    sim_process = multiprocessing.Process(target=run_sim, args =[Position_out, Position_in, Position_Sim, Buttons])
    process3 = multiprocessing.Process(target=SIMULATOR_process(),args =[Position_out,Position_in,Position_Sim,Buttons])


    core_process.start()
    gui_process.start()
    sim_process.start()
    core_process.join()
    gui_process.join()
    sim_process.join()

    core_process.terminate()
    gui_process.terminate()
    sim_process.terminate()





