
import serial
import time
import platform
import os
import logging


from gui.gui import run_gui
from gui.simulator import run_sim
from core.main import main
from io_data import *

platform = platform.system()
logging.basicConfig(level = logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)

if platform == "Windows":
    logging.debug("Os is Windows")
    Image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)))
    STARTING_PORT = 3 # COM3
    try:
        ser = serial.Serial(port='COM' + str(STARTING_PORT), baudrate=3000000, timeout=0)
    except:
        ser = serial.Serial()

else: 
    logging.debug("Os is Linux")
    Image_path = os.path.join(os.path.dirname(os.path.realpath(__file__)))
    STARTING_PORT = 0
    try:
        ser = serial.Serial(port='/dev/ttyACM' + str(STARTING_PORT), baudrate=3000000, timeout=0)
    except:
        ser = serial.Serial()
logging.disable(logging.DEBUG)




# u PROCES kao argumenti idu multi proc arrays tu dolje u initi
# Gore u thredovima i funkcijama to nazovem kako oćem i pozivam stvari iz toga i tjt
if __name__ == '__main__':

    print("Running")
    time.sleep(0.01) 

    try:
        ser.close()
    except:
        None

    # Process
    core_process = multiprocessing.Process(target=main, args=[shared_string, Position_out, Speed_out, Command_out, Affected_joint_out, InOut_out, Timeout_out, Gripper_data_out,
                                                              Position_in, Speed_in, Homed_in, InOut_in, Temperature_error_in, Position_error_in, Timeout_error, Timing_data_in,
                                                              XTR_data, Gripper_data_in,
                                                              Joint_jog_buttons, Cart_jog_buttons, Jog_control, General_data, Buttons, ])
    
    gui_process = multiprocessing.Process(target=run_gui, args=[shared_string, Position_out, Speed_out, Command_out, Affected_joint_out, InOut_out, Timeout_out, Gripper_data_out,
                                                                Position_in, Speed_in, Homed_in, InOut_in, Temperature_error_in, Position_error_in, Timeout_error, Timing_data_in,
                                                                XTR_data, Gripper_data_in,
                                                                Joint_jog_buttons, Cart_jog_buttons, Jog_control, General_data, Buttons, ])

    sim_process = multiprocessing.Process(target=run_sim, args =[Position_out, Position_in, Position_Sim, Buttons])
   # process3 = multiprocessing.Process(target=SIMULATOR_process(),args =[Position_out,Position_in,Position_Sim,Buttons])


    core_process.start()
    gui_process.start()
    sim_process.start()
    core_process.join()
    gui_process.join()
    sim_process.join()

    core_process.terminate()
    gui_process.terminate()
    sim_process.terminate()





