from gui.gui import *
from gui.gui import run_gui
from gui.simulator import run_sim
from core.main import main
from io_data import *
logging.basicConfig(level = logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(levelname)s:\t%(message)s',
    datefmt='%H:%M:%S'
)
logging.disable(logging.DEBUG)

if __name__ == '__main__':

    core_process = multiprocessing.Process(target=main, args=[shared_string, position_out, speed_out, command_out, affected_joint_out, in_out_out, timeout_out, gripper_data_out,
                                                              position_in, speed_in, homed_in, in_out_in, temperature_error_in, position_error_in, timeout_error, timing_data_in,
                                                              xtr_data, gripper_data_in,
                                                              joint_jog_buttons, cart_jog_buttons, jog_control, general_data, buttons, ])

    gui_process = multiprocessing.Process(target=run_gui, args=[shared_string, position_out, speed_out, command_out, affected_joint_out, in_out_out, timeout_out, gripper_data_out,
                                                                position_in, speed_in, homed_in, in_out_in, temperature_error_in, position_error_in, timeout_error, timing_data_in,
                                                                xtr_data, gripper_data_in,
                                                                joint_jog_buttons, cart_jog_buttons, jog_control, general_data, buttons, ])

    sim_process = multiprocessing.Process(target=run_sim, args =[position_out, position_in, position_sim, buttons])

    core_process.start()
    gui_process.start()
    sim_process.start()
    core_process.join()
    gui_process.join()
    sim_process.join()
    core_process.terminate()
    gui_process.terminate()
    sim_process.terminate()