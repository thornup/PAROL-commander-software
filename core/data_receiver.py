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
import serial as ser

from core.common import get_data


# Task that receives data and saves to the multi proc array
def receive_data(shared_string, position_in, speed_in, homed_in, in_out_in, temperature_error_in, position_error_in,
                 timeout_error, timing_data_in,
                 xtr_data, gripper_data_in, general_data):
    while True:
        #  PYTHON
        # https://pyserial.readthedocs.io/en/latest/pyserial_api.html#serial.Serial.in_waiting
        # https://stackoverflow.com/questions/17553543/pyserial-non-blocking-read-loop
        # inWaiting govori koliko imamo bytes u serial input bufferu.
        # Pošto šaljem serial bez pauze uvijek će biti nešto
        # time on čita dok ima nečekga; a to čita sa .read
        # .read prima kao parametar koliko bytes da čita i u tome loopa
        # pošto prima inwaiting pročitati će ih sve koji su trenutačno u bufferu
        # npr ako dolaze pre sporo neće ih biti u bufferu i kod ide dalje
        # i kada se opet vrati i vidi da nečega ima čita to

        # ARDUINO
        # https://forum.arduino.cc/t/sending-command-over-serial-64-bytes-128-bytes/121598/3
        # https://github.com/stm32duino/Arduino_Core_STM32/wiki/HardwareTimer-library
        # http://www.gammon.com.au/serial

        # ovako jako slično radi i od arduina
        # serial.available govori koliko imamo bytes u bufferu
        # serial.read čita samo JEDAN byte (tu je velika razlika jer moram sam onda spremati u buffer)
        # zato se stavi onaj while(serial.availabe) i onda u loopu ide serial.read i spremanje u buffer
        # kada se pojavi ili neki naš znak tipa /n ili duljina buffera parasa se to i gleda da li je dobro
        # isto kao i gore ako je data pre spor serial.available će javiti da nema ničega i idemo dalje
        # javiti će to makar je tamo while petlja. ako bi bila if petlja onda bi očitao jedan i radio ostatak koda
        # pa se vratio pročitao jedan itd. tako bi možda pre sporo primali serial ako bi ostatak koda bio spor
        try:
            # ToDo: check if this should get data_buffer
            get_data(position_in, speed_in, homed_in, in_out_in, temperature_error_in, position_error_in, timeout_error,
                     timing_data_in,
                     xtr_data, gripper_data_in)
        except:
            try:
                my_os = platform.system()
                if my_os == "Windows":
                    com_port = 'COM' + str(general_data[0])
                    logging.debug("Os is Windows")
                else:
                    com_port = '/dev/ttyACM' + str(general_data[0])
                    logging.debug("Os is Linux")


                ser.port = com_port
                ser.baudrate = 3000000
                ser.close()
                time.sleep(0.5)
                ser.open()
                time.sleep(0.5)
            except:
                time.sleep(0.5)
                logging.debug("no serial available, reconnecting!")
                # Get_data_old()
        # print("Task 2 alive")
        # time.sleep(2)

