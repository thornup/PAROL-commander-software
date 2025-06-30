# Task that receives data and saves to the multi proc array
def Task2(shared_string, Position_in, Speed_in, Homed_in, InOut_in, Temperature_error_in, Position_error_in,
          Timeout_error, Timing_data_in,
          XTR_data, Gripper_data_in, General_data):
    while 1:

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
            Get_data(Position_in, Speed_in, Homed_in, InOut_in, Temperature_error_in, Position_error_in, Timeout_error,
                     Timing_data_in,
                     XTR_data, Gripper_data_in)
            # Get_data_old()
        except:
            try:

                if my_os == 'Linux':
                    com_port = '/dev/ttyACM' + str(General_data[0])
                elif my_os == 'Windows':
                    com_port = 'COM' + str(General_data[0])

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
                # Get_data_old()
        # print("Task 2 alive")
        # time.sleep(2)

