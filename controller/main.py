import crc as crc_lib
from inputs import *
import threading
import sys, os
import ctypes
import struct
import serial
import time

from lib import *  # PS3


#===============================================|
# threading                                     |
#==============================================/
class thread(threading.Thread):
    def __init__(self, *args, **kwargs):
        super(thread, self).__init__(*args, **kwargs)

    def stop(self):
        if not self.is_alive(): return True
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(self.ident), ctypes.py_object(SystemExit))
        if res > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(self.ident, None)
            return False
        return True


def controller_update_loop(controller: PS3) -> None:
    while True: controller.update()


#===============================================|
# functions                                     |
#==============================================/
def package(throttle: int, steering: int) -> bytes:
    pack_data = struct.pack("<Hh", throttle, steering)
    pack_crc = struct.pack("<L", crc.checksum(pack_data))
    return pack_data + pack_crc


#===============================================|
# entry                                         |
#==============================================/
if __name__ == '__main__':
    # commands
    if "-help" in sys.argv: print(
            "|----------------|-----|-----------|-----------------------------------------------------------------------|",
            "| command        | req | args      | description                                                           |",
            "|----------------|-----|-----------|-----------------------------------------------------------------------|",
            "| -adapter       |  *  | %s        | port connected to the receiver of the car                             |",
            "| -baud          |  *  | %d        | baud rate for serial communication                                    |",
            "| -timeout       |     | %f        | time that one write/read operation is allowed to take (in seconds)    |",
            "| -display       |     |           | display the packet that is being sent                                 |",
            "|----------------|-----|-----------|-----------------------------------------------------------------------|",
            sep="\n"
        ); exit(0)

    # argument checking
    if "-adapter" not in sys.argv:  raise Exception("no serial port provided")
    if "-baud" not in sys.argv:     raise Exception("no baud rate provided")

    # argument interpretation
    try: adapter = sys.argv[sys.argv.index("-adapter") + 1]
    except IndexError: raise Exception("no serial port provided")
    try: baud = int(sys.argv[sys.argv.index("-baud") + 1])
    except IndexError or ValueError: raise Exception("invalid or no baud rate provided")
    timeout = 0  # no timeout as default
    if "-timeout" in sys.argv:
        try: timeout = float(sys.argv[sys.argv.index("-timeout") + 1])
        except IndexError or ValueError:  raise Exception("invalid or no timeout provided")
    display = "-display" in sys.argv

    # find inputs device
    for dev in devices:  # TODO: more devices
        if dev.name == "Sony PLAYSTATION(R)3 Controller":
            ps3 = PS3(dev); break
    else: raise Exception("device not found")

    # initialize crc
    stm32_crc = crc_lib.Configuration(
        width=              32,
        polynomial=         0x04C11DB7,
        init_value=         0xFFFFFFFF,
        final_xor_value=    0x00,
        reverse_input=      False,
        reverse_output=     False
    ); crc = crc_lib.Calculator(stm32_crc)

    # setup controller update function
    ps3_t = thread(target=controller_update_loop, args=(ps3,))
    ps3_t.start()

    # open serial port
    ser = serial.Serial(adapter, baud, timeout=timeout)

    # main loop
    while True:
        packet = package(
            int(ps3.trigger_R.x * 0xFFFF),  # map the right trigger value to an unsigned short
            int(ps3.joystick_R.x * 0x7FFF)  # map the x coordinate of the right joystick to a signed short
        )
        if display: print(f"{ps3.trigger_R.x}, {ps3.joystick_R.x} -> {packet.hex()}" + (" " * 50), end="\r")
        #ser.write(packet); time.sleep(1)
        #print(ps3, end="\r")

        time.sleep(0.015)  # TODO: remove