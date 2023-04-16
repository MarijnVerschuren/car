import crc as crc_lib
from inputs import *
import threading
import ctypes
import struct
import serial
import time
import sys
import os

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
def package(throttle: int, steering: int, boost: bool) -> bytes:
    # struct packet {
    #   uint8_t throttle;
    #   uint8_t steering;
    #   uint16_t reverse: 1;
    #   uint16_t flags : 15;  # not used at this time
    # }
    flags = 0x0000
    if throttle < 0: flags |= 0x1; throttle *= -1
    if boost: flags |= 0x2;
    pack_data = struct.pack("<BBH", throttle, steering, flags)
    pack_crc = struct.pack("<L", crc.checksum(pack_data[::-1]))  # flip bytes because python
    return pack_data + pack_crc


#===============================================|
# entry                                         |
#==============================================/
if __name__ == '__main__':
    # commands
    if "-help" in sys.argv: print(
            "|----------------|-----|-----------|-------|--------------------------------------------------------|",
            "| command        | req | args      | unit  | description                                            |",
            "|----------------|-----|-----------|-------|--------------------------------------------------------|",
            "| -adapter       |  *  | %s        | -     | port connected to the receiver of the car              |",
            "| -baud          |  *  | %d        | bit/S | baud rate for serial communication                     |",
            "| -timeout       |     | %f        | S     | time that one write/read operation is allowed to take  |",
            "| -freq          |     | %d        | Hz    | send frequency (default: 50 Hz, max: <500 Hz)          |",
            "| -display       |     |           | -     | display the packet that is being sent                  |",
            "|----------------|-----|-----------|-------|--------------------------------------------------------|",
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
    send_delay = 1 / 50  # 50 Hz
    if "-freq" in sys.argv:
        try: send_delay = 1 / int(sys.argv[sys.argv.index("-freq") + 1])
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
    try:
        while True:
            packet = package(
                ps3.trigger_R.raw - ps3.trigger_L.raw,    # -255 - 255
                ps3.joystick_L.raw_x,  # 0 - 255  =[encoding]=>  -128 - 127
                bool(ps3.x_button)  # boost
            )
            if display: print(f"{packet.hex()} -> {ps3.trigger_R.raw - ps3.trigger_L.raw}, {ps3.joystick_L.x}" + (" " * 50), end="\r")
            ser.write(packet)
            time.sleep(send_delay)
    except KeyboardInterrupt:  # stop gracefully
        ps3_t.stop()
        os._exit(0)
