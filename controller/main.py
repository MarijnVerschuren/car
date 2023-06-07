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
#===============================================|
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
#===============================================|
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
#===============================================|
if __name__ == '__main__':
    # commands
    if "-help" in sys.argv: print(
            "|----------------|-----|-----------|-------|---------------------------------------------------------------|",
            "| command        | req | args      | unit  | description                                                   |",
            "|----------------|-----|-----------|-------|---------------------------------------------------------------|",
            "| -adapter       |  *  | %s        | -     | port connected to the receiver of the car                     |",
            "| -baud          |  *  | %d        | bit/S | baud rate for serial communication                            |",
            "| -timeout       |     | %f        | S     | time that one write/read operation is allowed to take         |",
            "| -freq          |     | %d        | Hz    | send frequency (default: 50 Hz, max: <500 Hz)                 |",
            "| -display       |     |           | -     | display the packet that is being sent                         |",
            "| -test          |     |           | -     | display incoming packet without requiring a PS3 controller    |",
            "|----------------|-----|-----------|-------|---------------------------------------------------------------|",
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
    test = "-test" in sys.argv
    display = ("-display" in sys.argv) or test

    ps3 = None; ps3_t = None
    if not test:  # init controller
        # find inputs device
        for dev in devices:  # TODO: more devices
            if dev.name == "Sony PLAYSTATION(R)3 Controller":
                ps3 = PS3(dev); break
        else: raise Exception("device not found")

        # setup controller update function
        ps3_t = thread(target=controller_update_loop, args=(ps3,))
        ps3_t.start()

    # initialize crc
    stm32_crc = crc_lib.Configuration(
        width=              32,
        polynomial=         0x04C11DB7,
        init_value=         0xFFFFFFFF,
        final_xor_value=    0x00,
        reverse_input=      False,
        reverse_output=     False
    ); crc = crc_lib.Calculator(stm32_crc)

    # open serial port
    ser = serial.Serial(adapter, baud, timeout=timeout)

    # main loop
    try:
        while True:
            sys.stdout.flush()
            time.sleep(send_delay)
            print("\033[1A\x1b[2K" * 2, flush=True)

            if not test:
                throttle = ps3.trigger_R.raw - ps3.trigger_L.raw,    # -255 - 255
                steering = ps3.joystick_L.raw_x,  # 0 - 255  =[encoding]=>  -128 - 127
                boost = bool(ps3.x_button)
            else: throttle = 0; steering = 0; boost = False
            packet = package(
                throttle,
                steering,
                boost
            )
            if display: print(f"{packet.hex()} -> {throttle}, {steering}, {boost}")
            ser.write(packet)

            data = b"\x00" * 16
            if ser.inWaiting() >= 16:
                data = ser.read(16)
                #data_crc = ser.read(4)
                #if data_crc != struct.pack("<L", crc.checksum(data)): continue
            print(" ".join([hex(x) for x in struct.unpack("<LLLL", data)]), end=" " * 24)

    except KeyboardInterrupt:  # stop gracefully
        if not test: ps3_t.stop()
        os._exit(0)
