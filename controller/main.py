from inputs import *
import sys, os
import time

from lib import *  # PS3


# entry
if __name__ == '__main__':
    # help command
    if "-help" in sys.argv: print(
            "| command              | args      | description                       |",
            "|----------------------|-----------|-----------------------------------|",
            sep="\n"
        ); exit(0)

    for dev in devices:
        if dev.name == "Sony PLAYSTATION(R)3 Controller":
            ps3 = PS3(dev); break
    else: raise Exception("device not found")

    while 1:
        ps3.update()
        if ps3.x_button: print(ps3); sys.stdout.flush()


    # https://pypi.org/project/aioserial/#quick-start