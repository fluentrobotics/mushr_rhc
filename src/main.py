#!/usr/bin/env python3

import controlnode
import threading
import signal
import time
import sys


if __name__ == '__main__':

    time.sleep(0.5)
    node = controlnode.ControlNode("controller")

    signal.signal(signal.SIGINT, node.shutdown)

    controller = threading.Tread(start=node.start)
    controller.start()
    

    while controller.run:
        signal.pause()

    try:
        controller.join()
    except Exception as e:
        print("Exception Handled in Main, Details of the Exception:", e)
