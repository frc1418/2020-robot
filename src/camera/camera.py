#!/usr/bin/env python3
from cscore import CameraServer, UsbCamera


def main():
    cs = CameraServer.getInstance()
    cs.enableLogging()

    usb0 = cs.startAutomaticCapture(dev=0)
    usb1 = cs.startAutomaticCapture(dev=1)

    cs.waitForever()


if __name__ == "__main__":
    main()
