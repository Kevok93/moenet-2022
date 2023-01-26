#!/usr/bin/env python3
#
# This is a NetworkTables client (eg, the DriverStation/coprocessor side).
# You need to tell it the IP address of the NetworkTables server (the
# robot or simulator).
#
# When running, this will continue incrementing the value 'dsTime', and the
# value should be visible to other networktables clients and the robot.
#

import sys
import time
from networktables import NetworkTables

# To see messages from networktables, you must setup logging
import logging

logging.basicConfig(level=logging.DEBUG)

ip = '10.3.65.2'

NetworkTables.initialize(server=ip)

sd = NetworkTables.getTable("SmartDashboard")

def sfloat(param: str, value: float):
    sd.putNumber(param, value)
    sd.flush()


def send_pose(pose):
    if sd.putNumberArray("pose", pose) == False:
        print("NOT WORKING")
    else:
        NetworkTables.flush()