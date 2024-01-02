#!/usr/bin/env python3
#
# This is a NetworkTables client (eg, the DriverStation/coprocessor side).
# You need to tell it the IP address of the NetworkTables server (the
# robot or simulator).
#
# When running, this will continue incrementing the value 'dsTime', and the
# value should be visible to other networktables clients and the robot.
#

from networktables import (
    NetworkTables,
    NetworkTable,
    NetworkTableEntry,
)

# To see messages from networktables, you must setup logging
import logging
#logging.basicConfig(level=logging.DEBUG)

class NetworkTablesSender:
    ip: str
    smart_dashboard: NetworkTable
    camera_data: NetworkTable
    camera_debug: NetworkTable

    def __init__(self, ip : str):
        self.ip = ip
        NetworkTables.initialize(server=ip)
        self.smart_dashboard = NetworkTables.getTable("SmartDashboard")
        self.camera_data = NetworkTables.getTable("CameraData")
        self.camera_debug = NetworkTables.getTable("CameraDebug")

    def flush(self):
        NetworkTables.flush()

    @staticmethod
    def init(ip : str):
        global nts
        nts = NetworkTablesSender(ip)


nts: NetworkTablesSender = None
