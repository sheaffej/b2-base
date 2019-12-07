#!/usr/bin/env python

from __future__ import print_function
import sys
from roboclaw_driver import Roboclaw

if len(sys.argv) == 1:
   print()
   print("Example Usage: {} /dev/ttyACM0".format(sys.argv[0]))
   print()
   exit()

dev_name = sys.argv[1]
baud_rate = 115200
address = 0x80

roboclaw = Roboclaw(dev_name, baud_rate)
roboclaw.Open()

temp_response = roboclaw.ReadTemp(address)[1]/10.0
volts_response = roboclaw.ReadMainBatteryVoltage(address)[1]/10.0

print("Reading Roboclaw at: {}".format(dev_name))
print("Temp: {}C, Battery: {}V".format(temp_response, volts_response))
