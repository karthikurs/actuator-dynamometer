#!/usr/bin/env python3

# Based on example.py from https://github.com/tatobari/hx711py
# and https://github.com/odriverobotics/ODrive/blob/master/tools/odrive_demo.py

from __future__ import print_function

import moteus
import moteus.moteus_tool as mt

import asyncio
import sys
import os
import argparse
import csv
import json
import time
import math

async def main():
    print("bringing up CAN...")
    os.system("sudo ip link set can0 down")
    os.system("sudo ip link set can0 up type can   tq 25 prop-seg 13 phase-seg1 12 phase-seg2 14 sjw 5   dtq 25 dprop-seg 3 dphase-seg1 1 dphase-seg2 3 dsjw 3   restart-ms 1000 fd on")
    c1 = moteus.Controller(id=1)
    c2 = moteus.Controller(id=2)
    
    stream1 = moteus.Stream(c1)
    stream2 = moteus.Stream(c2)
    
    cal_file_1 = "moteus_testing/moteus_logs/ri50_cal_1.log"
    cal_file_2 = "moteus_testing/moteus_logs/ri50_cal_2.log"
    
    print("loading moteus controller calibration from " + cal_file_1 + ", " +cal_file_2 + " ...")
    cmd1 = "python3 -m moteus.moteus_tool --target 1 --restore-cal " + cal_file_1
    cmd2 = "python3 -m moteus.moteus_tool --target 2 --restore-cal " + cal_file_2
    print(cmd1)        
    os.system(cmd1)
    print(cmd2)
    os.system(cmd2)
    
    cal1 = json.load(open(cal_file_1, "r"))
    kt_1 = 30/(math.pi*cal1["kv"])
    cal2 = json.load(open(cal_file_2, "r")) 
    kt_2 = 30/(math.pi*cal2["kv"])
    
    await c1.set_stop()
    await c2.set_stop()
    
    pos = 0.5
    while True:
        try:
            await c1.set_position(position=pos, watchdog_timeout=1.0, kp_scale=0.1, kd_scale=0.001)
            await c2.set_position(position=pos, watchdog_timeout=1.0, kp_scale=0.1, kd_scale=0.001)
            pos = -pos
            time.sleep(0.5)
        except (KeyboardInterrupt, SystemExit):
            print("stopping actuators and cleaning...")
            await c1.set_stop()
            await c2.set_stop()
            await asyncio.sleep(0.2)
            os.system("sudo ip link set can0 down")
            print("done")
            sys.exit()
        
        except :
            os.system("sudo ip link set can0 down")

if __name__ == "__main__" :
    asyncio.run(main())