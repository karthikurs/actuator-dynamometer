#!/usr/bin/env python3

from __future__ import print_function

import moteus
import moteus.moteus_tool as mt
import moteus_pi3hat


import numpy as np
import asyncio
import sys
import os
import argparse
import csv
import json
import time
import math

async def e_stop():
#Defines the transport and stops the error that prevents the transport from not being able to be found
    c1, c2, kt_1, kt_2 = await init_controllers()
    print(kt_1)
    print(kt_2)
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            1:[1, 2]
        },
    )

    for _ in range(2):
        replies = await transport.cycle(\
                                    [c1.make_stop(query=True),\
                                    c2.make_stop(query=True)])
    # os.system("sudo ip link set can0 up type can   tq 25 prop-seg 13 phase-seg1 12 phase-seg2 14 sjw 5   dtq 25 dprop-seg 3 dphase-seg1 1 dphase-seg2 3 dsjw 3   restart-ms 1000 fd on")
    c1 = moteus.Controller(id=1)
    c2 = moteus.Controller(id=2)
    await c1.set_stop()
    await c2.set_stop()

async def rezero():
#Resets position angles to within one rotation
    c1, c2, kt_1, kt_2 = await init_controllers()
    print(kt_1)
    print(kt_2)
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            1:[1, 2]
        },
    )

    for _ in range(2):
        replies = await transport.cycle(\
                                    [c1.make_rezero(rezero=0.0, query=True),\
                                    c2.make_rezero(rezero=0.0,query=True)])

    print('Angles reset to within one rotation')
    

async def init_controllers():
    # print("bringing up CAN...")
    # os.system("sudo ip link set can0 down")
    # os.system("sudo ip link set can0 up type can   tq 25 prop-seg 13 phase-seg1 12 phase-seg2 14 sjw 5   dtq 25 dprop-seg 3 dphase-seg1 1 dphase-seg2 3 dsjw 3   restart-ms 1000 fd on")
    c1 = moteus.Controller(id=1)
    c2 = moteus.Controller(id=2)
    
    stream1 = moteus.Stream(c1)
    stream2 = moteus.Stream(c2)
    
    cal_file_1 = "moteus-setup/moteus-cal/ri50_cal_1_leg.log"
    cal_file_2 = "moteus-setup/moteus-cal/ri50_cal_2_leg.log"
    
    print("loading moteus controller calibration from " + cal_file_1 + ", " +cal_file_2 + " ...")
    cmd1 = "sudo python3 -m moteus.moteus_tool --target 1 --pi3hat-cfg '1=1,2' --restore-cal " + cal_file_1
    cmd2 = "sudo python3 -m moteus.moteus_tool --target 2 --pi3hat-cfg '1=1,2' --restore-cal " + cal_file_2
    print(cmd1)        
    os.system(cmd1)
    print(cmd2)
    os.system(cmd2)
    
    cal1 = json.load(open(cal_file_1, "r"))
    kt_1 = 0.78*cal1["v_per_hz"]/np.pi
    cal2 = json.load(open(cal_file_2, "r")) 
    kt_2 = 0.78*cal2["v_per_hz"]/np.pi
    
    await c1.set_stop()
    await c2.set_stop()

    return c1, c2, kt_1, kt_2

def parse_reply(reply, g):
    pos = reply.values[1]
    vel = reply.values[2]
    trq = reply.values[3]
    return pos*np.pi*2/g, vel*np.pi*2/g, trq*g

def raw_reply_list(reply):
    data = [val for key, val in reply.values.items()] if reply is not None else [0]*7
    return data

def raw_reply_headers():
    headers = ["mode", "position [rev]", "vel [Hz]",\
                "torque [Nm]", "voltage [V]",\
                "temp [C]", "fault"]
    return headers