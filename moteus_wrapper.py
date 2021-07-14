#!/usr/bin/env python3

from __future__ import print_function

import moteus
import moteus.moteus_tool as mt

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
	os.system("sudo ip link set can0 up type can   tq 25 prop-seg 13 phase-seg1 12 phase-seg2 14 sjw 5   dtq 25 dprop-seg 3 dphase-seg1 1 dphase-seg2 3 dsjw 3   restart-ms 1000 fd on")
	c1 = moteus.Controller(id=1)
	c2 = moteus.Controller(id=2)
	await c1.set_stop()
	await c2.set_stop()

async def init_controllers(transport=None):
	# print("bringing up CAN...")
	# os.system("sudo ip link set can0 down")
	# os.system("sudo ip link set can0 up type can   tq 25 prop-seg 13 phase-seg1 12 phase-seg2 14 sjw 5   dtq 25 dprop-seg 3 dphase-seg1 1 dphase-seg2 3 dsjw 3   restart-ms 1000 fd on")
	if transport is not None:
		print("using pi3hat transport...")
		c1 = moteus.Controller(id=1, transport=transport)
		c2 = moteus.Controller(id=2, transport=transport)
	else:
		print("finding default transport...")
		c1 = moteus.Controller(id=1)
		c2 = moteus.Controller(id=2)
	
	stream1 = moteus.Stream(c1)
	stream2 = moteus.Stream(c2)
	
	cal_file_1 = "moteus-setup/moteus-cal/ri50_cal_1_dyn.log"
	cal_file_2 = "moteus-setup/moteus-cal/ri50_cal_2_dyn.log"
	
	print("loading moteus controller calibration from " + cal_file_1 + ", " +cal_file_2 + " ...")
	cmd1 = "python3 -m moteus.moteus_tool --target 1 --pi3hat-cfg '3=1;4=2' --restore-cal " + cal_file_1
	cmd2 = "python3 -m moteus.moteus_tool --target 2 --pi3hat-cfg '3=1;4=2' --restore-cal " + cal_file_2
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