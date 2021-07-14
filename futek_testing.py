#!/usr/bin/env python3

from __future__ import print_function

import moteus
import moteus.moteus_tool as mt
import moteus_pi3hat

import numpy as np
from numpy import sin, cos
from numpy import linalg as LA
from numpy import linspace

import asyncio
import sys
import os
import argparse
import argcomplete
import csv
import json
import time
import math
from moteus_wrapper import *
from kinematics import *
from utils import *

import Adafruit_ADS1x15
from ina260.controller import Controller

# def adc2temp(temp):
#     R0 = 100000
#     T0 = 25

#     Rf = 100000
#     V0 = 5

#     Vt = round(6.144*(2.0*temp/(65536)), 6)

#     Rt = Rf*Vt / (V0 - Vt)

#     temp = (1/298.15) + (1/3950)*math.log(Rt/R0)
#     return 1/temp - 273.15

# def adc2futek(adc, gain=1, zero_val=2.500):
#     torque = 6.144*(2.0*adc/(65536))
#     torque = -2.0*(torque-zero_val) * gain
#     return torque

async def finish(c1, c2, data=None):
	if data is not None:
		print("writing data...")
		with open("futek_data/futek_test" + time.strftime("_%d_%m_%Y_%H-%M-%S") + ".csv","w+") as my_csv:
			csvWriter = csv.writer(my_csv,delimiter=',')
			csvWriter.writerows(data)
	
	print("stopping actuators and cleaning...")
	await asyncio.sleep(0.1)
	await c1.set_stop()
	await c2.set_stop()
	await asyncio.sleep(0.1)
	# os.system("sudo ip link set can0 down")

	print("done.\n\n")

async def main():
	parser = argparse.ArgumentParser(description='Runs dual-actuator moteus controller futek dynamometer setup')
	parser.add_argument("-c", "--comment",\
		help="enter comment string to be included in output csv. DO NOT INCLUDE ANY COMMAS.",
		type=str, required=True)
	parser.add_argument("-g1", "--gear1",\
		help="specify gear ratio of actuator test sample",
		type=float, required=True)
	parser.add_argument("-g2", "--gear2",\
		help="specify gear ratio of load actuator",
		type=float, required=True)
	parser.add_argument("-d", "--damping",\
		help="load damping scale. unitless",
		type=float)
	parser.add_argument("--duration",\
		help="test duration in seconds",
		type=float)

	input_group = parser.add_mutually_exclusive_group()
	input_group.add_argument("-s", "--step",\
		help="run a step response test. provide step input current to test actuator in A",
		type=float)
	input_group.add_argument("--stair",\
		help="run stairstep input (configured in the file)",action='store_true')
	input_group.add_argument("--grp",\
		help="run a lowpass filtered gaussian random process (configure in file)", action='store_true')

	parser.add_argument("--antihysteresis",\
		help="runs anti-hysteresis pattern",action='store_true')
	parser.add_argument("--torquesensor",\
		help="specify which torque sensor is being used", choices=['trd605-18', 'trs605-5'],\
		type=str, required=True)
	parser.add_argument("--tsflip",\
		help="flips torque sensor orientation",action='store_true')
	parser.add_argument("--loadmode",\
		help="specify how load actuator behaves", choices=['damping', 'damp', 'stall', 'stop', 'idle', 'velocity'],\
		type=str, required=True)
	parser.add_argument("--drivemode",\
		help="specify how driving actuator behaves", choices=['velocity', 'current'],\
		type=str, required=True)
	parser.add_argument("--hitemp",\
		help="higher temperature allowance",action='store_true')

	parser.add_argument("--actuatorflip",\
		help="Actuator 2 takes precedence instead of 1",action='store_true')

	speed_group = parser.add_mutually_exclusive_group()
	speed_group.add_argument("--fast",\
		help="skip temp readings, only command c1",action='store_true')
	speed_group.add_argument("--ultrafast",\
		help="skip temp and futek readings, only command c1",action='store_true')

	standard_test_group = parser.add_mutually_exclusive_group()
	standard_test_group.add_argument("--standard-kt-test",\
		help="runs standardized KT test: -8A to 8A, 2A increments, 5s hold, stalled load actuator.\nNOTE: This overrides other options",action='store_true')
	standard_test_group.add_argument("--standard-direct-damping-test",\
		help="runs standardized direct damping measurement test: -20Hz to 20HzA, 2.5Hz increments, 4s hold, idle load actuator.\nNOTE: This overrides other options",action='store_true')
	standard_test_group.add_argument("--standard-grp-test",\
		help="runs standardized Gaussian Random Process test for inertia and damping: 8A amplitude, 45Hz LPF, 90s duration, ultrafast sampling mode, idling load actuator.\nNOTE: This overrides other options",action='store_true')
	standard_test_group.add_argument("--standard-tv-sweep",\
		help="runs standardized torque-velocity sweep.\nNOTE: This overrides other options",action='store_true')

	argcomplete.autocomplete(parser)
	args = parser.parse_args()

	g1 = args.gear1 if args.gear1 is not None else 6.0
	g2 = args.gear2 if args.gear2 is not None else 6.0

	ts_gain = 0; ts_overload = 0
	if args.torquesensor == 'trd605-18': ts_gain = 18.0/5.0; ts_overload = 18
	elif args.torquesensor == 'trs605-5': ts_gain = 5.0/5.0; ts_overload = 5

	TP_latch = 45
	TM_latch = 85
	TP_unlatch = 40
	TM_unlatch = 70

	if args.hitemp:
		TP_latch = 80
		TP_unlatch = 70
	
	### Parameters for stairstep command
	max_cmd = 8.1   # A     or rotation Hz in velocity mode
	hold = 5        # s
	incr = 2.0      # A     or rotation Hz in velocity mode

	end_cycle = -1
	if args.standard_kt_test:
		args.step = None
		args.stair = True
		args.grp = False
		args.damping = None
		args.drivemode = 'current'
		args.loadmode = 'stall'
		args.antihysteresis = False
		args.fast = False
		args.ultrafast = False
		end_cycle = 4
		max_cmd = 8.1
		hold = 5
		incr = 2
	elif args.standard_direct_damping_test:
		args.step = None
		args.stair = True
		args.grp = False
		args.damping = None
		args.drivemode = 'velocity'
		args.loadmode = 'idle'
		# args.loadmode = 'damping'
		args.antihysteresis = False
		args.fast = False
		args.ultrafast = False
		end_cycle = 4
		max_cmd = 20.1
		hold = 5
		incr = 2.5
	elif args.standard_grp_test:
		args.step = None
		args.stair = False
		args.grp = True
		args.damping = None
		args.drivemode = 'current'
		args.loadmode = 'idle'
		args.antihysteresis = False
		args.fast = True
		# args.fast = False
		# args.ultrafast = True
		args.ultrafast = False
		args.duration = 300
	elif args.standard_tv_sweep:
		args.step = None
		args.stair = False
		args.grp = False
		args.damping = None
		args.drivemode = 'current'
		args.loadmode = 'velocity'
		args.antihysteresis = False
		args.fast = False
		args.ultrafast = False
		hold = 3
	
	rate = incr/hold      # A/s   or rotation Hz/s in velocity mode

	step_mag = args.step if args.step is not None else 0.0 
	damping = args.damping if args.damping is not None else 0.1

	### SENSOR SETUP
	adc = Adafruit_ADS1x15.ADS1015()
	GAIN = 1.0
	DATARATE = 3300

	ina1 = Controller(address= 0x40); ina1_v = 0; ina1_i = 0
	ina2 = Controller(address= 0x41); ina2_v = 0; ina2_i = 0

	zero_val = adc.read_adc(0, gain=GAIN, data_rate=DATARATE)
	zero_val = round(4.096/DATARATE*(2.0*zero_val/(4096)), 6)
	print(zero_val)

	if args.fast: adc.start_adc(0, gain=GAIN, data_rate=DATARATE)
	if not (args.fast or args.ultrafast):
		temp1 = adc.read_adc(2, gain=GAIN, data_rate=DATARATE); temp1 = adc2temp(temp1)
		temp2 = adc.read_adc(3, gain=GAIN, data_rate=DATARATE); temp2 = adc2temp(temp2)
	else:
		temp1 = 0; temp2 = 0
	
	### MOTEUS SETUP
	moteus_transport = moteus_pi3hat.Pi3HatRouter(
		servo_bus_map = {
			3:[1],
			4:[2],
		},
	)
	# c1, c2, kt_1, kt_2 = await init_controllers(transport = moteus_transport)
	c1, c2, kt_1, kt_2 = await init_controllers()

	await c1.set_rezero()
	await c2.set_rezero()
	ca = c1; cb = c2
	orient_a_1 = True # True when a1 is driving
	if args.actuatorflip:
		ca = c2
		cb = c1
		orient_a_1 = False # False when a2 is driving
	
	pos_neg = 1 # positive or negative command
	cmd1 = 0; cmd2 = 0
	replya = await ca.set_stop(query=True)
	replyb = await cb.set_stop(query=True)
	print(replya)
	print(replyb)
	
	t0 = time.monotonic()
	data = []
	if True: # if statement here just to collapse block in editor
		data.append(["# Test started at " + time.asctime()])
		data.append(["# Labels:"])
		data.append(["# \t a1 = actuator 1. Values at *actuator* level after gearbox"])
		data.append(["# \t a2 = actuator 2. Values at *actuator* level after gearbox"])
		data.append(["# \t c1 = moteus controller 1. Values at *motor* level (no gearbox)"])
		data.append(["# \t c2 = moteus controller 2. Values at *motor* level (no gearbox)"])
		data.append(["# Note: Data assumes gearbox is 100pc efficient"])
		data.append(["# "])
		data.append(["# kt_1 = {}; kt_2 = {} from calibration logs".format(kt_1, kt_2)])
		data.append(["# g1 = {}; g2 = {}".format(g1, g2)])
		data.append(["# torque sensor: {}".format(args.torquesensor)])
		data.append(["# tsflip = {}".format(args.tsflip)])
		data.append(["# load mode: {}".format(args.loadmode)])
		data.append(["# drive mode: {}".format(args.drivemode)])
		data.append(["# load damping scale = {}".format(damping)])
		if args.step is not None: data.append(["# step input test; magnitude = {} A".format(step_mag)])
		if args.comment is not None: data.append(["# User comment: " + args.comment.replace(',',';')])

	cmd_label = 'q-axis cmd [A]'
	if args.drivemode == 'velocity': cmd_label = 'velocity cmd [Hz]'

	data.append(["time [s]",\
				"time function [s]",\
				"a1 {}".format(cmd_label),\
				"a1 torque cmd [Nm]",\
				"a1 position [rad]", "a1 velocity [rad/s]", "a1 torque [Nm]", "a1 q-axis [A]",\
				"a2 {}".format(cmd_label),\
				"a2 torque cmd [Nm]",\
				"a2 position [rad]", "a2 velocity [rad/s]", "a2 torque [Nm]", "a2 q-axis [A]",\
				"c1 mode", "c1 position [rev]", "c1 vel [Hz]",\
				"c1 torque [Nm]", "c1 voltage [V]",\
				"c1 temp [C]", "c1 fault",\
				"c2 mode", "c2 position [rev]", "c2 vel [Hz]",\
				"c2 torque [Nm]", "c2 voltage [V]",\
				"c2 temp [C]", "c2 fault",\
				"{} torque [Nm]".format(args.torquesensor),
				"motor temp [C]", "housing temp [C]",
				"ina1 voltage [V]", "ina1 current [A]", "ina1 power [W]",\
				"ina2 voltage [V]", "ina2 current [A]", "ina2 power [W]",\
				"load velocity cmd [Hz]" if args.loadmode == 'velocity' else ""])

	### COMMAND SETUP
	overtemp = False
	old_cmd = 0.0
	cmd = 0.0
	load_v = 0.0
	t0_fcn = t0

	safety_max = 8.1
	
	Ts = 0.0025
	GRP = GaussianRandomProcess(mean=0, amplitude=8, Ts=Ts)
	# Set GRP cutoff frequency here
	GRP.set_fc(45)
	grp_tstart = 0 # GRP delayed start
	grp_n = 100
	grp_buffer = GRP.sample(n=grp_n)
	grp_ii = 0

	driving_i_cmd_vec = []
	loading_v_cmd_vec = []
	condition_time_vec = []
	nom_i_A = np.linspace(0, 5, 11)
	nom_v_Hz = np.linspace(0.5, 20.5, 11)
	i_pos_neg = [1, 1]
	v_pos_neg = [1, -1]
	if args.standard_tv_sweep:
		hold = 3.75
		rest = 0.25
		for v_nom in nom_v_Hz:
			for i_nom in nom_i_A:
				for pn in range(2):
					driving_i_cmd_vec.append(i_pos_neg[pn]*i_nom)
					loading_v_cmd_vec.append(v_pos_neg[pn]*v_nom)
					condition_time_vec.append(hold)
					driving_i_cmd_vec.append(0)
					loading_v_cmd_vec.append(0)
					condition_time_vec.append(rest)
		end_cycle = len(driving_i_cmd_vec)-1
	condition_time_vec.append(0)
	cycle = 0
	t_fcn = 0
	t_vec = np.array([])
	t_old = 0
	t = 0
	### MAIN TESTING LOOP
	# moteus_transport = c1._get_transport()
	
	while True:
		try:
			t = time.monotonic() - t0

			### Setup t_fcn -- freezes when overtemp is latched
			if not overtemp: t_fcn += (t - t_old)
			# print(t_fcn)
			
			### Figure out sampling period
			dt_med = t - t_old
			t_old = t
			np.append(t_vec,t)
			frame = min(max(len(t_vec) - 2, 1), 20)
			if frame > 1:
				dt_vec = t_vec[-frame:] - t_vec[-frame-1:-1]
				dt_med = np.median(dt_vec)
			else:
				dt_med = 0.005
			GRP.set_Ts(dt_med)
			
			### Cycle input function if requested
			if not args.standard_tv_sweep and not args.standard_grp_test and t_fcn > (max_cmd+incr)/rate:
				# Swap driving and loading actuators
				if not args.fast and not args.ultrafast:
					ctemp = ca
					ca = cb
					cb = ctemp
					orient_a_1 = not orient_a_1 # swap driving actuator
					print("reverse! reverse!")
				if orient_a_1: pos_neg = -pos_neg # swap cmd sign every other cycles
				# t0_fcn = time.monotonic()
				t_fcn = 0
				# print("cycle")
				cycle += 1
			elif args.standard_tv_sweep and t_fcn > sum(condition_time_vec[:cycle+1]):
				cycle += 1

			### Timed Finish
			if args.duration is not None and t > args.duration:
				print("test duration done")
				await finish(c1, c2, data)
				return
			### Cycle Finish
			if end_cycle > 0 and cycle > end_cycle:
				print("{} cycles complete".format(cycle-1))
				await finish(c1, c2, data)
				return
			
			old_cmd = cmd
			freq_hz = 0
			
			### Generate Command
			if args.step: cmd = step_mag
			elif args.stair:
				cmd0 = pos_neg * incr*(min(rate*(t_fcn), max_cmd)//incr)
				# modulate command to combat/balance out hysteresis effects
				if args.antihysteresis:
					arg = ((rate*t_fcn)%incr)/incr
					if( arg > 0.01 and arg <= 0.2): cmd = cmd0
					elif(arg > 0.2 and arg <= 0.4): cmd = cmd0 + min(0.5*incr, 0.3)
					elif(arg > 0.4 and arg <= 0.6): cmd = cmd0
					elif(arg > 0.6 and arg <= 0.8): cmd = cmd0 - min(0.5*incr, 0.3)
					elif(arg > 0.8 and arg <= 0.9): cmd = cmd0
					elif(arg > 0.9): cmd = 0
					else: cmd = 0
				else:
					cmd = cmd0
				# freq_hz = ((0.5*t)//1.0) # exponent
				# freq_hz = min(1.0*(1.1**freq_hz), 45) # increase freq by 10% every 2 sec
				# cmd = max_cmd*math.cos(freq_hz*np.pi*t)
			elif args.grp:
				if t < grp_tstart:
					# variable low frequency sweep to get more low frequency data
					cmd = 2.0*math.sin((15/grp_tstart)*t*2*np.pi  *  t)
				else:
					if grp_ii >= grp_n:
						grp_buffer = GRP.sample(n=grp_n)
						grp_ii = 0
						# print("grp_refresh")
					cmd = grp_buffer[grp_ii]
					grp_ii += 1
			elif args.standard_tv_sweep:
				cmd = driving_i_cmd_vec[cycle]
				load_v = loading_v_cmd_vec[cycle]

			### Overtemp Detection and Latch
			if min(temp1, temp2) > TP_latch or max(temp1, temp2) > TM_latch or overtemp:
				if t % 1.0 < 0.019 or overtemp == False and not args.fast:
					print("over temp: temp1 = {}, temp2 = {}".format(round(temp1, 2), round(temp2, 2)))
				# await finish(c1,c2,data)
				# return
				overtemp = True
				cmd = 0.0

			### Overtemp unlatch
			if min(temp1, temp2) < TP_unlatch and max(temp1, temp2) < TM_unlatch and overtemp:
				overtemp = False

			moteus_cmds = []
			### Load Actuator
			if not args.fast and not args.ultrafast:
				if args.loadmode == 'damping' or args.loadmode == 'damp':
					# replyb = (await cb.set_position(position=math.nan, velocity=0,\
					#     watchdog_timeout=1.0, kp_scale=0.0, kd_scale=damping, query=True))
					moteus_cmds.append(cb.make_position(position=math.nan, velocity=0,\
						watchdog_timeout=1.0, kp_scale=0.0, kd_scale=damping, query=True))
				elif args.loadmode == 'stall':
					# replyb = (await cb.set_position(position=0.0, velocity=math.nan,\
					#     watchdog_timeout=2.0, kp_scale=15, kd_scale=5, query=True))
					moteus_cmds.append(cb.make_position(position=0.0, velocity=math.nan,\
						watchdog_timeout=2.0, kp_scale=15, kd_scale=5, query=True))
				elif args.loadmode == 'velocity':
					# replyb = (await cb.set_position(position=math.nan, velocity=load_v,\
					#     watchdog_timeout=2.0, kp_scale=5, kd_scale=5, query=True))
					moteus_cmds.append(cb.make_position(position=math.nan, velocity=load_v,\
						watchdog_timeout=2.0, kp_scale=5, kd_scale=5, query=True))
				# elif args.loadmode == 'idle':
				#     replyb = replyb
				else:
					# replyb = await cb.set_stop(query=True)
					moteus_cmds.append(cb.make_stop(query=True))
				
			### Driving Actuator
			if args.drivemode == 'current':
				cmd = min(max(cmd, -safety_max), safety_max)
				# replya = (await ca.set_current(q_A=cmd, d_A=0.0, query=True))
				moteus_cmds.append(ca.make_current(q_A=cmd, d_A=0.0, query=True))
			elif args.drivemode == 'velocity':
				# replya = (await ca.set_position(position=math.nan, velocity=cmd,\
				#     watchdog_timeout=2.0, query=True))
				moteus_cmds.append(ca.make_position(position=math.nan, velocity=cmd,\
					watchdog_timeout=2.0, query=True))
			else:
				cmd = 0
				# replya = await ca.set_stop(query=True)
				moteus_cmds.append(ca.make_stop(query=True))

			# print(moteus_cmds)
			
			replies = await moteus_transport.cycle(moteus_cmds)
			if len(replies) > 1: replyb = replies[0]; replya = replies[1]
			else: replya = replies[0]; replyb = None

			print(replya)
			print(replyb)
			### Keep track of data when actuators swap driving/driven
			if orient_a_1:
				reply1 = replya
				reply2 = replyb
				cmd1 = cmd
				cmd2 = 0
			else:
				reply2 = replya
				reply1 = replyb
				cmd1 = 0
				cmd2 = cmd

			# Position, velocity, torque
			p1, v1, t1 = parse_reply(reply1, g1) if reply1 is not None else 0,0,0
			p2, v2, t2 = parse_reply(reply2, g2) if reply2 is not None else 0,0,0

			### Read from analog sensors (torque, thermistors, power meters)
			if not args.fast and not args.ultrafast:
				# futek_torque = adc.read_adc(1, gain=GAIN, data_rate=DATARATE); futek_torque = adc2futek(futek_torque, gain=5/5)
				futek_torque = adc.read_adc(0, gain=GAIN, data_rate=DATARATE); futek_torque = adc2futek(futek_torque, gain=5/5)
				temp1 = adc.read_adc(2, gain=GAIN, data_rate=DATARATE); temp1 = adc2temp(temp1)
				temp2 = adc.read_adc(3, gain=GAIN, data_rate=DATARATE); temp2 = adc2temp(temp2)

				ina1_v = ina1.voltage(); ina1_i = ina1.current()
				ina2_v = ina2.voltage(); ina2_i = ina2.current()
			elif args.fast:
				futek_torque = adc.get_last_result(); futek_torque = adc2futek(futek_torque, gain=5/5)
				temp1=0; temp2=0
				ina1_v=0; ina1_i=0
				ina2_v=0; ina2_i=0
			else:
				futek_torque=0;temp1=0;temp2=0
				ina1_v=0; ina1_i=0
				ina2_v=0; ina2_i=0
			
			### Terminal print for monitoring
			if t % 1.0 < 3.0*dt_med and not args.fast and not args.ultrafast: 
				print("t = {}s, temp1 = {}, temp2 = {}, cycle = {}, cmd = {}, load_v = {}, t1 = {}, t2 = {}".format(\
				round(t, 1), round(temp1, 1), round(temp2, 1), cycle, round(cmd, 2), round(load_v, 2), round(t1, 1), round(t2, 1)))

			observed_kt = 0 if np.abs(cmd) < 0.001 else futek_torque/cmd

			### Log data
			row = [t, t_fcn] +\
				[cmd1/g1 if args.drivemode=='velocity' else cmd1] + [cmd1*kt_1*g1 if args.drivemode=='current' else 0] +\
				[p1, v1, t1, t1/(kt_1*g1)] +\
				[cmd2/g2 if args.drivemode=='velocity' else cmd2] + [cmd2*kt_2*g2 if args.drivemode=='current' else 0] +\
				[p2, v2, t2, t2/(kt_2*g2)] +\
				raw_reply_list(reply1) + raw_reply_list(reply2) +\
				[futek_torque, temp1, temp2] +\
				[ina1_v, ina1_i, ina1_v*ina1_i] +\
				[ina2_v, ina2_i, ina2_v*ina2_i]
			if args.loadmode == 'velocity': row += [load_v]
			data.append(row)

			### Torque overload protection
			if abs(t1) > ts_overload or abs(t2) > ts_overload or abs(futek_torque) > ts_overload:
				print("torque overload detected")
				await finish(c1, c2, data)
				# sys.exit()
				return

			### This sleep seems to help with loop rate consistency
			# if args.fast: await asyncio.sleep(0.0001)
			await asyncio.sleep(0.5)

		except (KeyboardInterrupt, SystemExit):
			print("\nCaught keyboard interrupt; exiting...")
			print(dt_med)
			await finish(c1, c2, data)
			# sys.exit()
			return
		except:
			print("\n\nUnanticipated exception: something went wrong\n\n")
			await finish(c1, c2, data)
			print(dt_med)
			raise
			# sys.exit()
			return
	
	await finish(c1,c2,data)

if __name__ == "__main__" :
	asyncio.run(main())

