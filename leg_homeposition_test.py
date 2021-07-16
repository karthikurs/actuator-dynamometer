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
import csv
import json
import time
import math
from moteus_wrapper import *
from kinematics import *



async def main():
    c1, c2, kt_1, kt_2 = await init_controllers()
    print(kt_1)
    print(kt_2)
#Defines the transport and stops the error that prevents the transport from not being able to be found
    transport = moteus_pi3hat.Pi3HatRouter(
        servo_bus_map = {
            1:[1, 2]
        },
    )

    for _ in range(2):
        replies = await transport.cycle(\
                                    [c1.make_stop(query=True),\
                                    c2.make_stop(query=True)])
        # import ipdb; ipdb.set_trace()
    reply1 = replies[0]
    reply2 = replies[1]
    p1, v1, t1 = parse_reply(reply1, 1)
    p2, v2, t2 = parse_reply(reply2, 1)

#

    print([p1,v1,t1])

    kin = Kinematics()

    [x,y] = kin.fk_vec(p1,p2)[-1]

    [x0,y0] = [x+20,y-10]
 
    k = 0.5
    torques=np.empty([0,3],dtype=float)
    print(torques)
    start=time.time()
    ctime=time.time()-start
    ctimeprev=time.time()
    freqency=np.empty([0,1],dtype=float)
    while True:
        try:
            fx = -k*(x-x0)
            fy = -k*(y-y0)

            J = kin.jacobian_theta(p1,p2)
            Jinv = LA.inv(J)
            dt = np.matmul(Jinv, np.array([[fx],[fy]]))
            t1 = dt[0,0]
            t2 = dt[1,0]
            # print('fx={} fy={} t1={} t2={}'.format(fx, fy, t1, t2))
            # replies = await transport.cycle(\
            #                 [c1.make_stop(query=True),\
            #                 c2.make_stop(query=True)])


            # reply1 = replies[0]
            # reply2 = replies[1]
            # p1, v1, t1 = parse_reply(reply1, 6)
            # p2, v2, t2 = parse_reply(reply2, 6)


            # p1, v1, t1 = parse_reply(await c1.set_current(q_A=t1/kt_1, d_A=0.0, query=True), g=6)
            # p2, v2, t2 = parse_reply(await c2.set_current(q_A=t2/kt_2, d_A=0.0, query=True), g=6)
            # print('p1={},v1={},t1={}'.format(p1, v1, t1))
            # print('p2={},v2={},t2={}'.format(p2, v2, t2))


        
            adj_1=0.31862059281708005
            adj_2=1.164099670406716
            
            # adj_1=0.15433981845496693
            # adj_2=0.34315861931457507
            d_p1=0+adj_1
            d_p2=0+adj_2
            d_pos1=(d_p1*6)/(2*np.pi)
            d_pos2=(d_p2*6)/(2*np.pi)


            p1, v1, t1 = parse_reply(await c1.set_position(position=np.NaN, velocity=0.5, maximum_torque=0.3,stop_position=d_pos1,query=True), g=6)
            p2, v2, t2 = parse_reply(await c2.set_position(position=np.NaN, velocity=0.5, maximum_torque=0.3,stop_position=d_pos2,query=True), g=6)

            p1_h=p1-adj_1
            p2_h=p2-adj_2
            ctimeprev=ctime
            ctime=time.time()-start
            freq=1/(ctime-ctimeprev)
            frequency=np.append(frequency,np.array([ [freq] ]),axis=0)
            torques=np.append(torques,np.array([ [ctime,\
                                                t1,\
                                                t2] ]),axis=0)
                                                

            # np.savetxt("pose_trace.csv", data, delimiter=",")
            
            # print('p1={},v1={},t1={}'.format(p1, v1, t1))
            # print('p1_h={},v1={},t1={}'.format(p1_h, v1, t1))
            # print('p2={},v2={},t2={}'.format(p2, v2, t2))
            # print('p2_h={},v2={},t2={}'.format(p2_h, v2, t2))
 
            #Zero out tests
            #Home pose defaults
            # p1=0.16087623512950663
            # p2=1.0025842766155904

            #IMPORTANT LEG NOTES
            #Looking at leg head on (as if robot facing to the right) and the leg is one of the right legs of the robot
            #Actuator 1: CCW negative CW Positive
            #Actuator 2: CCW negative CW Positive

            #Ranges of Motion in terms of Encoder angle readings (normalized to home position)

            #ACTUATOR 1:
            #Easily visualized from femur angle, but generally for safety with current wire config +- PI

            #ACTUATOR 2
            #p2_h=-0.8247064210870179,v2=0.02996056226339143,t2=0.0
            #p2_h=0.5899434446743528,v2=0.02996056226339143,t2=0.0


            # time.sleep(2)

            # p1, v1, t1 = parse_reply(await c1.set_rezero(rezero=0.0, query=True), g=6)
            # p2, v2, t2 = parse_reply(await c2.set_rezero(rezero=0.0, query=True), g=6)    
            
            # print('ZEROED OUT')
            # print('p1={},v1={},t1={}'.format(p1, v1, t1))
            # print('p2={},v2={},t2={}'.format(p2, v2, t2))

            [x,y] = kin.fk_vec(p1,p2)[-1]
            time.sleep(0.005)
        except (KeyboardInterrupt, SystemExit):
            print("stopping actuators and cleaning...")
            print('Saving torques')
            # np.savetxt("home_pos_torque_log.csv", torques, delimiter=",")
            print('Torques saved')
            await c1.set_stop()
            await c2.set_stop()
            await asyncio.sleep(0.2)
            os.system("sudo ip link set can0 down")
            print("done")
            sys.exit()
        except:
            os.system("sudo ip link set can0 down")
            print('Saving torques')
            # np.savetxt("home_pos_torque_log.csv", torques, delimiter=",")
            print('Torques saved')
            print("something went wrong")
            raise
            sys.exit()

if __name__ == "__main__" :
    asyncio.run(main())
