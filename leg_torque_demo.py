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
import csv

async def main():
    c1, c2, kt_1, kt_2 = await init_controllers()
    print('Kt_1')
    print(kt_1)
    print('Kt_2')
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


    adj_1=0.10433981845496693
    adj_2=0.34315861931457507
    p1_h=p1-adj_1
    p2_h=p2-adj_2

    # print([p1,v1,t1])

    kin = Kinematics()

    [x,y] = kin.fk_vec(p1,p2)[-1]

    [x0,y0] = [x+20,y-10]
 
    k = 0.5
    j=0
    dtimes=[]
    dtimes2=[]
    t1des=[]
    t2des=[]
    tracked_torques=np.empty([0,5],dtype=float)

    kp=1.05
    kd=0.000
    t1=0
    t2=0




    # with open('home_pos_torque_log.csv') as des_torques:
    with open('torque_test.csv') as des_torques:
        csvReader = csv.reader(des_torques)
        for row in csvReader:
            dtimes.append(float(row[0]))
            t1des.append(float(row[1]))
            dtimes2.append(float(row[2]))
            t2des.append(float(row[3]))


    start=time.time()
    err=0
    prev_err=0
    ctime_prev=0.02

    while True:
        try:
            # fx = -k*(x-x0)
            # fy = -k*(y-y0)

            # J = kin.jacobian_theta(p1,p2)
            # Jinv = LA.inv(J)
            # dt = np.matmul(Jinv, np.array([[fx],[fy]]))
            # t1 = dt[0,0]
            # t2 = dt[1,0]
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

            #kt_1 is 0.1095229868838896
            
            # print('p1={},v1={},t1={}'.format(p1, v1, t1))
            # print('p2={},v2={},t2={}'.format(p2, v2, t2))
            
            ###############################
            #Set up probe to query current time and compare it to next time index, 
            #if time is greater tan next time index swiwtch torque signal
            ###############################

        
            adj_1=0.10433981845496693
            adj_2=0.34315861931457507
            d_p1=0+adj_1
            d_p2=0+adj_2
            d_pos1=(d_p1*6)/(2*np.pi)
            d_pos2=(d_p2*6)/(2*np.pi)

            ctime=time.time()-start
            
            
        
            if j<len(dtimes)-1:  
                err=t1des[j]-t1
                P=kp*err
                D=kd*(err-prev_err)/(ctime-ctime_prev)

                ctime_prev=ctime
                prev_err=err

                t1PID=P+D
                if t1PID>3:
                    t1PID=3


                if abs(ctime-dtimes[j]<0.009):
                    # print("Current Time:",end='')  
                    # print("Desired Time:",end='')
                    # print(dtimes[j],end='   ')
                    # print("Diff:",end='')
                    # print(ctime-dtimes[j])
                    j=j+1

                if abs(p1_h)>=1.3:
                    replies = await transport.cycle(\
                            [c1.make_stop(query=True),\
                            c2.make_stop(query=True)])
                    reply1 = replies[0]
                    reply2 = replies[1]
                    p1, v1, t1 = parse_reply(reply1, 6)
                    p2, v2, t2 = parse_reply(reply2, 6)
                    print('############################STOPPING#############')
                        
                else:
                    p1, v1, t1 = parse_reply(await c1.set_position(position=np.NaN, velocity=np.NaN, feedforward_torque=t1PID,\
                                                                kp_scale=0,kd_scale=0,query=True), g=6)
                # p1, v1, t1 = parse_reply(await c1.set_current(q_A=t1PID/kt_1, d_A=0.0, query=True), g=6)
                # p2, v2, t2 = parse_reply(await c2.set_current(q_A=t2/kt_2, d_A=0.0, query=True), g=6)

                tracked_torques=np.append(tracked_torques,np.array([ [ctime,\
                                                t1,\
                                                t1des[j],\
                                                err,\
                                                t1PID] ]),axis=0)

                p1_h=p1-adj_1
                p2_h=p2-adj_2
                print('p1_h={},v1={},t1={},t1des={}'.format(p1_h, v1, t1,t1des[j]))
                # print('p2={},v2={},t2={}'.format(p2, v2, t2))
                print('p2_h={},v2={},t2={}'.format(p2_h, v2, t2))

            else:
                err=0-t1
                P=kp*err
                D=kd*(err-prev_err)/(ctime-ctime_prev)

                ctime_prev=ctime
                prev_err=err

                t1PID=P+D
                if t1PID>3:
                    t1PID=3

                replies = await transport.cycle(\
                            [c1.make_stop(query=True),\
                            c2.make_stop(query=True)])

                reply1 = replies[0]
                reply2 = replies[1]
                p1, v1, t1 = parse_reply(reply1, 6)
                p2, v2, t2 = parse_reply(reply2, 6)

                tracked_torques=np.append(tracked_torques,np.array([ [ctime,\
                                                t1,\
                                                0,\
                                                err,\
                                                t1PID] ]),axis=0)


                p1_h=p1-adj_1
                p2_h=p2-adj_2

                print('p1_h={},v1={},t1={}'.format(p1_h, v1, t1))
                # print('p2={},v2={},t2={}'.format(p2, v2, t2))
                print('p2_h={},v2={},t2={}'.format(p2_h, v2, t2))
                

        
 
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
            np.savetxt("tracked_torque_log.csv", tracked_torques, delimiter=",")
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
            np.savetxt("tracked_torque_log.csv", tracked_torques, delimiter=",")
            print('Torques saved')
            print("something went wrong")
            raise
            sys.exit()
            

if __name__ == "__main__" :
    asyncio.run(main())
