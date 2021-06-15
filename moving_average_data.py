#!/usr/bin/env python3

from __future__ import print_function

import matplotlib.pyplot as plt
import numpy as np
import csv

time=[]
t1=[]
t2=[]
t1des=[]
t2des=[]
err=[]
t1PID=[]

################PID TORQUES

# # Data for plotting
# with open('tracked_torque_log.csv') as des_torques:
#         csvReader = csv.reader(des_torques)
#         for row in csvReader:
#             time.append(float(row[0]))
#             t1.append(float(row[1]))
#             t1des.append(float(row[2]))
#             err.append(float(row[3]))
#             t1PID.append(float(row[4]))
            

# fig, ax=plt.subplots(2)

# ax[0].plot(time,t1,label="Actual Torques")
# ax[0].plot(time,t1des,label="Desired Torques")
# ax[0].plot(time,t1PID,label="PID Torques")

# ax[0].xlabel=('Time (s)')
# ax[0].ylabel=('Torques (Nm)')

# ax[0].legend()

# ax[1].plot(time,err,label="Error")

# ax[1].xlabel=('Time (s)')
# ax[1].ylabel=('Torques (Nm)')

# ax[1].legend()

# plt.show()



N=20
################Actuator1&2 TORQUES

# Data for plotting
with open('leg_jump1.csv') as des_torques:
        csvReader = csv.reader(des_torques)
        for row in csvReader:
            time.append(float(row[0]))
            t1.append(float(row[1]))
            t2.append(float(row[2]))
            
            

fig, ax=plt.subplots()

move_av1=np.convolve(t1,np.ones(N)/N, mode='valid')
move_av2=np.convolve(t2,np.ones(N)/N, mode='valid')
gh=len(move_av1)
av_time1=np.arange(0,time[-2],(time[-1]-time[0])/gh)
av_time2=np.arange(0,time[-2],(time[-1]-time[0])/gh)

av_torques=np.empty([0,4],dtype=float)

for i in range(0,len(move_av1)):
  av_torques=np.append(av_torques,np.array([ [av_time1[i],\
                                                move_av1[i],\
                                                av_time2[i],\
                                                move_av2[i]] ]),axis=0)

np.savetxt("torque_test.csv", av_torques, delimiter=",")

# hh=len(av_time)

ax.plot(time,t1,label="Actuator 1")
ax.plot(time,t2,label="Actuator 2")
ax.plot(av_time1,move_av1,label="A1 Move-Ave")
ax.plot(av_time2,move_av2,label="A2 Move-Ave")
# ax[0].plot(time,t1PID,label="PID Torques")

ax.xlabel=('Time (s)')
ax.ylabel=('Torques (Nm)')

ax.legend()

plt.show()