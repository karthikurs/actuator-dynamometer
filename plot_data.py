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

# Data for plotting
with open('tracked_torque_log.csv') as des_torques:
        csvReader = csv.reader(des_torques)
        for row in csvReader:
            time.append(float(row[0]))
            t1.append(float(row[1]))
            t1des.append(float(row[2]))
            err.append(float(row[3]))
            t1PID.append(float(row[4]))
            

fig, ax=plt.subplots(2)

ax[0].plot(time,t1,label="Actual Torques")
ax[0].plot(time,t1des,label="Desired Torques")
ax[0].plot(time,t1PID,label="PID Torques")

ax[0].xlabel=('Time (s)')
ax[0].ylabel=('Torques (Nm)')

ax[0].legend()

ax[1].plot(time,err,label="Error")

ax[1].xlabel=('Time (s)')
ax[1].ylabel=('Torques (Nm)')

ax[1].legend()

plt.show()




################Actuator1&2 TORQUES

# Data for plotting
# with open('leg_jump.csv') as des_torques:
#         csvReader = csv.reader(des_torques)
#         for row in csvReader:
#             time.append(float(row[0]))
#             t1.append(float(row[1]))
#             t2.append(float(row[2]))
            
            

# fig, ax=plt.subplots()

# ax.plot(time,t1,label="Actuator 1")
# ax.plot(time,t2,label="Actuator 2")
# # ax[0].plot(time,t1PID,label="PID Torques")

# ax.xlabel=('Time (s)')
# ax.ylabel=('Torques (Nm)')

# ax.legend()

# plt.show()