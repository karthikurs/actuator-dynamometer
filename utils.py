#!/usr/bin/env python3

from __future__ import print_function

import numpy as np
from numpy import sin, cos
from numpy import linalg as LA
from numpy import linspace

from scipy import stats
from scipy.signal import butter, lfilter, freqz

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

import Adafruit_ADS1x15

def adc2temp(temp, V0 = 3.3):
    R0 = 100000
    T0 = 25

    Rf = 100000
    # V0 = 5

    Vt = round(6.144*(2.0*temp/(65536)), 6)

    Rt = Rf*Vt / (V0 - Vt)

    temp = (1/298.15) + (1/3950)*math.log(Rt/R0)
    return 1/temp - 273.15

def adc2futek(adc, gain=1, zero_val=2.500):
    torque = 6.144*(2.0*adc/(65536))
    torque = -2.0*(torque-zero_val) * gain
    return torque

# https://stackoverflow.com/questions/25191620/creating-lowpass-filter-in-scipy-understanding-methods-and-units
def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

class GaussianRandomProcess:
    def __init__(self, mean=0, amplitude=1, Ts=0.01):
        self.mean = mean
        self.amplitude = amplitude
        self.buffer = np.zeros(100)
        self.Ts = Ts
        self.fc = 0.45/self.Ts
        self.filtered_buffer = butter_lowpass_filter(self.buffer, self.fc, 1/self.Ts)
    
    def set_Ts(self, Ts_in):
        self.Ts = Ts_in

    def set_fc(self, fc_in):
        self.fc = fc_in

    def cycle(self, n=1):
        self.buffer = np.roll(self.buffer, n)
        data = np.random.random(n)
        data = (2*data)-1.0
        data *= self.amplitude
        data += self.mean
        self.buffer[0:n] = data
        self.filtered_buffer = butter_lowpass_filter(self.buffer, self.fc, 1/self.Ts)
    
    def sample(self, n=1, cycle=True):
        if cycle: self.cycle(n)
        return self.filtered_buffer[-n:]
    
    def raw_sample(self, n=1, cycle=True):
        if cycle: self.cycle(n)
        return self.buffer[-n:]