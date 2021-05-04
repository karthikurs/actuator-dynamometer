import argparse
import pandas as pd
import numpy as np
import matplotlib as mpl
from matplotlib import pyplot as plt
from scipy import stats
from scipy.signal import butter, lfilter, freqz

from utils import *

rng = np.random.default_rng()
Ts = 0.01
N = 500
# c_ratio = 0.05

time = np.arange(0,N*Ts, Ts)
# data = rng.random(N)
# data_filt = butter_lowpass_filter(data, c_ratio/Ts, 1/Ts)


GRP = GaussianRandomProcess(mean=0, amplitude=3, Ts=Ts)
GRP.set_fc(0.1/Ts)
data_filt = []
data = []

for ii in range(len(time)):
    data_filt = np.append(data_filt, GRP.sample())
    data = np.append(data, GRP.raw_sample(cycle=False))

# import ipdb; ipdb.set_trace()
fig = plt.figure()
ax = fig.gca()

ax.scatter(time, data)
ax.plot(time, data_filt)
ax.plot(time, butter_lowpass_filter(data, GRP.fc, 1/GRP.Ts))

plt.show()