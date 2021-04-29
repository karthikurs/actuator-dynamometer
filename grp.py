import argparse
import pandas as pd
import numpy as np
import matplotlib as mpl
from matplotlib import pyplot as plt
from scipy import stats
from scipy.signal import butter, lfilter, freqz


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

rng = np.random.default_rng()
Ts = 0.01
N = 500
c_ratio = 0.45

time = np.arange(0,N*Ts, Ts)
data = rng.random(N)
data_filt = butter_lowpass_filter(data, c_ratio/Ts, 1/Ts)

# import ipdb; ipdb.set_trace()

fig = plt.figure()
ax = fig.gca()

ax.scatter(time, data)
ax.plot(time, data_filt)

plt.show()