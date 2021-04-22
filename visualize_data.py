#!/usr/bin/env python3

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


def main() :
    parser = argparse.ArgumentParser(description='Plots data from csv')
    parser.add_argument("filename",\
        help="filename of csv", type=str)
    parser.add_argument("-i", "--interactive",\
        help="interactive mode: decide which data to plot of what is present in csv",
        action='store_true')
    parser.add_argument("-o", "--outlier",\
        help="outlier rejection: ignore rows in the csv for which brake torque data are in the <arg> extremes of the data: -o 0.05 will drop the top and bottom 5%.",
        type=float)
    parser.add_argument("-g", "--gear",\
        help="specify gear ratio of actuator test sample",
        type=float)
    parser.add_argument("-a", "--averaging",\
        help="specify number of samples to use for running average filtering",
        type=int)

    args = parser.parse_args()
    
    data = pd.read_csv(args.filename, comment='#', header=0)

    # if args.outlier is not None:
        # import ipdb; ipdb.set_trace()
        # data = data[data['brake torque [Nm]']\
        #     .between(\
        #         data['brake torque [Nm]'].quantile(args.outlier),\
        #         data['brake torque [Nm]'].quantile(1-args.outlier)\
        #         )\
        #     ]

    gear_ratio = 6
    if args.gear is not None:
        gear_ratio = args.gear

    averaging_num_samples = 1
    if args.averaging is not None:
        averaging_num_samples = args.averaging
        # data['brake torque [Nm]'] = np.convolve(data['brake torque [Nm]'],\
        #     np.ones(averaging_num_samples)/averaging_num_samples, mode='same')
        # data['motor torque measured [Nm]'] = np.convolve(data['motor torque measured [Nm]'],\
        #     np.ones(averaging_num_samples)/averaging_num_samples, mode='same')
        # data['motor torque setpoint [Nm]'] = np.convolve(data['motor torque setpoint [Nm]'],\
        #     np.ones(averaging_num_samples)/averaging_num_samples, mode='same')
    
    # if args.filename[0:5] == "speed":
        # data['efficiency from measurement []'] = data['brake torque [Nm]'] / (gear_ratio * data['motor torque measured [Nm]'])
        # data['efficiency from setpoint []'] = data['brake torque [Nm]'] / (gear_ratio * data['motor torque setpoint [Nm]'])
    
    if args.interactive:
        print('the following data series are available:')
        num_cols = data.shape[1]
        headers = data.columns.values
        for i in range(num_cols) :
            print('\t{}:\t'.format(i) + headers[i])

        time = data[headers[0]]
        Ts = np.abs(np.mean(np.array(time[1:-1]) - np.array(time[0:-2])))
        fs = 1/Ts
        cutoff = 0.1*fs
        order = 6
        b, a = butter_lowpass(cutoff, fs, order)

        # import ipdb; ipdb.set_trace()
        # print(Ts)
        # return
        print('enter which data you would like to plot by their indices separated by pipes \'|\'')
        print('the first index will be taken as the x-axis and the rest will be plotted as separate series')
        while True:
            options = input('\t> ')
            if options[0] == 'q':
                return
            plot_strings = options.split(';')
            for plot_str in plot_strings:
                fig = plt.figure()
                ax = fig.gca()
                indices = plot_str.split('|')
                
                xidx=None
                filt_x = False

                xstr = indices[0]
                if "filter" in xstr:
                    filt_x = True
                    xstr = xstr.replace("filter",'')
                if "f" in xstr:
                    filt_x = True
                    xstr = xstr.replace("f",'')
                if not xstr.isdigit():
                    print("invalid input")
                    return
                xidx = int(xstr)
                xlabel = headers[xidx]
                xseries = data[xlabel] if not filt_x else butter_lowpass_filter(data[xlabel], cutoff, fs, order)

                for index in indices[1:]:
                    sidx = None
                    filt_s = False
                    sstr = index
                    if "filter" in sstr:
                        filt_s = True
                        sstr = sstr.replace("filter",'')
                    if "f" in sstr:
                        filt_s = True
                        sstr = sstr.replace("f",'')

                    scatter = False
                    if "scatter" in sstr:
                        scatter = True
                        sstr = sstr.replace("scatter",'')
                    if "s" in sstr:
                        scatter = True
                        sstr = sstr.replace("s",'')

                    if not sstr.isdigit():
                        print("invalid input")
                        return
                    sidx = int(sstr)
                    label = headers[sidx]
                    series = data[label] if not filt_s else butter_lowpass_filter(data[label], cutoff, fs, order)
                    
                    ratio = 1
                    if label.find("torque") > -1 and (label.find("c1") > -1 or label.find("c2") > -1):
                        ratio = gear_ratio
                    if filt_s: label += ", filtered"
                    if scatter: ax.scatter(xseries, ratio*series, label=label, s=1)
                    else: ax.plot(xseries, ratio*series, label=label)
                if filt_x: xlabel += ", filtered"
                plt.xlabel(xlabel)
                plt.legend()
                plt.title(args.filename)
            plt.show()
        return

    # plt.subplot(111)
    # plt.plot(data['time [s]'], data['motor torque measured [Nm]'], label='Motor Torque [Nm]')
    # plt.plot(data['time [s]'], data['brake torque [Nm]'], label='Brake Torque [Nm]')
    # plt.xlabel('time [s]')
    # plt.legend()
    # plt.title(args.filename)


    # brake = data['brake torque [Nm]']
    # motor = data['motor torque measured [Nm]']
    # print('average brake torque = {}'.format(np.mean(brake[abs(brake) < 1.0])))
    # print('average motor torque = {}'.format(np.mean(motor)))

    # plt.show()

if __name__ == '__main__' :
    main()