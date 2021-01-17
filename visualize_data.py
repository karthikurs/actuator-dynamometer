#!/usr/bin/env python3

import argparse
import pandas as pd
import numpy as np
import matplotlib as mpl
from matplotlib import pyplot as plt
from scipy import stats


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

    if args.outlier is not None:
        # import ipdb; ipdb.set_trace()
        data = data[data['brake torque [Nm]']\
            .between(\
                data['brake torque [Nm]'].quantile(args.outlier),\
                data['brake torque [Nm]'].quantile(1-args.outlier)\
                )\
            ]
    gear_ratio = 1
    if args.gear is not None:
        gear_ratio = args.gear

    averaging_num_samples = 1
    if args.averaging is not None:
        averaging_num_samples = args.averaging
        data['brake torque [Nm]'] = np.convolve(data['brake torque [Nm]'],\
            np.ones(averaging_num_samples)/averaging_num_samples, mode='same')
        data['motor torque measured [Nm]'] = np.convolve(data['motor torque measured [Nm]'],\
            np.ones(averaging_num_samples)/averaging_num_samples, mode='same')
        data['motor torque setpoint [Nm]'] = np.convolve(data['motor torque setpoint [Nm]'],\
            np.ones(averaging_num_samples)/averaging_num_samples, mode='same')
    
    data['efficiency from measurement []'] = data['brake torque [Nm]'] / (gear_ratio * data['motor torque measured [Nm]'])
    data['efficiency from setpoint []'] = data['brake torque [Nm]'] / (gear_ratio * data['motor torque setpoint [Nm]'])
    
    if args.interactive:
        print('the following data series are available:')
        num_cols = data.shape[1]
        headers = data.columns.values
        for i in range(num_cols) :
            print('\t{}: '.format(i) + headers[i])
        print('enter which data you would like to plot by their indices separated by commas')
        print('the first index will be taken as the x-axis and the rest will be plotted as separate series')
        while True:
            options = input('\t> ')
            plot_strings = options.split(';')
            for plot_str in plot_strings:
                fig = plt.figure()
                ax = fig.gca()
                indices = plot_str.split(',')
                xlabel = headers[int(indices[0])]
                # import ipdb; ipdb.set_trace()
                xseries = data[xlabel]
                for index in indices[1:]:
                    label = headers[int(index)]
                    series = data[label]
                    ax.plot(xseries, series, label=label)
                plt.xlabel(xlabel)
                plt.legend()
                plt.title(args.filename)
            plt.show()
        return

    plt.subplot(111)
    plt.plot(data['time [s]'], data['motor torque [Nm]'], label='Motor Torque [Nm]')
    plt.plot(data['time [s]'], data['brake torque [Nm]'], label='Brake Torque [Nm]')
    plt.xlabel('time [s]')
    plt.legend()
    plt.title(args.filename)


    brake = data['brake torque [Nm]']
    motor = data['motor torque [Nm]']
    print('average brake torque = {}'.format(np.mean(brake[abs(brake) < 1.0])))
    print('average motor torque = {}'.format(np.mean(motor)))

    plt.show()

if __name__ == '__main__' :
    main()