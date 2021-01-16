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
        help="outlier rejection: ignore rows in the csv for which brake torque data are >3 sigma away",
        action='store_true')

    args = parser.parse_args()
    
    data = pd.read_csv(args.filename, comment='#', header=0)

    import ipdb; ipdb.set_trace()
    # if args.outlier:
    #     data = data[(np.abs(stats.zscore(data['brake torque [Nm]'])) < 3)]

    if args.interactive:
        print('the following data are available:')
        num_cols = data.shape[1]
        headers = data.columns.values
        for i in range(num_cols) :
            print('\t{}: '.format(i) + headers[i])
        print('enter which data you would like to plot by their indices separated by commas')
        options = input('the first index will be taken as the x-axis and the rest will be plotted as separate series\n')
        plot_strings = options.split(';')
        for plot_str in plot_strings:
            fig = plt.figure()
            ax = fig.gca()
            indices = plot_str.split(',')
            xlabel = headers[int(indices[0])]
            xseries = data[headers[xlabel]]
            for index in indices[1:]:
                label = headers[int(index)]
                series = data[label]
                ax.plot(xseries, series, label=label)
            ax.xlabel(xlabel)
            ax.legend()
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