import argparse
import pandas as pd
import numpy as np
import matplotlib as mpl
from matplotlib import pyplot as plt

def main() :
    parser = argparse.ArgumentParser(description='Plots data from csv')
    parser.add_argument("filename",\
        help="filename of csv", type=str)
    parser.add_argument("-i", "--interactive",\
        help="interactive mode: decide which data to plot of what is present in csv",
        type=float)

    args = parser.parse_args()
    
    data = pd.read_csv(args.filename, comment='#', header=0)

    plt.subplot(111)
    plt.plot(data['time [s]'], data['motor torque [Nm]'], label='Motor Torque [Nm]')
    plt.plot(data['time [s]'], data['brake torque [Nm]'], label='Brake Torque [Nm]')
    plt.xlabel('time [s]')
    plt.legend()
    plt.title(args.filename)

    plt.show()

if __name__ == '__main__' :
    main()