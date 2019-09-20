#! /usr/bin/env python3

import numpy as np
import argparse
from utils import *
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('--files', type=str, nargs='+', required=True, help='output of several ODE simulations')
parser.add_argument('--out', type=str, default="GUI")
args = parser.parse_args()

all_data = None

for f in args.files:
    data = remove_next_sim_data(np.loadtxt(f))
    if all_data is None:
        all_data = data
    else:
        all_data = np.concatenate((all_data, data), axis=0)

omega = all_data[:,1]

density, bin_edges = np.histogram(omega, bins=100, density=True)

fig = plt.figure(0)
ax = fig.add_subplot(111)

bin_centers = 0.5 * (bin_edges[:-1] + bin_edges[1:])

ax.plot(bin_centers, density, '-')

ax.set_xlabel(r'$\omega$')
ax.set_ylabel(r'$density$')

if args.out == "GUI":
    plt.show()
else:
    plt.savefig(args.out, transparent=True)

