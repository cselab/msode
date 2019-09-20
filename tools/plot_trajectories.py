#! /usr/bin/env python3

import numpy as np
import argparse
from utils import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

parser = argparse.ArgumentParser()
parser.add_argument('--file', type=str, required=True, help='output of ODE simulation')
parser.add_argument('--out', type=str, default="GUI")
parser.add_argument('--multi_color', action='store_true', default=False)
parser.add_argument('--L', type=float, default=50, help='initial half box length')
args = parser.parse_args()

def plot_trajectory(ax, x, y, z):
    ax.plot(x, y, z, '-')
    ax.plot([x[0]], [y[0]], [z[0]], 'og')
    ax.plot([x[-1]], [y[-1]], [z[-1]], 'or')

def plot_trajectory_time_colored(ax, t, x, y, z, cmap):
    # ax.scatter(x, y, z, c = plt.cm.jet(t))

    n = len(x)
    for i in range(n-1):
        ax.plot(x[i:i+2], y[i:i+2], z[i:i+2], color = cmap(i/(n-1)))

data = remove_next_sim_data(np.loadtxt(args.file))

ncolumnds_per_rigid = 4 + 3 + 3 # q, r, w

ncolumns = len(data[0,:])
nrigids = (ncolumns-1) // ncolumnds_per_rigid
t = data[:,0]

fig = plt.figure(0)
ax = fig.add_subplot(111, projection='3d')

cmaps = [plt.cm.viridis, plt.cm.jet]

for i in range(nrigids):
    start = 5 + i * ncolumnds_per_rigid
    end = start + ncolumnds_per_rigid
    q, pos, omega = read_rigid_data(data[:, start:end]) 

    if args.multi_color:
        plot_trajectory_time_colored(ax, t, pos[:,0], pos[:,1], pos[:,2], cmaps[i%len(cmaps)])
    else:
        plot_trajectory(ax, pos[:,0], pos[:,1], pos[:,2])
    

ax.set_xlabel(r'$x$')
ax.set_ylabel(r'$y$')
ax.set_zlabel(r'$z$')

L = args.L
lims = [-L, L]
ax.set_xlim(lims)
ax.set_ylim(lims)
ax.set_zlim(lims)

plt.grid()


if args.out == "GUI":
    plt.show()
else:
    plt.savefig(args.out, transparent=True)

