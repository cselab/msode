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

xG=[0,0,0]

max_swimmers = 32
colors = ['C'+str(i%10) for i in range(max_swimmers)]

def plot_trajectory(i, ax, x, y, z):
    # shadows
    x0 = -np.ones(len(x)) * args.L
    y0 = x0
    z0 = x0
    shade_color='0.75'
    ax.plot([xG[0], xG[0], x0[0],   x[0], x[0], x0[0]],
            [xG[1], y0[0], xG[1],   y[0], y0[0], y[0]],
            [z0[2], xG[2], xG[2],   z0[0], z[0], z[0]],
            'o', color = shade_color)
    
    ax.plot(x, y, z0, '-', color = shade_color)
    ax.plot(x, y0, z, '-', color = shade_color)
    ax.plot(x0, y, z, '-', color = shade_color)

    ax.plot(x, y, z, '-' + colors[i])
    ax.plot([x[0]], [y[0]], [z[0]], 'o' + colors[i])
    #ax.plot([x[-1]], [y[-1]], [z[-1]], 'or')


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
ax.view_init(elev=20., azim=45)

cmaps = [plt.cm.viridis, plt.cm.jet]

for i in range(nrigids):
    start = 5 + i * ncolumnds_per_rigid
    end = start + ncolumnds_per_rigid
    q, pos, omega = read_rigid_data(data[:, start:end]) 

    if args.multi_color:
        plot_trajectory_time_colored(ax, t, pos[:,0], pos[:,1], pos[:,2], cmaps[i%len(cmaps)])
    else:
        plot_trajectory(i, ax, pos[:,0], pos[:,1], pos[:,2])

ax.plot([xG[0]], [xG[1]], [xG[2]], 'or')

ax.set_xlabel(r'$x$')
ax.set_ylabel(r'$y$')
ax.set_zlabel(r'$z$')

ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_zticklabels([])

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

