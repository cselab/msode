#! /usr/bin/env python3

import numpy as np
import argparse
from utils import *
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('file',   type=str, help='Output of ODE simulation.')
parser.add_argument('--out',  type=str, default="GUI")
parser.add_argument('--L',    type=float, default=40, help='initial half box length')
args = parser.parse_args()

data = remove_next_sim_data(np.loadtxt(args.file))

ncolumnds_per_rigid = 4 + 3 + 3 # q, r, w

ncolumns = len(data[0,:])
nrigids = (ncolumns-1) // ncolumnds_per_rigid
t = data[:,0]
w = data[:,1]
ex = data[:,2]
ey = data[:,3]
ez = data[:,4]

targets_lo = np.ones_like(t) * -2
targets_hi = np.ones_like(t) *  2

fig, axes = plt.subplots(2,2, sharex='col')

for ax in axes.flatten()[:-1]:
    ax.fill_between(t, targets_lo, targets_hi, color='grey', alpha=0.2)
    ax.set_ylim(-args.L, args.L)

for i in range(nrigids):
    start = 5 + i * ncolumnds_per_rigid
    end = start + ncolumnds_per_rigid
    q, pos, omega = read_rigid_data(data[:, start:end])

    for i, ax in enumerate(axes.flatten()[:-1]):
        ax.plot(t, pos[:,i], '-')

axes[1,1].plot(t, w * ex, '--')
axes[1,1].plot(t, w * ey, '--')
axes[1,1].plot(t, w * ez, '--')


axes[0,0].set_ylabel(r'$x$')

axes[0,1].set_ylabel(r'$y$')
axes[0,1].yaxis.tick_right()
axes[0,1].yaxis.set_label_position("right")

axes[1,0].set_xlabel(r'$t$')
axes[1,0].set_ylabel(r'$z$')

axes[1,1].set_xlabel(r'$t$')
axes[1,1].set_ylabel(r'$\omega$')
axes[1,1].yaxis.tick_right()
axes[1,1].yaxis.set_label_position("right")

for ax in axes.flatten():
    ax.grid()
    ax.set_xlim(t[0], t[-1])

plt.tight_layout()

if args.out == "GUI":
    plt.show()
else:
    plt.savefig(args.out, transparent=True)
