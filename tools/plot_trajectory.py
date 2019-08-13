#! /usr/bin/env python3

import numpy as np
import argparse
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

parser = argparse.ArgumentParser()
parser.add_argument('--file', type=str, required=True, help='output of ODE simulation')
parser.add_argument('--out', type=str, default="GUI")
args = parser.parse_args()

data = np.loadtxt(args.file)

t     = data[:,0]
q     = data[:,1:5]
pos   = data[:,5:8]
omega = data[:,8:11]

fig = plt.figure(0)
ax = fig.add_subplot(111, projection='3d')

ax.plot(pos[:,0], pos[:,1], pos[:,2], '-k')

ax.set_xlabel(r'$x$')
ax.set_ylabel(r'$y$')
ax.set_zlabel(r'$z$')
plt.grid()

if args.out == "GUI":
    plt.show()
else:
    plt.savefig(args.out, transparent=True)

