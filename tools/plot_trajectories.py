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

ncolumnds_per_rigid = 4 + 3 + 3 # q, r, w

ncolumns = len(data[0,:])
nrigids = (ncolumns-1) // ncolumnds_per_rigid

def readRigidData(data):
    q = data[:,0:4]
    r = data[:,4:7]
    w = data[:,7:10]
    return q, r, w

t     = data[:,0]

fig = plt.figure(0)
ax = fig.add_subplot(111, projection='3d')

for i in range(nrigids):
    start = 1 + i * ncolumnds_per_rigid
    end = start + ncolumnds_per_rigid
    q, pos, omega = readRigidData(data[:, start:end]) 

    ax.plot(pos[:,0], pos[:,1], pos[:,2], '-')

ax.set_xlabel(r'$x$')
ax.set_ylabel(r'$y$')
ax.set_zlabel(r'$z$')
plt.grid()

if args.out == "GUI":
    plt.show()
else:
    plt.savefig(args.out, transparent=True)

