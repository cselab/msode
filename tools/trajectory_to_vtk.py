#! /usr/bin/env python3

import numpy as np
import argparse
from utils import *

parser = argparse.ArgumentParser()
parser.add_argument('--file', type=str, required=True, help='output of ODE simulation')
parser.add_argument('--out', type=str, default="GUI")
args = parser.parse_args()

data = remove_next_sim_data(np.loadtxt(args.file))

ncolumnds_per_rigid = 4 + 3 + 3 # q, r, w
ncolumns = len(data[0,:])
nrigids = (ncolumns-1) // ncolumnds_per_rigid

t = data[:,0]

for i in range(nrigids):
    start = 5 + i * ncolumnds_per_rigid
    end = start + ncolumnds_per_rigid
    q, pos, omega = read_rigid_data(data[:, start:end]) 

    x = pos[:,0]
    y = pos[:,1]
    z = pos[:,2]
    np = len(x)
    
    f = open(args.out + '.%02d.vtk' % i, 'w')
    f.write('# vtk DataFile Version 2.0\n')
    f.write('generated from msode trajectories\n')
    f.write('ASCII\n')
    f.write('DATASET POLYDATA\n')
    f.write('POINTS %d FLOAT\n' % np)

    for j in range(np):
        f.write("%f %f %f\n" % (x[j], y[j], z[j]))

    nl = np-1
    f.write('LINES %d %d\n' % (nl, 3 * nl))

    for j in range(np-1):
        f.write("2 %d %d\n" % (j, j+1))
    
    f.close()
