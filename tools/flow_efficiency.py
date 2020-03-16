#! /usr/bin/env python3

import numpy as np
import argparse
from utils import *
import velocity_field as vf

parser = argparse.ArgumentParser(description='plot "flow usage" over time for all swimmers')
parser.add_argument('trajectory', type=str, help='output of ODE simulation')
parser.add_argument('config', type=str, default=None, metavar='config.json',
                    help='the background velocity field will be evaluated at every point of the trajectory. \
                    The velocity fielf is described in the json file passed in this argument.')
parser.add_argument('out_basename', type=str, help='output base name, will be basename.dat')
args = parser.parse_args()

data = np.loadtxt(args.trajectory)

ncolumnds_per_rigid = 4 + 3 + 3 # q, r, w

ncolumns = len(data[0,:])
nrigids = (ncolumns-1) // ncolumnds_per_rigid
t = data[:,0]
dt = t[:-1] - t[1:]

# time, eta0, eta1, ...
all_data = [t[:-1]]

for i in range(nrigids):
    start = 5 + i * ncolumnds_per_rigid
    end = start + ncolumnds_per_rigid
    q, pos, omega = read_rigid_data(data[:, start:end])
    x = pos[:,0]
    y = pos[:,1]
    z = pos[:,2]

    field_vel = vf.evaluate_field_from_config_filename(x, y, z, args.config)[:-1,:]
    displacements = pos[1:,:] - pos[:-1,:]
    velocities = displacements / np.transpose(np.vstack((dt, dt, dt)))
    eta = np.sum(field_vel * velocities, axis=1)
    all_data.append(np.sqrt(np.sum(field_vel * field_vel, axis=1)))
    all_data.append(np.sqrt(np.sum(velocities * velocities, axis=1)))
    all_data.append(eta)
    

fname = args.out_basename + ".dat"
np.savetxt(fname, np.transpose(np.vstack(all_data)))
