#! /usr/bin/env python3

import numpy as np
import argparse, os
from utils import *
import trimesh
import transforms3d

parser = argparse.ArgumentParser()
parser.add_argument('--file', type=str, required=True, help='output of ODE simulation')
parser.add_argument('--mesh', type=str, nargs='+', required=True, help="input triangle mesh of the swimmers")
parser.add_argument('--out_folder', type=str, default="ply", help="output folder mesh")
parser.add_argument('--scale', type=float, default=0.5, help="length scaling")
args = parser.parse_args()


data = remove_next_sim_data(np.loadtxt(args.file))

ncolumnds_per_rigid = 4 + 3 + 3 # q, r, w

ncolumns = len(data[0,:])
nrigids = (ncolumns-1) // ncolumnds_per_rigid
assert(nrigids == len(args.mesh))
t = data[:,0]

if not os.path.exists(args.out_folder):
    os.mkdir(args.out_folder)

for objid in range(nrigids):
    start = 1 + objid * ncolumnds_per_rigid
    end = start + ncolumnds_per_rigid
    quaternions, positions, omegas = read_rigid_data(data[:, start:end])
    orig_mesh = trimesh.load_mesh(args.mesh[objid])
    orig_mesh.vertices *= args.scale

    for i in range(len(t)):
        q = quaternions[i,:]
        x = positions[i,:]
        R = transforms3d.quaternions.quat2mat(q)
        mesh = orig_mesh.copy()
        mesh.vertices = [x + transforms3d.quaternions.rotate_vector(v, q) for v in mesh.vertices]
        mesh_name = args.out_folder + "/swimmer_{:02d}_{:05d}.ply".format(objid, i)
        mesh.export(mesh_name)
