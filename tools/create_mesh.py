#!/usr/bin/env python

import numpy as np
import argparse, os
from utils import *
import trimesh
import transforms3d


def align_along_x(mesh):
    vv = mesh.vertices
    extents = [max(vv[:,i]) - min(vv[:,i]) for i in range(3)]
    xaxis=np.argmax(extents)
    if xaxis != 0:
        mesh.vertices[:,[0,xaxis]] = mesh.vertices[:,[xaxis,0]]
    return mesh

# assume already correctly oriented
def scale_to_length(mesh, length):
    vv = mesh.vertices
    l = max(vv[:,0]) - min(vv[:,0])
    mesh.vertices *= (length / l)
    return mesh

def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('trajectories', type=str, help='output of ODE simulation')
    parser.add_argument('--mesh', type=str, nargs='+', required=True, help="input triangle mesh of the swimmers")
    parser.add_argument('--out_folder', type=str, default="ply", help="output folder mesh")
    parser.add_argument('--length', type=float, default=1.0, help="length output of the mesh (input mesh will be rescaled)")
    parser.add_argument('--flip', action='store_true', default=False, help="if need to flip the mesh (wrong screw direction)")
    parser.add_argument('--nframes', type=int, default=-1, help="number of time frames to process; -1 : process all")
    args = parser.parse_args(argv)

    data = remove_next_sim_data(np.loadtxt(args.trajectories))

    if args.nframes != -1:
        data = data[:args.nframes,:]

    ncolumnds_per_rigid = 4 + 3 + 3 # q, r, w

    ncolumns = len(data[0,:])
    nrigids = (ncolumns-5) // ncolumnds_per_rigid
    assert(nrigids == len(args.mesh))
    t = data[:,0]

    if not os.path.exists(args.out_folder):
        os.mkdir(args.out_folder)

    for objid in range(nrigids):
        start = 5 + objid * ncolumnds_per_rigid
        end = start + ncolumnds_per_rigid
        quaternions, positions, omegas = read_rigid_data(data[:, start:end])
        orig_mesh = align_along_x(trimesh.load_mesh(args.mesh[objid]))
        orig_mesh = scale_to_length(orig_mesh, args.length)

        if args.flip:
            orig_mesh.vertices[:,2] *=-1

        for i in range(len(t)):
            q = quaternions[i,:]
            x = positions[i,:]
            qinv = transforms3d.quaternions.qinverse(q)
            mesh = orig_mesh.copy()
            mesh.vertices = [x + transforms3d.quaternions.rotate_vector(v, qinv) for v in mesh.vertices]
            mesh_name = args.out_folder + "/swimmer_{:02d}_{:05d}.ply".format(objid, i)
            mesh.export(mesh_name)


if __name__ == '__main__':
    import sys
    main(sys.argv[1:])
