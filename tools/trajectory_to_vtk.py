#! /usr/bin/env python3

import numpy as np
import argparse

def read_rigid_data(data):
    q = data[:,0:4]
    r = data[:,4:7]
    w = data[:,7:10]
    return q, r, w


def write_point_data_scalar(f, point_data, name):
    f.write(f"SCALARS {name} FLOAT\n")
    f.write("LOOKUP_TABLE default\n")
    for val in point_data:
        f.write(f"{val}\n")

def write_point_data_vector(f, point_data, name):
    f.write(f"VECTORS {name} FLOAT\n")
    for val in point_data:
        f.write(f"{val[0]} {val[1]} {val[2]}\n")

def main(argv):
    parser = argparse.ArgumentParser(description="Create a vtk file from the trajectory files output by an msode simulation")
    parser.add_argument('file',         type=str, help='output of ODE simulation')
    parser.add_argument('out_basename', type=str, help='vtk file output base name, will be basename.<swimmer id>.vtk')
    parser.add_argument('--with_field_from', type=str, default=None, metavar='config.json',
                        help='if set, the background velocity field will be evaluated at every point of the trajectory. \
                        The velocity field is described in the json file passed in this argument.')
    args = parser.parse_args(argv)

    data = np.loadtxt(args.file)

    ncolumnds_per_rigid = 4 + 3 + 3 # q, r, w
    ncolumns = len(data[0,:])
    nrigids = (ncolumns-1) // ncolumnds_per_rigid

    time = data[:,0]
    omega_field = data[:,1]

    start_base = 5 # time, omega direction, omega magnitude

    for i in range(nrigids):
        start = start_base + i * ncolumnds_per_rigid
        end   = start + ncolumnds_per_rigid
        q, pos, omega = read_rigid_data(data[:, start:end])

        x = pos[:,0]
        y = pos[:,1]
        z = pos[:,2]
        n = len(x)

        filename = args.out_basename + ".{:04d}.vtk".format(i)

        with open(filename, 'w') as f:
            f.write("# vtk DataFile Version 2.0\n")
            f.write("generated from msode trajectories\n")
            f.write("ASCII\n")
            f.write("DATASET POLYDATA\n")
            f.write(f"POINTS {n} FLOAT\n")

            for j in range(n):
                f.write(f"{x[j]} {y[j]} {z[j]}\n")

            nl = n-1
            f.write(f"LINES {nl} {3*nl}\n")

            for j in range(n-1):
                f.write(f"2 {j} {j+1}\n")

            f.write(f"\nPOINT_DATA {n}\n")

            write_point_data_scalar(f, time, "time")
            write_point_data_scalar(f, np.abs(omega_field), "omega")

            if args.with_field_from is not None:
                import velocity_field as vf
                field_vel = vf.evaluate_field_from_config_filename(x, y, z, args.with_field_from)
                write_point_data_vector(f, field_vel, "velocity_field")

if __name__ == '__main__':
    import sys
    main(sys.argv[1:])
