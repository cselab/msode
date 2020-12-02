#! /usr/bin/env python3

import numpy as np
import os, argparse, json

parser = argparse.ArgumentParser()
parser.add_argument('--props', type=str, nargs='+', required=True, help='the file of propulsion given by the SDPD simulations (see msode/data/propulsion/)')
parser.add_argument('--out_dir', type=str, default=".", help='directory where to dump the configuration files required by msode')

parser.add_argument('--l_scale', type=float, default=20, help='length scale: 1 sim unit = this in um')
parser.add_argument('--B_target', type=float, default=0.1, help='B_xx in simulation unit')
parser.add_argument('--vmax', type=float, default=1, help='maximum velocity of all swimmers(assumed the same), in body length per second')
parser.add_argument('--magnetic_field_magnitude', type=float, default=1, help='magnetic field magnitude in simulation unit')
args = parser.parse_args()

def read_propulsion(fname):
    f = open(fname, "r")
    prop = {}

    for line in f.readlines():
        keys = line[:-1].split(' ') # [:-1] because of '\n'
        prop[keys[0]] = [float(val) for val in keys[1:]]

    f.close()
    return prop


def rescale_propulsion_length(prop, lscale):
    prop['B'] = [b*lscale    for b in prop['B']]
    prop['C'] = [c*lscale**2 for c in prop['C']]
    return prop

def compute_magn_moment(prop):
    return args.vmax / (args.magnetic_field_magnitude * prop['B'][0])



props = [read_propulsion(fname) for fname in args.props]
n = len(props)

props = [rescale_propulsion_length(prop, args.l_scale) for prop in props]
#print(props)

scaling = args.B_target / props[0]['B'][0]

for i in range(n):
    for key in props[i]:
        props[i][key] = [scaling * val for val in props[i][key]]

magn_moments = [compute_magn_moment(prop) for prop in props]

for i in range(n):
    config = {}
    config["moment"]     = [0.0, magn_moments[i], 0.0]
    config["quaternion"] = [0.0, 1.0, 0.0, 0.0]
    config["position"]   = [0.0, 0.0, 0.0]
    config["propulsion"] = props[i]

    filename = args.out_dir + 'swimmer_%02d.json'%(i)

    with open(filename, "w") as outfile:
        json.dump(config, outfile, indent=4)

    omegac = args.magnetic_field_magnitude * magn_moments[i] * props[i]['C'][0]
    print("omega_c = %f" % omegac)
