#! /usr/bin/env python

import json
import os
import numpy as np


def create_swimmer_cfg(*,
                       B: float,
                       wc: float,
                       Vmax: float,
                       m: float):
    cfg = dict()
    Tmax = m * B

    Bxx = Vmax / Vmax
    Cxx = wc / Vmax
    Cyy = 6 * Cxx # 6 times more difficult to steer (see DPD)
    Czz = Cyy

    cfg = {
        "moment": [0.0, m, 0.0],
        "quaternion": [0.0, 1.0, 0.0, 0.0],
	"position": [0.0, 0.0, 0.0],
        "propulsion": {
	    "A": [0.25, 0.2, 0.2], # arbitrary, will not be used.
	    "B": [Bxx, 0.0, 0.0],
	    "C": [Cxx, Cyy, Czz]
	}
        }
    return cfg


def main(argv: list):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('N', type=int, help='the number of swimmers')
    parser.add_argument('out', type=str, help='output json file')
    parser.add_argument('--B', type=float, default=1, help='magnetic field magnitude')
    parser.add_argument('--Vmax', type=float, default=1, help='max velocity of the swimmers')
    parser.add_argument('--m', type=float, default=1, help='magnetic moment of the swimmers')
    parser.add_argument('--L', type=float, default=50, help='domain size')
    args = parser.parse_args(argv)

    N = args.N
    B = args.B
    Vmax = args.Vmax
    m = args.m
    L = args.L

    wcs = np.linspace(1.0, N, N)
    x0 = np.random.uniform(low=-L/2, high=L/2, size=(N,3))

    cfg = dict()

    cfg['bodies'] = [create_swimmer_cfg(B=B, wc=wc, Vmax=Vmax, m=m) for wc in wcs]
    cfg['posIc'] = {
	"__type" : "Const",
	"positions" : x0.tolist()
    }

    cfg['fieldMagnitude'] = B
    cfg['velocityField'] = {
	"__type" : "None"
    }
    cfg['dumpEvery'] = 100

    filename = args.out
    with open(filename, "w") as outfile:
        json.dump(cfg, outfile, indent=4)



if __name__ == '__main__':
    import sys
    main(sys.argv[1:])
