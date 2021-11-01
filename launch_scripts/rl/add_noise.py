#!/usr/bin/env python

import copy
import json
import os
import sys

def main(argv):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('config', type=str, help='Input config. Must have TaylorGreen flow properties.')
    parser.add_argument('--out',  type=str, default="config_noise.json")
    parser.add_argument('--kBT', type=float, default=0, help='Temperature, energy units.')
    parser.add_argument('--no-dump', action='store_true', default=False, help='Set "dumpEvery" to 0.')
    args = parser.parse_args()

    with open(args.config, "r") as f:
        cfg = json.load(f)

    cfg["kBT"] = args.kBT

    if args.no_dump:
        cfg["dumpEvery"] = 0

    with open(args.out, "w") as f:
        cfg = json.dump(cfg, f, indent=4)



if __name__ == '__main__':
    main(sys.argv[1:])
