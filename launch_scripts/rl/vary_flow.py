#!/usr/bin/env python

import copy
import json
import os
import sys

def main(argv):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('config', type=str, help='Input config. Must have TaylorGreen flow properties.')
    parser.add_argument('--out',  type=str, default="config_scaled.json")
    parser.add_argument('--magnitude-factor', type=float, default=0, help='scale for the magnitude of the field.')
    parser.add_argument('--invPeriod-factor', type=float, default=1, help='scale for the inverse period of the field.')
    parser.add_argument('--no-dump', action='store_true', default=False, help='Set "dumpEvery" to 0.')
    args = parser.parse_args()

    with open(args.config, "r") as f:
        cfg = json.load(f)

    expected = "FieldTaylorGreenVortex"
    actual = cfg["velocityField"]["__type"]
    if actual != expected:
        raise RuntimeError(f"wrong velocity field in config. expected {expected}, got {actual}")

    v0 = copy.deepcopy(cfg["velocityField"])
    vpert = copy.deepcopy(v0)

    vpert["magnitude"] = [u * args.magnitude_factor for u in v0["magnitude"]]
    vpert["invPeriod"] = [u * args.invPeriod_factor for u in v0["invPeriod"]]

    cfg["velocityField"] = {"__type": "Sum",
                            "fields": [v0, vpert]}

    if args.no_dump:
        cfg["dumpEvery"] = 0

    with open(args.out, "w") as f:
        cfg = json.dump(cfg, f, indent=4)



if __name__ == '__main__':
    main(sys.argv[1:])
