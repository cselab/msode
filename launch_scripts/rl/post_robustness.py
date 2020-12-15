#! /usr/bin/env python3

import argparse
import numpy as np
import pandas as pd
import re
import sys

def read_file(fname: str):
    data = np.loadtxt(fname, skiprows=3)
    episode = data[:,0]
    tac = data[:,1]
    trl = data[:,2]
    return episode, tac, trl

def get_scales(fname: str):
    m = re.findall("p_(.+)_s_(.+).dat", fname)[0]
    p = float(m[0])
    s = float(m[1])
    return p, s

def get_success_number(trl):
    tmax = np.max(trl)
    idx = np.argwhere(trl < tmax)
    k = len(idx)
    n = len(trl)
    return k, n


def post(files: list):

    data = dict()

    for fname in files:
        _, _, trl = read_file(fname)

        k, n = get_success_number(trl)
        p, s = get_scales(fname)

        if not p in data.keys():
            data[p] = {"k": list(),
                       "n": list(),
                       "s": list()}

        data[p]["k"].append(k)
        data[p]["n"].append(n)
        data[p]["s"].append(s)

    for p, val in data.items():
        #print(p, val)

        df = pd.DataFrame(val)
        df.to_csv(f"p_{p}.csv", index=False)



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('files',  type=str, nargs='+', help='Comparison output of ODE simulation.')
    args = parser.parse_args()

    post(args.files)
