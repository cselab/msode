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

def get_diff(fname: str):
    m = re.findall("Dt_(.+)_Dr_(.+).dat", fname)[0]
    Dt = float(m[0])
    Dr = float(m[1])
    return Dt, Dr

def get_success_number(trl):
    tmax = np.max(trl)
    idx = np.argwhere(trl < tmax)
    k = len(idx)
    n = len(trl)
    return k, n


def post(files: list):

    all_num_success = []
    all_num_total = []
    all_Dt = []
    all_Dr = []

    for fname in files:
        _, _, trl = read_file(fname)

        num_success, num_total = get_success_number(trl)
        Dt, Dr = get_diff(fname)

        all_num_success.append(num_success)
        all_num_total.append(num_total)
        all_Dt.append(Dt)
        all_Dr.append(Dr)

    df = pd.DataFrame({"Dt": all_Dt,
                       "Dr": all_Dr,
                       "num_success": all_num_success,
                       "num_total": all_num_total})
    df.sort_values(by="Dr", inplace=True)
    df.to_csv(f"thermal_noise_success.csv", index=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('files',  type=str, nargs='+', help='Comparison output of ODE simulation.')
    args = parser.parse_args()

    post(args.files)
