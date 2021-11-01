#! /usr/bin/env python3

import argparse
import numpy as np
import pandas as pd
import sys

def read_file(fname: str):
    data = np.loadtxt(fname, skiprows=3)
    episode = data[:,0]
    tac = data[:,1]
    trl = data[:,2]
    return episode, tac, trl

def get_kBT(fname: str):
    kBT = float(fname.split('kBT_')[-1].split('.dat')[0])
    return kBT

def get_success_number(trl):
    tmax = np.max(trl)
    idx = np.argwhere(trl < tmax)
    k = len(idx)
    n = len(trl)
    return k, n


def post(files: list):

    all_num_success = []
    all_num_total = []
    all_kBT = []

    for fname in files:
        _, _, trl = read_file(fname)

        num_success, num_total = get_success_number(trl)
        kBT = get_kBT(fname)

        all_num_success.append(num_success)
        all_num_total.append(num_total)
        all_kBT.append(kBT)

    df = pd.DataFrame({"kBT": all_kBT,
                       "num_success": all_num_success,
                       "num_total": all_num_total})
    df.sort_values(by="kBT", inplace=True)
    df.to_csv(f"thermal_noise_success.csv", index=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('files',  type=str, nargs='+', help='Comparison output of ODE simulation.')
    args = parser.parse_args()

    post(args.files)
