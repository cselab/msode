#! /usr/bin/env python

import argparse
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def plot(fname: str):
    df = pd.read_csv(fname)
    Dt = df["Dt"].to_numpy()
    Dr = df["Dr"].to_numpy()
    p = df["num_success"].to_numpy() / df["num_total"].to_numpy()

    fig, ax = plt.subplots()
    ax.scatter(Dr, Dt, c=p)
    ax.set_xscale('log')
    ax.set_yscale('log')
    ax.set_xlim(1e-2, 1)
    ax.set_ylim(1e-2, 1)

    ax.set_xlabel(r"$D_r$")
    ax.set_ylabel(r"$D_t$")
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('csv_file',  type=str, help='Success rate file.')
    args = parser.parse_args()

    plot(args.csv_file)
