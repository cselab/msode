#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def plot(fname: str):
    df = pd.read_csv(fname)
    print(df)
    fig, ax = plt.subplots()

    ax.plot(df['omega'], df['V'], '+')

    plt.show()


if __name__ == '__main__':
    plot('data/V.csv')
