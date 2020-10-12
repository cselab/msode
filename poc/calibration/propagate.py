#!/usr/bin/env python

import json
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from model import compute_V
from scipy.stats import norm

def get_samples(fname: str):
    with open(fname) as json_file:
        e = json.load(json_file)
        samples = np.array(e["Results"]["Sample Database"])

    return samples

def propagate(fname: str,
              samples: np.ndarray,
              out: str):

    df = pd.read_csv(fname)
    wexp = df['omega'].to_numpy()
    Vexp = df['V'].to_numpy()

    nws = 200
    nsamples = samples.shape[0]
    ws = np.linspace(0, 1.2*np.max(wexp), nws)

    all_Vs = np.zeros((nsamples, nws))
    all_sigmas = np.zeros(nsamples)

    for i, sample in enumerate(samples):
        bmb, cmb, sigma = sample
        all_Vs[i,:] = [compute_V(bmb, cmb, w) for w in ws]
        all_sigmas = sigma


    # compute confidence intervals:
    # discretize the axis, compute CDF and invert at percentiles
    y = np.linspace(0, 1.5 * np.max(Vexp), 5000)

    cdf = np.sum( [norm.cdf(y_, loc=all_Vs, scale=all_sigmas) for y_ in y], axis=1 ) / nsamples

    Vlo = y[np.argmin(np.abs(cdf - 0.05), axis=0)]
    Vhi = y[np.argmin(np.abs(cdf - 0.95), axis=0)]

    Vmean = np.mean(all_Vs, axis=0)

    fig, ax = plt.subplots()

    ax.plot(ws, Vmean, '-', label='ODE model')
    ax.fill_between(ws, Vlo, Vhi, alpha=0.5, lw=0)
    ax.plot(wexp, Vexp, 'ok', label='Mhanna 2014')

    ax.set_xlim(0, np.max(ws))
    ax.set_ylim(0)

    ax.set_xlabel(r'$\omega \; [Hz]$')
    ax.set_ylabel(r'$V \; [\mu m / s]$')

    plt.legend()

    if out == 'GUI':
        plt.show()
    else:
        plt.savefig(out, transparent=True)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=str, default='data/V.csv', help="The experimental data.")
    parser.add_argument('--korali-res', type=str, default='_korali_result/latest', help="json file that contains the results of the inference script.")
    parser.add_argument('--out', type=str, default='GUI', help="Output name for the figure.")
    args = parser.parse_args()

    samples = get_samples(args.korali_res)
    propagate(args.data, samples[:200,:], args.out)
