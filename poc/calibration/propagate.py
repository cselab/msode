#!/usr/bin/env python

import json
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
from model import compute_V

def get_samples(fname: str):

    with open(fname) as json_file:
        e = json.load(json_file)
        samples = np.array(e["Results"]["Sample Database"])

    return samples


def propagate(fname: str,
              samples: np.ndarray):
    df = pd.read_csv(fname)
    wexp = df['omega'].to_numpy()
    Vexp = df['V'].to_numpy()

    nws = 50
    nsamples = samples.shape[0]
    ws = np.linspace(0, 1.2*np.max(wexp), nws)

    all_Vs = np.zeros((nsamples, nws))

    for i, sample in enumerate(samples):
        bmb, cmb, sigma = sample
        all_Vs[i,:] = [compute_V(bmb, cmb, w) for w in ws]
        all_Vs[i,:] += np.random.normal(loc=0, scale=sigma, size=nws)


    Vmean = np.mean(all_Vs, axis=0)
    Vhi = np.percentile(all_Vs, q=95, axis=0)
    Vlo = np.percentile(all_Vs, q=5, axis=0)

    fig, ax = plt.subplots()


    ax.plot(ws, Vmean, '-')
    ax.fill_between(ws, Vlo, Vhi, alpha=0.25)
    ax.plot(wexp, Vexp, '+')

    ax.set_xlim(0, np.max(ws))
    ax.set_ylim(0)

    ax.set_xlabel(r'$\omega \; [Hz]$')
    ax.set_ylabel(r'$V \; [\mu m / s]$')

    plt.show()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=str, default='data/V.csv', help="The experimental data.")
    parser.add_argument('--korali-dir', type=str, default='_korali_result/latest', help="json file that contains the results of the inference script.")
    args = parser.parse_args()

    samples = get_samples(args.korali_dir)
    propagate(args.data, samples[:200,:])
