#!/usr/bin/env python

import korali
import matplotlib.pyplot as plt
import numpy as np
import os
import pandas as pd
import sys

sys.path.insert(0, os.path.join('..', '1D'))
from model import ABFs

def compute_V(bmb: float,
              cmb: float,
              omega: float,
              tend: float=50):

    x0 = [0]
    abf = ABFs(x0, [bmb], [cmb])
    abf.advance(omega, tend)
    V = abf.x[0] / tend
    return V

def plot_comparison(fname: str,
                    bmb: float,
                    cmb: float):
    df = pd.read_csv(fname)
    omegasexp = df['omega'].to_numpy()
    Vexp = df['V'].to_numpy()

    omegas = np.linspace(0, 1.2*max(omegasexp), 100)
    V = [compute_V(bmb, cmb, w) for w in omegas]

    fig, ax = plt.subplots()

    ax.plot(omegasexp, Vexp, '+')
    ax.plot(omegas, V, '-')
    ax.set_xlim(left=0)

    ax.set_xlabel(r'$\omega [Hz]$')
    ax.set_xlabel(r'$V [\mu m/s]$')

    plt.show()


def infer_TMCMC(fname: str):
    df = pd.read_csv(fname)
    omegas = df['omega'].to_numpy()
    Vexp = df['V'].to_numpy()

    def model(sample):
        bmb, cmb, sig = sample["Parameters"]
        sample["Reference Evaluations"] = [compute_V(bmb, cmb, w) for w in omegas]
        sample["Standard Deviation"] = [sig] * len(omegas)

    e = korali.Experiment()

    # Setting up the reference likelihood for the Bayesian Problem
    e["Problem"]["Type"] = "Bayesian/Reference"
    e["Problem"]["Likelihood Model"] = "Normal"
    e["Problem"]["Reference Data"] = Vexp.tolist()
    e["Problem"]["Computational Model"] = model

    # Configuring TMCMC parameters
    e["Solver"]["Type"] = "Sampler/TMCMC"
    e["Solver"]["Population Size"] = 1000

    # Configuring the problem's random distributions
    e["Distributions"][0]["Name"] = "Uniform 0"
    e["Distributions"][0]["Type"] = "Univariate/Uniform"
    e["Distributions"][0]["Minimum"] = 5.0
    e["Distributions"][0]["Maximum"] = 15.0

    e["Distributions"][1]["Name"] = "Uniform 1"
    e["Distributions"][1]["Type"] = "Univariate/Uniform"
    e["Distributions"][1]["Minimum"] = 5.0
    e["Distributions"][1]["Maximum"] = 15.0

    e["Distributions"][2]["Name"] = "Uniform 2"
    e["Distributions"][2]["Type"] = "Univariate/Uniform"
    e["Distributions"][2]["Minimum"] = 0.0
    e["Distributions"][2]["Maximum"] = +5.0

    # Configuring the problem's variables and their prior distributions
    e["Variables"][0]["Name"] = "BmB"
    e["Variables"][0]["Prior Distribution"] = "Uniform 0"

    e["Variables"][1]["Name"] = "CmB"
    e["Variables"][1]["Prior Distribution"] = "Uniform 1"

    e["Variables"][2]["Name"] = "[Sigma]"
    e["Variables"][2]["Prior Distribution"] = "Uniform 2"

    # Starting Korali's Engine and running experiment
    #e["Console Output"]["Verbosity"] = "Detailed"
    k = korali.Engine()
    k.run(e)



if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=str, default='data/V.csv', help="The experimental data.")
    args = parser.parse_args()

    #infer_TMCMC(args.data)
    plot_comparison(args.data, bmb=9.1, cmb=9.1)
