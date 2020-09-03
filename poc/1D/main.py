#!/usr/bin/env python

import argparse
import numpy as np
from scipy.integrate import odeint

import matplotlib.pyplot as plt

class ABFs:
    def __init__(self,
                 initial_positions: list,
                 bmb: list,
                 cmb: list):
        self.x = np.array(initial_positions) # positions of the ABFs
        self.thetas = np.zeros_like(self.x) # orientations of the ABFs
        self.cmb = np.array(cmb) # C_ * m * B
        self.bmb = np.array(bmb) # B_ * m * B
        self.n = len(self.x)

        self.theta_B = 0 # orientation of the magnetic field

        if len(self.cmb) != self.n or len(self.bmb) != self.n:
            raise RuntimeError("x and coefficients must have the same size.")

    def advance(self, w: float, dt: float):
        n = self.n

        def rhs(y, t):
            # xs = y[:n] # Not used
            thetas = y[n:-1]
            theta_B = y[-1]

            dydt = np.zeros_like(y)

            T = np.sin(thetas - theta_B)

            dydt[0:n ] = self.bmb * T
            dydt[n:-1] = self.cmb * T
            dydt[-1  ] = w
            return dydt

        y0 = np.zeros(2 * n + 1) # x, thetas, theta_B
        y0[:n  ] = self.x
        y0[n:-1] = self.thetas
        y0[-1  ] = self.theta_B

        t = [0, dt]
        sol = odeint(rhs, y0, t)

        self.x       = sol[1, :n]
        self.thetas  = sol[1, n:-1]
        self.theta_B = sol[1, -1]


if __name__ == '__main__':
    # parser = argparse.ArgumentParser()
    # parser.add_argument('--kpow', type=float, default=0.125, help="Disspative kernel exponent.")
    # args = parser.parse_args()

    bmb = [1, 1]
    cmb = [1, 2]
    x0 = [0, 0]

    abfs = ABFs(x0, bmb, cmb)

    nsteps = 1000

    dt = 0.1

    x = np.zeros((len(x0), nsteps+1))
    x[:,0] = x0
    t = np.arange(nsteps+1) * dt

    for i in range(nsteps):
        w = 10

        abfs.advance(w, dt)
        x[:,i+1] = abfs.x

    fig, ax = plt.subplots()
    for j in range(len(x0)):
        ax.plot(t, x[j,:])
    plt.show()
