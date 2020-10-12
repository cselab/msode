#!/usr/bin/env python

import numpy as np
import scipy.integrate as integrate

def compute_V(bmb: float,
              cmb: float,
              omega: float):
    wc = cmb

    if omega <= wc:
        return bmb / cmb * omega
    else:
        # helper constant:
        a1 = np.sqrt(omega**2 - wc**2)
        a2 = np.arctan(wc / a1)
        period = 2 * np.pi / a1

        def integrand(t):
            theta = 2 * np.arctan(wc / omega - a1 / omega * np.tan(0.5 * a1 * t - a2))
            return np.sin(theta)

        return bmb * integrate.quad(integrand, 0, period)[0] / period
