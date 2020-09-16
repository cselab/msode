#!/usr/bin/env python3

import numpy as np


def sample_uniform_l1_ball(nsamples: int=100,
                           d: int=2):

    U = np.random.uniform(size=(nsamples, d))
    U = np.sort(U, axis=1)
    V = np.zeros((nsamples, d+1))
    V[:,1:] = U
    X = np.diff(V, axis=1)
    signs = np.random.randint(low=0, high=2, size=(nsamples, d)) * 2 - 1
    X *= signs
    return X


if __name__ == '__main__':
    X = sample_uniform_l1_ball(nsamples=1000)

    import matplotlib.pyplot as plt
    plt.plot(X[:,0], X[:,1], '+')
    plt.show()
