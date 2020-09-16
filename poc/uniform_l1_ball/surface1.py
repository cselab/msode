#!/usr/bin/env python3

import numpy as np

def get_signs(nsamples: int, d: int):
    return np.random.randint(low=0, high=2, size=(nsamples, d)) * 2 - 1

def non_uniform(nsamples: int,
                d: int=2):
    X = np.random.exponential(scale=1, size=(nsamples, d))

    X1 = np.sum(np.abs(X), axis=1)
    X /= X1[:,None]
    return X * get_signs(nsamples, d)



if __name__ == '__main__':

    X1 = non_uniform(nsamples=1000, d=3)

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    plt.plot(X1[:,0], X1[:,1], X1[:,2], '+')
    plt.show()
