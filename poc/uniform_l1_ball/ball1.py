#!/usr/bin/env python3

import numpy as np


def sample_uniform_l1_shell_positive(R0: float,
                                     R1: float,
                                     nsamples: int,
                                     d: int):
    assert R1 >= R0

    U = np.random.uniform(0, R1, size=(nsamples, d))
    U = np.sort(U, axis=1)
    V = np.zeros((nsamples, d+1))
    V[:,1:] = U
    X = np.diff(V, axis=1)
    return X


def sample_uniform_l1_ball_positive(R: float,
                                    nsamples: int,
                                    d: int):
    return sample_uniform_l1_shell_positive(0, 1, nsamples, d)

def get_signs(nsamples: int, d: int):
    return np.random.randint(low=0, high=2, size=(nsamples, d)) * 2 - 1


def sample_uniform_l1_ball(R: float,
                           nsamples: int,
                           d: int=2):
    X = sample_uniform_l1_ball_positive(R, nsamples, d)
    X *= get_signs(nsamples, d)
    return X

def sample_uniform_l1_shell(R0: float,
                            R1: float,
                            nsamples: int,
                            d: int=2):

    X = sample_uniform_l1_shell_positive(R0, R1, nsamples, d)
    X *= get_signs(nsamples, d)
    return X



def exponential_method(nsamples: int,
                     d: int=2):
    X = np.random.exponential(scale=1, size=(nsamples, d))
    Y = np.random.exponential(scale=1, size=nsamples)

    X1 = np.sum(np.abs(X), axis=1)
    X /= (Y + X1)[:,None]
    return X

if __name__ == '__main__':
    #X1 = sample_uniform_l1_ball(R=1, nsamples=1000)
    X1 = exponential_method(nsamples=1000)


    import matplotlib.pyplot as plt
    plt.plot(X1[:,0], X1[:,1], '+')
    #plt.plot(X2[:,0], X2[:,1], '+')
    plt.show()
