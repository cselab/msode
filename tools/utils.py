#! /usr/bin/env python3

def remove_next_sim_data(data):
    import numpy as np
    t = data[:,0]
    end = np.argmax(t)
    return data[:end,:]


def read_rigid_data(data):
    q = data[:,0:4]
    r = data[:,4:7]
    w = data[:,7:10]
    return q, r, w

