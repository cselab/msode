#! /usr/bin/env python3

def evaluate_field(x, y, z, config):
    import numpy as np
    
    cfg = config["velocityField"]
    strtype = cfg["__type"]

    n = len(x)
    vx = np.zeros(n)
    vy = np.zeros(n)
    vz = np.zeros(n)

    if strtype == "FieldTaylorGreenVortex":
        magn = cfg["magnitude"]
        invP = cfg["invPeriod"]

        factor = np.cos(invP[0] * x) * np.sin(invP[1] * y) * np.sin(invP[2] * z)
        vx = factor * magn[0]
        vy = factor * magn[1]
        vz = factor * magn[2]
        
    elif strtype == "None":
        # already zero
        pass
    
    elif strtype == "Constant":
        vel = cfg["vel"]
        vx = np.ones(n) * vel[0]
        vy = np.ones(n) * vel[1]
        vz = np.ones(n) * vel[2]

    else:
        print("Could not create a velocity field of type {}".format(strtype))
        exit(1)
    
    return np.transpose(np.vstack((vx, vy, vz)))


def evaluate_field_from_config_filename(x, y, z, fname):
    import json
    with open(fname) as conf_file:
        config = json.load(conf_file)
        return evaluate_field(x, y, z, config)

