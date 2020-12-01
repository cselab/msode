#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np

def get_w(fname: str):
    import re
    rexf = '[-+]?\d*\.\d+|\d+'
    matches = re.findall(f"w_({rexf}).dat", fname)
    assert len(matches) == 1
    return float(matches[0])


def plot(files: list,
         wc: float,
         l: float,
         i0: int=5):

    files = np.array(files)
    ws = np.array([get_w(f) for f in files])
    idx = np.argsort(ws)
    ws = ws[idx]
    files = files[idx]

    fig, ax = plt.subplots()

    tmax = None
    xmax = None

    for w, f in zip(ws, files):
        data = np.loadtxt(f)
        t = data[:,0] * wc
        x = -data[:,9] / l
        x -= x[i0]
        t -= t[i0]
        ax.plot(t, x, label=f"$\omega = {w/wc} \omega_c$")

        if tmax is None:
            tmax = np.max(t)
            xmax = np.max(x)
        else:
            tmax = min([tmax, np.max(t)])
            xmax = min([xmax, np.max(x)])

    ax.set_xlabel('$t \omega_c$')
    ax.set_ylabel('$x / l$')
    ax.set_xlim(0, tmax)
    ax.set_ylim(0, xmax)
    ax.legend()
    plt.show()


def main(argv):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('files', type=str, nargs='+', help='output of ODE simulation (forward)')
    parser.add_argument('--out', type=str, default="GUI")
    parser.add_argument('--wc', type=float, default=69.8364)
    parser.add_argument('--l', type=float, default=1)
    args = parser.parse_args(argv)

    plot(args.files, args.wc, args.l)


if __name__ == '__main__':
    import sys
    main(sys.argv[1:])
