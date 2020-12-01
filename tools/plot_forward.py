#! /usr/bin/env python3

import numpy as np
import argparse
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('--files', type=str, required=True, nargs='+', help='output of ODE simulation (forward_curve)')
parser.add_argument('--out', type=str, default="GUI")
parser.add_argument('--show_omegac', action='store_true', default=False)
args = parser.parse_args()

fig = plt.figure(0)
ax = fig.add_subplot(111)

for filename in args.files:
    data = np.loadtxt(filename)
    w = data[:,0]
    v = data[:,1]
    ax.plot(w, v, '-')

    if args.show_omegac:
        i = np.argmax(v)
        wc = w[i]
        vmax = v[i]
        ax.plot([wc,wc], [0,vmax], '--k')
        locs   = ax.get_xticks()
        labels = [str(x) for x in locs]
        locs   = np.append(locs, wc)
        labels = np.append(labels, r'$\omega_c$')

        ax.set_xticks(locs)
        ax.set_xticklabels(labels)

ax.set_xlabel(r'$\omega$ [Hz]')
ax.set_ylabel(r'$V$ [body length / s]')

plt.grid()
plt.tight_layout()

ax.set_xlim([min(w),max(w)])
ax.set_ylim([min(v),max(v)])

if args.out == "GUI":
    plt.show()
else:
    plt.savefig(args.out, transparent=True)
