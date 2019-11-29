#! /usr/bin/env python3

import numpy as np
import argparse
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('--file', type=str, required=True, help='console output file of app_rl_comp')
parser.add_argument('--out', type=str, default="GUI")
args = parser.parse_args()

data = np.loadtxt(args.file, skiprows=2)

ids = data[:,0]
tAC = data[:,1]
tRL = data[:,2]

speedup = tRL / tAC

print(np.mean(speedup), np.sqrt(np.var(speedup)))

fig = plt.figure(0)
ax = fig.add_subplot(111)

ax.hist(speedup, bins=50, range=[0.50, 1.75], density=True)

best = np.argmin(speedup)
print("best RL is id", ids[best], "with speedup", speedup[best])

ax.set_xlabel(r'$t_{RL} / t_{AC}$')
ax.set_ylabel(r'$pdf$')
#plt.grid()

if args.out == "GUI":
    plt.show()
else:
    plt.savefig(args.out, transparent=True)

