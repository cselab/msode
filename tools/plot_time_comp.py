#! /usr/bin/env python3

import numpy as np
import argparse
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('file', type=str, help='Console output file of app_rl_comp.')
parser.add_argument('--start', type=int, default=0, help="Only count episodes after this number.")
parser.add_argument('--out', type=str, default="GUI")
args = parser.parse_args()

data = np.loadtxt(args.file, skiprows=3)

ids = data[args.start:,0]
tAC = data[args.start:,1]
tRL = data[args.start:,2]
end_dist = data[args.start:,3]
init_dist = data[args.start:,4]


success = np.argwhere(end_dist < 2).flatten()

ids = ids[success]
tAC = tAC[success]
tRL = tRL[success]
end_dist = end_dist[success]
init_dist = init_dist[success]

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
