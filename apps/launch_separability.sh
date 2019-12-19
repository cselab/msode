#!/bin/bash

base="sep"
nsamples=1000

ae () { awk 'BEGIN {printf "%g", '"$@"'}'; }

for N in `seq 2 10`; do
    omegaMax=`ae "$N * 25.0"`
    echo "starting N = $N with wmax = $omegaMax"
    ./app_ac_separability $N $omegaMax $nsamples > ${base}_N_${N}.dat
    echo done
done
