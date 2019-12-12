#!/bin/sh

base="sep"
nsamples=1000

for N in `seq 2 5`; do
    echo "starting N = $N"
    ./app_ac_separability $N 50.0 $nsamples > ${base}_N_${N}.txt
    echo done
done
