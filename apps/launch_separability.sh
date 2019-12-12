#!/bin/sh

base="sep"

for N in `seq 2 4`; do
    echo "starting N = $N"
    ./app_ac_separability $N 50.0 200 > ${base}_N_${N}.txt
    echo done
done
