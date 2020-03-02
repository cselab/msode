#!/bin/bash

apps_bin=../../build/apps/
result_base_name="results/sep"
nsamples=1000

ae () { awk 'BEGIN {printf "%g", '"$@"'}'; }

for N in `seq 2 12`; do
    omega_max=`ae "$N * 25.0"`
    echo "starting N = $N with wmax = $omega_max"
    $apps_bin/ac_separability $N $omega_max $nsamples > ${result_base_name}_N_${N}.dat
    echo done
done
