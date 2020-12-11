#! /bin/bash

set -eu

nevals=10000
res_dir="res_robustness"

mkdir -p $res_dir

for s in 0.5 0.55 0.6 0.65 0.7 0.75 0.8 0.85 0.9 0.95 1.0 1.05 1.1 1.15 1.2 1.25 1.3 1.35 1.4 1.45 1.5 1.55 1.6; do
    conf=config_s_$s.json
    simdir=results/eval_flow_$s
    echo $conf

    if [[ -f $res_dir/s_$s.dat ]]; then
	continue
    fi

    ./vary_flow.py config/dpd_2_d_eu_gaussian_flow_1.0.json --scale $s --out $conf
    rm -rf $simdir
    ./eval_rl.sh `pwd`/results/training_dpd_2_d_eu_gaussian_flow_1.0/ flow_$s $conf --comp --n_evals=$nevals
    mv $simdir/simulation_001_00000/output_000 $res_dir/s_$s.dat
    rm -rf $simdir
    rm -f $conf
done
