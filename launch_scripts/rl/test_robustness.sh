#! /bin/bash

set -eu

nevals=10000
res_dir="res_robustness"

mkdir -p $res_dir

task()
(
    s=$1; shift
    conf=config_s_$s.json
    simdir=results/eval_flow_$s
    echo $conf

    if [[ -f $res_dir/s_$s.dat ]]; then
        exit 0
    fi

    ./vary_flow.py config/dpd_2_d_eu_gaussian_flow_1.0.json --scale $s --no-dump --out $conf
    rm -rf $simdir
    ./eval_rl.sh `pwd`/results/training_dpd_2_d_eu_gaussian_flow_1.0/ flow_$s $conf --comp --n_evals=$nevals
    mv $simdir/simulation_001_00000/output_000 $res_dir/s_$s.dat
    rm -rf $simdir
    rm -f $conf
)

for s in `seq 0.00 0.05 2.00`; do
    task $s &
done

wait
echo Done with all jobs
