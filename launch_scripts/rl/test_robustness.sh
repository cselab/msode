#! /bin/bash

set -eu

nevals=10000
res_dir="res_robustness"

mkdir -p $res_dir

task()
(
    p=$1; shift
    s=$1; shift

    id=p_${p}_s_${s}

    conf=config_${id}.json
    simdir=results/eval_flow_${id}
    echo $conf

    if [[ -f $res_dir/${id}.dat ]]; then
	echo Skipping s=$s.
        exit 0
    fi

    ./vary_flow.py config/dpd_2_d_eu_gaussian_flow_1.0.json --magnitude-factor $s --invPeriod-factor $p --no-dump --out $conf
    rm -rf $simdir
    ./eval_rl.sh `pwd`/results/training_dpd_2_d_eu_gaussian_flow_1.0/ flow_${id} $conf --comp --n_evals=$nevals
    mv $simdir/simulation_001_00000/output_000 $res_dir/${id}.dat
    rm -rf $simdir
    rm -f $conf
)

# for debugging
#task() (echo start $@; sleep 1; echo end $@)


jobid=0
nprocs=8
p=$1; shift

for s in `seq 0.00 0.05 1.0`; do
    task $p $s &
    jobid=$((jobid+1))

    if ! (( $jobid % $nprocs )) ; then
        wait
    fi
done

wait
echo Done with all $jobid jobs
