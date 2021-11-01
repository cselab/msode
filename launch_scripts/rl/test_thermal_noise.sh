#! /bin/bash

set -eu

nevals=1000
res_dir="res_thermal_noise"

mkdir -p $res_dir

task()
(
    kBT=$1; shift

    id=kBT_${kBT}

    conf=config_${id}.json
    simdir=results/eval_flow_${id}
    echo $conf

    if [[ -f $res_dir/${id}.dat ]]; then
	echo Skipping id=$id.
        exit 0
    fi

    ./add_noise.py config/dpd_2_d_eu_gaussian_flow_1.0.json \
		   --kBT $kBT \
		   --no-dump \
		   --out $conf

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

n=32

# ambiant temperature in simulation units
kBT0=0.0041034345047923325
kBTs=`python -c "import numpy as np; print(*np.logspace(np.log($kBT0/10)/np.log(10), np.log($kBT0*1000)/np.log(10), $n).tolist())"`

for kBT in $kBTs; do
    task $kBT &
    jobid=$((jobid+1))

    if ! (( $jobid % $nprocs )) ; then
        wait
    fi
done

wait
echo Done with all $jobid jobs
