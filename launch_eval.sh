#! /bin/bash

set -eu

src=$1; shift
name=$1; shift

nevals=1

if test $# -ge 1; then
    nevals=$1; shift
fi

rundir=eval_$name

mkdir -p $rundir
cp $src/agent* $rundir/
cd $rundir

. mir.load

for i in `seq 0 $nevals`; do
    id=`printf "%04d" $i`
    
    smarties.py .. \
		--nEvalSeqs 1 \
		--nThreads 1 \
		--nEnvironments 1

    sim_dir=simulation_001_0$id
    head -n -1 $sim_dir/trajectories.txt > $sim_dir/trajectories_clean.txt
done
