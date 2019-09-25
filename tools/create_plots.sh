#!/bin/bash

set -eu

name=$1; shift
n=$1; shift

out_dir=trajectories_$name

mkdir -p $out_dir

for i in `seq 0 $n`; do
    id=`printf "%02d" $i`

    tmp=/tmp/trajectory.txt
    head -n -1 eval_$name/simulation_001_000$id/trajectories.txt > $tmp
    
    mir.post ./tools/plot_trajectories.py \
	     --file $tmp  \
	     --out $out_dir/$id.pdf

    rm -rf $tmp
done 
