#!/bin/bash

n=19
name=two_test
out_dir=trajectories_$name

mkdir -p $out_dir

for i in `seq 0 $n`; do
    id=`printf "%02d" $i`
    mir.post ./tools/plot_trajectories.py \
	     --file eval_$name/simulation_001_000$id/trajectories.txt \
	     --out $out_dir/$id.pdf
done 
