#!/bin/bash

set -eu

n=$1; shift
name=$1; shift

for i in `seq 0 $n`; do
    id=`printf "%02d" $i`
    tmp=/tmp/trajectory_$id.txt
    head -n -1 eval_$name/simulation_001_000$id/trajectories.txt > $tmp
done 

mir.post ./tools/action_density.py \
	 --files /tmp/trajectory_??.txt

rm -rf /tmp/trajectory_*txt
