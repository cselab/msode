#!/bin/bash

for i in `seq 1 10`; do
    id=`printf "%02d" $i`
    echo $id
    ./launch_rl.sh --cluster \
		   --res_dir=$SCRATCH/amlucas/msode \
		   2d_tt_b_flow_0.2_$id \
		   config/helix_2d_tt_b_flow.json 
done
