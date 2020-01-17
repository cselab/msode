#!/bin/bash

# setup for smarties launch script 
# assume the app is compiled

# copy executable:
exec=$MSODE_ROOT/build/apps/app_rl
#exec=$MSODE_ROOT/build/apps/app_rl_comp
#exec=$MSODE_ROOT/build/apps/app_rl_curriculum

cp $exec $RUNDIR/exec

# copy config
CFG_DIR=$RUNDIR/config
mkdir -p $CFG_DIR

#data_dir=data/test
data_dir=data/helix
#data_dir=data/helix_many

cp $MSODE_ROOT/$data_dir/config/swimmer*cfg $CFG_DIR/

swimmer_list=$CFG_DIR/swimmers_list.cfg
rm -rf $swimmer_list
#for i in 0 4 8; do
for i in 0 1; do
    echo "$i $CFG_DIR/swimmer_0$i.cfg" >> $swimmer_list
done

