#!/bin/bash

# setup for smarties launch script 
# assume the app is compiled

# TODO: this should be changed
MSODE_ROOT=${SMARTIES_ROOT}/../msode

# copy executable:
exec=$MSODE_ROOT/apps/app_rl
#exec=$MSODE_ROOT/apps/app_rl_comp

cp $exec $RUNDIR/exec

# copy config
CFG_DIR=$RUNDIR/config
#data_dir=data/test
data_dir=data/helix
mkdir -p $RUNDIR/config
cp $MSODE_ROOT/$data_dir/config/swimmer*cfg $CFG_DIR/
echo "0 $CFG_DIR/swimmer_00.cfg" >  $CFG_DIR/swimmers_list.cfg
echo "1 $CFG_DIR/swimmer_01.cfg" >> $CFG_DIR/swimmers_list.cfg

