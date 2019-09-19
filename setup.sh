#!/bin/bash

# setup for smarties launch script 
# assume the app is compiled

# TODO: this should be changed
MSODE_ROOT=${SMARTIES_ROOT}/../msode

# copy executable:
cp $MSODE_ROOT/msode_rl $RUNDIR/exec

# copy config
# CFG_DIR=$RUNDIR/config
# mkdir -p $RUNDIR/config
# cp $MSODE_ROOT/config/swimmer*cfg $CFG_DIR/
# echo "0 $CFG_DIR/swimmer0.cfg" >  $CFG_DIR/swimmers_list.cfg
# echo "1 $CFG_DIR/swimmer1.cfg" >> $CFG_DIR/swimmers_list.cfg 

CFG_DIR=$RUNDIR/config
mkdir -p $RUNDIR/config
cp $MSODE_ROOT/data/config/swimmer*cfg $CFG_DIR/
echo "0 $CFG_DIR/swimmer_00.cfg" >  $CFG_DIR/swimmers_list.cfg
echo "1 $CFG_DIR/swimmer_01.cfg" >> $CFG_DIR/swimmers_list.cfg 
