#! /bin/bash

set -eu

EXEC=msode_rl
RUNDIR=$1; shift

mkdir -p $RUNDIR
cp $EXEC $RUNDIR/
cd $RUNDIR/
echo "OMP_NUM_THREADS=8 ./$EXEC $@"
OMP_NUM_THREADS=8 ./$EXEC $@

