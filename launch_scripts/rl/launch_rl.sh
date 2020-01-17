#! /bin/bash

set -eu
res_dir="results"

name=$1; shift

srcdir=`pwd`
rundir=$res_dir/training_$name

mkdir -p $rundir
cd $rundir

. mir.load
export MSODE_ROOT=$srcdir/../../

smarties.py $srcdir \
	    --nThreads 8 \
	    --nEnvironments 1 \
	    --nTrainSteps 20000000 \
	    --restart .
