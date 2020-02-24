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

#settings="NAF.json"
#settings="DPG.json"
settings="default.json"

smarties.py $srcdir $settings \
	    --nThreads 8 \
	    --nEnvironments 1 \
	    --nTrainSteps 20000000 \
	    --restart .
