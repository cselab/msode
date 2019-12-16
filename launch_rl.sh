#! /bin/bash

set -eu

name=$1; shift

rundir=training_$name

mkdir -p $rundir
cd $rundir

. mir.load

smarties.py .. \
	    --nThreads 8 \
	    --nEnvironments 1 \
	    --nTrainSteps 20000000 \
	    --restart .
