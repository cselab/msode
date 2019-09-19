#! /bin/bash

set -eu

src=$1; shift
name=$1; shift

nevals=1

if test $# -ge 1; then
    nevals=$1; shift
fi

rundir=eval_$name

mkdir -p $rundir
cp $src/agent* $rundir/
cd $rundir

. mir.load

for i in `seq 1 $nevals`; do
    smarties.py .. \
		--nEvalSeqs 1 \
		--nThreads 1 \
		--nEnvironments 1
done
