#! /bin/bash

set -eu

src=$1; shift
name=$1; shift

nevals=1

if test $# -ge 1; then
    nevals=$1; shift
fi

echo "launching $nevals evaluation "

rundir=eval_$name

mkdir -p $rundir
cp $src/agent*raw $rundir/
cd $rundir

. mir.load

smarties.py .. \
	    --nEvalSeqs $nevals \
	    --nThreads 8 \
	    --nEnvironments 1
