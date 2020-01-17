#! /bin/bash

set -eu
res_dir="results"

src=$1; shift
name=$1; shift

nevals=1

if test $# -ge 1; then
    nevals=$1; shift
fi

echo "launching $nevals evaluation "

srcdir=`pwd`
rundir=$res_dir/training_$name

mkdir -p $rundir
cp $src/agent*raw $rundir/
cd $rundir

. mir.load
export MSODE_ROOT=$srcdir/../../

smarties.py $srcdir \
	    --nEvalSeqs $nevals \
	    --nThreads 8 \
	    --nEnvironments 1
