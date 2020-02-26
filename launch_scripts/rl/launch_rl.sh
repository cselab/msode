#! /bin/bash

set -eu
res_dir="results"

usage()
{
    cat <<EOF
usage: ./launch_rl.sh <name_specifier> <simulation config file>

The simulation will run in $res_dir/training<name_specifier>

EOF
    exit 1
}

if test $# -ne 0 && test $1 = -h;     then usage; fi
if test $# -ne 0 && test $1 = --help; then usage; fi
if test $# -ne 2; then usage; fi

name=$1; shift
sim_comfig=$1; shift

srcdir=`pwd`
rundir=$res_dir/training_$name

mkdir -p $rundir
cd $rundir

. mir.load

export MSODE_ROOT=$srcdir/../../

export APP_NAME=app_rl
#export APP_NAME=app_rl_comp

export CONF_FILE=$sim_comfig

#settings="NAF.json"
#settings="DPG.json"
settings="default.json"

smarties.py $srcdir $settings \
	    --nThreads 8 \
	    --nEnvironments 1 \
	    --nTrainSteps 20000000 \
	    --restart .
