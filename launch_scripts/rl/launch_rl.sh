#! /bin/bash

set -eu
res_dir="results"

usage()
{
    cat <<EOF
usage: ./launch_rl.sh <name_specifier> <simulation config file>
    [-h | --help] Print this help message

The simulation will run in $res_dir/training_<name_specifier>
The simulation config file must be in the path relative to this launch script.

EOF
    exit 1
}

POSITIONAL_ARGS=""

# parse optional arguments
while test $# -ne 0; do
    case "$1" in
	-h|--help)
	    usage
	    ;;
	-*|--*)
	    echo "Error: unsupported option $1"
	    usage
	    ;;
	*)
	    POSITIONAL_ARGS="$POSITIONAL_ARGS $1"
	    shift
	    ;;
    esac
done

# parse positional arguments
set -- "$POSITIONAL_ARGS"
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
