#! /bin/bash

set -eu

res_dir="results"
launch_mode="interactive"

usage()
{
    cat <<EOF
usage: ./launch_rl.sh <name_specifier> <simulation config file>
    [-h | --help] Print this help message
    [-c | --comp] Launch the comparison rl + analytical methods
    [--cluster] Launch on euler or daint
    [--settings=<rl-settings.json>] Use specific smarties settings (default: see smarties docs)
    [--res_dir=<res_dir>] store the simulation folder output in this directory (default: ${res_dir}/)

The simulation will run in <res_dir>/training_<name_specifier>
The simulation config file must be in the path relative to this launch script.

EOF
    exit 1
}

# default values
app_name="run_rl"
settings="default.json"

POSITIONAL_ARGS=""

# parse optional arguments
while test $# -ne 0; do
    case "$1" in
	-h|--help)
	    usage
	    ;;
	-c|--comp)
	    app_name="run_rl_comp"
	    shift
	    ;;
	--cluster)
	    launch_mode="cluster"
	    shift
	    ;;
	--settings=*)
	    settings="${1#*=}"
	    shift
	    ;;
	--res_dir=*)
	    res_dir="${1#*=}"
	    shift
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
eval set -- "$POSITIONAL_ARGS"
if test $# -ne 2; then usage; fi
name=$1; shift
sim_config=$1; shift


srcdir=`pwd`
rundir=$res_dir/training_$name

cat<<EOF
launch settings
--------------------
smarties settings:  $settings
msode settings:     $sim_config
application in use: $app_name"
training output:    $rundir
--------------------
EOF

mkdir -p $rundir

# setup

# copy executable:
cp $srcdir/../../build/apps/$app_name $rundir/exec

# copy config
cp $srcdir/$sim_config $rundir/config.json

cd $rundir

num_train_steps=20000000

if [ $launch_mode = "cluster" ]; then

    . load
    echo $settings
    smarties.py $srcdir $settings \
	 --nThreads 8 \
	 --nEnvironments 1 \
	 --clockHours 24 \
	 --nTrainSteps $num_train_steps \
	 --restart .

else
    . mir.load

    smarties.py $srcdir $settings \
		--nThreads 8 \
		--nEnvironments 1 \
		--nTrainSteps $num_train_steps \
		--restart .
fi
