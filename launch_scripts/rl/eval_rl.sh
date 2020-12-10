#! /bin/bash

set -eu

nevals=100
res_dir="results"
launch_mode="interactive"

usage()
{
    cat <<EOF
usage: ./eval_rl.sh <trained_folder> <name_specifier> <simulation config file>
    [-h | --help] Print this help message
    [-c | --comp] Launch the comparison rl + analytical methods
    [--cluster] Launch on euler or daint
    [--settings=<rl-settings.json>] Use specific smarties settings (default: see smarties docs)
    [--res_dir=<res_dir>] store the simulation folder output in this directory (default: ${res_dir}/)
    [--n_evals=<num evaluations>] Number of episodes to perform (default: ${nevals})

The simulation will run in <res_dir>/eval_<name_specifier>
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
	--n_evals=*)
	    nevals="${1#*=}"
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
if test $# -ne 3; then usage; fi
trained_dir=$1; shift
name=$1; shift
sim_config=$1; shift


srcdir=`pwd`
rundir=$res_dir/eval_$name

cat<<EOF
launch settings
--------------------
smarties settings:  $settings
msode settings:     $sim_config
application in use: $app_name"
trained dir output: $trained_dir
running directory:  $rundir
num evals:          $nevals
--------------------
EOF

mkdir -p $rundir

# setup

# copy executable:
cp $srcdir/../../build/apps/$app_name $rundir/exec

# copy config
cp $srcdir/$sim_config $rundir/config.json

cd $rundir

if [ $launch_mode = "cluster" ]; then

    . load
    echo $settings
    smarties.py $srcdir $settings \
	 --nThreads 8 \
	 --nEnvironments 1 \
	 --nTrainSteps 20000000 \
	 --clockHours 24 \
	 --restart $trained_dir \
	 --nEvalEpisodes $nevals

else
    . mir.load

    smarties.py $srcdir $settings \
		--nThreads 8 \
		--nEnvironments 1 \
		--nTrainSteps 20000000 \
		--restart $trained_dir \
		--nEvalEpisodes $nevals
fi
