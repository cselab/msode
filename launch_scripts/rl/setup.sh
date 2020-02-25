#!/bin/bash

# setup for smarties launch script 
# assume the app is compiled

# copy executable:
exec=$MSODE_ROOT/build/apps/$APP_NAME
cp $exec $RUNDIR/exec

# copy config
cp $MSODE_ROOT/launch_scripts/rl/config/$CONF_FILE $RUNDIR/config.json

