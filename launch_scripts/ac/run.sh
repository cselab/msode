#!/bin/bash

set -eu

config=$1; shift
L=$1; shift

app=../../build/apps/run_ac

$app $config $L
