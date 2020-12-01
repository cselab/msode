#!/bin/bash

set -eu

apps=../../build/apps
config=../../data/helix/config/swimmer_00.json
fps=120

res=results

mkdir -p $res

for w in 40 60 80 100 120; do
    $apps/forward $config $w $fps $res/xt_w_$w.dat
done
