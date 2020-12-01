#!/bin/bash

set -eu

apps=../../build/apps
config=../../data/helix/config/swimmer_00.json
fps=120

res=results

mkdir -p $res

wc=69.8364

for a in 0.6 0.8 1 1.2 1.4 1.6; do
    w=`python -c "print($wc * $a)"`
    $apps/forward $config $w $fps $res/xt_w_$w.dat
done
