#!/bin/bash

set -eu

apps=../../build/apps
config=../../data/helix/config/swimmer_00.json
fps=120

res=results

mkdir -p $res

wc=69.8364

for a in 0.6 1 1.4 1.8; do
    w=`python -c "print($wc * $a)"`
    $apps/forward $config $w $fps $res/xt_w_$w.dat
done
