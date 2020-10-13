#!/bin/bash

set -eu

wc=0
for i in `seq 0 279`; do
    /Applications/ParaView-5.8.1.app/Contents/bin/pvbatch scene.py $wc $i
done

wc=1
for i in `seq 0 279`; do
    /Applications/ParaView-5.8.1.app/Contents/bin/pvbatch scene.py $wc $i
done

# ffmpeg -r 30 -hide_banner -loglevel warning -stats -pattern_type glob -i 'wc_*.png' -c:v libx264 -pix_fmt yuv420p -vf scale=2000:1150 -y movie.mp4
