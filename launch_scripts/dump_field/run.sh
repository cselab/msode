#! /bin/sh

set -eu

out=$1; shift
conf=field.json

../../build/apps/app_dump_velocity_field $conf $out
