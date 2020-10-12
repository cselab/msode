#!/bin/bash

fps=120

w0=28
w1=69

mesh_dir=../data/helix/mesh

mesh0=$mesh_dir/helix_P_2.0.ply
mesh1=$mesh_dir/helix_P_5.0.ply

conf_dir=../data/helix/config
conf0=$conf_dir/swimmer_00.json
conf1=$conf_dir/swimmer_01.json

../build/apps/forward $conf0  $w0 $fps traj_swimmer_00_wc_0.dat
../build/apps/forward $conf0  $w1 $fps traj_swimmer_00_wc_1.dat
../build/apps/forward $conf1  $w0 $fps traj_swimmer_01_wc_0.dat
../build/apps/forward $conf1  $w1 $fps traj_swimmer_01_wc_1.dat

../tools/create_mesh.py traj_swimmer_00_wc_0.dat --mesh $mesh0 --flip --out ply_swimmer_00_wc_0
../tools/create_mesh.py traj_swimmer_00_wc_1.dat --mesh $mesh0 --flip --out ply_swimmer_00_wc_1
../tools/create_mesh.py traj_swimmer_01_wc_0.dat --mesh $mesh1 --flip --out ply_swimmer_01_wc_0
../tools/create_mesh.py traj_swimmer_01_wc_1.dat --mesh $mesh1 --flip --out ply_swimmer_01_wc_1
