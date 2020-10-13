#!/bin/bash

fps=120

w0=28
w1=69

root=../..

mesh_dir=$root/data/helix/mesh

mesh0=$mesh_dir/helix_P_2.0.ply
mesh1=$mesh_dir/helix_P_5.0.ply

conf_dir=$root/data/helix/config
conf0=$conf_dir/swimmer_00.json
conf1=$conf_dir/swimmer_01.json

forward=$root/build/apps/forward
create_mesh=$root/tools/create_mesh.py

$forward $conf0 $w0 $fps traj_swimmer_00_wc_0.dat
$forward $conf0 $w1 $fps traj_swimmer_00_wc_1.dat
$forward $conf1 $w0 $fps traj_swimmer_01_wc_0.dat
$forward $conf1 $w1 $fps traj_swimmer_01_wc_1.dat

$create_mesh traj_swimmer_00_wc_0.dat --mesh $mesh0 --flip --out ply_swimmer_00_wc_0
$create_mesh traj_swimmer_00_wc_1.dat --mesh $mesh0 --flip --out ply_swimmer_00_wc_1
$create_mesh traj_swimmer_01_wc_0.dat --mesh $mesh1 --flip --out ply_swimmer_01_wc_0
$create_mesh traj_swimmer_01_wc_1.dat --mesh $mesh1 --flip --out ply_swimmer_01_wc_1
