# msode

microswimmer ODE solver

## compilation

In release mode:

	make apps

In debug mode:

	debug=1 make apps

## usage

See `./app` help messages.
Configuration files for swimmers examples can be found in the `config/`directory.


## Reinforcement Learning

Compile the application:

	make msode_rl

See [smarties](https://github.com/cselab/smarties) for usage.


## create movie

* set dump rate in `rl/main.cpp` and evaluate
* create ply files:

        mir.post ../../msode/tools/create_mesh.py --file eval_two/simulation_001_00007/trajectories.txt --mesh ~/ymrRun/microswimmers/ABFs/data/helix_P_2.5.ply ~/ymrRun/microswimmers/ABFs/data/helix_P_5.0.ply --flip

* paraview: create planes; add texture
* import ply files; add color; add light; export to png
* combine:

	    ffmpeg -framerate 60 -start_number 0 -i movie/out.%4d.png -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -crf 20 -tune film -pix_fmt yuv420p ABFs.mov
