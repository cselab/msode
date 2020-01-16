# msode

microswimmer ODE solver

## compilation

### requirements

- korali https://github.com/cselab/korali
- smarties https://github.com/cselab/smarties


### debug mode (default)

	mkdir buils
	cd build
	cmake ..
	make -j <njobs>

### release mode (no contracts)

	mkdir buils
	cd build
	cmake -DBUILD_WITH_CONTRACTS=ON ..
	make -j <njobs>


## usage

See `./app` help messages.
Configuration files for swimmers examples can be found in the `data/*config/`directories.

## Non Dimensionalization

Input: 

* dimensions of microswimmer
* magnetic field intensity
* step out frequency `omega_c` or max velocity `v_max`
* propulsion matrix

Procedure:

* set length scale such that `1 (sim) = body` length (~20um typically)
* set time scale such that `1 (sim) = 1s`
* set energy scale such that `eta * B_xx = 0.1` (arbitrary) -> gives all entries of propulsion matrix
* set magnetic scale such that `B (sim) = 1`
* compute `m` given `B`, `B_xx` and `v_max` or `omega_c`

Output:
trajectories: length in body length, time in seconds


## Reinforcement Learning

See [smarties](https://github.com/cselab/smarties) for usage.


## create movie

* set dump rate in `rl/main.cpp` and evaluate
* create ply files:

        mir.post ../../msode/tools/create_mesh.py --file eval_two/simulation_001_00007/trajectories.txt --mesh ~/ymrRun/microswimmers/ABFs/data/helix_P_2.5.ply ~/ymrRun/microswimmers/ABFs/data/helix_P_5.0.ply --flip

* paraview: create planes; add texture
* import ply files; add color; add light; export to png
* combine:

	    ffmpeg -framerate 60 -start_number 0 -i movie/out.%4d.png -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -crf 20 -tune film -pix_fmt yuv420p ABFs.mov
