# msode

microswimmer ODE solver

## build the libraries, tests and executables:

	mkdir build
	cd build
	cmake <options> ..
	make -j <njobs>

The options are:

- build mode:

		-DCMAKE_BUILD_TYPE=Release (default)
		-DCMAKE_BUILD_TYPE=Debug

- for debugging with contracts:

		-DBUILD_WITH_CONTRACTS=ON (default: ON)
		-DENABLE_STACKTRACE=ON (default: ON)

- sanitizers (for debugging):

		-DSANITIZE_ADDRESS=ON
		-DSANITIZE_MEMORY=ON
		-DSANITIZE_THREAD=ON
		-DSANITIZE_UNDEFINED=ON

- skip smarties:

		-DUSE_SMARTIES=OFF



## usage

See `./app` help messages.
Configuration files for swimmers examples can be found in the `data/*/config/`directories.


## Reinforcement Learning

See [smarties](https://github.com/cselab/smarties) for usage.
