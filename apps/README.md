# msode apps

a set of apps for simple cases with ABFs in a magnetic field

## ac_separability

Collect optimal travel time in one dimension for N swimmers, along with properties of the velocity matrix.
See ../launch_scripts/separability/ for application.

## ac_stats

Compute mean travel time for multiple swimmers from a random position in a box to the origin.

## convex_hull

reports forward velocities for `N` given swimmers; 
output are "velocity vectors" (one for each omega sample) which can be processed to obtained the convex hull of that cloud 
this allows to choose a velocity matrix with optimal omegas.

## dump_velocity_field

Read a config file compatible with one of ``run_rl`` or ``run_rl_comp`` and dump the velocity field that is described there. 

## forward

ABF in a field rotating with frequency `omega` in the yz plane.

## forward_curve

ABF in a field rotating with frequencies `omega` in the yz plane.
report mean velocity along the x axis for many omegas.

## orient

A small case for reorienting randomly oriented ABFs along a given axis.
The method for reorienting consists in applying a field rotating along the target axis with alternating +/- `omega`.
The magnitude `omega` is lower than the step out frequency to allow the rotation of the ABF.
This is the method used used in `analytic_control`.

## run_ac

Finds a sequence of magnetic field rotation frequencies and directions to bring given ABFs to a target position optimally.

## run_rl

Use smarties to find optimal policy for the problem stated in `run_ac`.

## run_rl_comp

Use smarties to find optimal policy for the problem stated in `run_ac` and compare to the `run_ac` results for the same setup.

## rotating

ABF in a rotating field with direction changing direction over time (describes a circle).

