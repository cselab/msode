# msode apps

a set of apps for simple cases with ABFs in a magnetic field

## app_ac_separability

Collect optimal travel time in one dimension for N swimmers, along with properties of the velocity matrix.
See ../launch_scripts/separability/ for application.

## app_ac_stats

Compute mean travel time for multiple swimmers from a random position in a box to the origin.

## app_analytic_control

Finds a sequence of magnetic field rotation frequencies and directions to bring given ABFs to a target position optimally.

## app_convex_hull

reports forward velocities for `N` given swimmers; 
output are "velocity vectors" (one for each omega sample) which can be processed to obtained the convex hull of that cloud 
this allows to choose a velocity matrix with optimal omegas.

## app_dump_velocity_field

Read a config file compatible with one of ``app_rl`` or ``app_rl_comp`` and dump the velocity field that is described there. 

## app_forward

ABF in a field rotating with frequency `omega` in the yz plane.

## app_forward_curve

ABF in a field rotating with frequencies `omega` in the yz plane.
report mean velocity along the x axis for many omegas.

## app_orient

A small case for reorienting randomly oriented ABFs along a given axis.
The method for reorienting consists in applying a field rotating along the target axis with alternating +/- `omega`.
The magnitude `omega` is lower than the step out frequency to allow the rotation of the ABF.
This is the method used used in `app_analytic_control`.

## app_rl

Use smarties to find optimal policy for the problem stated in `app_analytic_control`.

## app_rl_comp

Use smarties to find optimal policy for the problem stated in `app_analytic_control` and compare to the `app_analytic_control` results for the same setup.

## app_rotating

ABF in a rotating field with direction changing direction over time (describes a circle).

