# msode apps

a set of apps for simple cases with ABFs in a magnetic field

## app_forward

ABF in a field rotating with frequency `omega` in the yz plane.

## app_forward_curve

ABF in a field rotating with frequencies `omega` in the yz plane.
report mean velocity along the x axis for many omegas.

## app_rotating

ABF in a rotating field with direction changing direction over time (describes a circle).

## app_analytic_control

## app_orient

A small case for reorienting randomly oriented ABFs along a given axis.
The method for reorienting consists in applying a field rotating along the target axis with alternating +/- omega.
The magnitude omega is lower than the step out frequency to allow the rotation of the ABF.
This is the method used used in app_analytic_control.
