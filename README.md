# Continuum Solver
A least-squares based solver for any continuously bending soft-robot driven by contraction.

For a technical explanation / writeup, see [here](https://bf01.notion.site/Reformulating-McKibben-Arm-Modelling-as-a-Least-Squares-Problem-84ed191f9c79442aa76b039cefe4606d)

## Code base
This code base primarily consists of three parts:
1. A library of Matlab functions that drive underlying Lie theory math (Exponential maps, constructing adjoints and pose matrices, etc)
2. A library of Matlab objects to handle the design and simulation of various continuously bending robot arms
3. A set of livescripts that utilizie the former two to create various simulations and animations of coninuously bending robot arms.

## Getting Started
1. Add the `/include` directory and all of its subdirectories to the MATLAB path.
2. Open `planar_arm.mlx`, `spatial_arm.mlx`, or `helical_arm.mlx`
3. Run the entire file to create a gif of a simulation of a robot arm.
