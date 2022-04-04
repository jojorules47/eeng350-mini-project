# MATLAB Simulation

Motor controls simulation and verification

## Published Documentation

Published MATLAB scripts can be found in `docs/` as PDFs, showing 
scripts and data.

## MATLAB Scripts

All scripts, data, and Simulink models used can be found in `scripts/`,
and were written in MATLAB Online **R2021b**.

`motor_function.m`: Analyze real data, and create approximated open loop
velocity, as well as closed loop PI controlled results.

`PI_tester.m`: Verify discrete time controller results, and compare to
MATLAB continuous PI controller. The **Arduino Controller** block
contains the example script found in the Mini Project Instructions.

`run_camera_sim.m`: Verify controls models for the robot. Currently differs from
our Arduino controls implementation, so results are not accurate.
