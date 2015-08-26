# Matlab_simulator

Checkout a branch for a chapter that you want to use! or keep reading.

## What is in this repo?

This repo contains the matlab and simulink simulator that is a result of doing the homework from the flight dynamics class associated with the [Small Unmaned Aircraft](http://uavbook.byu.edu/doku.php) book by Beard and McLain.  Each branch in the repo contains the finished homework from the chapter that it is named for. It also uses contains tools for porting the autopilot portion of the simulator to the pixhawk autopilot.  

## Wait it contains finished homework, and its posted online?

Yeah, it does, but its by no means perfect and could be imporved.  If you think you want to use this stuff for an autopilot then your better off doing the assignment on your own and comparing with what is here.  You'll learn more and have an easyer time.  If you want to turn this in as your assignment, well I guess there isn't anything to stop you, but you have to live with your own conscience. 

## How do I use the simulator?

The simulator used by first running the params_chap#.m file in matlab to create a constants/parameter struct "P".  This sturct is used throughout the simulator.  The simulator is run by opening the simulink block diagram named mavsim_chap#.mdl and clicking run.

## How is this used with the other repo's?

This matlab simulator is set up to be used as a hardware in the loop (HIL) and/or software in the loop (SIL) simulator.  Both of which can be used as development tools for migrating this simulator to an actual autopilot.

## HIL and/or SIL sounds great how do I use them?

Each brach contains a file called simulation_options.mdl.  This file cannot be run by itself but the blocks that it contain can run different levels of simulation; matlab simulation, SIL, or HIL.  Both SIL and HIL blocks run compiled mex funtions or that are built from the [SIL](https://github.com/MAGICC-UAVbook/SIL) and [MATLink](https://github.com/MAGICC-UAVbook/MATLink) repositories respectively. Keep in mind that the code from these repos need to be built and placed on the matlab path by typing "addpath ../build_SIL" or something similar.  Refer to the other repositories for more info.
