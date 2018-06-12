# object-localization
Object localization using particle filter algorithm - Udacity's Self-Driving Car Nanodegree.

## Description ##
Originally provided by Udacity

A robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

This project implements a 2 dimensional particle filter in C++. A particle filter is given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data. The main goal is to derive a robot position based on measurements and control data.

## Requirements ##
In order to successfully build and run the program, the following requirements should be fulfilled:
* cmake equal or above version 3.5
* make equal or above version 4.1ions
* gcc/g++ equal or above version 5.4

The software in this repository also requires the uWebSocketIO library which can be obtained [here](https://github.com/uNetworking/uWebSockets).

The contents of this repo were tested in Ubuntu Linux 18.04. It should work fine in Mac OS X once requirements are fulfilled. For Windows, please use Docker or other virtualization tools.

## Building and Running the code ##
In order to compile, please execute the following commands in a project folder:
1. `mkdir build` - do it once
2. `cmake ./` - do it once; eventually, you may need to delete `CMakeChache.txt`.
3. `./build.sh`

In order to execute the program:
* `./run.sh`

Program opens a port 4567 and wait for a connection of the simulator described below.

Note: `./clean.sh` - removes a build folder, use it with caution.

## Validation ##
Once compiled and launched, the program tries establish a connection with a simulator through the 4567 port. The Term 2 Simulator includes a graphical version of the Kidnapped Vehicle Project on a third screen. The simulator will provide a car path and landmark visualization as well as a validation of the particle filter algorithm performance.

The simulator can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

## License ##
All software included in this repository is licensed under MIT license terms. All additional programs and libraries have their owners and are distributed under their respective licenses. This repository contains Udacity's intellectual property. For any inquires on reuse or commercialization, please contact Udacity at [www.udacity.com](https://www.udacity.com/)
