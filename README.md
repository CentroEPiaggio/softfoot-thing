# softfoot-thing

This repository contains all required packages to use the SoftFoot for Thing EU project.

## Packages

* **softfoot_thing** - metapackage
* **softfoot_thing_description** - foot meshes, xacros and urdf
* **softfoot_thing_gazebo** - simulation plugin
* **softfoot_thing_visualization** - real time visualization using imus

More details about the individual packages can be found in the respective READMEs.

## Getting Started

### Prerequisites

The above packages are tested on ROS Melodic and their main external dependencies are the following:

* [`IMU`](https://github.com/NMMI/IMU) (branch `master`)

### Installing

To install the packages in this repo just clone it into your catkin_ws, make sure to clone also the external dependencies and `catkin build`.
