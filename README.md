# softfoot-thing

This repository contains all the required packages for using the SoftFoot for Thing EU project.

![SoftFootV2](https://github.com/CentroEPiaggio/softfoot-thing/blob/master/images/softfoot-gazebo.png)

## Packages

* **softfoot_thing** - metapackage
* **softfoot_thing_description** - foot meshes, xacros and urdf
* **softfoot_thing_gazebo** - simulation plugin
* **softfoot_thing_visualization** - real time visualization using imus

More details about the individual packages can be found in the respective READMEs.

## Getting Started

### Prerequisites

The above packages are tested on ROS Melodic and their main external dependencies are the following:

* [`NMMI/ROS-base`](https://github.com/NMMI/ROS-base) (branch `master`)
* [`NMMI/ROS-NMMI`](https://github.com/NMMI/ROS-NMMI) (branch `foot-devel`)

If you wish to use the old communication package with the IMUs, instead of the above packages use the following:

* [`IMU`](https://github.com/NMMI/IMU) (branch `master`)

### Installing

To install the packages in this repo just clone it (branch `master`) into your catkin_ws, make sure to clone also the external dependencies and then `catkin build`.
