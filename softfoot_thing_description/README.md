# softfoot_thing_description

This package contains all the basics for the foot model in ROS.

## Getting Started

### Foot model
The basic robot model of the to be attatched to the leg of any legged robot (e.g. Anymal) is `softfoot_thing.urdf.xacro` which can be found in the folder `model`.

### Prerequisites

The parent repository of this package should be cloned and built in your catkin workspace.

### Running a simulation

Several gazebo simulation launch files are provided within this package. The `softfoot_thing_gazebo` package is exclusively for the plugin.

A fixed singel foot simulation in Gazebo can be launched as follows:

`roslaunch softfoot_thing_description pisa_softfoot.launch`

A floating (free to fall) single foot simulation can be launched by

`roslaunch softfoot_thing_description floating_softfoot.launch`

Finally, four floating (free to fall) feet simulation can be launched by

`roslaunch softfoot_thing_description four_softfoot.launch`
