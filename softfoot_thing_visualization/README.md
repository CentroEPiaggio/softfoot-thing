# softfoot_thing_visualization

This package contains all the necessary utilities to work on ROS with a real SoftFoot. It provides basically two ros nodes for
* Calibration - Reads IMU measurements from a foot in stable position and saves a `.yaml` file with the necessary details.
* Visualization - Calibrates the foot on the fly or loads pre-existing calibration data and estimates current joint states from IMU measurements.

### Prerequisites

The parent repository of this package should be cloned and built in your catkin workspace.

### Calibrating a SoftFoot

The feet are provided with default calibration files which are more than enough for good estimation. Re-calibration is required only in extreme cases (e.g. if foot breaks and IMUs are mounted differently).

Connect the foot to your computer (one foot at a time), keep the foot on a flat surface and hold the leg (tube) perpendicular to the flat surface. Then launch the following, in order:

`roslaunch softfoot_thing_visualization pisa_softfoot_calibration_base.launch`
`roslaunch softfoot_thing_visualization pisa_softfoot_calibration.launch`

You will be asked to type the **Name** and **ID** of the foot. A good rule would be to give as **Name** a basic identifier (e.g. softfoot or footthing), which will be the same for all feet of the robot, and the **ID** should be the `board_id` of the real foot.

A file named `Name_ID.yaml` file will be saved to the `config` folder of this package. Then, make sure to add it to your foot visualization launch file so to load its parameters into the ROS parameter server.

### Visualizing SoftFeet on RViz

#### Prerequisites

##### Setup the foot on your robot

Setting up the robot model properly is vital for the estimated joint states to be published into the correct topic so that they can be sourced by the `joint_state_publisher`.

> For example:
> Suppose you calibrated a SoftFoot with **Name** = softfoot and **ID** = 4.
> When adding the model of that particular foot to the urdf.xacro of your robot, make sure to give that xacro the name softfoot_4.
> ```xml
> <xacro:softfoot_thing name="softfoot_4" parent="<parent-link>">
>     <origin xyz="0 0 0" rpy="0 0 0"/>
> </xacro:softfoot_thing>
> ```

##### Specify the visualization parameters

The file `pisa_softfoot_viz.yaml` in the folder `config` of this package gives the user some control over the visualization of the feet. Most of the parameters therewithin are self explanatory, but the following table explains them for the sake of completeness.

| Parameter             | Type          | Description  |
| ----------------------|:-------------:| -------------|
| `calibrate_online`    | `bool`        | If `true` an online calibration will be performed for all the feet name specified in `connected_feet_name` with ids specified in `connected_feet_ids`. |
| `use_filter`          | `bool`        | If `true` the acceleration and gyroscope measurements will be prelimilarly smoothed using the low pass filter defined by the parameter `low_pass_filter`. |
| `use_gyro`            | `bool`        | If `true` the gyro measurements will be integrated and fused to the joint angle estimations from accelerations through a complementary filter. |
| `publish_leg_pose`    | `bool`        | If `true` a raw estimate of the leg pose will be published to the joint states. This might not be useful in case the leg angle of your robot is already provided by other means. |
| `connected_feet_name` | `string`      | A sting containing the basic identifier of the feet of in your robot model (**Name**). |
| `connected_feet_ids`  | `int[]`       | An array of integers containing the IDs of the feet of in your robot model (**ID**). |

Some other parameters can also be found in the configuration file, but these are relative to future work and thus can be safely ignored.
