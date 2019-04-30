/* JOINT ESTIMATOR NODE - Creates estimators for specified feet and publish joint sates for them.
Author: Mathew Jose Pollayil
Email:  mathewjose.pollayil@phd.unipi.it  */

#include "softfoot_thing_visualization/joints_estimator.h"

#define     DEBUG_JEN      0       // Prints out debug info

int main(int argc, char** argv) {
    
    ros::init (argc, argv, "softfoot_thing_visualization_joints_estimator");
    ros::NodeHandle nh;

    // Get needed params
    bool online_calib = false;
    if (!nh.getParam("softfoot_viz/calibrate_online", online_calib)) {
         ROS_INFO_STREAM("SoftFoot Joint Estimator : No specific request for calibration.");
    };

    // Joint estimator object
    softfoot_thing_visualization::JointsEstimator joint_estimator(nh, 3, "softfoot");

    ROS_INFO_STREAM("SoftFoot Joint Estimator : started.");

    // If needed calibrating after waiting for some time
    sleep(2);

    if (online_calib) {
        ROS_INFO_STREAM("SoftFoot Joint Estimator : starting to calibrate the sensing.");
        joint_estimator.calibrate();
        ROS_INFO_STREAM("SoftFoot Joint Estimator : calibration finished.");
    }

    // Start to spin the estimator
    ROS_INFO_STREAM("SoftFoot Joint Estimator : starting to spin");
    joint_estimator.spinEstimator();

    // Shutting down when finished
    ros::shutdown();
    return 0;
}