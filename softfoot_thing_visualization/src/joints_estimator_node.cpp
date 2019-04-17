#include "softfoot_thing_visualization/joints_estimator.h"

#define     DEBUG_JEN      0       // Prints out debug info

int main(int argc, char** argv) {
    
    ros::init (argc, argv, "softfoot_thing_visualization_joints_estimator");
    ros::NodeHandle nh;

    // Joint estimator object
    softfoot_thing_visualization::JointsEstimator joint_estimator(nh, 3, "softfoot");

    ROS_INFO_STREAM("SoftFoot Joint Estimator : started. Please ensure that the feet are in neutral position.");

    // Calibrating after waiting for some time
    sleep(2);

    ROS_INFO_STREAM("SoftFoot Joint Estimator : starting to calibrate the sensing.");

    joint_estimator.calibrate();

    // Start to spin the estimator
    ROS_INFO_STREAM("SoftFoot Joint Estimator : calibration finished. Starting to spin");

    joint_estimator.spinEstimator();

    // Shutting down when finished
    ros::shutdown();
    return 0;
}