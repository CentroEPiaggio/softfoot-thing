#include "softfoot_thing_visualization/joints_estimator.h"

#define     DEBUG_JEN      0       // Prints out debug info

int main(int argc, char** argv) {
    
    ros::init (argc, argv, "softfoot_thing_visualization_joints_estimator");
    ros::NodeHandle nh;

    // Joint estimator object
    softfoot_thing_visualization::JointsEstimator joint_estimator(nh, 2);

    // Starting to spin
    ros::spin();

    // Shutting down when finished
    ros::shutdown();
    return 0;
}