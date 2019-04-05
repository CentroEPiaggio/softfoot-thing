/*  
    JOINTS ESTIMATOR CLASS
    This object tries to estimate the joint angles of foot from IMU poses.
*/

#ifndef JOINTS_ESTIMATOR_H
#define JOINTS_ESTIMATOR_H

// ROS includes
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <string>

// MSG includes
#include <sensor_msgs/JointState.h>
#include "qb_interface/anglesArray.h"

// Other includes
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

namespace softfoot_thing_visualization {

class JointsEstimator {

    public:

        JointsEstimator(ros::NodeHandle& nh, int foot_id);

        ~JointsEstimator();

    private:

        // Function to parse parameters
        bool parse_parameters(ros::NodeHandle& nh);

        // Callback to joint_states topic
        void imu_callback(const qb_interface::anglesArray::ConstPtr &msg);

        // Function to initialize all main variables
        bool compute_relative_poses();

        // ROS variables
        ros::NodeHandle je_nh_;
        ros::Subscriber sub_imu_;
        ros::Publisher pub_js_;

        // Relative poses

        // Messages
        sensor_msgs::JointState joint_states_;
        qb_interface::anglesArray imu_poses_;

        // Constants
        int foot_id_;
        std::string imu_topic_ = "/qb_class_imu/angles";

        // Parsed variables

};

}

#endif // JOINTS_ESTIMATOR_H