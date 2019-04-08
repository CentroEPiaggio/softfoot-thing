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
#include <mutex>

// MSG includes
#include <sensor_msgs/JointState.h>
#include "qb_interface/angles.h"
#include "qb_interface/anglesArray.h"

// Other includes
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

namespace softfoot_thing_visualization {

class JointsEstimator {

    public:

        JointsEstimator(ros::NodeHandle& nh, int foot_id, std::string foot_name);

        ~JointsEstimator();

    private:

        // Function to parse parameters
        bool parse_parameters(ros::NodeHandle& nh);

        // Function to echo transform from pair of frames
        Eigen::Affine3d getTransform(std::string frame_1, std::string frame_2);

        // Function to get the joint axes fron joint name
        Eigen::Vector3d get_joint_axis(std::string joint_name);

        // Function to compute transform from rpy (as there seem to be no prebuilt function in Eigen)
        Eigen::Matrix3d create_rotation_matrix(double ax, double ay, double az);

        // Function to compute perpendicular plane to a given vector
        bool compute_perpendiculars(Eigen::Vector3d in, Eigen::Vector3d &out_1, Eigen::Vector3d &out_2);

        // Function to compute the joint angle from pair of imu ids
        float compute_joint_state_from_pair(std::pair<int, int> imu_pair);

        // Function to compute the relative transforms for all pairs
        void compute_relative_trasforms(std::vector<std::pair<int, int>> imu_pairs);

        // Callback to imu angles topic
        void imu_callback(const qb_interface::anglesArray::ConstPtr &msg);

        // ROS variables
        ros::NodeHandle je_nh_;
        ros::Subscriber sub_imu_;
        ros::Publisher pub_js_;

        // Transform listener and stamped transform for lookupTransform
        tf::TransformListener tf_listener_;
        tf::StampedTransform stamped_transform_;

        // Poses and the mutex
        std::mutex imu_mutex_;          // Not used for now as everything is done in callback.
        std::vector<qb_interface::angles> imu_poses_;
        std::vector<Eigen::Matrix3d> rel_poses_;

        // Messages
        sensor_msgs::JointState joint_states_;

        // Constants
        std::string foot_name_;
        int foot_id_;
        std::string imu_topic_ = "/qb_class_imu/angles";

        // Parsed variables
        std::vector<std::pair<int, int>> joint_pairs_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> joint_frame_names_;

};

}

#endif // JOINTS_ESTIMATOR_H