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
#include "qb_interface/quaternion.h"
#include "qb_interface/quaternionArray.h"
#include "qb_interface/inertialSensor.h"
#include "qb_interface/inertialSensorArray.h"

// Softfoot visualization includes
#include "softfoot_thing_visualization/madgwick_filter.h"

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

        // Function to compute perpendicular plane to a given vector
        bool compute_perpendiculars(Eigen::Vector3d in, Eigen::Vector3d &out_1, Eigen::Vector3d &out_2);

        // Function to compute the joint angle from pair of imu ids
        float compute_joint_state_from_pair(std::pair<int, int> imu_pair);

        // Function to compute the relative transforms for all pairs
        void compute_relative_trasforms(std::vector<std::pair<int, int>> imu_pairs);

        // Function to fill joint states with est. values and publish
        void fill_and_publish(std::vector<float> joint_values);

        // Callback to imu angles topic
        void imu_callback(const qb_interface::quaternionArray::ConstPtr &msg);

        // Callback to imu accelerations topic
        void acc_callback(const qb_interface::inertialSensorArray::ConstPtr &msg);

        // Callback to imu angular velocities topic
        void gyro_callback(const qb_interface::inertialSensorArray::ConstPtr &msg);


        // Auxiliary funtion for deg2rad conversion
        inline double deg2rad (double degrees) {
            static const double k_pi_on_180 = 4.0 * atan (1.0) / 180.0;
            return degrees * k_pi_on_180;
        }

        // Auxiliary funtion for rad2deg conversion
        inline double rad2deg (double radians) {
            static const double k_180_on_pi = 180.0 / 4.0 * atan (1.0);
            return radians * k_180_on_pi;
        }

        // ROS variables
        ros::NodeHandle je_nh_;
        ros::Subscriber sub_imu_;
        ros::Subscriber sub_imu_acc_;
        ros::Subscriber sub_imu_gyro_;
        ros::Publisher pub_js_;

        // Transform listener and stamped transform for lookupTransform
        tf::TransformListener tf_listener_;
        tf::StampedTransform stamped_transform_;

        // Madgwick filter
        MadgwickFilter mw_filter;

        // Poses and the mutex
        std::mutex imu_mutex_;          // Not used for now as everything is done in callback.
        std::vector<qb_interface::quaternion> imu_poses_;
        std::vector<qb_interface::inertialSensor> imu_acc_;
        std::vector<qb_interface::inertialSensor> imu_gyro_;
        std::vector<Eigen::Quaternion<float>> rel_poses_;

        // Joint variables
        std::vector<float> joint_values_;
        std::vector<float> joint_offset_;
        std::vector<std::pair<float, float>> joint_limits_;
        sensor_msgs::JointState joint_states_;

        // Constants
        std::string foot_name_;
        int foot_id_;
        std::string imu_topic_ = "/qb_class_imu/quat";
        std::string imu_topic_acc_ = "/qb_class_imu/acc";
        std::string imu_topic_gyro_ = "/qb_class_imu/gyro";

        // Parsed variables
        bool use_filter;
        std::vector<std::pair<int, int>> joint_pairs_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> joint_frame_names_;

};

}

#endif // JOINTS_ESTIMATOR_H