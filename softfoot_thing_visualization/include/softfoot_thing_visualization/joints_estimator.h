/*  
    JOINTS ESTIMATOR CLASS
    This object tries to estimate the joint angles of foot from IMU poses.
*/

#ifndef JOINTS_ESTIMATOR_H
#define JOINTS_ESTIMATOR_H

// ROS includes
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <string>
#include <mutex>

// MSG includes
#include <sensor_msgs/JointState.h>
#include "qb_interface/inertialSensor.h"
#include "qb_interface/inertialSensorArray.h"

// Softfoot visualization includes

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

        // Function that calibrates the sensing
        void calibrate();

        // Function that calibrates the sensing and saves the calibration data to yaml
        void calibrate_and_save(std::string file_name);

        // Function that spins the estimator
        bool check_calibration();

        // Function that estimates the joint angles
        bool estimate();

    private:

        // Function to parse parameters
        bool parse_parameters(ros::NodeHandle& nh);

        // Function to get joint limits
        bool get_joint_limits(ros::NodeHandle& nh);

        // Function to enforce joint limits
        void enforce_limits();

        // Function to correct the offset form estimated angles
        void correct_offset();

        // Function to echo transform from pair of frames
        Eigen::Affine3d getTransform(std::string frame_1, std::string frame_2);

        // Function to get the joint axes fron joint name
        Eigen::Vector3d get_joint_axis(std::string joint_name);

        // Function to compute perpendicular plane to a given vector
        bool compute_perpendiculars(Eigen::Vector3d in, Eigen::Vector3d &out_1, Eigen::Vector3d &out_2);

        // Function to compute the joint angle from pair of imu ids
        float compute_joint_state_from_pair(std::pair<int, int> imu_pair);

        // Function to fill joint states with est. values and publish
        void fill_and_publish(std::vector<float> joint_values);

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
        ros::AsyncSpinner spinner;
        ros::Subscriber sub_imu_acc_;
        ros::Subscriber sub_imu_gyro_;
        ros::Publisher pub_js_;

        // Initialization variables
        bool calibrated_ = false;

        // Transform listener and stamped transform for lookupTransform
        tf::TransformListener tf_listener_;
        tf::StampedTransform stamped_transform_;

        // qb readings and the mutex
        std::mutex imu_mutex_;                                      // Not used yet
        std::vector<qb_interface::inertialSensor> imu_acc_;         // Raw acceleration msg from qb
        std::vector<qb_interface::inertialSensor> imu_gyro_;        // Raw gyro msg from qb

        // Acceleration vectors
        std::vector<Eigen::Vector3d> acc_vec_0_;                    // Calibration acceleration
        std::vector<Eigen::Vector3d> acc_vec_;                      // Current acceleration
        std::vector<Eigen::Vector3d> acc_vec_olds_;                 // Previous acceleration

        // Joint variables
        std::vector<float> joint_values_;                           // Raw joint values
        std::vector<float> js_values_;                              // Joint states 
        std::vector<std::pair<float, float>> joint_limits_;         // parsed fron robot model
        sensor_msgs::JointState joint_states_;                      // Joint states message

        // Chain variables
        std::string chain_name_ = "middle_chain";

        // Constants
        int foot_id_;
        std::string foot_name_;
        std::string robot_name_;                                    // Foot name + id
        std::string pkg_path;                                       // Path to this package (parsed later)
        std::string imu_topic_ = "/qb_class_imu/quat";
        std::string imu_topic_acc_ = "/qb_class_imu/acc";
        std::string imu_topic_gyro_ = "/qb_class_imu/gyro";

        // Parsed variables
        std::vector<std::pair<int, int>> joint_pairs_;              // Pairs of imu ids for each joint
        std::vector<std::string> joint_names_;                      // Names of each joint
        std::vector<std::string> joint_frame_names_;                // Names of the frames of each joint
        std::vector<std::pair<Eigen::Vector3d, 
            Eigen::Vector3d>> axes_pairs_;                          // For each imu pair, the axis of sensor frame aligned with joint axis
        XmlRpc::XmlRpcValue je_params_;                             // For nested params

};

}

#endif // JOINTS_ESTIMATOR_H