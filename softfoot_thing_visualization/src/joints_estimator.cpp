/*  
    JOINTS ESTIMATOR CLASS
    This object tries to estimate the joint angles of foot from IMU poses.
*/

#include "softfoot_thing_visualization/joints_estimator.h"

#define     DEBUG_JE        0       // Prints out additional info

using namespace softfoot_thing_visualization;

JointsEstimator::JointsEstimator(ros::NodeHandle& nh , int foot_id){

    // Initializing main variables
    this->foot_id_ = foot_id;
    this->je_nh_ = nh;
    this->sub_imu_ = this->je_nh_.subscribe<qb_interface::anglesArray>(this->imu_topic_, 1, &JointsEstimator::imu_callback, this);
    qb_interface::anglesArray::ConstPtr temp_msg = ros::topic::waitForMessage<qb_interface::anglesArray>(this->imu_topic_, ros::Duration(2.0));
    this->pub_js_ = this->je_nh_.advertise<sensor_msgs::JointState>("/softfoot_" + std::to_string(this->foot_id_) + "/joint_states", 1);

}

JointsEstimator::~JointsEstimator(){

    // Nothing to do here

}

// Callback to joint_states topic
void JointsEstimator::imu_callback(const qb_interface::anglesArray::ConstPtr &msg){

}