/*  
    JOINTS ESTIMATOR CLASS
    This object tries to estimate the joint angles of foot from IMU poses.
*/

#include "softfoot_thing_visualization/joints_estimator.h"

#define     DEBUG_JE        1       // Prints out additional info

using namespace softfoot_thing_visualization;

JointsEstimator::JointsEstimator(ros::NodeHandle& nh , int foot_id, std::string foot_name){

    // Initializing main variables
    this->foot_id_ = foot_id;
    this->foot_name_ = foot_name;

    // Initializing ros variables
    this->je_nh_ = nh;
    this->sub_imu_ = this->je_nh_.subscribe<qb_interface::anglesArray>(this->imu_topic_, 1, &JointsEstimator::imu_callback, this);
    qb_interface::anglesArray::ConstPtr temp_msg = ros::topic::waitForMessage<qb_interface::anglesArray>(this->imu_topic_, ros::Duration(2.0));
    this->pub_js_ = this->je_nh_.advertise<sensor_msgs::JointState>
        ("/" + this->foot_name_ + "_" + std::to_string(this->foot_id_) + "/joint_states", 1);

    // Parsing needed parameters
    this->parse_parameters(this->je_nh_);

    // Temporarily building parsable variables here (TODO: parse them)
    this->joint_pairs_ = {{0, 1}, {0, 3}, {1, 2}};
    this->joint_names_ = {"front_arch_joint", "back_arch_joint", "roll_joint"};
    this->joint_frame_names_ = {"front_arch_link", "back_arch_link", "roll_link"};

}

JointsEstimator::~JointsEstimator(){

    // Nothing to do here

}

// Function to parse parameters
bool JointsEstimator::parse_parameters(ros::NodeHandle& nh){
    
    // TODO: parse needed params

}

// Function to echo transform from pair of frames
Eigen::Affine3d JointsEstimator::getTransform(std::string frame_1, std::string frame_2){
    // tf echoing using input frame names
    try {
		this->tf_listener_.waitForTransform(std::string("/") + frame_1,
      std::string("/") + frame_2, ros::Time(0), ros::Duration(1.0) );
		this->tf_listener_.lookupTransform(std::string("/") + frame_1,
      std::string("/") + frame_2, ros::Time(0), this->stamped_transform_);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // Converting to Affine3d
    Eigen::Affine3d affine;
    tf::Transform transform(this->stamped_transform_.getRotation(),
      this->stamped_transform_.getOrigin());
    tf::transformTFToEigen(transform, affine);

    return affine;
}

// Function to get the joint axes fron joint name
Eigen::Vector3d JointsEstimator::get_joint_axis(std::string joint_name){

    // Getting the transform to joint frame
    int pos = std::find(this->joint_names_.begin(), this->joint_names_.end(),
         joint_name) - this->joint_names_.begin();
    if (pos >= this->joint_names_.size()) {
        ROS_FATAL("JointsEstimator::get_joint_axis : You specified a joint name which is unknown to me!");
        this->je_nh_.shutdown();
    }
    Eigen::Affine3d joint_frame = this->getTransform("world", this->foot_name_ + "_" + std::to_string(this->foot_id_) + "_" + this->joint_frame_names_[pos]);

    // Getting the joint's axis in world frame (TODO: parse local axis)
    Eigen::Vector3d loc_axis;
    if (pos == 0) {
        loc_axis << 0, 1, 0;
    } else if (pos == 1) {
        loc_axis << 0, -1, 0;
    } else if (pos == 2) {
        loc_axis << 1, 0, 0;
    } else {
        ROS_FATAL("JointsEstimator::get_joint_axis : You specified a joint name which is unknown to me! But this should have not happened!!!");
        this->je_nh_.shutdown();
    }

    if (DEBUG_JE) {
        std::cout << "Got axis for " << joint_name << ": " << joint_frame.rotation() * loc_axis << std::endl;
        std::cout << "Frame rotation is " << joint_frame.rotation() << std::endl;
        std::cout << "Local axis is " << loc_axis << std::endl;
    }

    // Return joint axis in global frame
    return joint_frame.rotation() * loc_axis;

}

// Function to compute the joint angle from pair of imu ids
float JointsEstimator::compute_angles_from_pair(std::pair<int, int> imu_pair){

    // Getting joint name for current imu pair
    int pos = std::find(this->joint_pairs_.begin(), this->joint_pairs_.end(),
         imu_pair) - this->joint_pairs_.begin();
    if (pos >= this->joint_pairs_.size()) {
        ROS_FATAL("JointsEstimator::compute_angles_from_pair : You specified an imu pair which is unknown to me!");
        this->je_nh_.shutdown();
    }

    // Getting the joint axis for the joint
    Eigen::Vector3d tmp_axis = this->get_joint_axis(this->joint_names_[pos]);

}

// Callback to imu angles topic
void JointsEstimator::imu_callback(const qb_interface::anglesArray::ConstPtr &msg){
    
    // Get the imu angles of foot after clearing old angles
    this->imu_poses_.resize(4);
    for (int i = 0; i < msg->m.size(); i++) {
        if(msg->m[i].board_id == this->foot_id_){
            this->imu_poses_[msg->m[i].id] = msg->m[i];
        }
    }

    if (DEBUG_JE) {
        ROS_INFO_STREAM("Saved angles for foot " << this->foot_id_ << ": imus no: ");
        std::cout << "/ -";
        for (auto it : this->imu_poses_) {
            std::cout << it.id << " - ";
        }
        std::cout << "/" << std::endl;
    }

    // Temporarily checking an angle for a pair
    float tmp_angle = this->compute_angles_from_pair(this->joint_pairs_[0]);

}