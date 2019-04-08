/*  
    JOINTS ESTIMATOR CLASS
    This object tries to estimate the joint angles of foot from IMU poses.
*/

#include "softfoot_thing_visualization/joints_estimator.h"

#define     DEBUG_JE        1       // Prints out additional info
#define     DEBUG_ANGLES    1       // Prints out only estimated angles

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

    // Debug print out
    if (DEBUG_JE) {
        ROS_INFO_STREAM("Got axis for " << joint_name << ": \n" << joint_frame.rotation() * loc_axis);
        ROS_INFO_STREAM("Frame rotation is \n" << joint_frame.rotation());
        ROS_INFO_STREAM("Local axis is \n" << loc_axis << "\n");
    }

    // Return joint axis in global frame
    return joint_frame.rotation() * loc_axis;

}

// Function to compute perpendicular plane to a given vector
bool JointsEstimator::compute_perpendiculars(Eigen::Vector3d in, Eigen::Vector3d &out_1,
    Eigen::Vector3d &out_2){

    // If input vector is near to null, return
    if((in - Eigen::Vector3d::Zero()).isMuchSmallerThan(0.0001)) return false;
    
    Eigen::Vector3d v; Eigen::Vector3d w;
    // Checking if the input vector is on some main plane, else get traditional perpendicular
    if (std::abs(in(0) - 0.0) < 0.0001) {
        v << 1, 0, 0;
    } else if (std::abs(in(1) - 0.0) < 0.0001) {
        v << 0, 1, 0;
    } else if (std::abs(in(2) - 0.0) < 0.0001) {
        v << 0, 0, 1;
    } else {
        v << -in(1), in(0), in(2);
    }

    // Computing the next vector normal to both in and w
    w = in.cross(v);

    // Normalizing v and w and retuning
    v.normalize(); w.normalize();
    out_1 = v; out_2 = w;

    // Debug print out
    if (DEBUG_JE) {
        ROS_INFO_STREAM("The normals to \n" << in << "\n are \n" << out_1 << "\n and \n" << out_2 << "\n");
    }

    return true;
}

// Function to compute the joint angle from pair of imu ids
float JointsEstimator::compute_joint_state_from_pair(std::pair<int, int> imu_pair){

    // 1) Getting joint name for current imu pair
    int pos = std::find(this->joint_pairs_.begin(), this->joint_pairs_.end(),
         imu_pair) - this->joint_pairs_.begin();
    if (pos >= this->joint_pairs_.size()) {
        ROS_FATAL("JointsEstimator::compute_joint_state_from_pair : You specified an imu pair which is unknown to me!");
        this->je_nh_.shutdown();
    }

    // 2) Getting the normalized joint axis for the joint
    Eigen::Vector3d tmp_axis = this->get_joint_axis(this->joint_names_[pos]);
    tmp_axis.normalize();

    // 3) Getting the plane normal to joint axis
    Eigen::Vector3d perp_1; Eigen::Vector3d perp_2;
    if(!this->compute_perpendiculars(tmp_axis, perp_1, perp_2)){
        ROS_FATAL_STREAM("Could not compute the normal plane to the joint axis of " << this->joint_names_[pos]);
        this->je_nh_.shutdown();
    }

    // 4) Trasforming one of the normals using the relative rotation of the imu pair
    Eigen::Vector3d transformed = this->rel_poses_[pos] * perp_1;

    // Debug print out
    if (DEBUG_JE) {
        ROS_INFO_STREAM("The rotated vector to \n" << perp_1 << "\n is \n" << transformed << "\n");
    }

    // 5) Re-projecting it to the normal plane
    Eigen::Vector3d projected = transformed - (transformed.dot(tmp_axis) * tmp_axis);
    projected.normalize();

    // Debug print out
    if (DEBUG_JE) {
        ROS_INFO_STREAM("The normalized re-projected vector is \n" << projected << "\n");
    }

    // 6) Computing the angle between the initial normal and the rotated - projected one
    double sin_js = (perp_1.cross(projected)).norm() / (perp_1.norm() * projected.norm());
    double cos_js = perp_1.dot(projected);
    float js = (float) atan2(sin_js, cos_js);
    // float js = (float) acos((double) perp_1.dot(projected));

    // 7) Return the found joint state
    return js;

}

// Function to compute transform from rpy (as there seem to be no prebuilt function in Eigen)
Eigen::Matrix3d JointsEstimator::create_rotation_matrix(double ax, double ay, double az){
  
  // Create elementary rpy, then compose and return
  Eigen::Matrix3d rx =
      Eigen::Matrix3d(Eigen::AngleAxisd(this->deg2rad(ax), Eigen::Vector3d(1, 0, 0)));
  Eigen::Matrix3d ry =
      Eigen::Matrix3d(Eigen::AngleAxisd(this->deg2rad(ay), Eigen::Vector3d(0, 1, 0)));
  Eigen::Matrix3d rz =
      Eigen::Matrix3d(Eigen::AngleAxisd(this->deg2rad(az), Eigen::Vector3d(0, 0, 1)));

  return rz * ry * rx;

}

// Function to compute the relative transforms for all pairs
void JointsEstimator::compute_relative_trasforms(std::vector<std::pair<int, int>> imu_pairs){
    
    // Running through pairs and computing relative transforms
    this->rel_poses_.clear();
    Eigen::Matrix3d pose_1; Eigen::Matrix3d pose_2;
    for (auto it : imu_pairs) {
        pose_1 = this->create_rotation_matrix(this->imu_poses_.at(it.first).r,
            this->imu_poses_.at(it.first).p, this->imu_poses_.at(it.first).y);
        pose_2 = this->create_rotation_matrix(this->imu_poses_.at(it.second).r,
            this->imu_poses_.at(it.second).p, this->imu_poses_.at(it.second).y);
        this->rel_poses_.push_back(pose_1.transpose() * pose_2);
    }

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

    // Debug print out
    if (DEBUG_JE) {
        std::cout << "\n *-----------* \n IMU messages recieved \n *-----------* \n";
        ROS_INFO_STREAM("Saved angles for foot " << this->foot_id_ << ": imus no: ");
        std::cout << "/ -";
        for (auto it : this->imu_poses_) {
            std::cout << it.id << " - ";
        }
        std::cout << "/\n" << std::endl;
    }

    // Computing all relevant transforms between imu pairs
    this->compute_relative_trasforms(this->joint_pairs_);

    if (this->joint_pairs_.size() != this->rel_poses_.size()) {
        ROS_FATAL("Number of joints and relative trasforms do not match! This should not happen!");
        this->je_nh_.shutdown();
    }

    // Debug print out
    if (DEBUG_JE) {
        ROS_INFO_STREAM("Saved relative trasforms between imu pairs: ");
        std::cout << "---\n";
        for (auto it : this->rel_poses_) {
            std::cout << it << std::endl;
            std::cout << "--" << std::endl;
        }
        std::cout << "---\n" << std::endl;
    }
    
    // Temporarily checking an angle for a pair
    float tmp_angle = this->compute_joint_state_from_pair(this->joint_pairs_[0]);
    if (DEBUG_ANGLES) {
        ROS_INFO_STREAM("The estimated joint angle for imu pair (" << this->joint_pairs_[0].first 
            << ", " << this->joint_pairs_[0].second << ") is " << this->rad2deg(tmp_angle) << "degs \n");
    }

}