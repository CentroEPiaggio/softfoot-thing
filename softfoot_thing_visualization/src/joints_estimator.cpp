/*  
    JOINTS ESTIMATOR CLASS
    This object tries to estimate the joint angles of foot from IMU poses.
*/

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "softfoot_thing_visualization/joints_estimator.h"

#define     DEBUG_JE        0       // Prints out additional info about the Object
#define     DEBUG_CB        0       // Prints out only info about callbacks execution
#define     DEBUG_JS        0       // Prints out info about estimated joint states
#define     DEBUG_PARSED    0       // Prints out info about parsed stuff
#define     DEBUG_ANGLES    0       // Prints out only raw estimated angles

#define     PI              3.1415
#define     _OFFSET_STEP_   200     // Number of steps for offset computation

using namespace softfoot_thing_visualization;

JointsEstimator::JointsEstimator(ros::NodeHandle& nh , int foot_id, std::string foot_name){

    // Initializing main variables
    this->foot_id_ = foot_id;
    this->foot_name_ = foot_name;
    this->robot_name_ = this->foot_name_ + "_" + std::to_string(this->foot_id_);

    // Initializing ros variables
    this->je_nh_ = nh;

    this->sub_imu_acc_ = this->je_nh_.subscribe<qb_interface::inertialSensorArray>(this->imu_topic_acc_, 1, &JointsEstimator::acc_callback, this);
    qb_interface::inertialSensorArray::ConstPtr temp_msg_2 = ros::topic::waitForMessage<qb_interface::inertialSensorArray>(this->imu_topic_acc_, ros::Duration(2.0));
    
    this->sub_imu_gyro_ = this->je_nh_.subscribe<qb_interface::inertialSensorArray>(this->imu_topic_gyro_, 1, &JointsEstimator::gyro_callback, this);
    qb_interface::inertialSensorArray::ConstPtr temp_msg_3 = ros::topic::waitForMessage<qb_interface::inertialSensorArray>(this->imu_topic_gyro_, ros::Duration(2.0));
    
    this->sub_imu_ = this->je_nh_.subscribe<qb_interface::quaternionArray>(this->imu_topic_, 1, &JointsEstimator::imu_callback, this);
    qb_interface::quaternionArray::ConstPtr temp_msg_1 = ros::topic::waitForMessage<qb_interface::quaternionArray>(this->imu_topic_, ros::Duration(2.0));
    
    this->pub_js_ = this->je_nh_.advertise<sensor_msgs::JointState>
        ("/" + this->foot_name_ + "_" + std::to_string(this->foot_id_) + "/joint_states", 1);

    // Debud publishers
    this->pub_rel_rpy_ = this->je_nh_.advertise<geometry_msgs::QuaternionStamped>
        ("/" + this->foot_name_ + "_" + std::to_string(this->foot_id_) + "/debug_rel_rpy", 1);

    // Initializing the Madgwick filter
    this->mw_filter.initialize(this->je_nh_);

    // Temporarily building parsable variables here (TODO: parse them)
    this->use_filter = true;
    this->joint_pairs_ = {{0, 1}, {0, 3}, {1, 2}};
    this->joint_names_ = {"front_arch_joint", "back_arch_joint", "roll_joint"};
    this->joint_frame_names_ = {"front_arch_link", "back_arch_link", "roll_link"};
    this->joint_offset_ = {1.90, 1.98, 2.74};

    // Parsing needed parameters
    if (!this->parse_parameters(this->je_nh_)) {
        ROS_FATAL("JointsEstimator::JointsEstimator : Could not get parameters for building estimator!");
        this->je_nh_.shutdown();
    }

    // Filling up main parts of the joint state msg and setting size of values
    for (auto it : this->joint_names_) {
        this->joint_states_.name.push_back(this->foot_name_ + "_" + std::to_string(this->foot_id_) + "_" + it);
        this->joint_states_.position.push_back(0.0);
    }
    this->joint_values_.resize(this->joint_pairs_.size());
    this->js_values_.resize(this->joint_pairs_.size());

    // Setting relative poses to identity at the beginning
    if (this->use_filter) {
        this->rel_poses_.clear();
        for (int j = 0; j < this->joint_pairs_.size(); j++) {
            this->rel_poses_.push_back(Eigen::Quaternion<float>(1.0, 0.0, 0.0, 0.0));
        }
        this->rel_poses_joints_.clear();
        for (int j = 0; j < this->joint_pairs_.size(); j++) {
            this->rel_poses_joints_.push_back(Eigen::Quaternion<float>(1.0, 0.0, 0.0, 0.0));
        }
        this->rel_poses_off_.clear();
        for (int j = 0; j < this->joint_pairs_.size(); j++) {
            this->rel_poses_off_.push_back(Eigen::Quaternion<float>(1.0, 0.0, 0.0, 0.0));
        }
    } else {
        this->offset_computed_ = true;
    }

    // Debug print out
    if (DEBUG_JE) {
        ROS_INFO_STREAM("Starting relative quaternions between imu pairs: ");
        std::cout << "---\n";
        for (auto it : this->rel_poses_) {
            std::cout << it.toRotationMatrix().eulerAngles(2, 1, 0) << std::endl;
            std::cout << "--" << std::endl;
        }
        std::cout << "---\n" << std::endl;
        ROS_INFO_STREAM("Starting relative_offset quaternions between imu pairs: ");
        std::cout << "---\n";
        for (auto it : this->rel_poses_off_) {
            std::cout << it.toRotationMatrix().eulerAngles(2, 1, 0) << std::endl;
            std::cout << "--" << std::endl;
        }
        std::cout << "---\n" << std::endl;
        ROS_INFO_STREAM("Starting relative_joints quaternions between imu pairs: ");
        std::cout << "---\n";
        for (auto it : this->rel_poses_joints_) {
            std::cout << it.toRotationMatrix().eulerAngles(2, 1, 0) << std::endl;
            std::cout << "--" << std::endl;
        }
        std::cout << "---\n" << std::endl;
    }

    // Setting old accelerations to null
    this->acc_1_olds_.resize(this->joint_pairs_.size());
    this->acc_2_olds_.resize(this->joint_pairs_.size());
    for (int i = 0; i < this->joint_pairs_.size(); i++) {
        this->acc_1_olds_[i] = Eigen::Vector3d::Zero();
        this->acc_2_olds_[i] = Eigen::Vector3d::Zero();
    }

    // Computing offsets
    // this->compute_initial_offset();

}

JointsEstimator::~JointsEstimator(){

    // Nothing to do here

}

// Function to initially compute offsets before starting to spin
void JointsEstimator::compute_initial_offset(){

    // Compute offsets
    for (int k=0; k < _OFFSET_STEP_; k++) {			

		// Creating the necessary inputs of the filter
        Eigen::Vector3d acc_1, gyro_1, acc_2, gyro_2;

        ros::spinOnce();

        int i = 0;
        for (auto it : this->joint_pairs_) {

            // Setting acceleration and angular velocity of pair
            acc_1 << this->imu_acc_.at(it.first).x, this->imu_acc_.at(it.first).y, this->imu_acc_.at(it.first).z;
            gyro_1 << this->imu_gyro_.at(it.first).x, this->imu_gyro_.at(it.first).y, this->imu_gyro_.at(it.first).z;
            acc_2 << this->imu_acc_.at(it.second).x, this->imu_acc_.at(it.second).y, this->imu_acc_.at(it.second).z;
            gyro_2 << this->imu_gyro_.at(it.second).x, this->imu_gyro_.at(it.second).y, this->imu_gyro_.at(it.second).z;

            // Filter the quaternion
            this->rel_poses_off_[i] = this->mw_filter.filter(acc_1, gyro_1, acc_2, gyro_2, this->acc_1_olds_[i],
                this->acc_2_olds_[i], this->rel_poses_off_[i]);

            // Save olds and increase index
            this->acc_1_olds_[i] = acc_1;
            this->acc_2_olds_[i] = acc_2;
            i++;

        }
		
		// Set the relative pose and joint relative pose to the offset
        this->rel_poses_ = this->rel_poses_off_;
        this->rel_poses_joints_ = this->rel_poses_off_;

        // Debug print out
        if (DEBUG_JE) {
            ROS_INFO_STREAM("Saved offset quaternions between imu pairs: ");
            std::cout << "---\n";
            for (auto it : this->rel_poses_off_) {
                std::cout << it.toRotationMatrix().eulerAngles(2, 1, 0) << std::endl;
                std::cout << "--" << std::endl;
            }
            std::cout << "---\n" << std::endl;
        }
	}

    // Signal that offset has been computed
    this->offset_computed_ = true;

}

// Function to run in a loop to estimate the joint states
void JointsEstimator::estimate_joint_states(){

    if (this->offset_computed_) {

        // Computing all relevant transforms between imu pairs
        if (!this->use_filter) {
            this->compute_relative_trasforms(this->joint_pairs_);
        } else {
            // Use madwick filter
            this->filter_relative_trasforms(this->joint_pairs_);
        }

        if (this->joint_pairs_.size() != this->rel_poses_.size()) {
            ROS_FATAL("Number of joints and relative trasforms do not match! This should not happen!");
            this->je_nh_.shutdown();
        }

        // Debug print out
        if (DEBUG_JE) {
            ROS_INFO_STREAM("Saved relative quaternions between imu pairs: ");
            std::cout << "---\n";
            for (auto it : this->rel_poses_) {
                std::cout << it.toRotationMatrix().eulerAngles(2, 1, 0) << std::endl;
                std::cout << "--" << std::endl;
            }
            std::cout << "---\n" << std::endl;
        }
        if (DEBUG_JE) {
            ROS_INFO_STREAM("Saved relative quaternions (no offset) between imu pairs: ");
            std::cout << "---\n";
            for (auto it : this->rel_poses_joints_) {
                std::cout << it.toRotationMatrix().eulerAngles(2, 1, 0) << std::endl;
                std::cout << "--" << std::endl;
            }
            std::cout << "---\n" << std::endl;
        }

        // Publish debug relative poses to topic
        this->debug_rel_rpy_.header.stamp = ros::Time::now();
        this->debug_rel_rpy_.quaternion.x = this->rel_poses_[0].x();
        this->debug_rel_rpy_.quaternion.y = this->rel_poses_[0].y();
        this->debug_rel_rpy_.quaternion.z = this->rel_poses_[0].z();
        this->debug_rel_rpy_.quaternion.w = this->rel_poses_[0].w();
        this->pub_rel_rpy_.publish(this->debug_rel_rpy_);

        // Estimating the angles
        for (int i = 0; i < this->joint_pairs_.size(); i++) {
            this->joint_values_[i] = this->compute_joint_state_from_pair(this->joint_pairs_[i]);
        }

        // Filling up jointstates and publishing
        this->fill_and_publish(this->joint_values_);

        if (DEBUG_ANGLES) {
            ROS_INFO_STREAM("The estimated raw joint angle for imu pair (" << this->joint_pairs_[0].first 
                << ", " << this->joint_pairs_[0].second << ") is " << this->joint_values_[0] << "rads \n");
            ROS_INFO_STREAM("The estimated raw joint angle for imu pair (" << this->joint_pairs_[1].first 
                << ", " << this->joint_pairs_[1].second << ") is " << this->joint_values_[1] << "rads \n");
            ROS_INFO_STREAM("The estimated raw joint angle for imu pair (" << this->joint_pairs_[2].first 
                << ", " << this->joint_pairs_[2].second << ") is " << this->joint_values_[2] << "rads \n");
        }

        if (DEBUG_JS) {
            ROS_INFO_STREAM("The estimated joint state for imu pair (" << this->joint_pairs_[0].first 
                << ", " << this->joint_pairs_[0].second << ") is " << this->js_values_[0] << "rads \n");
            ROS_INFO_STREAM("The estimated joint state for imu pair (" << this->joint_pairs_[1].first 
                << ", " << this->joint_pairs_[1].second << ") is " << this->js_values_[1] << "rads \n");
            ROS_INFO_STREAM("The estimated joint state for imu pair (" << this->joint_pairs_[2].first 
                << ", " << this->joint_pairs_[2].second << ") is " << this->js_values_[2] << "rads \n");
        }
    }

    ROS_INFO_STREAM("Saved relative quaternions between imu pairs: ");
    std::cout << "---\n";
    std::cout << this->rel_poses_[0].toRotationMatrix().eulerAngles(2, 1, 0) << std::endl;
    std::cout << "---\n" << std::endl;

}

// Function to parse parameters
bool JointsEstimator::parse_parameters(ros::NodeHandle& nh){
    
    // Parsing jont offsets
    if (!nh.getParam("softfoot_viz/" + this->robot_name_ + "/joint_offset", this->joint_offset_)) {
        ROS_FATAL("Cannot find the joint offsets, shutting down!");
        return false;
    }

    // TODO: parse needed params

    // Parsing joint limits of the foot (joint_names_ needs to be set before)
    if (!this->get_joint_limits(nh)) {
        ROS_FATAL("Unable to get the joint limits for %s!", this->robot_name_.c_str());
        return false;
    }

    // Everything parsed correctly
    return true;

}

// Function to get joint limits
bool JointsEstimator::get_joint_limits(ros::NodeHandle& nh){
    
    // Construct an URDF model from the xml string
    std::string xml_string;
    if (!nh.getParam("robot_description", xml_string)) {
        ROS_FATAL("Cannot find robot_description, shutting down!");
        return false;
    }
    
    if (xml_string.size() == 0) {
        ROS_FATAL("Unable to load robot model from robot_description!");
        return false;
    }
    
    // Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string)) {
        ROS_FATAL("Failed to get robot model from description!");
        return false;
    }
    
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
        ROS_ERROR("Failed to construct kdl tree");
        return false;
    }

    // Get the joint limits from the tree according to robot name
    bool all_limits_found = true;
    boost::shared_ptr<const urdf::Joint> joint;
    this->joint_limits_.resize(this->joint_names_.size());
    for (int i = 0; i < this->joint_names_.size(); i++) {
        joint = model.getJoint(this->robot_name_ + "_" + this->joint_names_[i]);
        this->joint_limits_[i] = {joint->limits->lower, joint->limits->upper};
    }

    if (DEBUG_PARSED) {
        ROS_INFO_STREAM("The joint limits of " << this->robot_name_ << " are ");
        std::cout << "/ - ";
        for (auto it : this->joint_limits_) {
            std::cout << "(" << it.first << ", " << it.second << ") ";
        }
        std::cout << "- /\n" << std::endl;
    }

    // After everything return success
    return true;

}

// Function to enforce joint limits
void JointsEstimator::enforce_limits(){
    
    // Saturate if estimated joint values outside limits
    for (int i = 0; i < this->js_values_.size(); i++) {
        if (this->js_values_[i] <= this->joint_limits_[i].first) {
            this->js_values_[i] = this->joint_limits_[i].first;
        } else if (this->js_values_[i] >= this->joint_limits_[i].second) {
            this->js_values_[i] = this->joint_limits_[i].second;
        }
    }

}

// Function to correct the offset form estimated angles
void JointsEstimator::correct_offset(){

    // Compute the real joint states by removing the offset
    for (int i = 0; i < this->joint_values_.size(); i++) {
        this->js_values_[i] = this->joint_values_[i] - this->joint_offset_[i];
    }

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
    // return loc_axis;                                // Using local axis changes nothing (obviously)

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
    if (!this->compute_perpendiculars(tmp_axis, perp_1, perp_2)) {
        ROS_FATAL_STREAM("Could not compute the normal plane to the joint axis of " << this->joint_names_[pos]);
        this->je_nh_.shutdown();
    }

    // 4) Trasforming one of the normals using the relative rotation of the imu pair
    // Eigen::Vector3d transformed = Eigen::Quaternion<double>(this->rel_poses_[pos]) * perp_1;
    Eigen::Vector3d transformed = Eigen::Quaternion<double>(this->rel_poses_joints_[pos]) * perp_1;

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
    tmp_axis.normalize(); perp_1.normalize(); projected.normalize();        // Making sure vectors are unitary
    double determinant = tmp_axis.dot(perp_1.cross(projected));             // Calculating det as triple product
    double dot_product = perp_1.dot(projected);                             // Getting the dot product
    
    float js = (float) atan2(determinant, dot_product);

    // 7) Return the found joint state
    return js;

}

// Function to compute the relative transforms for all pairs (use if quaternions from topic are wrt global frame)
void JointsEstimator::compute_relative_trasforms(std::vector<std::pair<int, int>> imu_pairs){
    
    // Running through pairs and computing relative transforms
    this->rel_poses_.clear();
    Eigen::Quaternion<float> pose_1; Eigen::Quaternion<float> pose_2;
    for (auto it : imu_pairs) {
        pose_1 = Eigen::Quaternion<float>(this->imu_poses_.at(it.first).w, this->imu_poses_.at(it.first).x,
            this->imu_poses_.at(it.first).y, this->imu_poses_.at(it.first).z);
        pose_2 = Eigen::Quaternion<float>(this->imu_poses_.at(it.second).w, this->imu_poses_.at(it.second).x,
            this->imu_poses_.at(it.second).y, this->imu_poses_.at(it.second).z);

        this->rel_poses_.push_back(pose_1.inverse() * pose_2);
    }

}

// Function to compute the relative transforms for all pairs (uses madgwick filter with acc and gyro)
void JointsEstimator::filter_relative_trasforms(std::vector<std::pair<int, int>> imu_pairs){

    // Creating the necessary inputs of the filter
    Eigen::Vector3d acc_1, gyro_1, acc_2, gyro_2;

    int i = 0;
    for (auto it : imu_pairs) {
        // Setting acceleration and angular velocity of pair
        acc_1 << this->imu_acc_.at(it.first).x, this->imu_acc_.at(it.first).y, this->imu_acc_.at(it.first).z;
        gyro_1 << this->imu_gyro_.at(it.first).x, this->imu_gyro_.at(it.first).y, this->imu_gyro_.at(it.first).z;
        acc_2 << this->imu_acc_.at(it.second).x, this->imu_acc_.at(it.second).y, this->imu_acc_.at(it.second).z;
        gyro_2 << this->imu_gyro_.at(it.second).x, this->imu_gyro_.at(it.second).y, this->imu_gyro_.at(it.second).z;

        // Filter the quaternion
        this->rel_poses_[i] = this->mw_filter.filter(acc_1, gyro_1, acc_2, gyro_2, this->acc_1_olds_[i],
            this->acc_2_olds_[i], this->rel_poses_[i]);

        // Correct the quaternion offset
        this->rel_poses_joints_[i] = this->mw_filter.correct_offset(this->rel_poses_[i], this->rel_poses_off_[i]);

        // Save olds and increase index
        this->acc_1_olds_[i] = acc_1;
        this->acc_2_olds_[i] = acc_2;
        i++;
    }

}

// Function to fill joint states with est. values and publish
void JointsEstimator::fill_and_publish(std::vector<float> joint_values){

    // Correcting offste and enforcing the limits on current estimation
    this->correct_offset();
    this->enforce_limits();

    // Filling up the msg and publishing
    this->joint_states_.header.stamp = ros::Time::now();
    for (int i = 0; i < this->joint_values_.size(); i++) {
        this->joint_states_.position[i] = (this->js_values_[i]);
    }
    this->pub_js_.publish(this->joint_states_);

}

// Callback to imu angles topic
void JointsEstimator::imu_callback(const qb_interface::quaternionArray::ConstPtr &msg){

    if (DEBUG_CB) std::cout << "Entered quaternion callback" << std::endl;
    
    // Get the imu angles of foot after clearing old angles
    this->imu_poses_.resize(4);
    for (int i = 0; i < msg->m.size(); i++) {
        if (msg->m[i].board_id == this->foot_id_) {
            this->imu_poses_[msg->m[i].id] = msg->m[i];
        }
    }

    // Debug print out
    if (DEBUG_JE) {
        std::cout << "\n *-----------* \n IMU messages recieved \n *-----------* \n";
        ROS_INFO_STREAM("Saved quaternions for foot " << this->foot_id_ << ": imus no: ");
        std::cout << "/ - ";
        for (auto it : this->imu_poses_) {
            std::cout << it.id << " - ";
        }
        std::cout << "/\n" << std::endl;
    }

}

// Callback to imu accelerations topic
void JointsEstimator::acc_callback(const qb_interface::inertialSensorArray::ConstPtr &msg){

    if (DEBUG_CB) std::cout << "Entered acceleration callback" << std::endl;

    // Get the imu angles of foot after clearing old angles
    this->imu_acc_.resize(4);
    for (int i = 0; i < msg->m.size(); i++) {
        if (msg->m[i].board_id == this->foot_id_) {
            this->imu_acc_[msg->m[i].id] = msg->m[i];
        }
    }

    // Debug print out
    if (DEBUG_JE) {
        std::cout << "\n *-----------* \n IMU messages recieved \n *-----------* \n";
        ROS_INFO_STREAM("Saved accelerations for foot " << this->foot_id_ << ": imus no: ");
        std::cout << "/ - ";
        for (auto it : this->imu_acc_) {
            std::cout << it.id << " - ";
        }
        std::cout << "/\n" << std::endl;
    }

}

// Callback to imu angular velocities topic
void JointsEstimator::gyro_callback(const qb_interface::inertialSensorArray::ConstPtr &msg){

    if (DEBUG_CB) std::cout << "Entered angular velocity callback" << std::endl;

    // Get the imu angles of foot after clearing old angles
    this->imu_gyro_.resize(4);
    for (int i = 0; i < msg->m.size(); i++) {
        if (msg->m[i].board_id == this->foot_id_) {
            this->imu_gyro_[msg->m[i].id] = msg->m[i];
        }
    }

    // Debug print out
    if (DEBUG_JE) {
        std::cout << "\n *-----------* \n IMU messages recieved \n *-----------* \n";
        ROS_INFO_STREAM("Saved angular velocities for foot " << this->foot_id_ << ": imus no: ");
        std::cout << "/ - ";
        for (auto it : this->imu_gyro_) {
            std::cout << it.id << " - ";
        }
        std::cout << "/\n" << std::endl;
    }

}