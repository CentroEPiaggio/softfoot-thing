/*  
    JOINTS ESTIMATOR CLASS
    This object tries to estimate the joint angles of foot from IMU poses.
*/

#include <fstream>
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include "yaml-cpp/yaml.h"

#include "softfoot_thing_visualization/utils/parsing_utilities.h"
#include "softfoot_thing_visualization/joints_estimator.h"

#define     DEBUG_JE        0       // Prints out additional info about the Object
#define     DEBUG_JS        0       // Prints out info about estimated joint states
#define     DEBUG_PARSED    0       // Prints out info about parsed stuff
#define     DEBUG_ANGLES    0       // Prints out only raw estimated angles

#define     N_CAL_IT        100     // Number of calibration iterations

using namespace softfoot_thing_visualization;

JointsEstimator::JointsEstimator(ros::NodeHandle& nh , int foot_id, std::string foot_name) : spinner(4) {

    // Initializing main variables
    this->foot_id_ = foot_id;
    this->foot_name_ = foot_name;
    this->robot_name_ = this->foot_name_ + "_" + std::to_string(this->foot_id_);
    this->pkg_path = ros::package::getPath("softfoot_thing_visualization");

    // Initializing ros variables
    this->je_nh_ = nh;

    this->sub_imu_acc_ = this->je_nh_.subscribe<qb_interface::inertialSensorArray>(this->imu_topic_acc_, 
        1, &JointsEstimator::acc_callback, this);
    qb_interface::inertialSensorArray::ConstPtr temp_msg_2 = ros::topic::waitForMessage
        <qb_interface::inertialSensorArray>(this->imu_topic_acc_, ros::Duration(2.0));
    
    this->sub_imu_gyro_ = this->je_nh_.subscribe<qb_interface::inertialSensorArray>(this->imu_topic_gyro_, 
        1, &JointsEstimator::gyro_callback, this);
    qb_interface::inertialSensorArray::ConstPtr temp_msg_3 = ros::topic::waitForMessage
        <qb_interface::inertialSensorArray>(this->imu_topic_gyro_, ros::Duration(2.0));
    
    this->pub_js_ = this->je_nh_.advertise<sensor_msgs::JointState>
        ("/" + this->foot_name_ + "_" + std::to_string(this->foot_id_) + "/joint_states", 1);

    // Temporarily building parsable variables here (TODO: parse them)
    this->joint_pairs_ = {{0, 1}, {0, 3}, {1, 2}};
    this->joint_names_ = {"front_arch_joint", "back_arch_joint", "roll_joint"};
    this->joint_frame_names_ = {"front_arch_link", "back_arch_link", "roll_link"};

    // Temporarily building parsable variables here (TODO: parse them)
    Eigen::Vector3d x_loc(1, 0, 0);
    Eigen::Vector3d y_loc(0, 1, 0);
    Eigen::Vector3d z_loc(0, 0, 1);
    this->axes_pairs_ = {{y_loc, x_loc}, {-y_loc, -y_loc}, {y_loc, -z_loc}};

    // Parsing needed parameters
    if (!this->parse_parameters(this->je_nh_)) {
        ROS_FATAL_STREAM("JointsEstimator::JointsEstimator : Could not get parameters for " 
            << this->robot_name_ << " !");
        this->je_nh_.shutdown();
    }

    // Filling up main parts of the joint state msg and setting size of values
    for (auto it : this->joint_names_) {
        this->joint_states_.name.push_back(this->foot_name_ + "_" + std::to_string(this->foot_id_) 
            + "_" + it);
        this->joint_states_.position.push_back(0.0);
    }
    this->joint_values_.resize(this->joint_pairs_.size());
    this->js_values_.resize(this->joint_pairs_.size());

    // Setting old accelerations to null
    this->acc_vec_0_.resize(4);
    this->acc_vec_.resize(4);
    this->acc_vec_olds_.resize(4);
    for (int i = 0; i < 4; i++) {
        this->acc_vec_0_[i] = Eigen::Vector3d::Zero();
        this->acc_vec_[i] = Eigen::Vector3d::Zero();
        this->acc_vec_olds_[i] = Eigen::Vector3d::Zero();
    }

}

JointsEstimator::~JointsEstimator(){

    // Nothing to do here

}

// Function that calibrates the sensing
void JointsEstimator::calibrate(){

    Eigen::Vector3d sample;

    for (int k = 0; k < N_CAL_IT; k++) {
        // Spin to recieve messages
        ros::spinOnce();

        // Fill up the initial accelerations and others
        for (int i = 0; i < 4; i++) {
            sample << this->imu_acc_[i].x, this->imu_acc_[i].y, this->imu_acc_[i].z;
            this->acc_vec_0_[i] = (this->acc_vec_0_[i] + sample);
        }
    }

    // Divide by N_CAL_IT to get the mean
    for (int i = 0; i < 4; i++) {
        this->acc_vec_0_[i] = (this->acc_vec_0_[i] / N_CAL_IT);
    }

    // Setting current and old accs to initial
    this->acc_vec_ = this->acc_vec_0_;
    this->acc_vec_olds_ = this->acc_vec_0_;

    // Set calibrated flag
    this->calibrated_ = true;

}

// Function that calibrates the sensing and saves the calibration data to yaml
void JointsEstimator::calibrate_and_save(std::string file_name){

    // Get the path to the softfoot viz config folder and append the file name
    std::string config_file_path = this->pkg_path + "/configs/" + file_name + ".yaml";
    std::cout << "File path is " << config_file_path << "." << std::endl;

    // Create yaml emitter and prepare it with foot details
    YAML::Emitter yaml_out;
    yaml_out << YAML::Comment("Calibration Data for SoftFoot Joint Estimation");

    // Calibrating the foot sensors
    this->calibrate();
    std::cout << "The calibrated accelerations are: " << std::endl;
    for (auto it : this->acc_vec_0_) {
        std::cout << it.transpose() << std::endl;
    }
    
    // Writing result of calibration to yaml
    yaml_out << YAML::BeginMap;                     // begin: foot name
    yaml_out << YAML::Key << this->robot_name_;

    yaml_out << YAML::BeginMap;                     // begin: base_accelerations
    yaml_out << YAML::Key << "base_accelerations";

    // Saving all of the base calibrated accelerations
    yaml_out << YAML::BeginSeq;
    for (auto it : this->acc_vec_0_) {
        std::vector<double> temp = {it(0), it(1), it(2)};
        yaml_out << YAML::Flow << temp;
    }
    yaml_out << YAML::EndSeq;

    yaml_out << YAML::EndMap;                       // end: foot name
    yaml_out << YAML::EndMap;                       // end: base_accelerations

    // Save the emmitter to file
    std::ofstream file_out(config_file_path);
    file_out << yaml_out.c_str();

}

// Function that spins the estimator
bool JointsEstimator::check_calibration(){

    // Check if the foot has been calibrated
    if (!this->calibrated_) {
        ROS_WARN_STREAM("No on the fly calibration was requested for " << this->robot_name_ 
            << ", looking for an offline calibration data file for this foot!");

        // Trying to parse an offline calibration yaml
        if (!this->je_nh_.getParam(this->robot_name_, this->je_params_)) {
            ROS_FATAL_STREAM("No offline calibration found for " << this->robot_name_ 
            << ", this is bad!");
            return false;
        } else {
            // Getting parameters
            Eigen::MatrixXd base_accelerations(4, 3);
            parseParameter(this->je_params_, base_accelerations, "base_accelerations");

            // Check for parameter consistency
            if (base_accelerations.rows() != this->acc_vec_0_.size() || 
                base_accelerations.cols() != 3) {
                    ROS_FATAL_STREAM("The parsed calibration for " << this->robot_name_ 
                        << " is inconsistent, this is bad!");
                    return false;
            }

            // Filling vector of base accelerations from parsed matrix
            Eigen::Vector3d sample;
            for (int i = 0; i < this->acc_vec_0_.size(); i++) {
                sample << base_accelerations(i, 0), base_accelerations(i, 1), base_accelerations(i, 2);
                this->acc_vec_0_[i] = sample;
            }

            std::cout << "The parsed calibration matrix is " << std::endl;
            std::cout << base_accelerations << std::endl;
        }

        // Set calibrated
        this->calibrated_ = true;
    }

    return true;

}

// Function that estimates the joint angles
bool JointsEstimator::estimate(){

    // Check if the foot has been calibrated
    if (!this->calibrated_) {
        ROS_FATAL_STREAM("Foot " << this->robot_name_  
            << "has not been calibrated, this is bad!");
        return false;
    }

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

    // Returning
    return true;

}

// Function to parse parameters
bool JointsEstimator::parse_parameters(ros::NodeHandle& nh){
    
    // TODO: parse needed params

    // Parsing joint limits of the foot (joint_names_ needs to be set before)
    if (!this->get_joint_limits(nh)) {
        ROS_FATAL_STREAM("Unable to get the joint limits for " << this->robot_name_ << "!");
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
        ROS_ERROR("Failed to construct kdl tree of robot!");
        return false;
    }

    // Get the joint limits from the tree according to robot name
    bool all_limits_found = true;
    std::shared_ptr<const urdf::Joint> joint;
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
        this->js_values_[i] = this->joint_values_[i];
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
        ROS_FATAL_STREAM("JointsEstimator::get_joint_axis : You specified for " << this->robot_name_ 
            << " a joint name which is unknown to me!");
        this->je_nh_.shutdown();
    }
    Eigen::Affine3d joint_frame = this->getTransform("world", this->foot_name_ + "_" 
        + std::to_string(this->foot_id_) + "_" + this->joint_frame_names_[pos]);

    // Getting the joint's axis in world frame (TODO: parse local axis)
    Eigen::Vector3d loc_axis;
    if (pos == 0) {
        loc_axis << 0, 1, 0;
    } else if (pos == 1) {
        loc_axis << 0, -1, 0;
    } else if (pos == 2) {
        loc_axis << 1, 0, 0;
    } else {
        ROS_FATAL_STREAM("JointsEstimator::get_joint_axis : You specified for " << this->robot_name_ 
            << " a joint name which is unknown to me! But this should have not happened!!!");
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
        ROS_FATAL_STREAM("JointsEstimator::compute_joint_state_from_pair : You specified for " 
            << this->robot_name_ << "  an imu pair which is unknown to me!");
        this->je_nh_.shutdown();
    }

    // Defining some variables
    Eigen::Vector3d axis, acc_0, acc_1;
    double determinant, dot_product;
    float angle_1, angle_2, js;
    
    // 2.1) Case imu1 of pair: compute the angle variation in acc vector around joint axis
    axis = this->axes_pairs_[pos].first; axis.normalize();
    acc_0 = this->acc_vec_0_[this->joint_pairs_[pos].first]; acc_0.normalize();
    acc_1 = this->acc_vec_[this->joint_pairs_[pos].first]; acc_1.normalize();

    // 2.2) Compute the first angle (variation of first imu inclination around the joint axis)
    determinant = axis.dot(acc_0.cross(acc_1));             // Calculating det as triple product
    dot_product = acc_0.dot(acc_1);                         // Getting the dot product
    angle_1 = (float) atan2(determinant, dot_product);

    // 3.1) Case imu2 of pair: compute the angle variation in acc vector around joint axis
    axis = this->axes_pairs_[pos].second; axis.normalize();
    acc_0 = this->acc_vec_0_[this->joint_pairs_[pos].second]; acc_0.normalize();
    acc_1 = this->acc_vec_[this->joint_pairs_[pos].second]; acc_1.normalize();

    // 3.2) Compute the second angle (variation of second imu inclination around the joint axis)
    determinant = axis.dot(acc_0.cross(acc_1));             // Calculating det as triple product
    dot_product = acc_0.dot(acc_1);                         // Getting the dot product
    angle_2 = (float) atan2(determinant, dot_product);

    // 4) Compute the final joint state combining the two angles
    js = angle_1 - angle_2;

    // 5) Return the found joint state of the joint relative to the specifies pair
    return js;

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

// Callback to imu accelerations topic
void JointsEstimator::acc_callback(const qb_interface::inertialSensorArray::ConstPtr &msg){

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

    // Save old accelerations and push new ones
    this->acc_vec_olds_ = this->acc_vec_;
    for (int i = 0; i < 4; i++) {
        this->acc_vec_[i] << this->imu_acc_[i].x, this->imu_acc_[i].y, this->imu_acc_[i].z;
    }

}

// Callback to imu angular velocities topic
void JointsEstimator::gyro_callback(const qb_interface::inertialSensorArray::ConstPtr &msg){

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