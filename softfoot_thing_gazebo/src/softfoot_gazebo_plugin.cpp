// Boost
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// Custom include
#include "softfoot_thing_gazebo/softfoot_gazebo_plugin.hpp"

// Custom defines
#define     DEBUG_SFGP      1       // Prints out additional info
#define     UPPER_COUP      1.04    // Upper (downwards) coupling angle limit for arch links
#define     LOWER_COUP      0.0     // Lower (upwards) coupling angle limit for arch links
#define     TORQUE_LIM      0.01    // Simulation limits on joint torques

using namespace gazebo;

// Constructor
SoftFootGazeboPlugin::SoftFootGazeboPlugin(){

}

// Plugin Load Function
void SoftFootGazeboPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf){

    // Safety check
    if (model->GetJointCount() == 0) {
        ROS_FATAL_STREAM("Invalid number joints, SoftFoot Gazebo Plugin not loaded!\n");
        return;
    }

    // Save the model and link pointers for later use.
    this->model_ = model;
    this->sdf_ = sdf;

    // Check if model can be found
    if (!model) {
        ROS_FATAL_STREAM("Parent model is NULL! SoftFoot Gazebo Plugin cannot be loaded!\n");
        return;
    }

    // Get the namespace of the foot
    if (sdf->HasElement("robotNamespace")) {
        this->foot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
    } else {
        this->foot_namespace_ = model->GetName();
    }

    ROS_WARN_STREAM("Namespace is " << this->foot_namespace_);

    // Get the roll joints
    this->front_roll_joint_ = model->GetJoint(this->foot_namespace_ + "_front_roll_joint");
    this->back_roll_joint_ = model->GetJoint(this->foot_namespace_ + "_back_roll_joint");

    // Get the arch joints
    this->front_arch_joint_ = model->GetJoint(this->foot_namespace_ + "_front_arch_joint");
    this->back_arch_joint_ = model->GetJoint(this->foot_namespace_ + "_back_arch_joint");

    // Get the links to be controlled
    this->link_ = model->GetLink(this->foot_namespace_ + "_middle_chain_9_link");

    // Get the desired link
    this->link_des_ = model->GetLink(this->foot_namespace_ + "_back_roll_link");

    // Set the fixed transforms (TODO: change this with getting the transforms from SDF)
    this->roll_to_ins_.Set(-0.002, 0.000, -0.012,-1.676, 0.000, -1.571);
    this->chain_9_to_tip_.Set(0.000, 0.000, 0.013, 0.000, 0.000, 0.000);

    // Listen to the update event
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SoftFootGazeboPlugin::OnUpdate, this));

    // Plugin loaded successfully
    ROS_WARN_STREAM("Loaded plugin for SoftFoot simulation!\n");

}

// Plugin Update Function
void SoftFootGazeboPlugin::OnUpdate(){

    /* 1) Control the roll joint to mimic symmetrically each other */
    if (std::abs(this->front_arch_joint_->Position() + this->back_roll_joint_->Position()) > 0.1) {
        this->back_roll_joint_->SetForce(0, 0.1 * (this->back_roll_joint_->Position()
                                                  + this->front_roll_joint_->Position()));
    }

    /* 2) Control the coupling between the arch joints */
    if ((this->front_arch_joint_->Position() + this->back_arch_joint_->Position()) > double(UPPER_COUP)) {
        this->back_arch_joint_->SetForce(0, 0.1 * (double(UPPER_COUP) - this->front_arch_joint_->Position()));
        this->front_arch_joint_->SetForce(0, 0.1 * (double(UPPER_COUP) - this->back_arch_joint_->Position()));
    } else if ((this->front_arch_joint_->Position() + this->back_arch_joint_->Position()) < double(LOWER_COUP)) {
        this->back_arch_joint_->SetForce(0, 0.1 * (double(LOWER_COUP) - this->front_arch_joint_->Position()));
        this->front_arch_joint_->SetForce(0, 0.1 * (double(LOWER_COUP) - this->back_arch_joint_->Position()));
    }

    /* 3) Control the tip of the chain to be inserted where it should be */
    // Get the linear and anguar errors between the desired and real poses of the tip link
    this->lin_error_ = (this->chain_9_to_tip_.Inverse() * this->roll_to_ins_
                        * this->link_des_->WorldPose()).Pos() - (this->link_->WorldPose()).Pos();
    this->ang_error_ = this->ComputeAngularVel(this->chain_9_to_tip_.Inverse() * this->roll_to_ins_
                                               * this->link_des_->WorldPose(), this->link_->WorldPose());

    // Apply a velocity control to the chain 9 link
    this->link_->SetLinearVel(10 * this->lin_error_);
    this->link_->SetAngularVel(10 * this->ang_error_);

    // Apply opposite linear velocity control to the back_roll_link
    if (this->lin_error_.SquaredLength() > 0.01) {
        this->link_des_->SetLinearVel(-10.0 * this->lin_error_);
    }

    // Debug print
    if (DEBUG_SFGP) {
        std::cout << "Back vel. torque control: \n" << 0.1 * (this->back_roll_joint_->Position()
                                                             + this->front_roll_joint_->Position()) << std::endl;
        std::cout << "Front arch torque control: \n" << 0.1 * (double(UPPER_COUP)
                                                               - this->back_arch_joint_->Position()) << std::endl;
        std::cout << "Back arch torque control: \n" << 0.1 * (double(UPPER_COUP)
                                                              - this->front_arch_joint_->Position()) << std::endl;
        std::cout << "Tip lin. vel. control: \n" << 10 * this->lin_error_ << std::endl;
        std::cout << "Tip ang. vel. control: \n" << 10 * this->ang_error_ << std::endl;
        std::cout << "\n ---- \n" << std::endl;
    }

}

// Function to compute angular velocity between two frames
ignition::math::Vector3d SoftFootGazeboPlugin::ComputeAngularVel(ignition::math::Pose3d f_1,
                                                                 ignition::math::Pose3d f_2){

    // Get the residual transformation
    ignition::math::Quaterniond Q_res = f_2.Rot() * f_1.Rot().Inverse();

    // Get the axis-angle
    ignition::math::Vector3d axis;
    double angle;
    Q_res.ToAxis(axis, angle);

    // Return the angular displacement
    return angle * axis;

}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SoftFootGazeboPlugin);
