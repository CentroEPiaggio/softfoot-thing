// Boost
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// Custom include
#include "softfoot_thing_gazebo/softfoot_gazebo_plugin.hpp"

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

    // Get the links to be controlled (chain tips)
    this->link_l_ = model->GetLink(this->foot_namespace_ + "_left_chain_9_link");
    this->link_m_ = model->GetLink(this->foot_namespace_ + "_middle_chain_9_link");
    this->link_r_ = model->GetLink(this->foot_namespace_ + "_right_chain_9_link");

    // Get the desired link
    this->link_des_ = model->GetLink(this->foot_namespace_ + "_back_roll_link");

    // Set the fixed transforms (TODO: change this with getting the transforms from SDF)
    this->roll_to_ins_.Set(-0.002, 0.000, -0.012,-1.676, 0.000, -1.571);
    this->chain_9_to_tip_.Set(0.000, 0.000, 0.013, 0.000, 0.000, 0.000);

    // Adding joint between left_chain 9_link and back_roll_link
    this->insertion_joint_ = model->CreateJoint(this->foot_namespace_ + "_left_chain_attach_joint",
                                                 "revolute", this->link_des_, this->link_l_);
    this->insertion_joint_->Load(this->link_des_, this->link_l_, this->chain_9_to_tip_);
    this->insertion_joint_->Attach(this->link_des_, this->link_l_);
    this->insertion_joint_->SetModel(model);
    this->insertion_axis_ = ignition::math::Vector3d(1.0, 0.0, 0.0);
    this->insertion_joint_->SetAxis(0, this->insertion_axis_);
    this->insertion_joint_->SetLowerLimit(0, this->deg2rad(-45.0));
    this->insertion_joint_->SetUpperLimit(0, this->deg2rad(45.0));
    this->insertion_joint_->SetEffortLimit(0, 1.0);
    this->insertion_joint_->SetVelocityLimit(0, 1.0);
    this->insertion_joint_->SetDamping(0, 0.0001);
    this->insertion_joint_->SetParam("friction", 0, 0.0001);

    // Adding joint between middle_chain 9_link and back_roll_link
    this->insertion_joint_ = model->CreateJoint(this->foot_namespace_ + "_middle_chain_attach_joint",
                                                 "revolute", this->link_des_, this->link_m_);
    this->insertion_joint_->Load(this->link_des_, this->link_m_, this->chain_9_to_tip_);
    this->insertion_joint_->Attach(this->link_des_, this->link_m_);
    this->insertion_joint_->SetModel(model);
    this->insertion_axis_ = ignition::math::Vector3d(1.0, 0.0, 0.0);
    this->insertion_joint_->SetAxis(0, this->insertion_axis_);
    this->insertion_joint_->SetLowerLimit(0, this->deg2rad(-45.0));
    this->insertion_joint_->SetUpperLimit(0, this->deg2rad(45.0));
    this->insertion_joint_->SetEffortLimit(0, 1.0);
    this->insertion_joint_->SetVelocityLimit(0, 1.0);
    this->insertion_joint_->SetDamping(0, 0.0001);
    this->insertion_joint_->SetParam("friction", 0, 0.0001);

    // Adding joint between right_chain 9_link and back_roll_link
    this->insertion_joint_ = model->CreateJoint(this->foot_namespace_ + "_right_chain_attach_joint",
                                                 "revolute", this->link_des_, this->link_r_);
    this->insertion_joint_->Load(this->link_des_, this->link_r_, this->chain_9_to_tip_);
    this->insertion_joint_->Attach(this->link_des_, this->link_r_);
    this->insertion_joint_->SetModel(model);
    this->insertion_axis_ = ignition::math::Vector3d(1.0, 0.0, 0.0);
    this->insertion_joint_->SetAxis(0, this->insertion_axis_);
    this->insertion_joint_->SetLowerLimit(0, this->deg2rad(-45.0));
    this->insertion_joint_->SetUpperLimit(0, this->deg2rad(45.0));
    this->insertion_joint_->SetEffortLimit(0, 1.0);
    this->insertion_joint_->SetVelocityLimit(0, 1.0);
    this->insertion_joint_->SetDamping(0, 0.0001);
    this->insertion_joint_->SetParam("friction", 0, 0.0001);

    // Listen to the update event
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SoftFootGazeboPlugin::OnUpdate, this));

    // Plugin loaded successfully
    ROS_WARN_STREAM("Loaded plugin for " << this->foot_namespace_ << " simulation!\n");

}

// Plugin Update Function
void SoftFootGazeboPlugin::OnUpdate(){

    /* Nothing to do here */

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SoftFootGazeboPlugin);
