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

    ROS_WARN_STREAM("Namespace is " << this->foot_namespace_);

    // Get the links to be controlled
    this->link_ = model->GetLink(this->foot_namespace_ + "_middle_chain_9_link");

    // Get the desired link
    this->link_des_ = model->GetLink(this->foot_namespace_ + "_back_roll_link");

    // ADDING A JOINT BETWEEN CHAIN TIP AND BACK ROLL
    this->insertion_joint_ = model_->GetWorld()->Physics()->CreateJoint("revolute", model);
    this->insertion_joint_->Load(this->link_des_, this->link_, this->roll_to_ins_);
    this->insertion_joint_->Attach(this->link_des_, this->link_);
    this->insertion_axis_ = ignition::math::Vector3d(1.0, 0.0, 0.0);
    this->insertion_joint_->SetAxis(0, this->insertion_axis_);

    // Listen to the update event
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SoftFootGazeboPlugin::OnUpdate, this));

    // Plugin loaded successfully
    ROS_WARN_STREAM("Loaded plugin for SoftFoot simulation!\n");

}

// Plugin Update Function
void SoftFootGazeboPlugin::OnUpdate(){

    /* Nothing to do here */

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SoftFootGazeboPlugin);
