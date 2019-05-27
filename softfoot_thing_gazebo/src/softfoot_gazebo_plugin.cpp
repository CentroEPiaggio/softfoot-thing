// Boost
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// Other includes
#include <ignition/math/Vector3.hh>

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

    // Get the link to be controlled
    this->link_ = model->GetLink(this->foot_namespace_ + "_middle_chain_9_link");

    // Get the desired link
    this->link_des_ = model->GetLink(this->foot_namespace_ + "_back_roll_link");

    // Set the fixed transforms (TODO: change this with getting the transforms from SDF)
    this->roll_to_ins_.Set(ignition::math::Vector3d(-0.002, 0.000, -0.012),
                           ignition::math::Vector3d(-1.676, 0.000, -1.571));
    this->chain_9_to_tip_.Set(ignition::math::Vector3d(0.000, 0.000, 0.013),
                              ignition::math::Vector3d(0.000, 0.000, 0.000));


    // Listen to the update event
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SoftFootGazeboPlugin::OnUpdate, this));

    // Plugin loaded successfully
    ROS_WARN_STREAM("Loaded plugin for SoftFoot simulation!\n");

}

// Plugin Update Function
void SoftFootGazeboPlugin::OnUpdate(){

    // Get the desired pose of the tip link
    this->error_ = (this->link_des_->WorldPose()).Pos()
                - (this->link_->WorldPose()).Pos();

//    std::cout << (this->link_des_->WorldPose() * this->roll_to_ins_).Pos() << std::endl;

    // Apply the desired pose to the chain tip link
    this->link_->SetLinearVel(10 * this->error_);

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SoftFootGazeboPlugin);
