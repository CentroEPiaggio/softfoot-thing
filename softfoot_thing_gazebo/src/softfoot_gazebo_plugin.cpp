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
void SoftFootGazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){

    // Safety check
    if (_model->GetJointCount() == 0) {
        ROS_FATAL_STREAM("Invalid number joints, SoftFoot Gazebo Plugin not loaded!\n");
        return;
    }

    // Save the model and link pointers for later use.
    this->model = _model;
    this->link = model->GetLink("softfoot_3_roll_link");

    // Listen to the update event
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&SoftFootGazeboPlugin::OnUpdate, this));

    // Plugin loaded successfully
    ROS_WARN_STREAM("Loaded plugin for SoftFoot simulation!\n");

}

// Plugin Update Function
void SoftFootGazeboPlugin::OnUpdate(){
    // Apply a small linear velocity to the model.
    this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SoftFootGazeboPlugin);