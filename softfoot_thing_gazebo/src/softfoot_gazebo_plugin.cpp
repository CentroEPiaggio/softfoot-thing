// Boost
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Custom include
#include "softfoot_thing_gazebo/softfoot_gazebo_plugin.hpp"

using namespace gazebo;

// Constructor
SoftFootGazeboPlugin::SoftFootGazeboPlugin(){

}

// Plugin Load function
void SoftFootGazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){

    // Safety check
    if (_model->GetJointCount() == 0) {
        std::cerr << "Invalid number joints, SoftFoot Gazebo Plugin not loaded!\n";
        return;
    }

    // Save the model pointer for later use.
    this->model = _model;

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SoftFootGazeboPlugin);