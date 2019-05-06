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
void SoftFootGazeboPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SoftFootGazeboPlugin);