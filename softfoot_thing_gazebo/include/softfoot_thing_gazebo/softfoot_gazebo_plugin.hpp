#ifndef SOFTFOOT_GAZEBO_PLUGIN_H_
#define SOFTFOOT_GAZEBO_PLUGIN_H_

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
    // A plugin to simulate the softfoot in gazebo.
    class SoftFootGazeboPlugin : public ModelPlugin {
        
        // Constructor
        public: SoftFootGazeboPlugin();

        // Plugin Load function
        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  };
}

#endif // SOFTFOOT_GAZEBO_PLUGIN_H_