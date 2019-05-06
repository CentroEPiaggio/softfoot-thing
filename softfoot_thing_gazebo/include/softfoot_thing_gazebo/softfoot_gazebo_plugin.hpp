#ifndef SOFTFOOT_GAZEBO_PLUGIN_H_
#define SOFTFOOT_GAZEBO_PLUGIN_H_

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// ROS
#include <ros/ros.h>

namespace gazebo
{
    // A plugin to simulate the softfoot in gazebo.
    class SoftFootGazeboPlugin : public ModelPlugin {
        
        public:

            // Constructor
            SoftFootGazeboPlugin();

            // Plugin Load Function
            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

            // Plugin Update Function
            virtual void OnUpdate();

        private:

            // Pointer to the model.
            physics::ModelPtr model;

            // Pointer to the joint
            physics::LinkPtr link;

            // Pointer to the update event connection
            event::ConnectionPtr updateConnection;

  };
}

#endif // SOFTFOOT_GAZEBO_PLUGIN_H_