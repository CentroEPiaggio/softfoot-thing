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
            virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);

            // Plugin Update Function
            virtual void OnUpdate();

        private:

            // Pointers to the model
            physics::ModelPtr model_;
            sdf::ElementPtr sdf_;

            // Pointer to the link to be contolled
            physics::LinkPtr link_;

            // Pointer to the update event connection
            event::ConnectionPtr updateConnection_;

            // Namespace of the foot
            std::string foot_namespace_;

  };
}

#endif // SOFTFOOT_GAZEBO_PLUGIN_H_