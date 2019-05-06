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
        
        public:

            // Constructor
            SoftFootGazeboPlugin();

            // Plugin Load function
            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

        private:

            /// \brief Pointer to the model.
            private: physics::ModelPtr model;

            /// \brief Pointer to the joint.
            private: physics::LinkPtr link;

  };
}

#endif // SOFTFOOT_GAZEBO_PLUGIN_H_