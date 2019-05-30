#ifndef SOFTFOOT_GAZEBO_PLUGIN_H_
#define SOFTFOOT_GAZEBO_PLUGIN_H_

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Gazebo Math
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

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

            // Function to compute angular velocity between two frames
            ignition::math::Vector3d ComputeAngularVel(ignition::math::Pose3d f_1,
                                                       ignition::math::Pose3d f_2);

        private:

            // Pointers to the model
            physics::ModelPtr model_;
            sdf::ElementPtr sdf_;

            // Insertion joint
            physics::JointPtr insertion_joint_;
            ignition::math::Vector3d insertion_axis_;

            // Pointers to the links to be contolled
            physics::LinkPtr link_;                     // chain_9_link
            physics::LinkPtr link_des_;                 // back_roll_link

            // Fixed trasforms
            ignition::math::Pose3d roll_to_ins_;

            // Pointer to the update event connection
            event::ConnectionPtr updateConnection_;

            // Namespace of the foot
            std::string foot_namespace_;

  };
}

#endif // SOFTFOOT_GAZEBO_PLUGIN_H_
