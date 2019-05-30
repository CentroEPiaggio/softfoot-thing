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

            // Pointers to the roll joints
            physics::JointPtr front_roll_joint_;
            physics::JointPtr back_roll_joint_;

            // Pointers to the arch joints
            physics::JointPtr front_arch_joint_;
            physics::JointPtr back_arch_joint_;

            // Joint controls
            double back_roll_control_;
            double front_arch_control_;
            double back_arch_control_;

            // Pointers to the links to be contolled
            physics::LinkPtr link_;                     // chain_9_link
            physics::LinkPtr link_des_;                 // back_roll_link

            // Fixed trasforms
            ignition::math::Pose3d roll_to_ins_;
            ignition::math::Pose3d chain_9_to_tip_;

            // Position errors for control
            ignition::math::Vector3d lin_error_;
            ignition::math::Vector3d lin_error_loc_;
            ignition::math::Vector3d int_error_;
            ignition::math::Vector3d ang_error_;

            // Time dependent variables
            double dt_ = 0.0;
            double last_time_;
            double error_integral_ = 0.0;               // integral of the error

            // Pointer to the update event connection
            event::ConnectionPtr updateConnection_;

            // Namespace of the foot
            std::string foot_namespace_;

  };
}

#endif // SOFTFOOT_GAZEBO_PLUGIN_H_
