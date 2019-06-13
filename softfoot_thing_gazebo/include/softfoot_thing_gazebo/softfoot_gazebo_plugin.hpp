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

            // Auxiliary funtion for deg2rad conversion
            inline double deg2rad (double degrees) {
                static const double k_pi_on_180 = 4.0 * atan (1.0) / 180.0;
                return degrees * k_pi_on_180;
            }

            // Pointers to physics variables
            sdf::ElementPtr sdf_;
            physics::ModelPtr model_;
            physics::WorldPtr world_;
            physics::PhysicsEnginePtr engine_;


            // Insertion joints
            physics::JointPtr insertion_joint_l_;
            physics::JointPtr insertion_joint_m_;
            physics::JointPtr insertion_joint_r_;
            ignition::math::Vector3d insertion_axis_;

            // Coupling spring joint
            physics::JointPtr coupling_joint_;
            ignition::math::Vector3d coupling_axis_;

            // Arch joints and spring constant
            physics::JointPtr fa_joint_;
            physics::JointPtr ba_joint_;
            double current_angle_;
            double rest_angle_ = 0.0;
            double spring_k_ = 0.01;

            // Pointers to the links to join for chains
            physics::LinkPtr link_l_;                   // left_chain_9_link
            physics::LinkPtr link_m_;                   // middle_chain_9_link
            physics::LinkPtr link_r_;                   // right_chain_9_link
            physics::LinkPtr link_des_;                 // back_roll_link

            // Pointers to the arch links
            physics::LinkPtr link_fa_;                  // front_arch_link
            physics::LinkPtr link_ba_;                  // back_arch_link

            // Fixed trasforms
            ignition::math::Pose3d roll_to_ins_;
            ignition::math::Pose3d chain_9_to_tip_;

            // Pointer to the update event connection
            event::ConnectionPtr updateConnection_;

            // Namespace of the foot
            std::string foot_namespace_;

  };
}

#endif // SOFTFOOT_GAZEBO_PLUGIN_H_
