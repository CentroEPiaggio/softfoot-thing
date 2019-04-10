/*  
    MADGWICK FILTER CLASS
    This object contains functions to estimate imu quaternion from acc and gyro measurements.
*/

#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

// ROS includes
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <string>
#include <mutex>

// Other includes
#include <Eigen/Dense>

namespace softfoot_thing_visualization {

class MadgwickFilter {

    public:

        MadgwickFilter();
        
        MadgwickFilter(ros::NodeHandle& nh);

        ~MadgwickFilter();

        // Function to initialize filter
        bool initialize(ros::NodeHandle& nh);

        // Function to filter out relative quaternion form old relative quaternion, acc anf gyro
        Eigen::Quaternion<float> filter(Eigen::Vector3d acc_1, Eigen::Vector3d gyro_1,
            Eigen::Vector3d acc_2, Eigen::Vector3d gyro_2, Eigen::Vector3d acc_1_old,
            Eigen::Vector3d acc_2_old, Eigen::Quaternion<float> Q_rel_old);

    private:

        // Function to parse parameters
        bool parse_parameters(ros::NodeHandle& nh);
            
        /* AUXILIARY FUNCTIONS */

        inline Eigen::Vector4d ConjQ(Eigen::Vector4d Q_in){
            Eigen::Vector4d Q_out;
	        Q_out(0) =  Q_in(0);
	        Q_out(1) = -Q_in(1);
	        Q_out(2) = -Q_in(2);
	        Q_out(3) = -Q_in(3);
	        return Q_out;
        }

        inline Eigen::Vector4d QxQ(Eigen::Vector4d Q_1, Eigen::Vector4d Q_2){
            Eigen::Vector4d Q_out;
	        Q_out(0) = Q_1(0)*Q_2(0) - (Q_1(1)*Q_2(1) + Q_1(2)*Q_2(2) + Q_1(3)*Q_2(3));
	        Q_out(1) = Q_1(0)*Q_2(1) + Q_1(1)*Q_2(0) + (Q_1(2)*Q_2(3) - Q_1(3)*Q_2(2));
	        Q_out(2) = Q_1(0)*Q_2(2) + Q_1(2)*Q_2(0) + (Q_1(3)*Q_2(1) - Q_1(1)*Q_2(3));
	        Q_out(3) = Q_1(0)*Q_2(3) + Q_1(3)*Q_2(0) + (Q_1(1)*Q_2(2) - Q_1(2)*Q_2(1));
	        return Q_out;
        }

        // ROS variables
        ros::NodeHandle mf_nh_;

        // Filter variables
        double sampleFreq_ = 50;
        double beta_ = 2.0;
        double thGyro_ = 18;

};

}

#endif // MADGWICK_FILTER_H