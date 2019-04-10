/*  
    MADGWICK FILTER CLASS
    This object contains functions to estimate imu quaternion from acc and gyro measurements.
*/

#include "softfoot_thing_visualization/madgwick_filter.h"

#define     DEBUG_MF        1       // Prints out additional info

using namespace softfoot_thing_visualization;

MadgwickFilter::MadgwickFilter(){

    // Initializing ros variables

}

MadgwickFilter::MadgwickFilter(ros::NodeHandle& nh){

    // Initializing ros variables
    if (!this->initialize(nh)) {
        ROS_FATAL("Could not initialize the Madgwick Filter!");
        nh.shutdown();
    }

}

MadgwickFilter::~MadgwickFilter(){

    // Nothing to do here

}

// Function to initialize filter
bool MadgwickFilter::initialize(ros::NodeHandle& nh){

    // Initializing ros variables
    this->mf_nh_ = nh;

    // Parsing needed parameters
    if (!this->parse_parameters(this->mf_nh_)) return false;

    return true;

}

// Function to parse parameters
bool MadgwickFilter::parse_parameters(ros::NodeHandle& nh){
    
    // TODO: parse needed params
    return true;

}

// Function to filter out relative quaternion form old relative quaternion, acc anf gyro
Eigen::Quaternion<float> MadgwickFilter::filter(Eigen::Vector3d acc_1, Eigen::Vector3d gyro_1,
    Eigen::Vector3d acc_2, Eigen::Vector3d gyro_2, Eigen::Quaternion<float> Q_rel_old){

    // P = 1 / N = 2

	float q1, q2 ,q3 ,q4;
    
    float dx, dy, dz;
    float sx, sy, sz;

    Eigen::Vector4d qL;                 // Q_rel_old in 4d
    qL(0) = Q_rel_old.w();
    qL(1) = Q_rel_old.x();
    qL(2) = Q_rel_old.y();
    qL(3) = Q_rel_old.z();

    Eigen::Vector3d aP = acc_1; 
    Eigen::Vector3d aN = acc_2;
    Eigen::Vector4d gP, gN, g;

    Eigen::Vector3d fa;
    Eigen::MatrixXd Ja(3,4); 
    Eigen::Vector4d qdot;

   	Eigen::Vector4d Napla;

    // Normalize 
 	aP = aP / aP.norm();
	aN = aN / aN.norm();

	gP(0)  = 0; 
	gP(1)  = gyro_1(0);  
	gP(2)  = gyro_1(1);  
	gP(3)  = gyro_1(2);

    gN(0)  = 0; 
    gN(1)  = gyro_2(0);  
    gN(2)  = gyro_2(1);  
    gN(3)  = gyro_2(2);

	gP = gP*(M_PI/180);
	gN = gN*(M_PI/180);
	
	// rotate the angular velocity
	g = QxQ(QxQ(qL, gP), ConjQ(qL)) - gN;	
	
	q1 = qL(0);  
	q2 = qL(1); 
	q3 = qL(2); 
	q4 = qL(3);
	
	//accelerometer
	dx = aN(0); 
	dy = aN(1); 
	dz = aN(2); 
	
	sx = aP(0); 
	sy = aP(1); 
	sz = aP(2); 
	
	fa(0) =  2*dx*(0.5 -q3*q3 -q4*q4) + 2*dy*(q1*q4 + q2*q3) + 2*dz*(q2*q4-q1*q3) - sx; 
	fa(1) =  2*dx*(q2*q3 -q1*q4) + 2*dy*(0.5 - q2*q2 - q4*q4) + 2*dz*(q1*q2 + q3*q4) - sy;
	fa(2) =  2*dx*(q1*q3 -q2*q4) + 2*dy*(q3*q4 - q1*q2) + 2*dz*(0.5 - q2*q2 -q3*q3) - sz; 

	// Compute the Jacobian
	Ja << 2*dy*q4-2*dz*q3,    2*dy*q3+2*dz*q4 ,        -4*dx*q3+2*dy*q2-2*dz*q1,  -4*dx*q4+2*dy*q1+2*dz*q2,
		  -2*dx*q4+2*dz*q2,   2*dx*q3-4*dy*q2+2*dz*q1, 2*dx*q2+2*dz*q4,           -2*dx*q1-4*dy*q4+2*dz*q3,
		  2*dx*q3-2*dy*q2,    2*dx*q4-2*dy*q1-4*dz*q2, 2*dx*q1+2*dy*q4-4*dz*q3,   2*dx*q2+2*dy*q3;


	// Compute the Napla
  	Napla = Ja.transpose() * fa;

	qdot = 0.5*QxQ( qL,g ) - ( beta_*Napla );				

	qL = qL + qdot / sampleFreq_;  
	
	qL = qL /qL.norm();

    // Convert qL back to Quaternion and return
	return Eigen::Quaternion<float>(qL(0), qL(1), qL(2), qL(3));

}