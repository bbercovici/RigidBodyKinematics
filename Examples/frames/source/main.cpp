/**
@file   main.cpp
@Author Benjamin Bercovici (bebe0705@colorado.edu)
@date   July, 2017
@brief  main.cpp Implementation of the frames example
*/

#include <RigidBodyKinematics.hpp>
#include <armadillo>
#include <iostream>


int main() {

	// This example features 2 frames: an inertial frame of reference "N" and a body frame "B"

	// Orienting B with respect to N means
	double yaw = 0.1;
	double pitch = 0.1;
	double roll = 0.1;

	arma::vec angles_321 = {yaw, pitch, roll};
	arma::mat dcm_BN = RBK::euler321_to_dcm(angles_321);

	// The position of B's origin is also expressed in N
	arma::vec origin_B_in_N = {150, 20, 40};

	// Some vector, for instance representative of a position in the inertial frame
	arma::vec pos_N = {150, 120, 20};

	// This vector is expressed in the B frame
	arma::vec pos_B = dcm_BN * (pos_N - origin_B_in_N);

	


	return 0;

}