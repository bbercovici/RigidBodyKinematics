/**
@file   main.cpp
@Author Benjamin Bercovici (bebe0705@colorado.edu)
@date   July, 2017
@brief  main.cpp Implementation of the dcm_transform example
*/

#include <RigidBodyKinematics.hpp>
#include <armadillo>
#include <iostream>


int main() {

	// Transforming a set of 3-1-3 euler angles into a Direction Cosine Matrix
	arma::vec angles_313 = {0.1, 0.2, 0.3};
	arma::mat dcm = RBK::euler313_to_dcm(angles_313);

	// Converting the angles to degrees and recomputing the dcm
	arma::vec angles_313_deg = 180 / arma::datum::pi * angles_313;
	arma::mat dcm_deg = RBK::euler313d_to_dcm(angles_313_deg);

	// Ensuring that the two are the same
	std::cout << " Are the DCMs equal? " << std::endl;
	if (arma::approx_equal(dcm, dcm_deg, "absdiff", 1e-6)) {
		std::cout << " Yes "  << std::endl;
	}
	else {
		std::cout << " No "  << std::endl;
	}

	return 0;

}