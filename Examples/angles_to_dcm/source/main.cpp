/** MIT License

Copyright (c) 2018 Benjamin Bercovici and Jay McMahon

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


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

	// Transforming a set of 313 euler angles sequence into a Direction Cosine Matrix
	arma::vec angles_313 = {0.1, 0.2, 0.3};
	arma::mat dcm = RBK::euler313_to_dcm(angles_313);

	// Converting the angles to degrees and recomputing the dcm
	arma::vec angles_313_deg = 180 / arma::datum::pi * angles_313;
	arma::mat dcm_deg = RBK::euler313d_to_dcm(angles_313_deg);

	// Ensuring that the two are the same
	std::cout << "Are the DCMs equal? ";
	if (arma::approx_equal(dcm, dcm_deg, "absdiff", 1e-6)) {
		std::cout << "Yes "  << std::endl;
	}
	else {
		std::cout << "No "  << std::endl;
	}

	// The dcm is then converted to a quaternion
	arma::vec quat = RBK::dcm_to_quat(dcm);

	// Is our quaternion of unit norm?
	std::cout << "Quaternion norm: " << arma::norm(quat) << std::endl;

	// The quaternion is then converted to a set of Modified Rodrigues Parameters
	arma::vec mrp = RBK::quat_to_mrp(quat);

	// The mrp is converted back to a set of 321 euler angles
	arma::vec angles_321 = RBK::mrp_to_euler321(mrp);

	// These angles are different from the 313 euler angles
	std::cout << "Are the 321 and 313 sequences the same? " << std::endl;
	std::cout << "321: " << angles_321.t();
	std::cout << "313: " << angles_313.t();

	// But these two sequences should be equivalent
	std::cout << "Are the 321 and 313 sequences equivalent? " ;

	if (arma::approx_equal(angles_313, RBK::mrp_to_euler313(RBK::euler321_to_mrp(angles_321)), "absdiff", 1e-6)) {
		std::cout << " Yes "  << std::endl;
	}
	else {
		std::cout << " No "  << std::endl;
	}










	return 0;

}