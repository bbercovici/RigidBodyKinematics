
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