// MIT License

// Copyright (c) 2018 Benjamin Bercovici 

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "Tests.hpp"
#include <armadillo>
#include <RigidBodyKinematics.hpp>
void Tests::test_euler321_to_mrp(){





};
void Tests::test_dXattitudedt(){

};
void Tests::test_domegadt(){

};
void Tests::test_dmrpdt(){
};
void Tests::test_shadow_mrp(){

};
void Tests::test_mrp_to_quat(){
};
void Tests::test_euler321d_to_dcm(){

		arma::vec theta_B = {30,-45,60};
		arma::vec theta_F = {10,25,-15};

		arma::mat BN = RBK::euler321d_to_dcm(theta_B);
		arma::mat FN = RBK::euler321d_to_dcm(theta_F);
		arma::mat BF = BN * FN.t();

		arma::vec theta_BF = RBK::dcm_to_euler321d(BF);
		
		assert(std::abs(theta_BF(0) + 0.933242)/0.933242 < 1e-6);
		assert(std::abs(theta_BF(1) + 72.3373)/72.3373 < 1e-6);
		assert(std::abs(theta_BF(2) - 79.9635)/79.9635 < 1e-6);



};
void Tests::test_euler313d_to_dcm(){
};
void Tests::test_euler313d_to_mrp(){
};
void Tests::test_euler321d_to_mrp(){





};
void Tests::test_euler313_to_mrp(){
};
void Tests::test_euler313_to_dcm(){
};
void Tests::test_tilde(){
};
void Tests::test_M1(){
};
void Tests::test_M2(){
};
void Tests::test_M3(){
};
void Tests::test_dcm_to_euler321(){
};
void Tests::test_dcm_to_euler313(){
};
void Tests::test_mrp_to_euler313(){
};
void Tests::test_mrp_to_euler321(){
};
void Tests::test_dcm_to_euler321d(){
};
void Tests::test_dcm_to_euler313d(){
};
void Tests::test_mrp_to_euler313d(){
};
void Tests::test_mrp_to_euler321d(){
};

void Tests::test_dcm_to_mrp(){

}

void Tests::test_longitude_latitude_to_dcm(){

}

void Tests::test_mrp_to_dcm(){
};

void Tests::test_dcm_to_quat(){

} ;
void Tests::test_dcm_to_prv(){

} ;
void Tests::test_prv_to_dcm(){
};
void Tests::test_prv_to_mrp(){
};
void Tests::test_Bmat(){
};
