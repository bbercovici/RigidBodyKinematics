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
#include <cassert>

void Tests::test_euler321_to_mrp(){


}
void Tests::test_dXattitudedt(){

}
void Tests::test_domegadt(){

}

void Tests::test_dmrpdt(){

}

void Tests::test_shadow_mrp(){

	arma::vec sigma = {0.1,0.4,0.3};

	arma::vec sigma_s_b = 2 * sigma / arma::norm(sigma);
	arma::vec sigma_s_a = RBK::shadow_mrp(sigma_s_b);

	assert(arma::norm(sigma - RBK::shadow_mrp(sigma)) == 0);
	assert(arma::norm(sigma_s_a  + sigma_s_b / arma::dot(sigma_s_b,sigma_s_b)) == 0);
	assert(arma::norm(RBK::shadow_mrp(sigma,true)  + sigma / arma::dot(sigma,sigma)) == 0);

}

void Tests::test_mrp_to_quat(){

	arma::vec sigma = {-0.0742431,0.103306,0.0573479};
	arma::vec quat = {0.961698,-0.145650,0.202665,0.112505};
	assert(arma::norm(quat - RBK::mrp_to_quat(sigma)) < 9e-3);

}

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

}
void Tests::test_euler313d_to_dcm(){
}
void Tests::test_euler313d_to_mrp(){
}
void Tests::test_euler321d_to_mrp(){

}
void Tests::test_euler313_to_mrp(){
}
void Tests::test_euler313_to_dcm(){
}
void Tests::test_tilde(){

	arma::vec x = {1,2,3};
	arma::vec y = {3,-1,0};

	arma::mat tilde_x = RBK::tilde(x);
	assert(arma::norm(tilde_x * x)< 1e-10);
	assert(arma::norm(tilde_x * y - arma::cross(x,y)) < 1e-10);
}
void Tests::test_M1(){
}
void Tests::test_M2(){
}
void Tests::test_M3(){
}
void Tests::test_dcm_to_euler321(){
}
void Tests::test_dcm_to_euler313(){
}
void Tests::test_mrp_to_euler313(){
}
void Tests::test_mrp_to_euler321(){
}
void Tests::test_dcm_to_euler321d(){
}
void Tests::test_dcm_to_euler313d(){
}
void Tests::test_mrp_to_euler313d(){
}
void Tests::test_mrp_to_euler321d(){
}

void Tests::test_dcm_to_mrp(){

	arma::vec sigma = {0.317587,-0.281456,0.230736};
	arma::mat dcm = arma::eye<arma::mat>(3,3) + (8 * RBK::tilde(sigma) * RBK::tilde(sigma) - 4 * (1 - arma::dot(sigma,sigma)) * RBK::tilde(sigma))/std::pow(1 + arma::dot(sigma,sigma),2);


	arma::mat error = dcm* RBK::mrp_to_dcm(sigma).t();
	assert(std::abs(arma::det(dcm) - 1) < 1e-5);
	assert(std::abs(3 - arma::trace(error))/3 < 1e-4);
	assert(arma::abs(error - arma::diagmat(arma::ones<arma::vec>(3))).max() < 1e-4);

}

void Tests::test_longitude_latitude_to_dcm(){

}

void Tests::test_mrp_to_dcm(){

	arma::vec sigma = {0.317587,-0.281456,0.230736};
	arma::mat dcm_0 = {
		{0.303372,-0.004942,0.952859},
		{-0.935315,0.189535,0.298769},
		{-0.182075,-0.981862,0.052877}
	};

	arma::mat dcm_1 = RBK::mrp_to_dcm(sigma);

	arma::mat error = dcm_0 * dcm_1.t();
	assert(std::abs(arma::det(dcm_1) - 1) < 1e-5);
	assert(std::abs(3 - arma::trace(error))/3 < 1e-4);
	assert(arma::abs(error - arma::diagmat(arma::ones<arma::vec>(3))).max() < 1e-4);


}

void Tests::test_dcm_to_quat(){

	arma::mat BN = {
		{0.612472, 0.353553,0.707107},
		{-0.780330,0.126826,0.612372},
		{0.126826,-0.926777,0.353553}
	};

	arma::vec quat = RBK::dcm_to_quat(BN);
	arma::mat dcm = RBK::mrp_to_dcm(RBK::quat_to_mrp(quat));
	arma::mat error = dcm* BN.t();
	assert(std::abs(arma::det(dcm) - 1) < 1e-5);
	assert(std::abs(3 - arma::trace(error))/3 < 1e-4);
	assert(arma::abs(error - arma::diagmat(arma::ones<arma::vec>(3))).max() < 1e-4);

}

void Tests::test_dcm_to_prv(){
	arma::mat dcm_0 = arma::eye<arma::mat>(3,3);
	arma::vec prv_0 = RBK::dcm_to_prv(dcm_0);

	assert(arma::norm(prv_0) < 1e-10);

	arma::vec euler_321d = {10,25,-15};
	arma::mat BN = RBK::euler321d_to_dcm(euler_321d);
	arma::vec prv_BN = RBK::dcm_to_prv(BN);
	arma::vec e = {-0.532035,0.740302,0.410964};
	assert(std::abs(arma::norm(prv_BN) * 180. / arma::datum::pi - 31.7762)/31.7762 < 1e-5);
	assert(std::abs(arma::norm(e) -1) < 1e-6);

}

void Tests::test_prv_to_dcm(){

	arma::vec e = {-0.532035,0.740302,0.410964};
	double angle = 0.55460;
	arma::vec prv_BN  = e * angle;

	arma::mat dcm = RBK::prv_to_dcm(prv_BN);
	assert(std::abs(dcm(0,0) - 0.892539)/892539 < 1e-6);

}

void Tests::test_prv_to_mrp(){

	arma::vec prv = {0.3,-0.2,0.1};
	double angle = arma::norm(prv);
	arma::vec axis = arma::normalise(prv);
	arma::vec mrp = std::tan(angle / 4) * axis;
	assert(arma::norm(RBK::prv_to_mrp(prv) - mrp) / arma::norm(mrp) < 1e-6);


}
void Tests::test_Bmat(){
}
