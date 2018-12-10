/** MIT License

Copyright (c) 2018 Benjamin Bercovici

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

#include "RigidBodyKinematics.hpp"
#include <math.h>

arma::mat::fixed<3,3> RBK::mrp_to_dcm(const arma::vec::fixed<3> & mrp) {	
	arma::mat::fixed<3,3> dcm = arma::eye<arma::mat>(3,3) + (8 * tilde(mrp) * (tilde(mrp))
		- 4 * (1 - pow(arma::norm(mrp), 2)) * tilde(mrp)) / pow(1 + pow(arma::norm(mrp), 2), 2);
	return dcm;
}

arma::mat::fixed<3,3> RBK::euler321_to_dcm(const arma::vec::fixed<3> & euler_angles) {
	arma::mat::fixed<3,3> M(RBK::M1(euler_angles(2)) * RBK::M2(euler_angles(1)) * RBK::M3(euler_angles(0)));
	return M;
}

arma::mat::fixed<3,3> RBK::longitude_latitude_to_dcm(const arma::vec::fixed<3> & euler_angles) {
	arma::mat::fixed<3,3> M(RBK::M2(-euler_angles(1)) * RBK::M3(euler_angles(0)));
	return M;
}


arma::mat::fixed<3,3> RBK::euler313_to_dcm(const arma::vec::fixed<3> & euler_angles) {
	arma::mat::fixed<3,3> M(RBK::M3(euler_angles(2)) * RBK::M1(euler_angles(1)) * RBK::M3(euler_angles(0)));
	return M;
}

arma::mat::fixed<3,3> RBK::euler321d_to_dcm(const arma::vec::fixed<3> & euler_angles) {
	double d_to_r = arma::datum::pi / 180.;
	arma::mat::fixed<3,3> M(RBK::M1(euler_angles(2)* d_to_r) * RBK::M2(euler_angles(1)* d_to_r) * RBK::M3(euler_angles(0)* d_to_r));
	return M;
}

arma::mat::fixed<3,3> RBK::euler313d_to_dcm(const arma::vec::fixed<3> & euler_angles) {
	double d_to_r = arma::datum::pi / 180.;
	arma::mat::fixed<3,3> M(RBK::M3(euler_angles(2) * d_to_r) * RBK::M1(euler_angles(1) * d_to_r) * RBK::M3(euler_angles(0) * d_to_r));
	return M;
}

arma::vec::fixed<4> RBK::mrp_to_quat(const arma::vec::fixed<3> & mrp) {
	return RBK::dcm_to_quat(RBK::mrp_to_dcm(mrp));
}

arma::mat::fixed<3,3> RBK::tilde(const arma::vec::fixed<3> & vec) {
	arma::mat::fixed<3,3> vec_tilde = {
		{0, - vec(2), vec(1)},
		{vec(2), 0, -vec(0)},
		{ - vec(1), vec(0), 0}
	};

	return vec_tilde;
}

arma::mat::fixed<3,3> RBK::M1(const double angle) {
	arma::mat::fixed<3,3> M = {
		{ 1, 0, 0},
		{0, cos(angle), sin(angle)},
		{0, - sin(angle), cos(angle)}
	};
	return M;
}


arma::mat::fixed<3,3> RBK::M2(const double angle) {
	arma::mat::fixed<3,3> M = {
		{ cos(angle), 0, - sin(angle)},
		{0, 1, 0},
		{sin(angle), 0, cos(angle)}
	};
	return M;
}

arma::mat::fixed<3,3> RBK::M3(const double angle) {
	arma::mat::fixed<3,3> M = {
		{cos(angle), sin(angle), 0},
		{ - sin(angle), cos(angle), 0},
		{0, 0, 1}
	};
	return M;
}

arma::vec::fixed<3> RBK::dmrpdt(double t, const arma::vec::fixed<6> & attitude_set ) {
	
	return 0.25 *  Bmat(attitude_set.subvec(0, 2)) * attitude_set.subvec(3, 5);
}

arma::mat::fixed<3,3> RBK::Bmat(const arma::vec::fixed<3> & mrp){	
	return ( (1 - arma::dot(mrp, mrp)) * arma::eye<arma::mat>(3, 3) + 2 * tilde(mrp) + 2 * mrp * mrp.t() );
}

arma::vec::fixed<3> RBK::domegadt(double t, 
	const arma::vec::fixed<6> & attitude_set, 
	const arma::mat::fixed<3,3> & inertia,
	const arma::vec::fixed<3> & L) {
	return arma::solve(inertia, - tilde(attitude_set.subvec(3,5)) * inertia * attitude_set.subvec(3,5) + L);
}

arma::vec::fixed<6> RBK::dXattitudedt(double t, const arma::vec::fixed<6> & attitude_set, 
	const arma::mat::fixed<3,3> & inertia,const arma::vec::fixed<3> & L) {

	arma::vec::fixed<6> dxdt;
	dxdt.subvec(0, 2) = RBK::dmrpdt(t, attitude_set);
	dxdt.subvec(3, 5) = RBK::domegadt(t, attitude_set, inertia,L);

	return dxdt;


}


arma::vec::fixed<3> RBK::shadow_mrp(const arma::vec::fixed<3> & mrp, bool force_switch) {
	if (arma::norm(mrp) > 1 || force_switch == true) {
		return - mrp / arma::dot(mrp, mrp);
	}
	else {
		return mrp;
	}
}

arma::vec::fixed<3> RBK::quat_to_mrp(const arma::vec::fixed<4> & Q , const bool short_rot) {
	arma::vec::fixed<3> mrp = {Q(1), Q(2), Q(3)};
	mrp = mrp / ( 1 + Q(0));

	if (short_rot == true) {
		if (arma::norm(mrp) > 1) {
			mrp = - mrp / pow(arma::norm(mrp), 2);
		}
	}

	return mrp;
}

arma::vec::fixed<4> RBK::dcm_to_quat(const arma::mat::fixed<3,3> & dcm) {

	arma::vec::fixed<4> Q = {0, 0, 0, 0};
	arma::vec::fixed<4> q_s = {
		1. / 4. * ( 1 + arma::trace(dcm) ),
		1. / 4. * (1 + 2 * dcm(0, 0) - arma::trace(dcm)),
		1. / 4. * (1 + 2 * dcm(1, 1) - arma::trace(dcm)),
		1. / 4. * (1 + 2 * dcm(2, 2) - arma::trace(dcm))
	};


	int max_coef_index = q_s.index_max();
	Q(max_coef_index) = std::sqrt(q_s.max());

	switch (max_coef_index) {
		case 0:
		Q(1) = 1. / 4. * (dcm(1, 2) - dcm(2, 1)) / Q(0);
		Q(2) = 1. / 4. * (dcm(2, 0) - dcm(0, 2)) / Q(0);
		Q(3) = 1. / 4. * (dcm(0, 1) - dcm(1, 0)) / Q(0);
		break;

		case 1:
		Q(0) = 1. / 4. * (dcm(1, 2) - dcm(2, 1)) / Q(1);
		Q(2) = 1. / 4. * (dcm(0, 1) + dcm(1, 0)) / Q(1);
		Q(3) = 1. / 4. * (dcm(2, 0) + dcm(0, 2)) / Q(1);
		break;

		case 2:
		Q(0) = 1. / 4. * (dcm(2, 0) - dcm(0, 2)) / Q(2);
		Q(1) = 1. / 4. * (dcm(0, 1) + dcm(1, 0)) / Q(2);
		Q(3) = 1. / 4. * (dcm(1, 2) + dcm(2, 1)) / Q(2);
		break;


		case 3:
		Q(0) = 1. / 4. * (dcm(0, 1) - dcm(1, 0)) / Q(3);
		Q(1) = 1. / 4. * (dcm(2, 0) + dcm(0, 2)) / Q(3);
		Q(2) = 1. / 4. * (dcm(1, 2) + dcm(2, 1)) / Q(3);
		break;

	}

	return Q;

}

arma::vec::fixed<3> RBK::dcm_to_prv(const arma::mat::fixed<3,3> & dcm) {
	double angle;
	arma::vec::fixed<3> axis;
	
	double	cos_angle = 0.5 * (arma::trace(dcm) - 1);
	
	if (cos_angle - 1 < 0){
		angle = std::acos(0.5 * (arma::trace(dcm) - 1));

		axis = {
			dcm(1, 2) - dcm(2, 1),
			dcm(2, 0) - dcm(0, 2),
			dcm(0, 1) - dcm(1, 0)
		};
		axis = 1. / (2 * std::sin(angle)) * axis;

	}

	else{
		angle = 0 ;
		axis = {1,0,0};
	}

	return angle * axis;
}

arma::vec::fixed<3> RBK::dcm_to_mrp(const arma::mat::fixed<3,3> & dcm, const bool short_rot) {
	return RBK::quat_to_mrp(RBK::dcm_to_quat(dcm), short_rot);
}

arma::vec::fixed<3> RBK::dcm_to_euler321(const arma::mat::fixed<3,3> & dcm) {
	arma::vec angles = {0, 0, 0};
	angles(0) = std::atan2(dcm(0, 1) , dcm(0, 0));
	angles(1) = - std::asin(dcm(0, 2));
	angles(2) = std::atan2(dcm(1, 2), dcm(2, 2));
	return angles;
}

arma::vec::fixed<3> RBK::dcm_to_euler313(const arma::mat::fixed<3,3> & dcm) {
	arma::vec angles = {0, 0, 0};
	angles(0) = std::atan2(dcm(2, 0) , - dcm(2, 1));
	angles(1) = std::acos(dcm(2, 2));
	angles(2) = std::atan2(dcm(0, 2), dcm(1, 2));

	return angles;
}

arma::vec::fixed<3> RBK::mrp_to_euler321(const arma::vec::fixed<3> & mrp) {
	return RBK::dcm_to_euler321(RBK::mrp_to_dcm(mrp));
}

arma::vec::fixed<3> RBK::mrp_to_euler313(const arma::vec::fixed<3> & mrp) {
	return RBK::dcm_to_euler313(RBK::mrp_to_dcm(mrp));
}


arma::vec::fixed<3> RBK::euler321_to_mrp(const arma::vec::fixed<3> & euler_angles) {
	return RBK::dcm_to_mrp(RBK::euler321_to_dcm(euler_angles));
}

arma::vec::fixed<3> RBK::euler313_to_mrp(const arma::vec::fixed<3> & euler_angles) {
	return RBK::dcm_to_mrp(RBK::euler313_to_dcm(euler_angles));
}

arma::vec::fixed<3> RBK::euler321d_to_mrp(const arma::vec::fixed<3> & euler_angles) {
	return RBK::dcm_to_mrp(RBK::euler321d_to_dcm(euler_angles));
}

arma::vec::fixed<3> RBK::euler313d_to_mrp(const arma::vec::fixed<3> & euler_angles) {
	return RBK::dcm_to_mrp(RBK::euler313d_to_dcm(euler_angles));
}

arma::vec::fixed<3> RBK::dcm_to_euler321d(const arma::mat::fixed<3,3> & dcm) {
	return 180. / arma::datum::pi * RBK::dcm_to_euler321(dcm);
}

arma::vec::fixed<3> RBK::dcm_to_euler313d(const arma::mat::fixed<3,3> & dcm) {
	return 180. / arma::datum::pi * RBK::dcm_to_euler313(dcm);
}

arma::vec::fixed<3> RBK::mrp_to_euler313d(const arma::vec::fixed<3> & mrp) {
	return RBK::dcm_to_euler313d(RBK::mrp_to_dcm(mrp));
}

arma::vec::fixed<3> RBK::mrp_to_euler321d(const arma::vec::fixed<3> & mrp) {
	return RBK::dcm_to_euler321d(RBK::mrp_to_dcm(mrp));
}

arma::vec::fixed<3> RBK::prv_to_mrp(const arma::vec::fixed<3> & prv) {
	return RBK::dcm_to_mrp(RBK::prv_to_dcm(prv));
}

arma::mat::fixed<3,3> RBK::prv_to_dcm(const arma::vec::fixed<3> & prv) {

	if (arma::norm(prv) > 0) {
		double Phi = arma::norm(prv);
		double Sigma = 1 - std::cos(Phi);
		arma::vec::fixed<3> e = prv / Phi;

		arma::mat::fixed<3,3> dcm = Sigma * e * e.t() + std::cos(Phi) * arma::eye<arma::mat>(3, 3) - std::sin(Phi) * tilde(e);


		return dcm;
	}

	else {
		return arma::eye<arma::mat>(3, 3);
	}


}

arma::mat::fixed<3,3> RBK::partial_mrp_dot_partial_mrp(const arma::vec::fixed<6> & attitude_set){

		const arma::vec::fixed<3> & mrp = attitude_set.subvec(0,2);
		const arma::vec::fixed<3> & omega = attitude_set.subvec(3,5);

		return 0.5 * (- omega * mrp.t() - RBK::tilde(omega) + arma::eye<arma::mat>(3,3) * arma::dot(omega,mrp) + mrp * omega.t());

	}







