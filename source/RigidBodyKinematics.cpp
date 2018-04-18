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

arma::mat RBK::mrp_to_dcm(const arma::vec & sigma) {
	arma::mat identity(3, 3);
	identity = identity.eye();

	arma::mat dcm = identity + (8 * tilde(sigma) * (tilde(sigma))
		- 4 * (1 - pow(arma::norm(sigma), 2)) * tilde(sigma)) / pow(1 + pow(arma::norm(sigma), 2), 2);
	return dcm;
}

arma::mat RBK::euler321_to_dcm(const arma::vec & euler_angles) {
	arma::mat M(RBK::M1(euler_angles(2)) * RBK::M2(euler_angles(1)) * RBK::M3(euler_angles(0)));
	return M;
}

arma::mat RBK::longitude_latitude_to_dcm(const arma::vec & euler_angles) {
	arma::mat M(RBK::M2(-euler_angles(1)) * RBK::M3(euler_angles(0)));
	return M;
}


arma::mat RBK::euler313_to_dcm(const arma::vec & euler_angles) {
	arma::mat M(RBK::M3(euler_angles(2)) * RBK::M1(euler_angles(1)) * RBK::M3(euler_angles(0)));
	return M;
}

arma::mat RBK::euler321d_to_dcm(arma::vec & euler_angles) {
	euler_angles = arma::datum::pi / 180. * euler_angles;
	arma::mat M(RBK::M1(euler_angles(2)) * RBK::M2(euler_angles(1)) * RBK::M3(euler_angles(0)));
	return M;
}

arma::mat RBK::euler313d_to_dcm(arma::vec & euler_angles) {
	euler_angles = arma::datum::pi / 180. * euler_angles;

	arma::mat M(RBK::M3(euler_angles(2)) * RBK::M1(euler_angles(1)) * RBK::M3(euler_angles(0)));
	return M;
}

arma::vec RBK::mrp_to_quat(const arma::vec & mrp) {
	return RBK::dcm_to_quat(RBK::mrp_to_dcm(mrp));
}

arma::mat RBK::tilde(const arma::vec & x) {
	arma::mat vec_tilde = {
		{0, - x(2), x(1)},
		{x(2), 0, -x(0)},
		{ - x(1), x(0), 0}
	};

	return vec_tilde;
}

arma::mat RBK::M1(const double angle) {
	/**
	Returns the matrix of the elemental M1 rotation
	@param euler_angle Euler angle
	@return m1 Elemental rotation matrix
	*/
	arma::mat M = {
		{ 1, 0, 0},
		{0, cos(angle), sin(angle)},
		{0, - sin(angle), cos(angle)}
	};
	return M;
}


arma::mat RBK::M2(const double angle) {
	/**
	Returns the matrix of the elemental M2 rotation
	@param euler_angle Euler angle
	@return m2 Elemental rotation matrix
	*/
	arma::mat M = {
		{ cos(angle), 0, - sin(angle)},
		{0, 1, 0},
		{sin(angle), 0, cos(angle)}
	};
	return M;
}

arma::mat RBK::M3(const double angle) {
	/**
	Returns the matrix of the elemental M3 rotation
	@param euler_angle Euler angle
	@return m3 Elemental rotation matrix
	*/
	arma::mat M = {
		{cos(angle), sin(angle), 0},
		{ - sin(angle), cos(angle), 0},
		{0, 0, 1}
	};
	return M;
}

arma::vec RBK::dmrpdt(double t, arma::vec attitude_set ) {
	arma::vec mrp = attitude_set.rows(0, 2);
	arma::vec omega = attitude_set.rows(3, 5);
	arma::mat I = arma::eye<arma::mat>(3, 3);

	return 0.25 *  Bmat(mrp) * omega;
}

arma::mat RBK::Bmat(const arma::vec & mrp){
	arma::mat I = arma::eye<arma::mat>(3, 3);
	
	return ( (1 - arma::dot(mrp, mrp)) * I + 2 * tilde(mrp) + 2 * mrp * mrp.t() );
}

arma::vec RBK::domegadt(double t, arma::vec attitude_set, arma::mat & inertia) {
	arma::vec omega = attitude_set.rows(3, 5);
	arma::vec omega_dot = arma::solve(inertia, - tilde(omega) * inertia * omega);

	return omega_dot;

}

arma::vec RBK::dXattitudedt(double t, arma::vec attitude_set, arma::mat inertia) {

	arma::vec dxdt(6);
	dxdt.rows(0, 2) = dmrpdt(t, attitude_set);
	dxdt.rows(3, 5) = domegadt(t, attitude_set, inertia);

	return dxdt;


}


arma::vec RBK::shadow_mrp(const arma::vec & mrp, bool force_switch) {
	if (arma::norm(mrp) > 1 || force_switch == true) {
		return - mrp / arma::dot(mrp, mrp);
	}
	else {
		return mrp;
	}
}

arma::vec RBK::quat_to_mrp(const arma::vec & Q , const bool short_rot) {
	arma::vec mrp = {Q(1), Q(2), Q(3)};
	mrp = mrp / ( 1 + Q(0));

	if (short_rot == true) {
		if (arma::norm(mrp) > 1) {
			mrp = - mrp / pow(arma::norm(mrp), 2);
		}
	}

	return mrp;
}

arma::vec RBK::dcm_to_quat(const arma::mat & dcm) {

	arma::vec Q = {0, 0, 0, 0};
	arma::vec q_s = {
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

arma::vec RBK::dcm_to_prv(const arma::mat & dcm) {
	double angle;
	arma::vec axis;
	
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

arma::vec RBK::dcm_to_mrp(const arma::mat & dcm, const bool short_rot) {
	return RBK::quat_to_mrp(RBK::dcm_to_quat(dcm), short_rot);
}

arma::vec RBK::dcm_to_euler321(const arma::mat & dcm) {
	arma::vec angles = {0, 0, 0};
	angles(0) = std::atan2(dcm(0, 1) , dcm(0, 0));
	angles(1) = - std::asin(dcm(0, 2));
	angles(2) = std::atan2(dcm(1, 2), dcm(2, 2));
	return angles;
}

arma::vec RBK::dcm_to_euler313(const arma::mat & dcm) {
	arma::vec angles = {0, 0, 0};
	angles(0) = std::atan2(dcm(2, 0) , - dcm(2, 1));
	angles(1) = std::acos(dcm(2, 2));
	angles(2) = std::atan2(dcm(0, 2), dcm(1, 2));

	return angles;
}

arma::vec RBK::mrp_to_euler321(const arma::vec & sigma) {
	return RBK::dcm_to_euler321(RBK::mrp_to_dcm(sigma));
}

arma::vec RBK::mrp_to_euler313(const arma::vec & sigma) {
	return RBK::dcm_to_euler313(RBK::mrp_to_dcm(sigma));
}


arma::vec RBK::euler321_to_mrp(const arma::vec & euler_angles) {
	return RBK::dcm_to_mrp(RBK::euler321_to_dcm(euler_angles));
}

arma::vec RBK::euler313_to_mrp(const arma::vec & euler_angles) {
	return RBK::dcm_to_mrp(RBK::euler313_to_dcm(euler_angles));
}

arma::vec RBK::euler321d_to_mrp(arma::vec & euler_angles) {
	return RBK::dcm_to_mrp(RBK::euler321d_to_dcm(euler_angles));
}

arma::vec RBK::euler313d_to_mrp(arma::vec & euler_angles) {
	return RBK::dcm_to_mrp(RBK::euler313d_to_dcm(euler_angles));
}

arma::vec RBK::dcm_to_euler321d(const arma::mat & dcm) {
	return 180. / arma::datum::pi * RBK::dcm_to_euler321(dcm);
}

arma::vec RBK::dcm_to_euler313d(const arma::mat & dcm) {
	return 180. / arma::datum::pi * RBK::dcm_to_euler313(dcm);
}

arma::vec RBK::mrp_to_euler313d(const arma::vec & sigma) {
	return RBK::dcm_to_euler313d(RBK::mrp_to_dcm(sigma));
}

arma::vec RBK::mrp_to_euler321d(const arma::vec & sigma) {
	return RBK::dcm_to_euler321d(RBK::mrp_to_dcm(sigma));
}

arma::vec RBK::prv_to_mrp(const arma::vec & prv) {
	return RBK::dcm_to_mrp(RBK::prv_to_dcm(prv));
}

arma::mat RBK::prv_to_dcm(const arma::vec & prv) {

	if (arma::norm(prv) > 0) {
		double Phi = arma::norm(prv);
		double Sigma = 1 - std::cos(Phi);
		arma::vec e = prv / Phi;

		arma::mat dcm = Sigma * e * e.t() + std::cos(Phi) * arma::eye<arma::mat>(3, 3) - std::sin(Phi) * tilde(e);


		return dcm;
	}

	else {
		return arma::eye<arma::mat>(3, 3);
	}


}







