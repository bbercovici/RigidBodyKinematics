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
 * @file   RigidBodyKinematics.hpp
 * @Author Benjamin Bercovici (bebe0705@colorado.edu)
 * @date   July, 2017
 * @brief  Header of the RigidBodyKinematics libary
 *
 * Rigid Body Kinematics library implementating a handful of useful rigid body routines
 */


#ifndef RIGIDBODYKINEMATICS_HPP
#define RIGIDBODYKINEMATICS_HPP

#include <armadillo>

namespace RBK {


/**
Converts MRP to DCM
@param sigma MRP vector
@return dcm DCM matrix
*/
	arma::mat::fixed<3,3> mrp_to_dcm(const arma::vec::fixed<3> & mrp);

/**
Converts DCM to MRP.
@param dcm DCM
@param short_rot True if short rotation is desired (default), false otherwise
@return mrp MRP set
*/
	arma::vec::fixed<3> dcm_to_mrp(const arma::mat::fixed<3,3> & dcm, const bool short_rot = true);

/**
Converts Quaternions to MRP.
@param Q Unit Quaternion
@param short_rot True if short rotation is desired (default), false otherwise
@return mrp MRP set
*/
	arma::vec::fixed<3> quat_to_mrp(const arma::vec::fixed<4> & Q , const bool short_rot = true);

/**
Converts a set of 321 Euler angles to DCM
@param euler_angles 321 sequence of Euler angles (rad)
@return dcm DCM
*/
	arma::mat::fixed<3,3> euler321_to_dcm(const arma::vec::fixed<3> & euler_angles);


/**
Converts a set of (longitude,latitude) angles to DCM
@param (longitude,latitude) angles (rad)
@return dcm DCM
*/
	arma::mat::fixed<3,3> longitude_latitude_to_dcm(const arma::vec::fixed<3> & euler_angles);


/**
Converts a set of 321 Euler angles to mrp
@param euler_angles 321 sequence of Euler angles (rad)
@return mrp MRP set
*/
	arma::vec::fixed<3> euler321_to_mrp(const arma::vec::fixed<3> & euler_angles);

/**
Computes the time derivative of the attitude set
assuming torque free rotational dynamics and MRP as attitude coordinate
@param[in] t Current time
@param[in] attitude_set mrp + angular velocities
@param[in] inertia inertia matrix
@param[in] L external torque (defaults to (0,0,0))
@return time derivative of the input attitude set
*/
	arma::vec::fixed<6> dXattitudedt(double t, const arma::vec::fixed<6> & attitude_set, const arma::mat::fixed<3,3> & inertia,
		const arma::vec::fixed<3> & L = arma::zeros<arma::vec>(3)) ;

/**
Computes the time derivative of the angular velocity set
@param[in] t Current time
@param[in] attitude_set mrp + angular velocities
@param[in] inertia inertia matrix
@param[in] L external torque (defaults to (0,0,0))
@return time derivative of the input angular velocity
*/
	arma::vec::fixed<3> domegadt(double t, 
		const arma::vec::fixed<6> & attitude_set, 
		const arma::mat::fixed<3,3> & inertia,
		const arma::vec::fixed<3> & L = arma::zeros<arma::vec>(3)) ;

/**
Computes the time derivative of a mrp set given
a corresponding angular velocity
@param t Current time
@param attitude_set mrp + angular velocities
@return time derivative of the input mrp
*/
	arma::vec::fixed<3> dmrpdt(double t, const arma::vec::fixed<6> & attitude_set );


/**
Returns the shadow set of the input mrp if crossing
surface is reached
@param mrp MRP set
@param force_switch if true, will force the switching of the MRP to its shadow without checking its norm
@return mrp or its shadow set

*/
	arma::vec::fixed<3> shadow_mrp(const arma::vec::fixed<3> & mrp, bool force_switch = false) ;


/**
Converts MRP to quaternions
@param sigma MRP vector
@return quat Unit quaternion
*/
	arma::vec::fixed<4> mrp_to_quat(const arma::vec::fixed<3> & mrp);


/**
Converts a set of 321 Euler angles to DCM
@param euler_angles 321 sequence of Euler angles (deg)
@return dcm DCM
*/
	arma::mat::fixed<3,3> euler321d_to_dcm(const arma::vec::fixed<3> & euler_angles);

/**
Converts a set of 321 Euler angles to mrp
@param euler_angles 321 sequence of Euler angles (deg)
@return mrp MRP set
*/
	arma::vec::fixed<3> euler321d_to_mrp(const arma::vec::fixed<3> & euler_angles);


/**
Converts a set of 313 Euler angles to DCM
@param euler_angles 313 sequence of Euler angles (deg)
@return dcm DCM
*/
	arma::mat::fixed<3,3> euler313d_to_dcm(const arma::vec::fixed<3> & euler_angles);

/**
Converts a set of 313 Euler angles to mrp
@param euler_angles 313 sequence of Euler angles (deg)
@return mrp MRP set
*/
	arma::vec::fixed<3> euler313d_to_mrp(const arma::vec::fixed<3> & euler_angles);

/**
Converts a set of 321 Euler angles expressed in degrees to DCM
@param euler_angles 321 sequence of Euler angles (deg)
@return dcm DCM
*/
	arma::mat::fixed<3,3> euler321d_to_dcm(const arma::vec::fixed<3> & euler_angles);

/**
Converts a set of 321 Euler angles expressed in degrees to mrp
@param euler_angles 321 sequence of Euler angles (deg)
@return mrp MRP set
*/
	arma::vec::fixed<3> euler321d_to_mrp(const arma::vec::fixed<3> & euler_angles);


/**
Converts a set of 313 Euler angles to mrp
@param euler_angles 313 sequence of Euler angles (rad)
@return mrp MRP set
*/
	arma::vec::fixed<3> euler313_to_mrp(const arma::vec::fixed<3> & euler_angles);


/**
Converts a set of 313 Euler angles to DCM
@param euler_angles 313 sequence of Euler angles (rad)
@return dcm DCM
*/
	arma::mat::fixed<3,3> euler313_to_dcm(const arma::vec::fixed<3> & euler_angles);


/**
Returns the matrix tilde[x] of the linear operator v|---> cross(x,v)
@param vec 3-by-1 vector
@return M Skew-symmetric matrix of the said linear operator
*/
	arma::mat::fixed<3,3> tilde(const arma::vec::fixed<3> & vec);

/**
Matrix of the elemental rotation about the first axis of the current frame
@param angle Rotation angle (rad)
@return M Elemental rotation matrix
*/
	arma::mat::fixed<3,3> M1(const double angle);

/**
Matrix of the elemental rotation about the second axis of the current frame
@param angle Rotation angle (rad)
@return M Elemental rotation matrix
*/
	arma::mat::fixed<3,3> M2(const double angle);

/**
Matrix of the elemental rotation about the third axis of the current frame
@param angle Rotation angle (rad)
@return M Elemental rotation matrix
*/
	arma::mat::fixed<3,3> M3(const double angle);

/**
Converts a DCM to the corresponding set of Euler angles
@param m DCM
@return angles Sequence of 321 Euler angles angles [yaw,pitch,roll]
*/
	arma::vec::fixed<3> dcm_to_euler321(const arma::mat::fixed<3,3> & dcm);

/**
Converts a DCM to the corresponding set of Euler angles
@param m DCM
@return angles Sequence of 313 Euler angles angles [right ascension,inclination,longitude]
*/
	arma::vec::fixed<3> dcm_to_euler313(const arma::mat::fixed<3,3> & dcm);

/**
Converts a MRP to the corresponding set of Euler angles
@param sigma MRP vector
@return angles Sequence of 313 Euler angles angles [right ascension,inclination,longitude]
*/
	arma::vec::fixed<3> mrp_to_euler313(const arma::vec::fixed<3> & mrp);

/**
Converts a MRP to the corresponding set of Euler angles
@param sigma MRP vector
@return angles Sequence of 321 Euler angles angles [right ascension,inclination,longitude]
*/
	arma::vec::fixed<3> mrp_to_euler321(const arma::vec::fixed<3> & mrp);

/**
Converts a DCM to the corresponding set of Euler angles in degrees
@param m DCM
@return angles Sequence of 321 Euler angles angles [yaw,pitch,roll]
*/
	arma::vec::fixed<3> dcm_to_euler321d(const arma::mat::fixed<3,3> & dcm);

/**
Converts a DCM to the corresponding set of Euler angles in degrees
@param m DCM
@return angles Sequence of 313 Euler angles angles [right ascension,inclination,longitude]
*/
	arma::vec::fixed<3> dcm_to_euler313d(const arma::mat::fixed<3,3> & dcm);

/**
Converts a MRP to the corresponding set of Euler angles in degrees
@param sigma MRP vector
@return angles Sequence of 313 Euler angles angles [right ascension,inclination,longitude]
*/
	arma::vec::fixed<3> mrp_to_euler313d(const arma::vec::fixed<3> & mrp);

/**
Converts a MRP to the corresponding set of Euler angles in degrees
@param sigma MRP vector
@return angles Sequence of 321 Euler angles angles [right ascension,inclination,longitude]
*/
	arma::vec::fixed<3> mrp_to_euler321d(const arma::vec::fixed<3> & mrp);

/**
Converts a DCM to a quaternion corresponding to the short-path rotation
@param dcm DCM
@return Q Unit quaternion
*/
	arma::vec::fixed<4> dcm_to_quat(const arma::mat::fixed<3,3> & dcm) ;

/**
Converts a dcm to the principal rotation vector
@param dcm DCM
@return prv Principal rotation vector
*/
	arma::vec::fixed<3> dcm_to_prv(const arma::mat::fixed<3,3> & dcm) ;

/**
Converts a PRV to the corresponding DCM
@param prv Principal rotation vector
@return DCM
*/

	arma::mat::fixed<3,3> prv_to_dcm(const arma::vec::fixed<3> & prv);


/**
Converts a PRV to a well-behaved MRP set
@param prv Principal rotation vector
@return MRP set
*/

	arma::vec::fixed<3> prv_to_mrp(const arma::vec::fixed<3> & prv);

/**
Returns the B matrix in the evaluation of the MRP's time derivative (sigma_dot = 1/4 * Bmat(sigma) * omega)
@param mrp MRP set
@return instantiated B mtrix
*/
	arma::mat::fixed<3,3> Bmat(const arma::vec::fixed<3> & mrp);

/**
Returns the partial derivative of mrp_dot with respect to the mrp
@param attitude_set attitude set comprised of the mrp set and its associated angular velocity
@return partial derivative of mrp_dot with respect to the mrp  
*/
	arma::mat::fixed<3,3> partial_mrp_dot_partial_mrp(const arma::vec::fixed<6> & attitude_set);



}


#endif


