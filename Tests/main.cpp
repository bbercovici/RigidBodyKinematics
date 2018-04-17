
#include <RigidBodyKinematics.hpp>
#include "Tests.hpp"
#include <cassert>

bool run_tests();

int main(){


	std::cout << "--- Running RBK tests...\n";

	run_tests();

	std::cout << "--- RBK tests successful\n";

	return 0;
}

bool run_tests(){

	Tests::test_euler321_to_mrp();
	Tests::test_dXattitudedt() ;
	Tests::test_domegadt() ;
	Tests::test_dmrpdt();
	Tests::test_shadow_mrp() ;
	Tests::test_mrp_to_quat();
	Tests::test_euler321d_to_dcm();
	Tests::test_euler313d_to_dcm();
	Tests::test_euler313d_to_mrp();
	Tests::test_euler321d_to_mrp();
	Tests::test_mrp_to_dcm();
	Tests::test_dcm_to_mrp();
	Tests::test_euler313_to_mrp();
	Tests::test_euler313_to_dcm();
	Tests::test_longitude_latitude_to_dcm();
	Tests::test_tilde();
	Tests::test_M1();
	Tests::test_M2();
	Tests::test_M3();
	Tests::test_dcm_to_euler321();
	Tests::test_dcm_to_euler313();
	Tests::test_mrp_to_euler313();
	Tests::test_mrp_to_euler321();
	Tests::test_dcm_to_euler321d();
	Tests::test_dcm_to_euler313d();
	Tests::test_mrp_to_euler313d();
	Tests::test_mrp_to_euler321d();
	Tests::test_dcm_to_quat() ;
	Tests::test_dcm_to_prv() ;
	Tests::test_prv_to_dcm();
	Tests::test_prv_to_mrp();
	Tests::test_Bmat();

	return true;
}


