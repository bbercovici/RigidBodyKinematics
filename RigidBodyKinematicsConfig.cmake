
if (EXISTS /home/bebe0705/.am_fortuna)
	set(IS_FORTUNA ON)
	message("-- This is Fortuna")

else()
	set(IS_FORTUNA OFF)
endif()


if (${IS_FORTUNA})
	set(RBK_INCLUDE_HEADER /home/bebe0705/libs/local/include/RigidBodyKinematics.hpp)
	set(RBK_LIBRARY /home/bebe0705/libs/local/lib/libRigidBodyKinematics.so)
else()
	set(RBK_INCLUDE_HEADER /usr/local/include/RigidBodyKinematics.hpp)

	if (APPLE)
		set(RBK_LIBRARY /usr/local/lib/libRigidBodyKinematics.dylib)
	elseif(UNIX AND NOT APPLE)
		set(RBK_LIBRARY /usr/local/lib/libRigidBodyKinematics.so)
	else()
		message(FATAL_ERROR "Unsupported platform")
	endif()

endif()


message("-- Found RigidBodyKinematics: " ${RBK_LIBRARY})
