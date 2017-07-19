set(RBK_INCLUDE_HEADER /usr/local/include/RigidBodyKinematics.hpp)

if (APPLE)
	set(RBK_LIBRARY /usr/local/lib/libRigidBodyKinematics.dylib)
elseif(UNIX AND NOT APPLE)
	set(RBK_LIBRARY /usr/local/lib/libRigidBodyKinematics.so)
else()
	message(FATAL_ERROR "Unsupported platform")
endif()

message("-- Found RigidBodyKinematics: " ${RBK_LIBRARY})
