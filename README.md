# RigidBodyKinematics
Implementation of a handful of useful rigid body kinematics routines relying on Armadillo (Required). CMake is also required.

Installation: 

1. Clone or download this repository 
2. Edit the install location (set by default to `/usr/local/lib`) in CMakeLists.txt if need be.
3. `make`
4. `make install`

Usage:
1. Assuming that you are using CMake to build your project, add the following: 
`find_package(RigidBodyKinematics REQUIRED)`
`include_directories(${RBK_INCLUDE_DIRS})`
2. Add `#include <RigidBodyKinematics.hpp>` to your include lists 








