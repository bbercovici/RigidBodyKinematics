# RigidBodyKinematics
Implementation of a handful of useful rigid body kinematics routines relying on Armadillo (Required). CMake is also required.

## Requires
1. Armadillo
2. CMake


## Installation: 

1. Clone or download this repository 
2. Edit the install location (set by default to `/usr/local/lib`) in CMakeLists.txt if need be.
3. `make`
4. `make install`

## Usage:
1. Assuming that you are using CMake to build your project, add the following: 

```
find_package(RigidBodyKinematics REQUIRED)
include_directories(${RBK_INCLUDE_DIRS})

...
...
...

# Linking against Armadillo, Boost and RigidBodyKinematics.
set(library_dependencies
${ARMADILLO_LIBRARIES}
${Boost_LIBRARIES}
${RBK_LIBRARY})

target_link_libraries(${EXE_NAME} ${library_dependencies})

```
2. Add `#include <RigidBodyKinematics.hpp>` to your includes
3. Look at Examples/ for more insight into using RBK!








