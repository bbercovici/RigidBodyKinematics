# RigidBodyKinematics

Shared library implementation of a handful of useful rigid body kinematics routines relying on Armadillo.

## Requires
1. Armadillo
2. CMake


## Installation: 

1. Clone or download this repository 
2. `make`
3. `make install`

## Usage:
1. Assuming that you are using CMake to build your project, add the following to your main CMakeLists.txt (see Examples/): 

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

## Getting updates
Assuming you have cloned this repository and did not apply any local changes to the library, cd to the corresponding folder
and type
1. `git pull`
2. `make`
3. `make install`


## License

[This software is distributed under the MIT License](https://choosealicense.com/licenses/mit/)




