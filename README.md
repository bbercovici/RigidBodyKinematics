# RigidBodyKinematics

Library implementation of a handful of useful Rigid Body Kinematics (RBK) routines relying on Armadillo.

## Requires
1. Armadillo
2. CMake

## Installation: 

### Mac users

   brew tap bbercovici/self
   brew update
   brew install rbk

### Linux & Mac users

    git clone https://github.com/bbercovici/RigidBodyKinematics.git
    cd RigidBodyKinematics/build
    cmake ..
    make
    make install

## Getting updates

### Mac users

Assuming that RBK was installed with Homebrew

    brew update
    brew upgrade rbk

### Linux & Mac users

    git pull
    cd build
    cmake ..
    make
    make install
    

## Usage:
1. In the CMakeLists.txt of your project: 

```
...
find_package(RigidBodyKinematics REQUIRED )
include_directories(${RBK_INCLUDE_DIRS})

...

# Linking against RigidBodyKinematics.
set(library_dependencies
${RBK_LIBRARY})

target_link_libraries(${EXE_NAME} ${library_dependencies})

```

2. Add `#include <RigidBodyKinematics.hpp>` to your includes


## License

[This software is distributed under the MIT License](https://choosealicense.com/licenses/mit/)




