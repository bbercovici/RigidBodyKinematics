# RigidBodyKinematics

Library implementation of a handful of useful Rigid Body Kinematics (RBK) routines relying on Armadillo.

## Requires
1. Armadillo
2. CMake


## Installation: 

### Mac users

RBK can be retrieved from Homebrew:

`brew tap bbercovici/rbk`
`brew update`
`brew install rbk`

### Linux & Mac users

1. Clone or download this repository 
2. `make`
3. `make install`

## Getting updates

### Mac users

Assuming that RBK was installed with Homebrew

`brew update`
`brew upgrade rbk`

### Linux & Mac users

Assuming you have cloned this repository and did not apply any local changes to the library, cd to the corresponding folder
and type
1. `git pull`
2. `make`
3. `make install`

## Usage:
1. Assuming that you are using CMake to build your project, add the following to your main CMakeLists.txt (see Examples/): 

```

set(RBK_LOC "/usr/local/Cellar/rbk/*")

...

find_package(RigidBodyKinematics REQUIRED PATHS RBK_LOC)
include_directories(${RBK_INCLUDE_DIRS})

...

# Linking against RigidBodyKinematics.
set(library_dependencies
${RBK_LIBRARY})

target_link_libraries(${EXE_NAME} ${library_dependencies})

```

2. Add `#include <RigidBodyKinematics.hpp>` to your includes
3. Look at Examples/ for more insight into using RBK!


## License

[This software is distributed under the MIT License](https://choosealicense.com/licenses/mit/)




