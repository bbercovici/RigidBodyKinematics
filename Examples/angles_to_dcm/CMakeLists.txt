
# @file   CMakeLists.txt
# @Author Benjamin Bercovici (bebe0705@colorado.edu)
# @date   July, 2017
# @brief  CMake listing enabling compilation and installation of the dcm_transform example
# 
cmake_minimum_required(VERSION 3.7)

get_filename_component(dirName ${CMAKE_CURRENT_SOURCE_DIR} NAME)
set(EXE_NAME ${dirName} CACHE STRING "Name of executable to be created.")

project(${EXE_NAME})

# Compiler flags
add_definitions(-Wall -O2 )
set(CMAKE_CXX_FLAGS "-std=c++11")

# Include directories
include_directories(include)

# Find Armadillo 
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

# Find RBK 
find_package(RigidBodyKinematics REQUIRED)
include_directories(${RBK_INCLUDE_DIRS})

# Add source files in root directory
add_executable(${EXE_NAME}
source/main.cpp)

# Linking
set(library_dependencies
${ARMADILLO_LIBRARIES}
${RBK_LIBRARY})

target_link_libraries(${EXE_NAME} ${library_dependencies})

