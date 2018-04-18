#!/bin/bash
if cd Tests; then
	if cd build; then
		echo "Building RigidBodyKinematics tests..."
		cmake ..
		ls
		cat Makefile
		# make
		# ./Tests
	fi
fi