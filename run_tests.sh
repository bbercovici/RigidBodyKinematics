#!/bin/bash
if cd Tests; then
	if cd build; then
		echo "Building RigidBodyKinematics tests..."
		cmake ..
		ls
		cp Makefile /Users/bbercovici/Desktop/
		# make
		# ./Tests
	fi
fi