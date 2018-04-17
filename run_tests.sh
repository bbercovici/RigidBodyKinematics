if cd Tests; then
	if cd build; then
		echo "Building RigidBodyKinematics tests..."
		cmake ..
		# make
		# ./Tests
	fi
fi