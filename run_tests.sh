if cd Tests; then
	if cd build; then
		echo "Building RigidBodyKinematics tests..."
		cmake ..
		cp Makefile /Users/bbercovici/Desktop/.
		# make
		# ./Tests
	fi
fi