if cd Tests; then
	if cd build; then
		pwd
		echo "Building RigidBodyKinematics tests..."
		pwd
		cmake ..
		pwd
		make
		./Tests
	fi
fi