############################################################
# Build test cases
############################################################

rock_testsuite(TestSolver
	SOURCES
		solver.cpp
	DEPS
		slam3d)

rock_testsuite(TestPclSensor
	SOURCES
		pcl_sensor.cpp
	DEPS
		slam3d)
