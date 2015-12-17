# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
set(ROCK_TEST_ENABLED ON)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

rock_init(slam3d 0.1)
rock_standard_layout()

############################################################
# Define the slam3d library
############################################################
rock_library(slam3d
	SOURCES
		src/GraphMapper.cpp
		src/PointCloudSensor.cpp
		src/LineScanSensor.cpp
		src/G2oSolver.cpp
	HEADERS
		src/Clock.hpp
		src/Logger.hpp
		src/FileLogger.hpp
		src/Solver.hpp
		src/G2oSolver.hpp
		src/GraphMapper.hpp
		src/Sensor.hpp
		src/PointCloudSensor.hpp
		src/LineScanSensor.hpp
		src/GICPConfiguration.hpp
		src/Types.hpp
		src/Odometry.hpp
	DEPS_PKGCONFIG 
		pcl_registration-1.7
		eigen3
		g2o
		graph_analysis
	DEPS_CMAKE
		Cholmod
)
		

############################################################
# Define a simple test executable
############################################################
rock_executable(dummy
	SOURCES
		test/dummy.cpp
	DEPS
		slam3d)

############################################################
# Build test cases
############################################################
rock_testsuite(TestPoseGraph
	SOURCES
		test/pose_graph.cpp
	DEPS
		slam3d)

rock_testsuite(TestSolver
	SOURCES
		test/solver.cpp
	DEPS
		slam3d)

rock_testsuite(TestPclSensor
	SOURCES
		test/pcl_sensor.cpp
	DEPS
		slam3d)
