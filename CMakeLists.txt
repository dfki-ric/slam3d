# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
rock_init(slam3d 0.1)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../g2o/cmake_modules)
rock_standard_layout()

include_directories(include)

set(CMAKE_BUILD_TYPE "Debug")

############################################################
# Define the slam3d library
############################################################
rock_library(slam3d
	SOURCES
		src/PoseGraph.cpp
		src/G2oSolver.cpp
		src/GraphMapper.cpp
	HEADERS
		include/Measurement.hpp
		include/PoseGraph.hpp
		include/Solver.hpp
		include/G2oSolver.hpp
		include/GraphMapper.hpp
	DEPS_PKGCONFIG 
		pcl_registration-1.7
		eigen3
		g2o
	DEPS_CMAKE
		Cholmod)

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