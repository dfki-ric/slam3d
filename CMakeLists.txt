# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
rock_init(slam3d 0.1)
rock_standard_layout()

include_directories(include)

# Define the slam3d library
rock_library(slam3d
	SOURCES
		src/PoseGraph.cpp
	HEADERS
		include/Measurement.hpp
		include/PoseGraph.hpp
	DEPS_PKGCONFIG 
		pcl_registration-1.7
		eigen3)

# Build test cases
rock_testsuite(TestPoseGraph
SOURCES
  test/pose_graph.cpp
DEPS
  slam3d)