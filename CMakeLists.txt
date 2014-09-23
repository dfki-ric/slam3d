# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)
rock_init(slam3d 0.1)
rock_standard_layout()

rock_library(slam3d
	SOURCES
		src/PoseGraph.cpp
	HEADERS
		src/PoseGraph.hpp
	DEPS_PKGCONFIG 
        pcl_registration-1.7 eigen3)
