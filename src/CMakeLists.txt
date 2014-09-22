rock_library(slam3d
	SOURCES
		PoseGraph.cpp
	HEADERS
		PoseGraph.hpp
	DEPS_PKGCONFIG 
        pcl_registration-1.7 eigen3)
