############################################################
# Define the slam3d library
############################################################

set(SLAM3D_INCLUDES
	BoostMapper.hpp
	Clock.hpp
	FileLogger.hpp
	G2oSolver.hpp
	GICPConfiguration.hpp
	GraphMapper.hpp
	Logger.hpp
	Odometry.hpp
	PointCloudSensor.hpp
	Sensor.hpp
	Solver.hpp
	Types.hpp)

set(SLAM3D_SOURCES
	GraphMapper.cpp
	BoostMapper.cpp
	PointCloudSensor.cpp
	G2oSolver.cpp)

find_package(PCL 1.7 REQUIRED COMPONENTS registration)
add_definitions(-DPCL_WITH_VIEWPOINT)

if(Rock_FOUND)
	rock_library(slam3d
		SOURCES
			${SLAM3D_SOURCES}
		HEADERS
			${SLAM3D_INCLUDES}
		DEPS_PKGCONFIG
			eigen3
			g2o
			flann
			pcl_registration-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
		DEPS_CMAKE
			Cholmod
	)
elseif(catkin_FOUND)
	
else()
	
endif()

