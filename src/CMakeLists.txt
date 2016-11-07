############################################################
# Define the slam3d library
############################################################

find_package(PCL 1.7 REQUIRED COMPONENTS registration)
add_definitions(-DPCL_WITH_VIEWPOINT)

rock_library(slam3d
	SOURCES
		GraphMapper.cpp
		BoostMapper.cpp
		GraphAnalysisMapper.cpp
		PointCloudSensor.cpp
		G2oSolver.cpp
	HEADERS
		Clock.hpp
		Logger.hpp
		FileLogger.hpp
		Solver.hpp
		G2oSolver.hpp
		GraphMapper.hpp
		BoostMapper.hpp
		GraphAnalysisMapper.hpp
		Sensor.hpp
		PointCloudSensor.hpp
		GICPConfiguration.hpp
		Types.hpp
		Odometry.hpp
	DEPS_PKGCONFIG
		eigen3
		g2o
		flann
		graph_analysis
		pcl_registration-${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}
	DEPS_CMAKE
		Cholmod
)
