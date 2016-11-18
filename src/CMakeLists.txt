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
	list(APPEND CMAKE_MODULE_PATH ${slam3d_SOURCE_DIR}/cmake)
	find_package(Boost REQUIRED)
	find_package(Eigen3 REQUIRED)
	find_package(Cholmod REQUIRED)
	find_package(G2O REQUIRED)

	include_directories(
		${EIGEN3_INCLUDE_DIR}
		${PCL_INCLUDE_DIRS}
		${G2O_INCLUDE_DIR}
		${CHOLMOD_INCLUDE_DIR}
	)

	add_library(slam3d ${SLAM3D_SOURCES})
	target_link_libraries(slam3d
		${CHOLMOD_LIBRARIES}
		${G2O_CORE_LIBRARY}
		${G2O_TYPES_SLAM3D}
		${G2O_SOLVER_CHOLMOD}
		${PCL_REGISTRATION_LIBRARIES}
	)
	
	install(TARGETS slam3d
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)
	
	install(DIRECTORY .
		DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
		FILES_MATCHING PATTERN "*.hpp"
	)
else()
	
endif()

