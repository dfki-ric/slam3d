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
else()
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
	
	if(catkin_FOUND)
		set(INSTALL_LIB_DIR ${CATKIN_PACKAGE_LIB_DESTINATION})
		set(INSTALL_BIN_DIR ${CATKIN_PACKAGE_BIN_DESTINATION})
		set(INSTALL_INC_DIR ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
	else()
		set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX}/lib)
		set(INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/bin)
		set(INSTALL_INC_DIR ${CMAKE_INSTALL_PREFIX}/include/slam3d)
	endif()
	
	message("${INSTALL_LIB_DIR}")
	message("${INSTALL_BIN_DIR}")
	message("${INSTALL_INC_DIR}")
	
	install(TARGETS slam3d
		ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
		LIBRARY DESTINATION ${INSTALL_LIB_DIR}
		RUNTIME DESTINATION ${INSTALL_BIN_DIR}
	)
	
	install(DIRECTORY .
		DESTINATION ${INSTALL_INC_DIR}
		FILES_MATCHING PATTERN "*.hpp"
	)
endif()

