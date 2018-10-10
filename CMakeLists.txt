cmake_minimum_required(VERSION 2.6)
cmake_policy(VERSION 3.5.1)

project(SLAM3D VERSION "1.0.0")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(PCL 1.7 REQUIRED COMPONENTS registration)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Cholmod REQUIRED)
find_package(G2O REQUIRED)

find_package(OpenMP)
if(OPENMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_library(slam3d
	src/Graph.cpp
	src/BoostGraph.cpp
	src/PointCloudSensor.cpp
	src/G2oSolver.cpp
	src/g2o/edge_direction_prior.cpp
)

target_compile_features(slam3d
	PUBLIC
		cxx_alias_templates
)

target_include_directories(slam3d
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
		$<INSTALL_INTERFACE:include/slam3d>
		${EIGEN3_INCLUDE_DIR}
		${PCL_INCLUDE_DIRS}
		${CHOLMOD_INCLUDE_DIR}
)

target_link_libraries(slam3d
	PRIVATE
		CHOLMOD_LIBRARIES
		G2O_CORE_LIBRARY
		G2O_STUFF_LIBRARY
		G2O_SOLVER_CHOLMOD
		G2O_TYPES_SLAM3D
		PCL_REGISTRATION_LIBRARIES
)

# Install the binaries
install(TARGETS slam3d EXPORT slam3d-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Export the library interface
install(EXPORT slam3d-targets
	NAMESPACE slam3d::
	DESTINATION lib/cmake/slam3d
	FILE slam3d-config.cmake
)

# Create and install the version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file("slam3d-version.cmake"
	VERSION ${SLAM3D_VERSION}
	COMPATIBILITY SameMajorVersion
)
install(FILES ${PROJECT_BINARY_DIR}/slam3d-version.cmake
	DESTINATION lib/cmake/slam3d)

# Install header files
install(DIRECTORY src/
	DESTINATION include/slam3d
	FILES_MATCHING PATTERN "*.hpp"
)

# Install catkin package-manifest
install(FILES package.xml
	DESTINATION share/slam3d
)

# Install pkg-config file
configure_file(slam3d.pc.in slam3d.pc @ONLY)
install(FILES ${PROJECT_BINARY_DIR}/slam3d.pc
	DESTINATION lib/pkgconfig
)

