cmake_minimum_required(VERSION 2.6)

set(SLAM3D_MAJOR_VERSION 0)
set(SLAM3D_MINOR_VERSION 1)
set(SLAM3D_PATCH_VERSION 0)
set(SLAM3D_VERSION ${SLAM3D_MAJOR_VERSION}.${SLAM3D_MINOR_VERSION}.${SLAM3D_PATCH_VERSION})

find_package(Rock)
find_package(catkin)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
message("CP1: ${CMAKE_MODULE_PATH}")

if(Rock_FOUND)
	message("=== Build using ROCK ===")
	set(ROCK_TEST_ENABLED ON)
	rock_init(slam3d ${SLAM3D_VERSION})
	rock_standard_layout()
elseif(catkin_FOUND)
	message("=== Build using Catkin ===")
	project(slam3d)	
	catkin_package(
		LIBRARIES slam3d
		DEPENDS Boost Eigen3 Cholmod G2O
	)
	add_subdirectory(src)
else()
	message("=== Build using standard CMake ===")
	project(slam3d)
	set(BUILD_SHARED_LIBS ON)
	add_subdirectory(src)
endif()

