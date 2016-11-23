cmake_minimum_required(VERSION 2.6)
project(slam3d)

set(SLAM3D_MAJOR_VERSION 0)
set(SLAM3D_MINOR_VERSION 1)
set(SLAM3D_PATCH_VERSION 0)
set(SLAM3D_VERSION ${SLAM3D_MAJOR_VERSION}.${SLAM3D_MINOR_VERSION}.${SLAM3D_PATCH_VERSION})

find_package(Rock)
find_package(catkin)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if(Rock_FOUND)
	message("=== Build using ROCK ===")
	set(ROCK_TEST_ENABLED ON)
	add_definitions(-DPCL_WITH_VIEWPOINT)
	rock_init(slam3d ${SLAM3D_VERSION})
	rock_standard_layout()
elseif(catkin_FOUND)
	message("=== Build using Catkin ===")
	if(NOT EXISTS ${PROJECT_SOURCE_DIR}/include)
		execute_process(
			COMMAND mkdir ${PROJECT_SOURCE_DIR}/include
			COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROJECT_SOURCE_DIR}/src ${PROJECT_SOURCE_DIR}/include/slam3d)
	endif()
	catkin_package(
		INCLUDE_DIRS include
		LIBRARIES slam3d
		DEPENDS Boost Eigen3 Cholmod G2O
	)
	add_subdirectory(src)
else()
	message("=== Build using standard CMake ===")
	set(BUILD_SHARED_LIBS ON)
	add_subdirectory(src)
endif()

