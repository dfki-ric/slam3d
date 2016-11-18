cmake_minimum_required(VERSION 2.6)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(SLAM3D_MAJOR_VERSION 0)
set(SLAM3D_MINOR_VERSION 1)
set(SLAM3D_PATCH_VERSION 0)
set(SLAM3D_VERSION ${SLAM3D_MAJOR_VERSION}.${SLAM3D_MINOR_VERSION}.${SLAM3DSLAM3D_PATCH_VERSION})

find_package(Rock)
find_package(catkin)

if(Rock_FOUND)
	message("Build using ROCK")
	set(ROCK_TEST_ENABLED ON)
	rock_init(slam3d ${SLAM3D_VERSION})
	rock_standard_layout()
elseif(catkin_FOUND)
	message("Build using Catkin")
else()
	message("Build using standard CMake")
endif()

