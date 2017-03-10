cmake_minimum_required(VERSION 2.6)
project(slam3d)

set(SLAM3D_MAJOR_VERSION 0)
set(SLAM3D_MINOR_VERSION 1)
set(SLAM3D_PATCH_VERSION 0)
set(SLAM3D_VERSION ${SLAM3D_MAJOR_VERSION}.${SLAM3D_MINOR_VERSION}.${SLAM3D_PATCH_VERSION})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_definitions(-std=c++11)
set(BUILD_SHARED_LIBS ON)

find_package(PCL 1.7 REQUIRED COMPONENTS registration)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Cholmod REQUIRED)
find_package(G2O REQUIRED)

include_directories(
	src
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	${G2O_INCLUDE_DIR}
	${CHOLMOD_INCLUDE_DIR}
)

add_library(slam3d 
	src/GraphMapper.cpp
	src/BoostMapper.cpp
	src/PointCloudSensor.cpp
	src/G2oSolver.cpp
)

target_link_libraries(slam3d
	${CHOLMOD_LIBRARIES}
	${G2O_CORE_LIBRARY}
	${G2O_TYPES_SLAM3D}
	${G2O_SOLVER_CHOLMOD}
	${PCL_REGISTRATION_LIBRARIES}
)

install(TARGETS slam3d
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

install(DIRECTORY src/
	DESTINATION include/slam3d
	FILES_MATCHING PATTERN "*.hpp"
)

install(FILES package.xml
	DESTINATION share/slam3d
)

