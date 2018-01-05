cmake_minimum_required(VERSION 2.6)
cmake_policy(VERSION 3.5.1)

project(SLAM3D VERSION "1.0.0")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_definitions(-std=c++11)

if(PCL_WITH_VIEWPOINT)
	add_definitions(-DPCL_WITH_VIEWPOINT)
endif()

set(BUILD_SHARED_LIBS ON)

find_package(PCL 1.7 REQUIRED COMPONENTS registration)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Cholmod REQUIRED)
find_package(G2O REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(
	src
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	${G2O_INCLUDE_DIR}
	${CHOLMOD_INCLUDE_DIR}
)

add_library(slam3d
	src/Graph.cpp
	src/BoostGraph.cpp
	src/PointCloudSensor.cpp
	src/G2oSolver.cpp
)

target_link_libraries(slam3d
	${CHOLMOD_LIBRARIES}
	${G2O_CORE_LIBRARY}
	${G2O_STUFF_LIBRARY}
	${G2O_TYPES_SLAM3D}
	${G2O_SOLVER_CHOLMOD}
	${PCL_REGISTRATION_LIBRARIES}
)

# Install the binaries
install(TARGETS slam3d
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY src/
	DESTINATION include/slam3d
	FILES_MATCHING PATTERN "*.hpp"
)

# Install catkin package-manifest
install(FILES package.xml
	DESTINATION share/slam3d
)

# Install cmake macro
install(FILES cmake/FindSLAM3D.cmake
	DESTINATION lib/cmake/slam3d
)

# Install pkg-config file
configure_file(slam3d.pc.in slam3d.pc @ONLY)
install(FILES ${PROJECT_BINARY_DIR}/slam3d.pc
	DESTINATION lib/pkgconfig
)

# Generate test cases
#enable_testing()
#find_package(Boost COMPONENTS unit_test_framework REQUIRED)
#include_directories (${Boost_INCLUDE_DIRS})
#add_definitions(-DBOOST_TEST_DYN_LINK)

#file(GLOB TEST_SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} test/*.cpp)
#foreach(testSrc ${TEST_SRCS})
#	get_filename_component(testName ${testSrc} NAME_WE)
#	add_executable(test_${testName} ${testSrc})
#	target_link_libraries(test_${testName} ${Boost_LIBRARIES} slam3d)
#	add_test(${testName} test_${testName}) 
#endforeach(testSrc)
