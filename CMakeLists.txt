cmake_minimum_required(VERSION 2.8)
cmake_policy(VERSION 3.5.1)

project(SLAM3D VERSION "1.0.0")

# Find all dependencies
find_package(Eigen3 NO_MODULE)
if (NOT TARGET Eigen3::Eigen)
	find_package(Eigen3 REQUIRED)
	add_library(Eigen3::Eigen INTERFACE IMPORTED)
	set_property(TARGET Eigen3::Eigen APPEND PROPERTY
		INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(Boost REQUIRED COMPONENTS graph)
find_package(PCL 1.7 REQUIRED COMPONENTS registration)
find_package(g2o REQUIRED)

# Define a global property to hold all libraries. This can be
# used to create a pkg-config file for all of SLAM3D.
set_property(GLOBAL PROPERTY SLAM3D_ALL_LIBRARIES)
function(add_slam3d_library)
    get_property(tmp GLOBAL PROPERTY SLAM3D_ALL_LIBRARIES)
    foreach(arg ${ARGV})
        set(tmp "${tmp} ${arg}")
    endforeach()
    set_property(GLOBAL PROPERTY SLAM3D_ALL_LIBRARIES "${tmp}")
endfunction(add_slam3d_library)

add_subdirectory(slam3d)

# Export the library interface
install(EXPORT slam3d-targets
	NAMESPACE slam3d::
	DESTINATION lib/cmake/slam3d
)

# Create and install the version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file("slam3d-version.cmake"
	VERSION ${SLAM3D_VERSION}
	COMPATIBILITY SameMajorVersion
)

install(
	FILES
		slam3d-config.cmake
		${PROJECT_BINARY_DIR}/slam3d-version.cmake
	DESTINATION
		lib/cmake/slam3d
)

# Install catkin package-manifest
install(
	FILES package.xml
	DESTINATION share/slam3d
)

# Install pkg-config file
get_property(PKG_REQUIRES GLOBAL PROPERTY SLAM3D_ALL_LIBRARIES)
configure_file(slam3d.pc.in slam3d.pc @ONLY)
install(
	FILES ${PROJECT_BINARY_DIR}/slam3d.pc
	DESTINATION lib/pkgconfig
)
