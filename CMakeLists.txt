cmake_minimum_required(VERSION 3.1)

project(SLAM3D VERSION "2.4.0")
enable_testing()

include(slam3d-dependencies.cmake)

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
write_basic_package_version_file("slam3d-config-version.cmake"
	VERSION ${SLAM3D_VERSION}
	COMPATIBILITY SameMajorVersion
)

install(
	FILES
		slam3d-config.cmake
		slam3d-dependencies.cmake
		${PROJECT_BINARY_DIR}/slam3d-config-version.cmake
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
