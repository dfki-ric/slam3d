cmake_minimum_required(VERSION 3.1)

project(slam3d VERSION "3.0.0")
enable_testing()

include(slam3d-dependencies.cmake)
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
