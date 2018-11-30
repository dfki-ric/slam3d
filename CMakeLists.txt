cmake_minimum_required(VERSION 2.8)
cmake_policy(VERSION 3.5.1)

project(SLAM3D VERSION "1.0.0")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_subdirectory(slam3d)

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
configure_file(slam3d.pc.in slam3d.pc @ONLY)
install(
	FILES ${PROJECT_BINARY_DIR}/slam3d.pc
	DESTINATION lib/pkgconfig
)

