cmake_minimum_required(VERSION 3.1...3.28.3)
project(slam3d VERSION "3.0.0")

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(slam3d-dependencies.cmake)

enable_testing()

add_subdirectory(slam3d)

# Export the library interface
install(EXPORT slam3d-targets
	NAMESPACE slam3d::
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/slam3d
)

# Create and install the version file
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
		${CMAKE_INSTALL_LIBDIR}/cmake/slam3d
)
