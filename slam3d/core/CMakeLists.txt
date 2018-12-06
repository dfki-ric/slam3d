add_library(core
	Mapper.cpp
	Graph.cpp
)

find_package(Eigen3 NO_MODULE)
if (TARGET Eigen3::Eigen)
	target_link_libraries(core PUBLIC Eigen3::Eigen)
else()
  find_package(Eigen3 REQUIRED)
  target_include_directories(core PUBLIC ${EIGEN3_INCLUDE_DIR})
endif ()

target_include_directories(core
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)

# Install header files
install(DIRECTORY ./
	DESTINATION include/slam3d/core
	FILES_MATCHING PATTERN "*.hpp"
)

# Install the binaries
install(TARGETS core EXPORT slam3d-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Install pkg-config file
configure_file(slam3d_core.pc.in slam3d_core.pc @ONLY)
install(
	FILES ${PROJECT_BINARY_DIR}/slam3d/core/slam3d_core.pc
	DESTINATION lib/pkgconfig
)

set_target_properties(core PROPERTIES OUTPUT_NAME slam3d_core)
add_slam3d_library(slam3d_core)
