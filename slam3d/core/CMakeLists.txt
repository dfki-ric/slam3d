find_package(Eigen3 REQUIRED)

add_library(core
	Mapper.cpp
	Graph.cpp
)

target_include_directories(core
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
		${EIGEN3_INCLUDE_DIR}
	)

# Install the binaries
install(TARGETS core EXPORT core-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Export the library interface
install(EXPORT core-targets
	NAMESPACE slam3d::
	DESTINATION lib/cmake/slam3d
)

set_target_properties(core PROPERTIES OUTPUT_NAME slam3d_core)
