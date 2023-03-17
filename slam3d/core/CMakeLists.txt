add_library(core SHARED
	Mapper.cpp
	Graph.cpp
	Sensor.cpp
	ScanSensor.cpp
	Types.cpp
	DataStorage.cpp
)

target_include_directories(core
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)

target_link_libraries(core
	PUBLIC
		Eigen3::Eigen
		Boost::thread
		PkgConfig::flann
	PRIVATE
		PkgConfig::pqxx
)

target_compile_features(core PUBLIC cxx_alias_templates)

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

set_target_properties(core PROPERTIES
	OUTPUT_NAME slam3d_core
)
