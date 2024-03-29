add_library(sensor-pointmatcher SHARED
	Scan2DSensor.cpp
)

target_include_directories(sensor-pointmatcher
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)

target_link_libraries(sensor-pointmatcher
	PUBLIC
		core
		libpointmatcher::libpointmatcher
)

# Install header files
install(
	FILES
		Scan2DSensor.hpp
	DESTINATION include/slam3d/sensor/pointmatcher
)

# Install the binaries
install(TARGETS sensor-pointmatcher EXPORT slam3d-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Install pkg-config file
configure_file(slam3d_sensor_pointmatcher.pc.in slam3d_sensor_pointmatcher.pc @ONLY)
install(
	FILES ${PROJECT_BINARY_DIR}/slam3d/sensor/pointmatcher/slam3d_sensor_pointmatcher.pc
	DESTINATION lib/pkgconfig
)

set_target_properties(sensor-pointmatcher PROPERTIES
	OUTPUT_NAME slam3d_sensor_pointmatcher
)
