add_library(sensor-scan
	ScanSensor.cpp
)

target_include_directories(sensor-scan
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)

target_link_libraries(sensor-scan
	PUBLIC core
)

# Install header files
install(
	FILES
		ScanSensor.hpp
	DESTINATION include/slam3d/sensor/scan
)

# Install the binaries
install(TARGETS sensor-scan EXPORT slam3d-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Install pkg-config file
configure_file(slam3d_sensor_scan.pc.in slam3d_sensor_scan.pc @ONLY)
install(
	FILES ${PROJECT_BINARY_DIR}/slam3d/sensor/scan/slam3d_sensor_scan.pc
	DESTINATION lib/pkgconfig
)

set_target_properties(sensor-scan PROPERTIES OUTPUT_NAME slam3d_sensor_scan)
add_slam3d_library(slam3d_sensor_scan)
