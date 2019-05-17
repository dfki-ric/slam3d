add_library(sensor-gdal
	GpsSensor.cpp
)

target_include_directories(sensor-gdal
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
		${GDAL_INCLUDE_DIRS}
)

target_link_libraries(sensor-gdal
	PUBLIC core ${GDAL_LIBRARIES}
)

# Install header files
install(
	FILES
		GpsSensor.hpp
	DESTINATION include/slam3d/sensor/gdal
)

# Install the binaries
install(TARGETS sensor-gdal EXPORT slam3d-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Install pkg-config file
configure_file(slam3d_sensor_gdal.pc.in slam3d_sensor_gdal.pc @ONLY)
install(
	FILES ${PROJECT_BINARY_DIR}/slam3d/sensor/gdal/slam3d_sensor_gdal.pc
	DESTINATION lib/pkgconfig
)

set_target_properties(sensor-gdal PROPERTIES OUTPUT_NAME slam3d_sensor_gdal)
add_slam3d_library(slam3d_sensor_gdal)
