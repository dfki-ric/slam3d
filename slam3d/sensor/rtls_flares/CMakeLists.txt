add_library(sensor-rtls
	Status.cpp
	FlareSensor.cpp
)

target_include_directories(sensor-rtls
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)

target_link_libraries(sensor-rtls
	PUBLIC core jsoncpp_lib
)

# Install header files
install(
	FILES
		FlareSensor.hpp
		Status.hpp
	DESTINATION include/slam3d/sensor/rtls_flares
)

# Install the binaries
install(TARGETS sensor-rtls EXPORT slam3d-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Install pkg-config file
configure_file(slam3d_sensor_rtls_flares.pc.in slam3d_sensor_rtls_flares.pc @ONLY)
install(
	FILES ${PROJECT_BINARY_DIR}/slam3d/sensor/rtls_flares/slam3d_sensor_rtls_flares.pc
	DESTINATION lib/pkgconfig
)

set_target_properties(sensor-rtls PROPERTIES OUTPUT_NAME slam3d_sensor_rtls_flares)
add_slam3d_library(slam3d_sensor_rtls_flares)
