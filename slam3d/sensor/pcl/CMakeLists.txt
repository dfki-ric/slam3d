find_package(PCL 1.7 REQUIRED COMPONENTS registration)

add_library(sensor-pcl
	PointCloudSensor.cpp
)

target_include_directories(sensor-pcl
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
		${PCL_INCLUDE_DIRS}
)

target_link_libraries(sensor-pcl
	PUBLIC core ${PCL_REGISTRATION_LIBRARIES}
)

# Install header files
install(
	FILES
		PointCloudSensor.hpp
		GICPConfiguration.hpp
	DESTINATION include/slam3d/sensor/pcl
)

# Install the binaries
install(TARGETS sensor-pcl EXPORT slam3d-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Install pkg-config file
configure_file(slam3d_sensor_pcl.pc.in slam3d_sensor_pcl.pc @ONLY)
install(
	FILES ${PROJECT_BINARY_DIR}/slam3d/sensor/pcl/slam3d_sensor_pcl.pc
	DESTINATION lib/pkgconfig
)

set_target_properties(sensor-pcl PROPERTIES OUTPUT_NAME slam3d_sensor_pcl)
add_slam3d_library(slam3d_sensor_pcl)
