find_package(PCL 1.7 REQUIRED COMPONENTS registration)

# Define core library
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
	PUBLIC core
)

# Install header files
install(
	FILES
		PointCloudSensor.hpp
		GICPConfiguration.hpp
	DESTINATION include/slam3d
)

# Install the binaries
install(TARGETS sensor-pcl EXPORT sensor-pcl-targets
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION bin
)

# Export the library interface
install(EXPORT sensor-pcl-targets
	NAMESPACE slam3d::
	DESTINATION lib/cmake/slam3d
)

set_target_properties(sensor-pcl PROPERTIES OUTPUT_NAME slam3d_sensor_pcl)

