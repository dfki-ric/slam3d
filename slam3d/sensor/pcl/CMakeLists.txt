add_library(sensor-pcl SHARED
	PointCloudSensor.cpp
)

target_include_directories(sensor-pcl
	PUBLIC
		$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
		${PCL_INCLUDE_DIRS}
)

if(TARGET pclomp::registration)
	set(SLAM3D_REGISTRATION pclomp::registration)
	target_compile_definitions(sensor-pcl PRIVATE USE_PCLOMP)
else()
	set(SLAM3D_REGISTRATION ${PCL_REGISTRATION_LIBRARIES})
endif()

target_link_libraries(sensor-pcl
	PUBLIC
		core
	PRIVATE
		Boost::serialization
		${SLAM3D_REGISTRATION}
		${PCL_SAMPLE_CONCENSUS_LIBRARIES}
		${PCL_LIBRARIES}
)

# Install header files
install(
	FILES
		PointCloudSensor.hpp
		RegistrationParameters.hpp
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

set_target_properties(sensor-pcl PROPERTIES
	OUTPUT_NAME slam3d_sensor_pcl
)

# Build test
add_executable(pcl_sensor_test PointCloudSensorTest.cpp)

target_link_libraries(pcl_sensor_test
	Boost::unit_test_framework
	Boost::system
	${PCL_IO_LIBRARIES}
	sensor-pcl)

target_compile_definitions(pcl_sensor_test PRIVATE -DBOOST_TEST_DYN_LINK)
add_test(pcl_sensor pcl_sensor_test)
